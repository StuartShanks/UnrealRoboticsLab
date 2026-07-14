// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Components/Controllers/MjMinkIKController.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"
#include "mujoco/mujoco.h"

// ============================================================================
// Rung C: URLab-integrated TidyBot mink IK repro. Runs the SAME scenario as
// Rung B (Source/URLabMink/Private/Tests/MinkTidybotTests.cpp) but through
// URLab's real layers: the MJCF importer (FMjXmlImportSession) and the real
// UMjMinkIKController component (solve -> ApplyControls -> mj_step). A
// NaN/instability failure in ClosedLoopStable is an EXPECTED, VALUABLE
// outcome here — it reproduces a known live-editor bug in a deterministic
// headless harness.
// ============================================================================

namespace MjMinkIKControllerTestsLocal // unity build: unique namespace name
{
constexpr int32 NSteps = 2500;
constexpr int32 MaxIters = 20;
constexpr double PosThreshold = 1e-4;
constexpr double OriThreshold = 1e-4;

// Same 10 drive joints as Rung B, in the same order: 3 base + 7 arm (no gripper).
const TCHAR* JointNames[10] = {TEXT("joint_x"), TEXT("joint_y"), TEXT("joint_th"),
	TEXT("joint_1"), TEXT("joint_2"), TEXT("joint_3"), TEXT("joint_4"),
	TEXT("joint_5"), TEXT("joint_6"), TEXT("joint_7")};

/** The shared deterministic target script — MUST match Rung B / gen_tidybot_trace.py. */
FVector TidybotTargetPos(int32 K, const FVector& P0)
{
	const FVector Reach(0.0, 0.35, 0.15);
	const FVector Far(0.9, 0.0, 0.0);
	if (K < 300)
		return P0;
	if (K < 400)
		return P0 + (double(K - 300) / 100.0) * Reach;
	if (K < 900)
		return P0 + Reach;
	if (K < 1000)
	{
		const double S = double(K - 900) / 100.0;
		return P0 + (1.0 - S) * Reach + S * Far;
	}
	const double Theta = 2.0 * PI * double(K - 1000) / 1200.0;
	return P0 + Far + FVector(0.4 * (FMath::Cos(Theta) - 1.0), 0.4 * FMath::Sin(Theta), 0.0);
}

/** Resolve a compiled MuJoCo object id by name suffix (import may prefix names). */
int32 FindIdBySuffix(const mjModel* M, mjtObj ObjType, int32 Count, const TCHAR* Suffix)
{
	for (int32 i = 0; i < Count; ++i)
	{
		const char* Nm = mj_id2name(const_cast<mjModel*>(M), ObjType, i);
		if (Nm && FString(UTF8_TO_TCHAR(Nm)).EndsWith(Suffix))
		{
			return i;
		}
	}
	return -1;
}

template <typename TComp>
TComp* FindComponentByMjId(const TArray<TComp*>& Comps, int32 MjId)
{
	for (TComp* C : Comps)
	{
		if (C && C->GetMjID() == MjId)
		{
			return C;
		}
	}
	return nullptr;
}

/** Copies a JSON number array's first N elements into Out. Returns false if too short. */
bool JsonArrayToDoubles(const TArray<TSharedPtr<FJsonValue>>* Arr, int32 N, double* Out)
{
	if (!Arr || Arr->Num() < N)
	{
		return false;
	}
	for (int32 i = 0; i < N; ++i)
	{
		Out[i] = (*Arr)[i]->AsNumber();
	}
	return true;
}

/**
 * Minimal local load of the golden trace JSON — URLabEditor has no dependency on
 * URLabMink (see URLabEditor.Build.cs), so MinkLoadFixture isn't reachable here.
 * We only need per-step target_pos/target_quat/ctrl, not the full MinkTestUtils
 * machinery (Eigen conversions, non-finite string decoding — none of which the
 * golden trace's target/ctrl fields ever contain).
 */
bool LoadTidybotTraceLocal(TSharedPtr<FJsonObject>& OutRoot)
{
	const FString Path = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/fixtures/tidybot_trace.json"));
	FString Contents;
	if (!FFileHelper::LoadFileToString(Contents, *Path))
	{
		return false;
	}
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Contents);
	return FJsonSerializer::Deserialize(Reader, OutRoot) && OutRoot.IsValid();
}
} // namespace MjMinkIKControllerTestsLocal

// ============================================================================
// URLab.MinkIK.TidyBot.ImportCompiles
//   Imports the merged TidyBot scene through URLab's pipeline and turns the
//   top NaN suspects (sim options, home keyframe, base actuator gains) into
//   pass/fail facts.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotImport,
	"URLab.MinkIK.TidyBot.ImportCompiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotImport::RunTest(const FString&)
{
	using namespace MjMinkIKControllerTestsLocal;

	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"));
	if (!FPaths::FileExists(XmlPath))
	{
		AddError(TEXT("fixture missing — run Task 4 Step 1"));
		return false;
	}

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Model();
	TestEqual(TEXT("nq"), (int32)M->nq, 18);
	TestEqual(TEXT("nu"), (int32)M->nu, 11);

	// Native compile of tidybot.xml — ground truth for both the home-keyframe
	// content check (Suspect 2, below) and the field-by-field actuator
	// gain/bias comparison further down. Loaded once and reused by both.
	const FString NativeXml = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot.xml"));
	char NativeErr[1024] = {0};
	mjModel* NativeModel = mj_loadXML(TCHAR_TO_UTF8(*NativeXml), nullptr, NativeErr, sizeof(NativeErr));
	TestNotNull(TEXT("native tidybot.xml compiled for keyframe/gain comparison"), NativeModel);

	// Suspect 1: the model's sim options must survive the import pipeline.
	TestEqual(TEXT("integrator == implicitfast"), (int32)M->opt.integrator, (int32)mjINT_IMPLICITFAST);
	TestEqual(TEXT("cone == elliptic"), (int32)M->opt.cone, (int32)mjCONE_ELLIPTIC);
	TestEqual(TEXT("impratio == 10"), M->opt.impratio, 10.0);

	// Suspect 2: home keyframe must survive with matching qpos CONTENT, not just
	// nq (suffix match — import may prefix). Compares element-by-element against
	// MuJoCo's own compile of tidybot.xml, tolerance 1e-6.
	const int32 KeyId = FindIdBySuffix(M, mjOBJ_KEY, M->nkey, TEXT("home"));
	TestTrue(TEXT("home keyframe exists"), KeyId >= 0);
	if (KeyId >= 0)
	{
		TestEqual(TEXT("home keyframe has 18 qpos"), (int32)M->nq, 18);

		if (NativeModel)
		{
			const int32 NativeKeyId = FindIdBySuffix(NativeModel, mjOBJ_KEY, NativeModel->nkey, TEXT("home"));
			if (TestTrue(TEXT("native home keyframe exists"), NativeKeyId >= 0)
				&& TestEqual(TEXT("nq matches native for keyframe comparison"), (int32)M->nq, (int32)NativeModel->nq))
			{
				const mjtNum* ImportedQpos = M->key_qpos + (size_t)KeyId * M->nq;
				const mjtNum* NativeQpos = NativeModel->key_qpos + (size_t)NativeKeyId * NativeModel->nq;
				double MaxAbsDiff = 0.0;
				for (int32 i = 0; i < M->nq; ++i)
				{
					MaxAbsDiff = FMath::Max(MaxAbsDiff, FMath::Abs((double)ImportedQpos[i] - (double)NativeQpos[i]));
				}
				TestTrue(FString::Printf(TEXT("home key_qpos matches native element-by-element (max |diff|=%.9f)"), MaxAbsDiff),
					MaxAbsDiff <= 1e-6);
			}
		}
	}

	// Base actuator gains survived (kp=1e6 on joint_x — find by name suffix).
	{
		const int32 JxAct = FindIdBySuffix(M, mjOBJ_ACTUATOR, M->nu, TEXT("joint_x"));
		TestTrue(TEXT("joint_x actuator found"), JxAct >= 0);
		if (JxAct >= 0)
		{
			TestEqual(TEXT("joint_x kp == 1e6"), M->actuator_gainprm[JxAct * mjNGAIN + 0], 1000000.0);
			// kv (biasprm[2] == -kv) must also survive — regression guard for the
			// always-non-null dampratio sentinel that used to drop biasprm[2].
			TestEqual(TEXT("joint_x biasprm[2] == -5e4"), M->actuator_biasprm[JxAct * mjNBIAS + 2], -50000.0);
		}
	}

	// Regression guard for Task 5 root cause 1: class-inherited actuator gains must
	// survive import. joint_1 inherits kp/kv from tidybot.xml's <default class=
	// "large_actuator"><position kp="2000" kv="100"/>. The historic -1 sentinel in
	// UMjPositionActuator::ExportTo clobbered gainprm[0] to -1 (positive feedback ->
	// NaN). We assert both against the literal native values AND field-by-field
	// against MuJoCo's own compile of the same XML (the strongest guard: if the
	// importer ever diverges from native gains again, this flips red).
	{
		const int32 J1Act = FindIdBySuffix(M, mjOBJ_ACTUATOR, M->nu, TEXT("joint_1"));
		TestTrue(TEXT("joint_1 actuator found"), J1Act >= 0);
		if (J1Act >= 0)
		{
			// Native values from <default class="large_actuator"> in tidybot.xml.
			TestEqual(TEXT("joint_1 kp (gainprm[0]) == 2000"), M->actuator_gainprm[J1Act * mjNGAIN + 0], 2000.0);
			TestEqual(TEXT("joint_1 biasprm[1] == -kp == -2000"), M->actuator_biasprm[J1Act * mjNBIAS + 1], -2000.0);
			TestEqual(TEXT("joint_1 biasprm[2] == -kv == -100"), M->actuator_biasprm[J1Act * mjNBIAS + 2], -100.0);
		}
	}

	// Field-by-field gain/bias comparison against the native compile of tidybot.xml
	// (NativeModel, loaded once above). NOTE: the imported fixture is
	// tidybot_scene_ue.xml (= tidybot.xml + scene wrapper + mocap target body); the
	// actuator definitions are identical, so native tidybot.xml is a valid
	// gain/bias ground truth (actuators matched by name suffix).
	if (NativeModel)
	{
		for (int32 ni = 0; ni < NativeModel->nu; ++ni)
		{
			const char* NmC = mj_id2name(NativeModel, mjOBJ_ACTUATOR, ni);
			if (!NmC)
			{
				continue;
			}
			const FString Nm = UTF8_TO_TCHAR(NmC);
			const int32 mi = FindIdBySuffix(M, mjOBJ_ACTUATOR, M->nu, *Nm);
			if (!TestTrue(*FString::Printf(TEXT("actuator '%s' present in import"), *Nm), mi >= 0))
			{
				continue;
			}
			TestEqual(*FString::Printf(TEXT("act '%s' gainprm[0]"), *Nm),
				M->actuator_gainprm[mi * mjNGAIN + 0], NativeModel->actuator_gainprm[ni * mjNGAIN + 0]);
			TestEqual(*FString::Printf(TEXT("act '%s' biasprm[1]"), *Nm),
				M->actuator_biasprm[mi * mjNBIAS + 1], NativeModel->actuator_biasprm[ni * mjNBIAS + 1]);
			TestEqual(*FString::Printf(TEXT("act '%s' biasprm[2]"), *Nm),
				M->actuator_biasprm[mi * mjNBIAS + 2], NativeModel->actuator_biasprm[ni * mjNBIAS + 2]);
		}
	}

	if (NativeModel)
	{
		mj_deleteModel(NativeModel);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.MinkIK.TidyBot.ClosedLoopStable
//   Same import; then configure the real UMjMinkIKController with the exact
//   example task stack (Frame on pinch_site, Posture on the 7 arm joints,
//   disabled Damping on the 3 base joints, ConfigurationLimit), stream
//   targets via ApplyConfig (the supported RPC path), and step the sim
//   directly. Finiteness every step; calibrated tracking checkpoints;
//   base-moved check. A NaN/instability failure here is an EXPECTED,
//   VALUABLE outcome — it reproduces the live-editor bug headlessly.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotClosedLoop,
	"URLab.MinkIK.TidyBot.ClosedLoopStable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotClosedLoop::RunTest(const FString&)
{
	using namespace MjMinkIKControllerTestsLocal;

	// --- 1. Import + compile -------------------------------------------------
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"));
	if (!FPaths::FileExists(XmlPath))
	{
		AddError(TEXT("fixture missing — run Task 4 Step 1"));
		return false;
	}

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Model();
	mjData* D = S.Data();
	if (!TestNotNull(TEXT("Robot spawned"), S.Robot) || !TestNotNull(TEXT("Model compiled"), M)
		|| !TestNotNull(TEXT("Data compiled"), D))
	{
		S.Cleanup();
		return false;
	}
	AMjArticulation* Robot = S.Robot;

	// --- 2. Find components (suffix-resolve across the import prefix) --------
	const int32 SiteMjId = FindIdBySuffix(M, mjOBJ_SITE, M->nsite, TEXT("pinch_site"));
	if (!TestTrue(TEXT("pinch_site found in compiled model"), SiteMjId >= 0))
	{
		S.Cleanup();
		return false;
	}

	TArray<UMjSite*> SiteComps;
	Robot->GetComponents<UMjSite>(SiteComps);
	UMjSite* PinchSite = FindComponentByMjId(SiteComps, SiteMjId);
	if (!TestNotNull(TEXT("pinch_site UMjSite component resolved"), PinchSite))
	{
		S.Cleanup();
		return false;
	}

	TArray<UMjJoint*> JointComps;
	Robot->GetComponents<UMjJoint>(JointComps);
	TArray<TObjectPtr<UMjJoint>> TenJoints;
	bool bAllJointsFound = true;
	for (const TCHAR* Nm : JointNames)
	{
		const int32 JMjId = FindIdBySuffix(M, mjOBJ_JOINT, M->njnt, Nm);
		UMjJoint* J = (JMjId >= 0) ? FindComponentByMjId(JointComps, JMjId) : nullptr;
		if (!J)
		{
			AddError(FString::Printf(TEXT("joint '%s' not resolved (mjId=%d)"), Nm, JMjId));
			bAllJointsFound = false;
			continue;
		}
		TenJoints.Add(J);
	}
	if (!bAllJointsFound || TenJoints.Num() != 10)
	{
		S.Cleanup();
		return false;
	}
	TArray<TObjectPtr<UMjJoint>> BaseJoints = {TenJoints[0], TenJoints[1], TenJoints[2]};
	TArray<TObjectPtr<UMjJoint>> ArmJoints = {TenJoints[3], TenJoints[4], TenJoints[5],
		TenJoints[6], TenJoints[7], TenJoints[8], TenJoints[9]};

	// --- 3. Create + configure the controller (not yet bound) ----------------
	UMjMinkIKController* Ctrl = NewObject<UMjMinkIKController>(Robot, TEXT("MinkIK"));

	FMinkTaskSpec Frame;
	Frame.Kind = EMinkTaskKind::Frame;
	Frame.Frame = PinchSite;
	Frame.TargetMocapBody = nullptr; // targets streamed via ApplyConfig (mocap is UE-authoritative)
	Frame.PositionCost = 1.0f;
	Frame.OrientationCost = 1.0f;
	Frame.LmDamping = 1.0f;

	FMinkTaskSpec Posture;
	Posture.Kind = EMinkTaskKind::Posture;
	Posture.Cost = 1e-3f;
	// Deliberately narrower than the literal example's cost[3:] (which spans
	// every non-base DOF, including the gripper): here it's arm joints only.
	// The gripper is undriven in this rig, so the divergence has no effect.
	Posture.Joints = ArmJoints;

	FMinkTaskSpec Damping;
	Damping.Kind = EMinkTaskKind::Damping;
	Damping.Cost = 100.0f;
	Damping.Joints = BaseJoints;
	Damping.bEnabled = false;

	Ctrl->Tasks = {Frame, Posture, Damping};

	FMinkLimitSpec ConfLimit; // mink.ConfigurationLimit(model) — Kind/Gain default to Configuration/0.95
	Ctrl->Limits = {ConfLimit};

	Ctrl->DriveJoints = TenJoints; // EXACTLY the example's 10 (not the gripper)
	Ctrl->MaxIters = MaxIters;
	Ctrl->PosThreshold = PosThreshold;
	Ctrl->OriThreshold = OriThreshold;
	Ctrl->RegisterComponent();

	// --- 4. Reset to home keyframe, mj_forward, capture P0/Q0 ----------------
	const int32 KeyId = FindIdBySuffix(M, mjOBJ_KEY, M->nkey, TEXT("home"));
	if (!TestTrue(TEXT("home keyframe found"), KeyId >= 0))
	{
		S.Cleanup();
		return false;
	}
	mj_resetDataKeyframe(M, D, KeyId);
	mj_forward(M, D);

	const FVector P0(D->site_xpos[3 * SiteMjId + 0], D->site_xpos[3 * SiteMjId + 1], D->site_xpos[3 * SiteMjId + 2]);
	double Q0[4];
	mju_mat2Quat(Q0, D->site_xmat + 9 * SiteMjId);

	// --- 5. Bind the controller against the live model -----------------------
	// AdoptRuntimeController builds the ActuatorIdMap from the articulation's
	// bound actuators (the same source PostSetup uses) and Bind()s the
	// controller before publishing it as CachedController — exactly the
	// "add a controller after compile" path the bridge uses.
	Robot->AdoptRuntimeController(Ctrl);
	if (!TestTrue(TEXT("controller bound"), Ctrl->IsBound()))
	{
		S.Cleanup();
		return false;
	}

	// --- 6. Closed loop: stream target -> solve -> step -----------------------
	const TSet<int32> Checkpoints = {299, 899, 999, 1600, 2200, 2499};
	bool bDiverged = false;
	for (int32 K = 0; K < NSteps; ++K)
	{
		const FVector TPos = TidybotTargetPos(K, P0);

		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> PosArr;
		PosArr.Add(MakeShared<FJsonValueNumber>(TPos.X));
		PosArr.Add(MakeShared<FJsonValueNumber>(TPos.Y));
		PosArr.Add(MakeShared<FJsonValueNumber>(TPos.Z));
		Cfg->SetArrayField(TEXT("target_pos"), PosArr);
		TArray<TSharedPtr<FJsonValue>> QuatArr;
		for (int32 i = 0; i < 4; ++i)
		{
			QuatArr.Add(MakeShared<FJsonValueNumber>(Q0[i]));
		}
		Cfg->SetArrayField(TEXT("target_quat"), QuatArr);
		Ctrl->ApplyConfig(Cfg);

		Ctrl->ComputeAndApply(M, D, 0);
		mj_step(M, D);

		for (int32 v = 0; v < M->nv; ++v)
		{
			if (!FMath::IsFinite(D->qacc[v]) || !FMath::IsFinite(D->qvel[v]))
			{
				FString CtrlStr;
				for (int32 a = 0; a < M->nu; ++a)
				{
					CtrlStr += FString::Printf(TEXT("%.4f "), D->ctrl[a]);
				}
				AddError(FString::Printf(
					TEXT("NON-FINITE qacc/qvel at step %d dof %d — ctrl=[%s]"), K, v, *CtrlStr));
				bDiverged = true;
				break;
			}
		}
		if (bDiverged)
		{
			break;
		}

		if (Checkpoints.Contains(K))
		{
			// Tolerances calibrated against the Python run (see task brief): 0.02 m at
			// settled checkpoints (299, 899); 0.30 m at 999 (actuator-lag transient at
			// the ramp corner); 0.10 m at the remaining moving checkpoints. NOTE the
			// controller integrates with sim dt (0.002) not the example's 0.005 — a
			// documented divergence; this rung asserts stability + tracking only, not
			// trace-parity vs Python.
			const double Tol = (K == 299 || K == 899) ? 0.02 : (K == 999 ? 0.30 : 0.10);
			const FVector Ee(D->site_xpos[3 * SiteMjId + 0], D->site_xpos[3 * SiteMjId + 1],
				D->site_xpos[3 * SiteMjId + 2]);
			const double TrackErr = (Ee - TPos).Length();
			TestTrue(FString::Printf(TEXT("checkpoint %d: track err %.4f <= %.2f m"), K, TrackErr, Tol),
				TrackErr <= Tol);
		}
	}

	if (!bDiverged)
	{
		// The base must actually have driven (far-circle phase; Damping/fix-base disabled).
		const int32 XQposAdr = M->jnt_qposadr[TenJoints[0]->GetMjID()];
		const int32 YQposAdr = M->jnt_qposadr[TenJoints[1]->GetMjID()];
		TestTrue(TEXT("base moved > 0.2 m"),
			FMath::Sqrt(FMath::Square(D->qpos[XQposAdr]) + FMath::Square(D->qpos[YQposAdr])) > 0.2);
	}

	S.Cleanup();
	return !bDiverged;
}

// ============================================================================
// URLab.MinkIK.TidyBot.BindSurvivesRefLoss
//   Regression for the PIE/Simulate world-duplication ref-loss bug. A UE world
//   copy nulls the controller's TObjectPtr UPROPERTYs (Frame / DriveJoints /
//   task+limit Joints) on the duplicated actor, so before the fix RebuildFromSpecs
//   skipped the frame task and bound 0 driven actuators. Here we model that exact
//   duplicated state: configure the SAME example stack as ClosedLoopStable but set
//   ONLY the name fields (FrameName / *JointNames / DriveJointNames) and leave every
//   component ref null. If the name-fallback resolution works, Bind rebuilds the
//   full stack from names alone and the closed loop tracks + drives the base — end
//   to end, without touching the controller's private state.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotBindSurvivesRefLoss,
	"URLab.MinkIK.TidyBot.BindSurvivesRefLoss",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotBindSurvivesRefLoss::RunTest(const FString&)
{
	using namespace MjMinkIKControllerTestsLocal;

	// --- 1. Import + compile -------------------------------------------------
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"));
	if (!FPaths::FileExists(XmlPath))
	{
		AddError(TEXT("fixture missing — run Task 4 Step 1"));
		return false;
	}

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Model();
	mjData* D = S.Data();
	if (!TestNotNull(TEXT("Robot spawned"), S.Robot) || !TestNotNull(TEXT("Model compiled"), M)
		|| !TestNotNull(TEXT("Data compiled"), D))
	{
		S.Cleanup();
		return false;
	}
	AMjArticulation* Robot = S.Robot;

	const int32 SiteMjId = FindIdBySuffix(M, mjOBJ_SITE, M->nsite, TEXT("pinch_site"));
	if (!TestTrue(TEXT("pinch_site found in compiled model"), SiteMjId >= 0))
	{
		S.Cleanup();
		return false;
	}

	// --- 2. Configure the controller with NAMES ONLY — every component ref
	//        left null, exactly as a duplicated (PIE/Simulate) world leaves it.
	UMjMinkIKController* Ctrl = NewObject<UMjMinkIKController>(Robot, TEXT("MinkIKRefLoss"));

	// Raw MjNames (same short names add_controller captures from JSON) — the
	// controller resolves them against the compiled model by exact/suffix match.
	const TArray<FString> ArmNames = {TEXT("joint_1"), TEXT("joint_2"), TEXT("joint_3"),
		TEXT("joint_4"), TEXT("joint_5"), TEXT("joint_6"), TEXT("joint_7")};
	const TArray<FString> BaseNames = {TEXT("joint_x"), TEXT("joint_y"), TEXT("joint_th")};
	TArray<FString> AllNames;
	for (const TCHAR* Nm : JointNames)
	{
		AllNames.Add(FString(Nm));
	}

	FMinkTaskSpec Frame;
	Frame.Kind = EMinkTaskKind::Frame;
	Frame.Frame = nullptr;                // ref dropped by duplication
	Frame.FrameName = TEXT("pinch_site"); // name fallback survives it
	Frame.TargetMocapBody = nullptr;
	Frame.PositionCost = 1.0f;
	Frame.OrientationCost = 1.0f;
	Frame.LmDamping = 1.0f;

	FMinkTaskSpec Posture;
	Posture.Kind = EMinkTaskKind::Posture;
	Posture.Cost = 1e-3f;
	Posture.Joints.Reset();        // refs dropped
	Posture.JointNames = ArmNames; // resolved by name

	FMinkTaskSpec Damping;
	Damping.Kind = EMinkTaskKind::Damping;
	Damping.Cost = 100.0f;
	Damping.Joints.Reset();
	Damping.JointNames = BaseNames;
	Damping.bEnabled = false;

	Ctrl->Tasks = {Frame, Posture, Damping};

	FMinkLimitSpec ConfLimit;
	Ctrl->Limits = {ConfLimit};

	Ctrl->DriveJoints.Reset(); // refs dropped
	Ctrl->DriveJointNames = AllNames;
	Ctrl->MaxIters = MaxIters;
	Ctrl->PosThreshold = PosThreshold;
	Ctrl->OriThreshold = OriThreshold;
	Ctrl->RegisterComponent();

	// --- 3. Reset to home, capture P0/Q0, bind -------------------------------
	const int32 KeyId = FindIdBySuffix(M, mjOBJ_KEY, M->nkey, TEXT("home"));
	if (!TestTrue(TEXT("home keyframe found"), KeyId >= 0))
	{
		S.Cleanup();
		return false;
	}
	mj_resetDataKeyframe(M, D, KeyId);
	mj_forward(M, D);

	const FVector P0(D->site_xpos[3 * SiteMjId + 0], D->site_xpos[3 * SiteMjId + 1], D->site_xpos[3 * SiteMjId + 2]);
	double Q0[4];
	mju_mat2Quat(Q0, D->site_xmat + 9 * SiteMjId);

	Robot->AdoptRuntimeController(Ctrl);
	if (!TestTrue(TEXT("controller bound"), Ctrl->IsBound()))
	{
		S.Cleanup();
		return false;
	}

	// Base joint qpos addresses (for the base-drove assert) — resolved directly
	// from the compiled model, independent of any component ref.
	const int32 JxId = FindIdBySuffix(M, mjOBJ_JOINT, M->njnt, TEXT("joint_x"));
	const int32 JyId = FindIdBySuffix(M, mjOBJ_JOINT, M->njnt, TEXT("joint_y"));
	if (!TestTrue(TEXT("base joints found"), JxId >= 0 && JyId >= 0))
	{
		S.Cleanup();
		return false;
	}
	const int32 XQposAdr = M->jnt_qposadr[JxId];
	const int32 YQposAdr = M->jnt_qposadr[JyId];

	// --- 4. Closed loop: same target script; assert tracking + base drive ----
	// If the frame task were skipped (bug) the EE would hold home; if the drive
	// actuators were unresolved (bug) nothing would move at all. Both are proven
	// false by tracking to tolerance AND the base translating on the far phase.
	const TSet<int32> Checkpoints = {299, 999, 2499};
	bool bDiverged = false;
	for (int32 K = 0; K < NSteps; ++K)
	{
		const FVector TPos = TidybotTargetPos(K, P0);

		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> PosArr;
		PosArr.Add(MakeShared<FJsonValueNumber>(TPos.X));
		PosArr.Add(MakeShared<FJsonValueNumber>(TPos.Y));
		PosArr.Add(MakeShared<FJsonValueNumber>(TPos.Z));
		Cfg->SetArrayField(TEXT("target_pos"), PosArr);
		TArray<TSharedPtr<FJsonValue>> QuatArr;
		for (int32 i = 0; i < 4; ++i)
		{
			QuatArr.Add(MakeShared<FJsonValueNumber>(Q0[i]));
		}
		Cfg->SetArrayField(TEXT("target_quat"), QuatArr);
		Ctrl->ApplyConfig(Cfg);

		Ctrl->ComputeAndApply(M, D, 0);
		mj_step(M, D);

		for (int32 v = 0; v < M->nv; ++v)
		{
			if (!FMath::IsFinite(D->qacc[v]) || !FMath::IsFinite(D->qvel[v]))
			{
				AddError(FString::Printf(TEXT("NON-FINITE qacc/qvel at step %d dof %d"), K, v));
				bDiverged = true;
				break;
			}
		}
		if (bDiverged)
		{
			break;
		}

		if (Checkpoints.Contains(K))
		{
			const double Tol = (K == 299) ? 0.02 : (K == 999 ? 0.30 : 0.10);
			const FVector Ee(D->site_xpos[3 * SiteMjId + 0], D->site_xpos[3 * SiteMjId + 1],
				D->site_xpos[3 * SiteMjId + 2]);
			const double TrackErr = (Ee - TPos).Length();
			TestTrue(FString::Printf(TEXT("checkpoint %d: track err %.4f <= %.2f m (frame+arm drive via name)"),
						 K, TrackErr, Tol),
				TrackErr <= Tol);
		}
	}

	if (!bDiverged)
	{
		// Base translation is the end-to-end proof the base drive joints (3 of the
		// 10) resolved by name too — the far-circle phase commands the base.
		const double BaseXY = FMath::Sqrt(FMath::Square(D->qpos[XQposAdr]) + FMath::Square(D->qpos[YQposAdr]));
		TestTrue(FString::Printf(TEXT("base drove > 0.2 m via name fallback (|xy|=%.3f)"), BaseXY), BaseXY > 0.2);
	}

	S.Cleanup();
	return !bDiverged;
}

// ============================================================================
// URLab.MinkIK.TidyBot.BindThenResetNoYank
//   Regression for the stale-open-loop-reference bug: Bind() seeds the internal
//   reference at whatever pose the sim is in (live workflow: the SPAWN pose),
//   and the bridge resets to the `home` keyframe BEFORE any stepping. The
//   backwards-time reset guard in ComputeAndApply never fires for a reset that
//   happens before the first integration (LastSimTime still -1), so the first
//   solve used to run from a spawn-pose reference while the robot sat at home —
//   the first ctrl frame commanded spawn, and kp*(ctrl-qpos) with kp=1e6 base
//   actuators yanked the robot violently (demo logs: "EE moved 0.5974 m").
//   This test encodes the exact ordering: Bind at spawn, THEN reset to home,
//   THEN step — and asserts the FIRST ctrl frame is near HOME qpos (the
//   first-integration re-seed), not spawn, and the sim stays finite.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotBindThenResetNoYank,
	"URLab.MinkIK.TidyBot.BindThenResetNoYank",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotBindThenResetNoYank::RunTest(const FString&)
{
	using namespace MjMinkIKControllerTestsLocal;

	// --- 1. Import + compile -------------------------------------------------
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"));
	if (!FPaths::FileExists(XmlPath))
	{
		AddError(TEXT("fixture missing — run Task 4 Step 1"));
		return false;
	}

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Model();
	mjData* D = S.Data();
	if (!TestNotNull(TEXT("Robot spawned"), S.Robot) || !TestNotNull(TEXT("Model compiled"), M)
		|| !TestNotNull(TEXT("Data compiled"), D))
	{
		S.Cleanup();
		return false;
	}
	AMjArticulation* Robot = S.Robot;

	// --- 2. Find components (same resolution as ClosedLoopStable) ------------
	const int32 SiteMjId = FindIdBySuffix(M, mjOBJ_SITE, M->nsite, TEXT("pinch_site"));
	if (!TestTrue(TEXT("pinch_site found in compiled model"), SiteMjId >= 0))
	{
		S.Cleanup();
		return false;
	}

	TArray<UMjSite*> SiteComps;
	Robot->GetComponents<UMjSite>(SiteComps);
	UMjSite* PinchSite = FindComponentByMjId(SiteComps, SiteMjId);
	if (!TestNotNull(TEXT("pinch_site UMjSite component resolved"), PinchSite))
	{
		S.Cleanup();
		return false;
	}

	TArray<UMjJoint*> JointComps;
	Robot->GetComponents<UMjJoint>(JointComps);
	TArray<TObjectPtr<UMjJoint>> TenJoints;
	bool bAllJointsFound = true;
	int32 CtrlIds[10];
	int32 QposAdrs[10];
	for (int32 i = 0; i < 10; ++i)
	{
		const TCHAR* Nm = JointNames[i];
		const int32 JMjId = FindIdBySuffix(M, mjOBJ_JOINT, M->njnt, Nm);
		UMjJoint* J = (JMjId >= 0) ? FindComponentByMjId(JointComps, JMjId) : nullptr;
		if (!J)
		{
			AddError(FString::Printf(TEXT("joint '%s' not resolved (mjId=%d)"), Nm, JMjId));
			bAllJointsFound = false;
			continue;
		}
		TenJoints.Add(J);
		QposAdrs[i] = M->jnt_qposadr[JMjId];
		CtrlIds[i] = FindIdBySuffix(M, mjOBJ_ACTUATOR, M->nu, Nm);
		if (CtrlIds[i] < 0)
		{
			AddError(FString::Printf(TEXT("actuator '%s' not resolved"), Nm));
			bAllJointsFound = false;
		}
	}
	if (!bAllJointsFound || TenJoints.Num() != 10)
	{
		S.Cleanup();
		return false;
	}
	TArray<TObjectPtr<UMjJoint>> BaseJoints = {TenJoints[0], TenJoints[1], TenJoints[2]};
	TArray<TObjectPtr<UMjJoint>> ArmJoints = {TenJoints[3], TenJoints[4], TenJoints[5],
		TenJoints[6], TenJoints[7], TenJoints[8], TenJoints[9]};

	// --- 3. Same example stack as ClosedLoopStable ----------------------------
	UMjMinkIKController* Ctrl = NewObject<UMjMinkIKController>(Robot, TEXT("MinkIKNoYank"));

	FMinkTaskSpec Frame;
	Frame.Kind = EMinkTaskKind::Frame;
	Frame.Frame = PinchSite;
	Frame.TargetMocapBody = nullptr;
	Frame.PositionCost = 1.0f;
	Frame.OrientationCost = 1.0f;
	Frame.LmDamping = 1.0f;

	FMinkTaskSpec Posture;
	Posture.Kind = EMinkTaskKind::Posture;
	Posture.Cost = 1e-3f;
	Posture.Joints = ArmJoints;

	FMinkTaskSpec Damping;
	Damping.Kind = EMinkTaskKind::Damping;
	Damping.Cost = 100.0f;
	Damping.Joints = BaseJoints;
	Damping.bEnabled = false;

	Ctrl->Tasks = {Frame, Posture, Damping};

	FMinkLimitSpec ConfLimit;
	Ctrl->Limits = {ConfLimit};

	Ctrl->DriveJoints = TenJoints;
	Ctrl->MaxIters = MaxIters;
	Ctrl->PosThreshold = PosThreshold;
	Ctrl->OriThreshold = OriThreshold;
	Ctrl->RegisterComponent();

	// --- 4. THE ORDERING UNDER TEST: Bind at the SPAWN pose (no reset first) --
	// This is the live workflow: add_controller binds against the freshly
	// compiled sim, and only afterwards does the bridge reset to `home`.
	mj_forward(M, D);
	double SpawnQpos[10];
	for (int32 i = 0; i < 10; ++i)
	{
		SpawnQpos[i] = D->qpos[QposAdrs[i]];
	}

	Robot->AdoptRuntimeController(Ctrl);
	if (!TestTrue(TEXT("controller bound"), Ctrl->IsBound()))
	{
		S.Cleanup();
		return false;
	}

	// --- 5. NOW reset to the home keyframe (after Bind, before any stepping) --
	const int32 KeyId = FindIdBySuffix(M, mjOBJ_KEY, M->nkey, TEXT("home"));
	if (!TestTrue(TEXT("home keyframe found"), KeyId >= 0))
	{
		S.Cleanup();
		return false;
	}
	mj_resetDataKeyframe(M, D, KeyId);
	mj_forward(M, D);

	double HomeQpos[10];
	double MaxSpawnHomeDelta = 0.0;
	for (int32 i = 0; i < 10; ++i)
	{
		HomeQpos[i] = D->qpos[QposAdrs[i]];
		MaxSpawnHomeDelta = FMath::Max(MaxSpawnHomeDelta, FMath::Abs(HomeQpos[i] - SpawnQpos[i]));
	}
	// Precondition: spawn and home must actually differ, or this test pins nothing.
	if (!TestTrue(FString::Printf(TEXT("spawn differs from home (max delta %.3f > 0.2)"), MaxSpawnHomeDelta),
			MaxSpawnHomeDelta > 0.2))
	{
		S.Cleanup();
		return false;
	}

	const FVector P0(D->site_xpos[3 * SiteMjId + 0], D->site_xpos[3 * SiteMjId + 1], D->site_xpos[3 * SiteMjId + 2]);
	double Q0[4];
	mju_mat2Quat(Q0, D->site_xmat + 9 * SiteMjId);

	// --- 6. Stream one reachable target at the HOME EE pose, then step --------
	{
		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> PosArr;
		PosArr.Add(MakeShared<FJsonValueNumber>(P0.X));
		PosArr.Add(MakeShared<FJsonValueNumber>(P0.Y));
		PosArr.Add(MakeShared<FJsonValueNumber>(P0.Z));
		Cfg->SetArrayField(TEXT("target_pos"), PosArr);
		TArray<TSharedPtr<FJsonValue>> QuatArr;
		for (int32 i = 0; i < 4; ++i)
		{
			QuatArr.Add(MakeShared<FJsonValueNumber>(Q0[i]));
		}
		Cfg->SetArrayField(TEXT("target_quat"), QuatArr);
		Ctrl->ApplyConfig(Cfg);
	}

	bool bDiverged = false;
	for (int32 K = 0; K < 200; ++K)
	{
		Ctrl->ComputeAndApply(M, D, 0);

		if (K == 0)
		{
			// THE assert: the very first ctrl frame must command ~HOME (the pose
			// the robot is actually in), NOT spawn (radians away for several
			// joints). Without the first-integration re-seed the reference is
			// still the spawn pose Bind captured, and this fails.
			for (int32 i = 0; i < 10; ++i)
			{
				const double Ctl = D->ctrl[CtrlIds[i]];
				TestTrue(FString::Printf(
							 TEXT("first ctrl frame ~home for '%s': |%.4f - %.4f| = %.4f < 0.05 (spawn=%.4f)"),
							 JointNames[i], Ctl, HomeQpos[i], FMath::Abs(Ctl - HomeQpos[i]), SpawnQpos[i]),
					FMath::Abs(Ctl - HomeQpos[i]) < 0.05);
			}
		}

		mj_step(M, D);

		for (int32 v = 0; v < M->nv; ++v)
		{
			if (!FMath::IsFinite(D->qacc[v]) || !FMath::IsFinite(D->qvel[v]))
			{
				AddError(FString::Printf(TEXT("NON-FINITE qacc/qvel at step %d dof %d"), K, v));
				bDiverged = true;
				break;
			}
		}
		if (bDiverged)
		{
			break;
		}
	}

	S.Cleanup();
	return !bDiverged;
}

// ============================================================================
// URLab.MinkIK.TidyBot.CtrlParity
//   Drives the REAL controller (same stack as ClosedLoopStable) on the imported
//   model with the golden trace's recorded targets, integrating with Kevin's
//   rate.dt (0.005) via IntegrateDtOverride, and compares its d->ctrl writes
//   against the trace's recorded ctrl arrays step-by-step. This is the one
//   remaining gap ClosedLoopStable documents (it integrates with sim dt, not
//   0.005) — with the override, the only source of divergence left is float
//   vs double and any residual code-path differences vs the Python reference,
//   both of which SolverParity already bounds at 1e-3 on q_out.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotCtrlParity,
	"URLab.MinkIK.TidyBot.CtrlParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotCtrlParity::RunTest(const FString&)
{
	using namespace MjMinkIKControllerTestsLocal;

	// --- 1. Import + compile -------------------------------------------------
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"));
	if (!FPaths::FileExists(XmlPath))
	{
		AddError(TEXT("fixture missing — run Task 4 Step 1"));
		return false;
	}

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Model();
	mjData* D = S.Data();
	if (!TestNotNull(TEXT("Robot spawned"), S.Robot) || !TestNotNull(TEXT("Model compiled"), M)
		|| !TestNotNull(TEXT("Data compiled"), D))
	{
		S.Cleanup();
		return false;
	}
	AMjArticulation* Robot = S.Robot;

	// --- 2. Load the golden trace ---------------------------------------------
	TSharedPtr<FJsonObject> Trace;
	if (!LoadTidybotTraceLocal(Trace))
	{
		AddError(TEXT("fixtures/tidybot_trace.json missing or unreadable — run gen_tidybot_trace.py"));
		S.Cleanup();
		return false;
	}
	const TArray<TSharedPtr<FJsonValue>>* Steps = nullptr;
	Trace->TryGetArrayField(TEXT("steps"), Steps);
	if (!Steps || Steps->Num() == 0)
	{
		AddError(TEXT("empty trace"));
		S.Cleanup();
		return false;
	}
	const int32 NStepsToRun = FMath::Min(NSteps, Steps->Num());

	// --- 3. Find components (suffix-resolve across the import prefix) --------
	const int32 SiteMjId = FindIdBySuffix(M, mjOBJ_SITE, M->nsite, TEXT("pinch_site"));
	if (!TestTrue(TEXT("pinch_site found in compiled model"), SiteMjId >= 0))
	{
		S.Cleanup();
		return false;
	}

	TArray<UMjSite*> SiteComps;
	Robot->GetComponents<UMjSite>(SiteComps);
	UMjSite* PinchSite = FindComponentByMjId(SiteComps, SiteMjId);
	if (!TestNotNull(TEXT("pinch_site UMjSite component resolved"), PinchSite))
	{
		S.Cleanup();
		return false;
	}

	TArray<UMjJoint*> JointComps;
	Robot->GetComponents<UMjJoint>(JointComps);
	TArray<TObjectPtr<UMjJoint>> TenJoints;
	bool bAllJointsFound = true;
	int32 CtrlIds[10];
	for (int32 i = 0; i < 10; ++i)
	{
		const TCHAR* Nm = JointNames[i];
		const int32 JMjId = FindIdBySuffix(M, mjOBJ_JOINT, M->njnt, Nm);
		UMjJoint* J = (JMjId >= 0) ? FindComponentByMjId(JointComps, JMjId) : nullptr;
		if (!J)
		{
			AddError(FString::Printf(TEXT("joint '%s' not resolved (mjId=%d)"), Nm, JMjId));
			bAllJointsFound = false;
			continue;
		}
		TenJoints.Add(J);
		// The trace's ctrl order == JointNames order; the actuators driving these
		// joints are named the same as the joints (position actuators), so a
		// name-suffix lookup gives us the ctrl index independently of the
		// controller's private DriveCtrlIds — same resolution mechanism the
		// controller itself uses internally (actuator_trntype==mjTRN_JOINT).
		CtrlIds[i] = FindIdBySuffix(M, mjOBJ_ACTUATOR, M->nu, Nm);
		if (CtrlIds[i] < 0)
		{
			AddError(FString::Printf(TEXT("actuator '%s' not resolved"), Nm));
			bAllJointsFound = false;
		}
	}
	if (!bAllJointsFound || TenJoints.Num() != 10)
	{
		S.Cleanup();
		return false;
	}
	TArray<TObjectPtr<UMjJoint>> BaseJoints = {TenJoints[0], TenJoints[1], TenJoints[2]};
	TArray<TObjectPtr<UMjJoint>> ArmJoints = {TenJoints[3], TenJoints[4], TenJoints[5],
		TenJoints[6], TenJoints[7], TenJoints[8], TenJoints[9]};

	// --- 4. Create + configure the controller (not yet bound) ----------------
	// Same example stack as ClosedLoopStable, with ONE difference: integrating
	// with Kevin's fixed rate.dt instead of the elapsed sim-time delta.
	UMjMinkIKController* Ctrl = NewObject<UMjMinkIKController>(Robot, TEXT("MinkIKCtrlParity"));

	FMinkTaskSpec Frame;
	Frame.Kind = EMinkTaskKind::Frame;
	Frame.Frame = PinchSite;
	Frame.TargetMocapBody = nullptr; // targets streamed via ApplyConfig
	Frame.PositionCost = 1.0f;
	Frame.OrientationCost = 1.0f;
	Frame.LmDamping = 1.0f;

	FMinkTaskSpec Posture;
	Posture.Kind = EMinkTaskKind::Posture;
	Posture.Cost = 1e-3f;
	Posture.Joints = ArmJoints;

	FMinkTaskSpec Damping;
	Damping.Kind = EMinkTaskKind::Damping;
	Damping.Cost = 100.0f;
	Damping.Joints = BaseJoints;
	Damping.bEnabled = false;

	Ctrl->Tasks = {Frame, Posture, Damping};

	FMinkLimitSpec ConfLimit;
	Ctrl->Limits = {ConfLimit};

	Ctrl->DriveJoints = TenJoints;
	Ctrl->MaxIters = MaxIters;
	Ctrl->PosThreshold = PosThreshold;
	Ctrl->OriThreshold = OriThreshold;
	Ctrl->IntegrateDtOverride = 0.005f; // Kevin's rate.dt — the only deliberate difference
	Ctrl->RegisterComponent();

	// --- 5. Reset to home keyframe, mj_forward, THEN bind (posture target = ---
	//        home, matching Kevin's order: reset -> forward -> configure tasks).
	const int32 KeyId = FindIdBySuffix(M, mjOBJ_KEY, M->nkey, TEXT("home"));
	if (!TestTrue(TEXT("home keyframe found"), KeyId >= 0))
	{
		S.Cleanup();
		return false;
	}
	mj_resetDataKeyframe(M, D, KeyId);
	mj_forward(M, D);

	Robot->AdoptRuntimeController(Ctrl);
	if (!TestTrue(TEXT("controller bound"), Ctrl->IsBound()))
	{
		S.Cleanup();
		return false;
	}

	// --- 6. Stream recorded targets -> solve -> compare ctrl -> step ---------
	// Optional evidence dump: set URLAB_CTRLPARITY_CSV=/abs/path.csv to record
	// both ctrl vectors (python trace vs live controller) per step, plus the
	// per-step max |diff| — user-inspectable ground truth for the parity claim.
	const FString CsvPath = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_CTRLPARITY_CSV"));
	FString Csv;
	if (!CsvPath.IsEmpty())
	{
		Csv = TEXT("step");
		for (int32 i = 0; i < 10; ++i)
		{
			Csv += FString::Printf(TEXT(",py_%s"), JointNames[i]);
		}
		for (int32 i = 0; i < 10; ++i)
		{
			Csv += FString::Printf(TEXT(",ue_%s"), JointNames[i]);
		}
		Csv += TEXT(",step_max_abs_diff\n");
	}

	double MaxAbsDiff = 0.0;
	int32 MaxAbsDiffStep = -1;
	bool bDiverged = false;
	FVector LastTargetPos = FVector::ZeroVector;
	for (int32 K = 0; K < NStepsToRun; ++K)
	{
		const TSharedPtr<FJsonObject> StepObj = (*Steps)[K]->AsObject();

		double TPos[3];
		double TQuat[4];
		double ExpectedCtrl[10];
		const TArray<TSharedPtr<FJsonValue>>* TPosArr = nullptr;
		const TArray<TSharedPtr<FJsonValue>>* TQuatArr = nullptr;
		const TArray<TSharedPtr<FJsonValue>>* CtrlArr = nullptr;
		StepObj->TryGetArrayField(TEXT("target_pos"), TPosArr);
		StepObj->TryGetArrayField(TEXT("target_quat"), TQuatArr);
		StepObj->TryGetArrayField(TEXT("ctrl"), CtrlArr);
		if (!JsonArrayToDoubles(TPosArr, 3, TPos) || !JsonArrayToDoubles(TQuatArr, 4, TQuat)
			|| !JsonArrayToDoubles(CtrlArr, 10, ExpectedCtrl))
		{
			AddError(FString::Printf(TEXT("step %d: malformed trace entry"), K));
			bDiverged = true;
			break;
		}
		LastTargetPos = FVector(TPos[0], TPos[1], TPos[2]);

		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> PosArr, QuatArr;
		for (int32 i = 0; i < 3; ++i)
		{
			PosArr.Add(MakeShared<FJsonValueNumber>(TPos[i]));
		}
		for (int32 i = 0; i < 4; ++i)
		{
			QuatArr.Add(MakeShared<FJsonValueNumber>(TQuat[i]));
		}
		Cfg->SetArrayField(TEXT("target_pos"), PosArr);
		Cfg->SetArrayField(TEXT("target_quat"), QuatArr);
		Ctrl->ApplyConfig(Cfg);

		Ctrl->ComputeAndApply(M, D, 0);

		double ActualCtrl[10];
		double StepMaxDiff = 0.0;
		for (int32 i = 0; i < 10; ++i)
		{
			const double Actual = D->ctrl[CtrlIds[i]];
			ActualCtrl[i] = Actual;
			if (!FMath::IsFinite(Actual))
			{
				AddError(FString::Printf(TEXT("step %d: non-finite ctrl[%d] ('%s')"), K, i, JointNames[i]));
				bDiverged = true;
				break;
			}
			const double Diff = FMath::Abs(Actual - ExpectedCtrl[i]);
			StepMaxDiff = FMath::Max(StepMaxDiff, Diff);
			if (Diff > MaxAbsDiff)
			{
				MaxAbsDiff = Diff;
				MaxAbsDiffStep = K;
			}
		}
		if (bDiverged)
		{
			break;
		}
		if (!CsvPath.IsEmpty())
		{
			Csv += FString::Printf(TEXT("%d"), K);
			for (int32 i = 0; i < 10; ++i)
			{
				Csv += FString::Printf(TEXT(",%.12g"), ExpectedCtrl[i]);
			}
			for (int32 i = 0; i < 10; ++i)
			{
				Csv += FString::Printf(TEXT(",%.12g"), ActualCtrl[i]);
			}
			Csv += FString::Printf(TEXT(",%.6e\n"), StepMaxDiff);
		}

		mj_step(M, D);

		for (int32 v = 0; v < M->nv; ++v)
		{
			if (!FMath::IsFinite(D->qacc[v]) || !FMath::IsFinite(D->qvel[v]))
			{
				AddError(FString::Printf(TEXT("NON-FINITE qacc/qvel at step %d dof %d"), K, v));
				bDiverged = true;
				break;
			}
		}
		if (bDiverged)
		{
			break;
		}
	}

	AddInfo(FString::Printf(TEXT("ctrl parity over %d step(s): max |diff|=%.9e at step %d"),
		NStepsToRun, MaxAbsDiff, MaxAbsDiffStep));

	if (!CsvPath.IsEmpty())
	{
		if (FFileHelper::SaveStringToFile(Csv, *CsvPath))
		{
			AddInfo(FString::Printf(TEXT("ctrl dump written: %s"), *CsvPath));
		}
		else
		{
			AddWarning(FString::Printf(TEXT("ctrl dump FAILED to write: %s"), *CsvPath));
		}
	}

	// Tolerance rationale: SolverParity proves the solver itself matches Python
	// to 1e-3 on q_out (single-step, given the SAME q_in). This test instead
	// runs the REAL controller open-loop over 2500 steps, so both sides
	// integrate their own reference from their own prior ctrl — errors can
	// compound across steps even though each side individually converges each
	// step. 2e-3 gives headroom above SolverParity's 1e-3 without masking a
	// real divergence (an actual bug would show as drift well past 1e-2, not
	// noise at the 2-3e-3 level).
	constexpr double CtrlTol = 2e-3;
	TestTrue(FString::Printf(TEXT("max ctrl |diff| %.6f <= %.4f (peak at step %d)"),
				 MaxAbsDiff, CtrlTol, MaxAbsDiffStep),
		MaxAbsDiff <= CtrlTol);

	// --- 7. Sanity: the run actually tracked (final EE vs final target) ------
	if (!bDiverged)
	{
		const FVector FinalEe(D->site_xpos[3 * SiteMjId + 0], D->site_xpos[3 * SiteMjId + 1],
			D->site_xpos[3 * SiteMjId + 2]);
		const double FinalTrackErr = (FinalEe - LastTargetPos).Length();
		TestTrue(FString::Printf(TEXT("final EE tracking err %.4f <= 0.10 m"), FinalTrackErr),
			FinalTrackErr <= 0.10);
	}

	S.Cleanup();
	return !bDiverged;
}
