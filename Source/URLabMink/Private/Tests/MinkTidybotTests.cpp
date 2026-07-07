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
#include "MinkTestUtils.h"
#include "MinkConfiguration.h"
#include "MinkSolveIK.h"
#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkPostureTask.h"
#include "Limits/MinkConfigurationLimit.h"
#include "Lie/MinkSE3.h"
#include "Lie/MinkSO3.h"
#include "mujoco/mujoco.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace MinkTidybotTestsLocal // unity build: unique namespace name
{
constexpr int32 NSteps = 2500;
constexpr double IntegrateDt = 0.005;
constexpr double PosThreshold = 1e-4;
constexpr double OriThreshold = 1e-4;
constexpr int32 MaxIters = 20;

const TCHAR* JointNames[10] = {TEXT("joint_x"), TEXT("joint_y"), TEXT("joint_th"),
	TEXT("joint_1"), TEXT("joint_2"), TEXT("joint_3"), TEXT("joint_4"),
	TEXT("joint_5"), TEXT("joint_6"), TEXT("joint_7")};

/** The shared deterministic target script — MUST match gen_tidybot_trace.py. */
FMinkVec3 TidybotTargetPos(int32 K, const FMinkVec3& P0)
{
	const FMinkVec3 Reach(0.0, 0.35, 0.15);
	const FMinkVec3 Far(0.9, 0.0, 0.0);
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
	return P0 + Far + FMinkVec3(0.4 * (FMath::Cos(Theta) - 1.0), 0.4 * FMath::Sin(Theta), 0.0);
}

struct FTidybotRig
{
	mjModel* m = nullptr;
	mjData* d = nullptr;
	TUniquePtr<FMinkConfiguration> Config;
	TUniquePtr<FMinkFrameTask> Ee;
	TUniquePtr<FMinkPostureTask> Posture;
	TUniquePtr<FMinkConfigurationLimit> Limit;
	TArray<int32> DofIds, QposAdrs, ActIds;
	int32 SiteId = -1, MocapBodyId = -1;

	bool Init(FAutomationTestBase& T)
	{
		m = MinkLoadModel(TEXT("stanford_tidybot/scene.xml"));
		if (!m)
		{
			T.AddError(TEXT("failed to load stanford_tidybot/scene.xml"));
			return false;
		}
		d = mj_makeData(m);

		for (const TCHAR* Nm : JointNames)
		{
			const int32 J = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_UTF8(Nm));
			const int32 A = mj_name2id(m, mjOBJ_ACTUATOR, TCHAR_TO_UTF8(Nm));
			if (J < 0 || A < 0)
			{
				T.AddError(FString::Printf(TEXT("missing joint/actuator %s"), Nm));
				return false;
			}
			DofIds.Add(m->jnt_dofadr[J]);
			QposAdrs.Add(m->jnt_qposadr[J]);
			ActIds.Add(A);
		}
		SiteId = mj_name2id(m, mjOBJ_SITE, "pinch_site");
		MocapBodyId = mj_name2id(m, mjOBJ_BODY, "pinch_site_target");
		if (SiteId < 0 || MocapBodyId < 0)
		{
			T.AddError(TEXT("missing site/mocap body"));
			return false;
		}

		const int32 KeyId = mj_name2id(m, mjOBJ_KEY, "home");
		if (KeyId < 0)
		{
			T.AddError(TEXT("no home keyframe"));
			return false;
		}
		mj_resetDataKeyframe(m, d, KeyId);

		Config = MakeUnique<FMinkConfiguration>(m);
		Config->Update(d->qpos);

		FMinkVec PostureCost = FMinkVec::Constant(m->nv, 1e-3);
		PostureCost.head(3).setZero();
		Posture = MakeUnique<FMinkPostureTask>(m, PostureCost);
		Posture->SetTargetFromConfiguration(*Config);

		Ee = MakeUnique<FMinkFrameTask>(TEXT("pinch_site"), EMinkFrameType::Site,
			FMinkVec::Constant(1, 1.0), FMinkVec::Constant(1, 1.0), /*gain*/ 1.0, /*lm*/ 1.0);
		Limit = MakeUnique<FMinkConfigurationLimit>(m);
		mj_forward(m, d);
		return true;
	}

	~FTidybotRig()
	{
		if (d)
			mj_deleteData(d);
		if (m)
			mj_deleteModel(m);
	}
};
} // namespace MinkTidybotTestsLocal

// ============================================================================
// URLab.Mink.TidyBot.SolverParity — replay recorded per-step solve inputs
// through the port; compare first-iteration velocity tightly and the
// integrated q loosely against Python mink v1.2.0.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTidybotSolverParity,
	"URLab.Mink.TidyBot.SolverParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMinkTidybotSolverParity::RunTest(const FString&)
{
	using namespace MinkTidybotTestsLocal;
	TSharedPtr<FJsonObject> Trace;
	if (!MinkLoadFixture(TEXT("tidybot_trace"), Trace))
	{
		AddError(TEXT("fixtures/tidybot_trace.json missing — run gen_tidybot_trace.py"));
		return false;
	}
	FTidybotRig R;
	if (!R.Init(*this))
		return false;

	const TArray<TSharedPtr<FJsonValue>>* Steps = nullptr;
	Trace->TryGetArrayField(TEXT("steps"), Steps);
	if (!Steps || Steps->Num() == 0)
	{
		AddError(TEXT("empty trace"));
		return false;
	}

	TArray<const FMinkBaseTask*> Tasks = {R.Ee.Get(), R.Posture.Get()};
	TArray<const FMinkLimit*> Limits = {R.Limit.Get()};

	int32 NChecked = 0;
	for (int32 K = 0; K < Steps->Num(); ++K)
	{
		const TSharedPtr<FJsonObject> S = (*Steps)[K]->AsObject();
		const FMinkVec QIn = MinkJsonVec(S->GetArrayField(TEXT("q_in")));
		const FMinkVec TPos = MinkJsonVec(S->GetArrayField(TEXT("target_pos")));
		const FMinkVec TQuat = MinkJsonVec(S->GetArrayField(TEXT("target_quat")));
		const FMinkVec V0Exp = MinkJsonVec(S->GetArrayField(TEXT("v0")));
		const FMinkVec QOutExp = MinkJsonVec(S->GetArrayField(TEXT("q_out")));

		R.Config->Update(QIn.data());
		const double Wxyz[4] = {TQuat[0], TQuat[1], TQuat[2], TQuat[3]};
		R.Ee->SetTarget(FMinkSE3::FromRotationAndTranslation(
			FMinkSO3::FromWxyz(Wxyz), FMinkVec3(TPos[0], TPos[1], TPos[2])));

		// Level 1 (tight): first-iteration velocity — pure function of (q_in, target).
		const FMinkIKResult R0 = MinkSolveIK(*R.Config, Tasks, IntegrateDt, 1e-3, false, &Limits);
		if (!R0.IsSuccess())
		{
			AddError(FString::Printf(TEXT("step %d: solve failed"), K));
			return false;
		}
		if (!MinkExpectNear(*this, *FString::Printf(TEXT("v0 @ step %d"), K),
				R0.Velocity, V0Exp, 1e-6))
			return false; // stop at first divergence — everything after compounds

		// Full inner loop for q_out (loose: iteration-count wobble near thresholds).
		R.Config->Update(QIn.data());
		for (int32 It = 0; It < MaxIters; ++It)
		{
			const FMinkIKResult Rs = MinkSolveIK(*R.Config, Tasks, IntegrateDt, 1e-3, false, &Limits);
			if (!Rs.IsSuccess())
				break;
			R.Config->IntegrateInplace(Rs.Velocity, IntegrateDt);
			FMinkVec Err;
			if (R.Ee->ComputeError(*R.Config, Err) && Err.size() >= 6
				&& Err.head(3).norm() <= PosThreshold && Err.tail(3).norm() <= OriThreshold)
				break;
		}
		if (!MinkExpectNear(*this, *FString::Printf(TEXT("q_out @ step %d"), K),
				FMinkMat(R.Config->GetQ()), FMinkMat(QOutExp), 1e-3))
			return false;
		++NChecked;
	}
	AddInfo(FString::Printf(TEXT("solver parity over %d recorded steps"), NChecked));
	return true;
}

// ============================================================================
// URLab.Mink.TidyBot.ClosedLoopStable — the literal 1:1 port of
// mobile_tidybot.py: full closed loop (solve -> ctrl -> mj_step) on raw
// MuJoCo. Finiteness every step; tracking + Python-trace agreement at
// checkpoints.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTidybotClosedLoop,
	"URLab.Mink.TidyBot.ClosedLoopStable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMinkTidybotClosedLoop::RunTest(const FString&)
{
	using namespace MinkTidybotTestsLocal;
	TSharedPtr<FJsonObject> Trace;
	MinkLoadFixture(TEXT("tidybot_trace"), Trace); // optional here; checkpoints compared if present

	FTidybotRig R;
	if (!R.Init(*this))
		return false;

	// move_mocap_to_frame equivalent: target starts at the EE site.
	FMinkVec3 P0(R.d->site_xpos[3 * R.SiteId + 0], R.d->site_xpos[3 * R.SiteId + 1],
		R.d->site_xpos[3 * R.SiteId + 2]);
	double Q0[4];
	mju_mat2Quat(Q0, R.d->site_xmat + 9 * R.SiteId);

	TArray<const FMinkBaseTask*> Tasks = {R.Ee.Get(), R.Posture.Get()};
	TArray<const FMinkLimit*> Limits = {R.Limit.Get()};

	const TSet<int32> Checkpoints = {299, 899, 999, 1600, 2200, 2499};
	for (int32 K = 0; K < NSteps; ++K)
	{
		const FMinkVec3 TPos = TidybotTargetPos(K, P0);
		R.Ee->SetTarget(FMinkSE3::FromRotationAndTranslation(
			FMinkSO3::FromWxyz(Q0), TPos));

		for (int32 It = 0; It < MaxIters; ++It)
		{
			const FMinkIKResult Rs = MinkSolveIK(*R.Config, Tasks, IntegrateDt, 1e-3, false, &Limits);
			if (!Rs.IsSuccess())
			{
				AddError(FString::Printf(TEXT("step %d: solve failed"), K));
				return false;
			}
			R.Config->IntegrateInplace(Rs.Velocity, IntegrateDt);
			FMinkVec Err;
			if (R.Ee->ComputeError(*R.Config, Err) && Err.size() >= 6
				&& Err.head(3).norm() <= PosThreshold && Err.tail(3).norm() <= OriThreshold)
				break;
		}

		const FMinkVec Q = R.Config->GetQ();
		for (int32 i = 0; i < 10; ++i)
			R.d->ctrl[R.ActIds[i]] = Q[R.QposAdrs[i]];
		mj_step(R.m, R.d);

		for (int32 v = 0; v < R.m->nv; ++v)
			if (!FMath::IsFinite(R.d->qacc[v]) || !FMath::IsFinite(R.d->qvel[v]))
			{
				AddError(FString::Printf(TEXT("NON-FINITE qacc/qvel at step %d dof %d"), K, v));
				return false;
			}

		if (Checkpoints.Contains(K))
		{
			// Settled checkpoints hold a fixed target; moving ones chase the circle. Tolerances
			// calibrated against the actual Task 2 Python run (see task-3-report.md): settled
			// checkpoints measured ~0.007-0.008 m; k=999 sits in the actuator-lag transient at
			// the ramp corner (measured ~0.24 m — real physics, not instability); the remaining
			// moving checkpoints measured ~0.05 m.
			const double Tol = (K == 299 || K == 899) ? 0.02 : (K == 999 ? 0.30 : 0.10);
			const FMinkVec3 Ee(R.d->site_xpos[3 * R.SiteId + 0],
				R.d->site_xpos[3 * R.SiteId + 1], R.d->site_xpos[3 * R.SiteId + 2]);
			const double TrackErr = (Ee - TPos).norm();
			TestTrue(FString::Printf(TEXT("checkpoint %d: track err %.4f <= %.2f m"), K, TrackErr, Tol),
				TrackErr <= Tol);

			if (Trace.IsValid()) // closed-loop similarity vs Python (loose)
			{
				const TArray<TSharedPtr<FJsonValue>>* Steps = nullptr;
				Trace->TryGetArrayField(TEXT("steps"), Steps);
				if (Steps && Steps->IsValidIndex(K))
				{
					const FMinkVec PyEe = MinkJsonVec(
						(*Steps)[K]->AsObject()->GetArrayField(TEXT("ee_sim_pos")));
					const double Diff = (Ee - FMinkVec3(PyEe[0], PyEe[1], PyEe[2])).norm();
					TestTrue(FString::Printf(TEXT("checkpoint %d: EE within 0.05 m of Python (%.4f)"), K, Diff),
						Diff <= 0.05);
				}
			}
		}
	}
	// The base must actually have driven (far-circle phase).
	TestTrue(TEXT("base moved > 0.2 m"),
		FMath::Sqrt(FMath::Square(R.d->qpos[R.QposAdrs[0]]) + FMath::Square(R.d->qpos[R.QposAdrs[1]])) > 0.2);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
