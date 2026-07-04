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

#include "MuJoCo/Components/Controllers/MjMinkIKController.h"

#include "MinkConfiguration.h"
#include "MinkSolveIK.h"
#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkPostureTask.h"
#include "Tasks/MinkDampingTask.h"
#include "Limits/MinkLimit.h"
#include "Limits/MinkConfigurationLimit.h"
#include "Lie/MinkSE3.h"
#include "Lie/MinkSO3.h"

#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"

#include <mujoco/mujoco.h>

namespace
{
/** Mink frame type + mjOBJ type inferred from the referenced component's class. */
EMinkFrameType MinkFrameTypeOf(const UMjComponent* C, int32& OutMjObjType)
{
	if (Cast<const UMjSite>(C))
	{
		OutMjObjType = mjOBJ_SITE;
		return EMinkFrameType::Site;
	}
	if (Cast<const UMjGeom>(C))
	{
		OutMjObjType = mjOBJ_GEOM;
		return EMinkFrameType::Geom;
	}
	OutMjObjType = mjOBJ_BODY;
	return EMinkFrameType::Body;
}

/** nv-sized cost vector: Scalar on the given joints' DOFs (all DOFs if none given). */
FMinkVec SubsetCost(const mjModel* m, const TArray<TObjectPtr<UMjJoint>>& Joints, double Scalar)
{
	if (Joints.Num() == 0)
	{
		return FMinkVec::Constant(m->nv, Scalar);
	}
	FMinkVec Cost = FMinkVec::Zero(m->nv);
	for (const UMjJoint* J : Joints)
	{
		if (!J || J->GetMjID() < 0 || J->GetMjID() >= m->njnt)
		{
			continue;
		}
		const int32 DofAdr = m->jnt_dofadr[J->GetMjID()];
		// Hinge/slide = 1 DOF; ball/free spread over 3/6 — cover the joint's full span.
		int32 DofNum = 1;
		switch (m->jnt_type[J->GetMjID()])
		{
			case mjJNT_FREE: DofNum = 6; break;
			case mjJNT_BALL: DofNum = 3; break;
			default: DofNum = 1; break;
		}
		for (int32 k = 0; k < DofNum && DofAdr + k < m->nv; ++k)
		{
			Cost[DofAdr + k] = Scalar;
		}
	}
	return Cost;
}

const TCHAR* IkStatusName(EMinkIKStatus S)
{
	switch (S)
	{
		case EMinkIKStatus::Success: return TEXT("Success");
		case EMinkIKStatus::NoSolutionFound: return TEXT("NoSolutionFound");
		case EMinkIKStatus::NotWithinConfigurationLimits: return TEXT("NotWithinConfigurationLimits");
		case EMinkIKStatus::TaskError: return TEXT("TaskError");
		case EMinkIKStatus::LimitError: return TEXT("LimitError");
	}
	return TEXT("Unknown");
}
} // namespace

/** All Mink-typed solver state; complete only in this TU (see TPimplPtr in the header). */
struct UMjMinkIKController::FMinkIKState
{
	/** A built task plus its per-step target routing. */
	struct FBuiltTask
	{
		TUniquePtr<FMinkBaseTask> Task;
		FMinkFrameTask* AsFrame = nullptr; // non-owning view iff Kind==Frame
		int32 SpecIndex = INDEX_NONE;
		int32 MocapIndex = -1; // m->body_mocapid slot, or -1
	};

	TUniquePtr<FMinkConfiguration> Config;
	TArray<FBuiltTask> BuiltTasks;
	TArray<TUniquePtr<FMinkLimit>> BuiltLimits;
};

UMjMinkIKController::UMjMinkIKController() = default;
UMjMinkIKController::~UMjMinkIKController() = default;

void UMjMinkIKController::Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& ActuatorIdMap)
{
	Super::Bind(m, d, ActuatorIdMap);

	Mink.Reset();
	DriveCtrlIds.Reset();
	DriveQposAddrs.Reset();
	{
		FScopeLock Lock(&TargetMutex);
		ManualTargets.Reset();
	}

	if (!m || !d)
	{
		return;
	}

	Mink = MakePimpl<FMinkIKState>();
	Mink->Config = MakeUnique<FMinkConfiguration>(m);
	FMinkConfiguration& Config = *Mink->Config;
	Config.Update(d->qpos);

	// --- build tasks from specs -------------------------------------------------
	for (int32 i = 0; i < Tasks.Num(); ++i)
	{
		const FMinkTaskSpec& S = Tasks[i];
		FMinkIKState::FBuiltTask Built;
		Built.SpecIndex = i;

		switch (S.Kind)
		{
			case EMinkTaskKind::Frame:
			{
				if (!S.Frame || S.Frame->GetMjID() < 0)
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] Tasks[%d]: Frame task has no resolved frame component — skipped."), i);
					continue;
				}
				int32 ObjType = mjOBJ_BODY;
				const EMinkFrameType FrameType = MinkFrameTypeOf(S.Frame, ObjType);
				const char* Nm = mj_id2name(m, ObjType, S.Frame->GetMjID());
				if (!Nm)
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] Tasks[%d]: frame id %d has no compiled name — skipped."), i, S.Frame->GetMjID());
					continue;
				}
				auto FrameTask = MakeUnique<FMinkFrameTask>(FString(UTF8_TO_TCHAR(Nm)), FrameType,
					FMinkVec::Constant(1, (double)S.PositionCost),
					FMinkVec::Constant(1, (double)S.OrientationCost),
					(double)S.Gain, (double)S.LmDamping);
				// Hold the current pose until a target arrives — never yank on start.
				FrameTask->SetTargetFromConfiguration(Config);
				if (S.TargetMocapBody && S.TargetMocapBody->GetMjID() >= 0
					&& S.TargetMocapBody->GetMjID() < m->nbody)
				{
					Built.MocapIndex = m->body_mocapid[S.TargetMocapBody->GetMjID()];
					if (Built.MocapIndex < 0)
					{
						UE_LOG(LogURLabRuntime, Warning,
							TEXT("[MinkIK] Tasks[%d]: TargetMocapBody is not a mocap body — falling back to SetIKTarget."), i);
					}
				}
				Built.AsFrame = FrameTask.Get();
				Built.Task = MoveTemp(FrameTask);
				break;
			}
			case EMinkTaskKind::Posture:
			{
				auto Posture = MakeUnique<FMinkPostureTask>(m, SubsetCost(m, S.Joints, (double)S.Cost),
					(double)S.Gain, (double)S.LmDamping);
				Posture->SetTargetFromConfiguration(Config);
				Built.Task = MoveTemp(Posture);
				break;
			}
			case EMinkTaskKind::Damping:
			{
				Built.Task = MakeUnique<FMinkDampingTask>(m, SubsetCost(m, S.Joints, (double)S.Cost));
				break;
			}
		}

		if (Built.Task.IsValid())
		{
			Mink->BuiltTasks.Add(MoveTemp(Built));
		}
	}

	// --- limits ------------------------------------------------------------------
	for (const FMinkLimitSpec& L : Limits)
	{
		if (L.Kind == EMinkLimitKind::Configuration)
		{
			Mink->BuiltLimits.Add(MakeUnique<FMinkConfigurationLimit>(m, (double)L.Gain, (double)L.MinDistance));
		}
	}

	// --- resolve driven actuators --------------------------------------------------
	if (DriveJoints.Num() == 0)
	{
		for (const FActuatorBinding& B : Bindings)
		{
			if (B.ActuatorMjID >= 0 && B.QposAddr >= 0)
			{
				DriveCtrlIds.Add(B.ActuatorMjID);
				DriveQposAddrs.Add(B.QposAddr);
			}
		}
	}
	else
	{
		for (const UMjJoint* J : DriveJoints)
		{
			if (!J || J->GetMjID() < 0)
			{
				continue;
			}
			const int32 JointId = J->GetMjID();
			bool bFound = false;
			for (int32 a = 0; a < m->nu; ++a)
			{
				if (m->actuator_trntype[a] == mjTRN_JOINT && m->actuator_trnid[a * 2] == JointId)
				{
					DriveCtrlIds.Add(a);
					DriveQposAddrs.Add(m->jnt_qposadr[JointId]);
					bFound = true;
					break;
				}
			}
			if (!bFound)
			{
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] DriveJoints: no joint actuator found for '%s' — it will not be commanded."),
					*J->GetName());
			}
		}
	}

	UE_LOG(LogURLabRuntime, Log,
		TEXT("[MinkIK] Bound: %d task(s), %d limit(s), %d driven actuator(s), nv=%d."),
		Mink->BuiltTasks.Num(), Mink->BuiltLimits.Num(), DriveCtrlIds.Num(), m->nv);
}

void UMjMinkIKController::SetIKTarget(int32 TaskIndex, FVector WorldPos, FQuat WorldRot)
{
	FManualTarget T;
	MjUtils::UEToMjPosition(WorldPos, T.Pos);
	MjUtils::UEToMjRotation(WorldRot, T.Quat);
	T.bSet = true;

	FScopeLock Lock(&TargetMutex);
	ManualTargets.Add(TaskIndex, T);
}

void UMjMinkIKController::ComputeAndApply(mjModel* m, mjData* d, uint8 /*Source*/)
{
	// Throttled diagnostics: first calls + every 2000th tell us this ran, what
	// target it saw, and what it wrote — ground truth for remote debugging.
	const bool bDiag = (DiagCounter < 3) || (DiagCounter % 2000 == 0);
	++DiagCounter;

	if (!bEnabled || !Mink.IsValid() || !Mink->Config.IsValid() || Mink->BuiltTasks.Num() == 0
		|| DriveCtrlIds.Num() == 0)
	{
		if (bDiag)
		{
			UE_LOG(LogURLabRuntime, Warning,
				TEXT("[MinkIK] call #%d EARLY-OUT: enabled=%d mink=%d tasks=%d drive=%d"),
				DiagCounter, bEnabled ? 1 : 0, Mink.IsValid() ? 1 : 0,
				Mink.IsValid() ? Mink->BuiltTasks.Num() : -1, DriveCtrlIds.Num());
		}
		return;
	}
	FMinkConfiguration& Config = *Mink->Config;

	if (bSyncFromLiveState)
	{
		Config.Update(d->qpos);
	}

	// Snapshot manual targets so the solve loop never holds the lock.
	TMap<int32, FManualTarget> Manual;
	{
		FScopeLock Lock(&TargetMutex);
		Manual = ManualTargets;
	}

	// Route targets + collect enabled tasks.
	TArray<const FMinkBaseTask*> Active;
	Active.Reserve(Mink->BuiltTasks.Num());
	const FMinkFrameTask* FirstFrame = nullptr;
	for (FMinkIKState::FBuiltTask& B : Mink->BuiltTasks)
	{
		const bool bTaskEnabled = !Tasks.IsValidIndex(B.SpecIndex) || Tasks[B.SpecIndex].bEnabled;
		if (!bTaskEnabled)
		{
			continue;
		}
		if (B.AsFrame)
		{
			// A streamed/manual target wins over the mocap body: UMjBody's tick
			// re-stamps the mocap from its UE transform every frame, so RPC
			// callers stream targets through ApplyConfig/SetIKTarget instead.
			if (const FManualTarget* T = Manual.Find(B.SpecIndex); T && T->bSet)
			{
				B.AsFrame->SetTarget(FMinkSE3::FromRotationAndTranslation(
					FMinkSO3::FromWxyz(T->Quat), FMinkVec3(T->Pos[0], T->Pos[1], T->Pos[2])));
			}
			else if (B.MocapIndex >= 0)
			{
				B.AsFrame->SetTarget(FMinkSE3::FromMocapId(d, B.MocapIndex));
			}
			// else: keep the last target (initialised to the bind pose).
			if (!FirstFrame)
			{
				FirstFrame = B.AsFrame;
			}
		}
		Active.Add(B.Task.Get());
	}
	if (Active.Num() == 0)
	{
		return;
	}

	// Limits: empty spec list => nullptr => mink's default ConfigurationLimit.
	TArray<const FMinkLimit*> LimitPtrs;
	for (const TUniquePtr<FMinkLimit>& L : Mink->BuiltLimits)
	{
		LimitPtrs.Add(L.Get());
	}
	const TArray<const FMinkLimit*>* LimitsArg = Mink->BuiltLimits.Num() > 0 ? &LimitPtrs : nullptr;

	// Inner solve/integrate loop (mink example pattern), early-out on convergence.
	const double Dt = m->opt.timestep;
	for (int32 It = 0; It < MaxIters; ++It)
	{
		const FMinkIKResult R = MinkSolveIK(Config, Active, Dt, (double)QpDamping,
			/*bSafetyBreak*/ false, LimitsArg);
		if (!R.IsSuccess())
		{
			static int32 ErrorLogBudget = 8;
			if (ErrorLogBudget-- > 0)
			{
				UE_LOG(LogURLabRuntime, Warning, TEXT("[MinkIK] solve failed: %s — holding last command."),
					IkStatusName(R.Status));
			}
			return; // hold the previous ctrl; never write a bad solution
		}
		Config.IntegrateInplace(R.Velocity, Dt);

		if (FirstFrame)
		{
			FMinkVec Err;
			if (FirstFrame->ComputeError(Config, Err) && Err.size() >= 6
				&& Err.head(3).norm() <= (double)PosThreshold
				&& Err.tail(3).norm() <= (double)OriThreshold)
			{
				break;
			}
		}
	}

	// data.ctrl[actuator_ids] = configuration.q[dof_ids]
	const FMinkVec Q = Config.GetQ();
	for (int32 i = 0; i < DriveCtrlIds.Num(); ++i)
	{
		if (DriveQposAddrs[i] < Q.size())
		{
			d->ctrl[DriveCtrlIds[i]] = Q[DriveQposAddrs[i]];
		}
	}

	if (bDiag)
	{
		FMinkVec3 Tgt(0, 0, 0);
		int32 Mc = -2;
		for (const FMinkIKState::FBuiltTask& B : Mink->BuiltTasks)
		{
			if (B.AsFrame)
			{
				Mc = B.MocapIndex;
				if (B.AsFrame->TransformTargetToWorld.IsSet())
					Tgt = B.AsFrame->TransformTargetToWorld.GetValue().Translation();
				break;
			}
		}
		UE_LOG(LogURLabRuntime, Log,
			TEXT("[MinkIK] call #%d OK: active=%d mocap_idx=%d tgt=(%.3f %.3f %.3f) "
				 "q_live0=%.3f ctrl_out=(%.3f %.3f %.3f)"),
			DiagCounter, Active.Num(), Mc, Tgt[0], Tgt[1], Tgt[2],
			d->qpos[DriveQposAddrs.Num() > 0 ? DriveQposAddrs[0] : 0],
			DriveCtrlIds.Num() > 0 ? d->ctrl[DriveCtrlIds[0]] : 0.0,
			DriveCtrlIds.Num() > 1 ? d->ctrl[DriveCtrlIds[1]] : 0.0,
			DriveCtrlIds.Num() > 2 ? d->ctrl[DriveCtrlIds[2]] : 0.0);
	}
}

// ---------------------------------------------------------------------------------
// Bridge config surface — flat solver params + per-task enable flags. Structural
// changes (task kinds, component refs, costs baked into cost vectors) rebuild on
// the next Bind, i.e. the next sim start.
// ---------------------------------------------------------------------------------
void UMjMinkIKController::GetConfigSchema(TSharedPtr<FJsonObject>& OutSchema) const
{
	OutSchema = MakeShared<FJsonObject>();
	OutSchema->SetStringField(TEXT("kind"), GetKindName());
	OutSchema->SetStringField(TEXT("max_iters"), TEXT("int"));
	OutSchema->SetStringField(TEXT("qp_damping"), TEXT("number"));
	OutSchema->SetStringField(TEXT("pos_threshold"), TEXT("number"));
	OutSchema->SetStringField(TEXT("ori_threshold"), TEXT("number"));
	OutSchema->SetStringField(TEXT("sync_from_live_state"), TEXT("bool"));
	OutSchema->SetStringField(TEXT("task_enabled"), TEXT("array<bool>"));
}

void UMjMinkIKController::GetCurrentConfig(TSharedPtr<FJsonObject>& OutParams) const
{
	OutParams = MakeShared<FJsonObject>();
	OutParams->SetNumberField(TEXT("max_iters"), MaxIters);
	OutParams->SetNumberField(TEXT("qp_damping"), QpDamping);
	OutParams->SetNumberField(TEXT("pos_threshold"), PosThreshold);
	OutParams->SetNumberField(TEXT("ori_threshold"), OriThreshold);
	OutParams->SetBoolField(TEXT("sync_from_live_state"), bSyncFromLiveState);
	TArray<TSharedPtr<FJsonValue>> Enabled;
	for (const FMinkTaskSpec& S : Tasks)
	{
		Enabled.Add(MakeShared<FJsonValueBoolean>(S.bEnabled));
	}
	OutParams->SetArrayField(TEXT("task_enabled"), Enabled);
}

void UMjMinkIKController::ApplyConfig(const TSharedPtr<FJsonObject>& InParams)
{
	if (!InParams.IsValid())
	{
		return;
	}
	double V = 0.0;
	if (InParams->TryGetNumberField(TEXT("max_iters"), V))
	{
		MaxIters = FMath::Max(1, (int32)V);
	}
	if (InParams->TryGetNumberField(TEXT("qp_damping"), V))
	{
		QpDamping = (float)FMath::Max(0.0, V);
	}
	if (InParams->TryGetNumberField(TEXT("pos_threshold"), V))
	{
		PosThreshold = (float)FMath::Max(0.0, V);
	}
	if (InParams->TryGetNumberField(TEXT("ori_threshold"), V))
	{
		OriThreshold = (float)FMath::Max(0.0, V);
	}
	bool B = false;
	if (InParams->TryGetBoolField(TEXT("sync_from_live_state"), B))
	{
		bSyncFromLiveState = B;
	}
	const TArray<TSharedPtr<FJsonValue>>* Enabled = nullptr;
	if (InParams->TryGetArrayField(TEXT("task_enabled"), Enabled) && Enabled)
	{
		for (int32 i = 0; i < Enabled->Num() && i < Tasks.Num(); ++i)
		{
			Tasks[i].bEnabled = (*Enabled)[i]->AsBool();
		}
	}

	// Streamed IK target in MuJoCo world coordinates (metres, wxyz). Routed to
	// the manual-target store, which takes precedence over the mocap body —
	// the supported way to drive the target over RPC (configure_controller).
	//   "target_pos":  [x, y, z]           (required to set a target)
	//   "target_quat": [w, x, y, z]        (optional; identity if omitted)
	//   "target_task": <spec index>        (optional; default first Frame spec)
	const TArray<TSharedPtr<FJsonValue>>* TPos = nullptr;
	if (InParams->TryGetArrayField(TEXT("target_pos"), TPos) && TPos && TPos->Num() == 3)
	{
		int32 TaskIdx = INDEX_NONE;
		double IdxV = 0.0;
		if (InParams->TryGetNumberField(TEXT("target_task"), IdxV))
		{
			TaskIdx = (int32)IdxV;
		}
		else
		{
			for (int32 i = 0; i < Tasks.Num(); ++i)
			{
				if (Tasks[i].Kind == EMinkTaskKind::Frame)
				{
					TaskIdx = i;
					break;
				}
			}
		}
		if (TaskIdx != INDEX_NONE)
		{
			FManualTarget T;
			for (int32 i = 0; i < 3; ++i)
			{
				T.Pos[i] = (*TPos)[i]->AsNumber();
			}
			const TArray<TSharedPtr<FJsonValue>>* TQuat = nullptr;
			if (InParams->TryGetArrayField(TEXT("target_quat"), TQuat) && TQuat && TQuat->Num() == 4)
			{
				for (int32 i = 0; i < 4; ++i)
				{
					T.Quat[i] = (*TQuat)[i]->AsNumber();
				}
			}
			T.bSet = true;
			FScopeLock Lock(&TargetMutex);
			ManualTargets.Add(TaskIdx, T);
		}
	}
}
