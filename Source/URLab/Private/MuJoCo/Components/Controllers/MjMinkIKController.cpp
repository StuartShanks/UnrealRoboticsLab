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
#include "Limits/MinkVelocityLimit.h"
#include "Limits/MinkCollisionAvoidanceLimit.h"
#include "Lie/MinkSE3.h"
#include "Lie/MinkSO3.h"

#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "MuJoCo/Navigation/MjBaseIntegrate.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"
#include "DrawDebugHelpers.h"

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

/**
 * Resolve a compiled MuJoCo object id from a name — the duplication-safe fallback
 * used when a component ref was dropped by a PIE/Simulate world copy. Exact
 * `mj_name2id` first, else a suffix match (compiled name EndsWith "_"+Name /
 * "/"+Name / Name) to tolerate the import prefix — same idiom as the test file's
 * FindIdBySuffix. Returns -1 if nothing matches (or Name is empty).
 */
int32 ResolveIdByName(const mjModel* m, mjtObj ObjType, int32 Count, const FString& Name)
{
	if (Name.IsEmpty())
	{
		return -1;
	}
	const int32 Exact = mj_name2id(const_cast<mjModel*>(m), ObjType, TCHAR_TO_UTF8(*Name));
	if (Exact >= 0)
	{
		return Exact;
	}
	const FString UnderSuffix = FString(TEXT("_")) + Name;
	const FString SlashSuffix = FString(TEXT("/")) + Name;
	for (int32 i = 0; i < Count; ++i)
	{
		const char* Nm = mj_id2name(const_cast<mjModel*>(m), ObjType, i);
		if (!Nm)
		{
			continue;
		}
		const FString Compiled = FString(UTF8_TO_TCHAR(Nm));
		if (Compiled.EndsWith(UnderSuffix) || Compiled.EndsWith(SlashSuffix) || Compiled.EndsWith(Name))
		{
			return i;
		}
	}
	return -1;
}

/**
 * Resolve a joint set into compiled joint mj-ids. Component refs win when valid;
 * captured names (the duplication-safe fallback) fill any gap — after a
 * PIE/Simulate world copy the refs are all null, so the whole set comes from
 * Names. Self-heals: if refs resolved but no names were captured yet, records the
 * compiled joint names into Names so the next duplication survives too.
 */
void ResolveJointIds(const mjModel* m, const TArray<TObjectPtr<UMjJoint>>& Refs, TArray<FString>& Names,
	TArray<int32>& OutIds)
{
	OutIds.Reset();
	for (const UMjJoint* J : Refs)
	{
		if (J && J->GetMjID() >= 0 && J->GetMjID() < m->njnt)
		{
			OutIds.AddUnique(J->GetMjID());
		}
	}
	// Self-heal: capture compiled names from the resolved refs so a future
	// duplicated world (which drops the refs) can rebuild this set from Names.
	if (Names.Num() == 0 && OutIds.Num() > 0)
	{
		for (int32 Jid : OutIds)
		{
			if (const char* Nm = mj_id2name(const_cast<mjModel*>(m), mjOBJ_JOINT, Jid))
			{
				Names.Add(FString(UTF8_TO_TCHAR(Nm)));
			}
		}
	}
	// Name fallback: fill any joint not already covered by a valid ref.
	for (const FString& Name : Names)
	{
		const int32 Id = ResolveIdByName(m, mjOBJ_JOINT, m->njnt, Name);
		if (Id >= 0)
		{
			OutIds.AddUnique(Id);
		}
	}
}

/**
 * Resolve a collision-avoidance geom group: each name resolves as a GEOM first
 * (suffix-tolerant), else as a BODY whose geoms are all added — imported mesh
 * geoms are commonly unnamed, so body expansion is the practical way to cover
 * a link. Warns per unresolved name.
 */
void ResolveGeomGroup(const mjModel* m, const TArray<FString>& Names, TArray<int32>& OutIds,
	int32 SpecIdx, const TCHAR* Side)
{
	for (const FString& N : Names)
	{
		const int32 Gid = ResolveIdByName(m, mjOBJ_GEOM, m->ngeom, N);
		if (Gid >= 0)
		{
			OutIds.AddUnique(Gid);
			continue;
		}
		const int32 Bid = ResolveIdByName(m, mjOBJ_BODY, m->nbody, N);
		if (Bid >= 0)
		{
			for (int32 g = 0; g < m->body_geomnum[Bid]; ++g)
			{
				OutIds.AddUnique(m->body_geomadr[Bid] + g);
			}
			continue;
		}
		UE_LOG(LogURLabRuntime, Warning,
			TEXT("[MinkIK] Limits[%d].%s: '%s' matched no geom or body — entry ignored."),
			SpecIdx, Side, *N);
	}
}

/** nv-sized cost vector: Scalar on the given joints' DOFs (all DOFs if none given). */
FMinkVec SubsetCost(const mjModel* m, const TArray<int32>& JointIds, double Scalar)
{
	if (JointIds.Num() == 0)
	{
		return FMinkVec::Constant(m->nv, Scalar);
	}
	FMinkVec Cost = FMinkVec::Zero(m->nv);
	for (int32 Jid : JointIds)
	{
		if (Jid < 0 || Jid >= m->njnt)
		{
			continue;
		}
		const int32 DofAdr = m->jnt_dofadr[Jid];
		// Hinge/slide = 1 DOF; ball/free spread over 3/6 — cover the joint's full span.
		int32 DofNum = 1;
		switch (m->jnt_type[Jid])
		{
			case mjJNT_FREE:
				DofNum = 6;
				break;
			case mjJNT_BALL:
				DofNum = 3;
				break;
			default:
				DofNum = 1;
				break;
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
		case EMinkIKStatus::Success:
			return TEXT("Success");
		case EMinkIKStatus::NoSolutionFound:
			return TEXT("NoSolutionFound");
		case EMinkIKStatus::NotWithinConfigurationLimits:
			return TEXT("NotWithinConfigurationLimits");
		case EMinkIKStatus::TaskError:
			return TEXT("TaskError");
		case EMinkIKStatus::LimitError:
			return TEXT("LimitError");
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
		FMinkFrameTask* AsFrame = nullptr;     // non-owning view iff Kind==Frame
		FMinkPostureTask* AsPosture = nullptr; // non-owning view iff Kind==Posture||TwistFollow
		int32 SpecIndex = INDEX_NONE;
		int32 MocapIndex = -1; // m->body_mocapid slot, or -1

		// TwistFollow only: the base-DOF integrator state (physics thread).
		bool bTwistFollow = false;
		int32 TwistQposAdr[3] = {-1, -1, -1}; // x, y, th qpos addresses
		int32 TwistJointId[3] = {-1, -1, -1}; // x, y, th joint ids (range clamps)
		double TwistTarget[3] = {0.0, 0.0, 0.0};
		bool bTwistSeeded = false; // (re)seed from the reference on first use
	};

	TUniquePtr<FMinkConfiguration> Config;
	TArray<FBuiltTask> BuiltTasks;
	TArray<TUniquePtr<FMinkLimit>> BuiltLimits;
};

UMjMinkIKController::UMjMinkIKController()
{
	// Ticking is off in the UMjArticulationController base (all real work runs
	// off ComputeAndApply on the physics thread); this subclass needs a game-
	// thread tick solely to draw the bDrawTarget debug markers, since
	// DrawDebug* is not safe to call from the physics thread. TickComponent
	// early-outs immediately when bDrawTarget is false, so this is free when
	// the feature is off.
	PrimaryComponentTick.bCanEverTick = true;
}
UMjMinkIKController::~UMjMinkIKController() = default;

void UMjMinkIKController::Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& ActuatorIdMap)
{
	Super::Bind(m, d, ActuatorIdMap);

	// Captured verbatim (not the transmission-type-filtered Bindings) so the
	// non-drive pass-through in ComputeAndApply can reach tendon/site-driven
	// actuators too — see the member comment in the header.
	AllActuatorIdMap = ActuatorIdMap;

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
	Mink->Config->Update(d->qpos);

	// Sibling twist source for TwistFollow tasks. Bind runs before stepping, so
	// component discovery is safe here (BaseDrive does the same); GetTwist() is
	// thread-safe for the physics-thread reads later.
	TwistSource = GetOwner() ? GetOwner()->FindComponentByClass<UMjTwistController>() : nullptr;

	RebuildFromSpecs(m, d);
	BuiltGeneration = SpecGeneration.GetValue();
	ErrorLogBudget = 8;
	// Fresh session: force the first-integration re-base and take the current
	// reset epoch as the baseline (a reset AFTER this point must re-base).
	LastSimTime = -1.0;
	LastSeenResetEpoch = GetSimResetEpoch();

	UE_LOG(LogURLabRuntime, Log,
		TEXT("[MinkIK] Bound: %d task(s), %d limit(s), %d driven actuator(s), nv=%d."),
		Mink->BuiltTasks.Num(), Mink->BuiltLimits.Num(), DriveCtrlIds.Num(), m->nv);
}

void UMjMinkIKController::RebuildFromSpecs(mjModel* m, mjData* d)
{
	if (!Mink.IsValid() || !Mink->Config.IsValid() || !m || !d)
	{
		return;
	}
	FMinkConfiguration& Config = *Mink->Config;

	// Reference is preserved across a live reconfigure; only the solver stack and
	// driven-actuator resolution are rebuilt from the current specs.
	Mink->BuiltTasks.Reset();
	Mink->BuiltLimits.Reset();
	DriveCtrlIds.Reset();
	DriveQposAddrs.Reset();

	// --- build tasks from specs -------------------------------------------------
	for (int32 i = 0; i < Tasks.Num(); ++i)
	{
		// Non-const: Bind self-heals the name fallbacks (writes captured compiled
		// names back into the spec) so Details-authored controllers become
		// duplication-proof after their first successful bind.
		FMinkTaskSpec& S = Tasks[i];
		FMinkIKState::FBuiltTask Built;
		Built.SpecIndex = i;

		switch (S.Kind)
		{
			case EMinkTaskKind::Frame:
			{
				FString FrameCompiledName;
				EMinkFrameType FrameType = EMinkFrameType::Site;

				// Preferred: a valid component ref resolves the frame directly.
				if (S.Frame && S.Frame->GetMjID() >= 0)
				{
					int32 ObjType = mjOBJ_BODY;
					FrameType = MinkFrameTypeOf(S.Frame, ObjType);
					if (const char* Nm = mj_id2name(m, ObjType, S.Frame->GetMjID()))
					{
						FrameCompiledName = FString(UTF8_TO_TCHAR(Nm));
						// Self-heal: capture the compiled name so this task survives
						// a future PIE/Simulate world copy that drops the ref.
						if (S.FrameName.IsEmpty())
						{
							S.FrameName = FrameCompiledName;
						}
					}
				}

				// Fallback: ref null/unresolved (e.g. duplicated world) — resolve the
				// captured name against the compiled model: site, then body, then geom.
				if (FrameCompiledName.IsEmpty() && !S.FrameName.IsEmpty())
				{
					mjtObj MatchedObj = mjOBJ_SITE;
					int32 Id = ResolveIdByName(m, mjOBJ_SITE, m->nsite, S.FrameName);
					if (Id >= 0)
					{
						FrameType = EMinkFrameType::Site;
						MatchedObj = mjOBJ_SITE;
					}
					else if ((Id = ResolveIdByName(m, mjOBJ_BODY, m->nbody, S.FrameName)) >= 0)
					{
						FrameType = EMinkFrameType::Body;
						MatchedObj = mjOBJ_BODY;
					}
					else if ((Id = ResolveIdByName(m, mjOBJ_GEOM, m->ngeom, S.FrameName)) >= 0)
					{
						FrameType = EMinkFrameType::Geom;
						MatchedObj = mjOBJ_GEOM;
					}
					if (Id >= 0)
					{
						if (const char* Nm = mj_id2name(m, MatchedObj, Id))
						{
							FrameCompiledName = FString(UTF8_TO_TCHAR(Nm));
							UE_LOG(LogURLabRuntime, Log,
								TEXT("[MinkIK] Tasks[%d]: frame resolved by name '%s' -> %s '%s' (ref dropped)."),
								i, *S.FrameName,
								MatchedObj == mjOBJ_SITE   ? TEXT("site")
								: MatchedObj == mjOBJ_GEOM ? TEXT("geom")
														   : TEXT("body"),
								*FrameCompiledName);
						}
					}
				}

				if (FrameCompiledName.IsEmpty())
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] Tasks[%d]: Frame task has no resolved frame component or name '%s' — skipped."),
						i, *S.FrameName);
					continue;
				}
				auto FrameTask = MakeUnique<FMinkFrameTask>(FrameCompiledName, FrameType,
					FMinkVec::Constant(1, (double)S.PositionCost),
					FMinkVec::Constant(1, (double)S.OrientationCost),
					(double)S.Gain, (double)S.LmDamping);
				// Hold the current pose until a target arrives — never yank on start.
				FrameTask->SetTargetFromConfiguration(Config);

				// Mocap target body: prefer the ref, else the captured name.
				int32 MocapBodyId = -1;
				if (S.TargetMocapBody && S.TargetMocapBody->GetMjID() >= 0
					&& S.TargetMocapBody->GetMjID() < m->nbody)
				{
					MocapBodyId = S.TargetMocapBody->GetMjID();
					if (S.TargetMocapBodyName.IsEmpty())
					{
						if (const char* Nm = mj_id2name(m, mjOBJ_BODY, MocapBodyId))
						{
							S.TargetMocapBodyName = FString(UTF8_TO_TCHAR(Nm));
						}
					}
				}
				else if (!S.TargetMocapBodyName.IsEmpty())
				{
					MocapBodyId = ResolveIdByName(m, mjOBJ_BODY, m->nbody, S.TargetMocapBodyName);
				}
				if (MocapBodyId >= 0)
				{
					Built.MocapIndex = m->body_mocapid[MocapBodyId];
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
				TArray<int32> JointIds;
				ResolveJointIds(m, S.Joints, S.JointNames, JointIds);
				auto Posture = MakeUnique<FMinkPostureTask>(m, SubsetCost(m, JointIds, (double)S.Cost),
					(double)S.Gain, (double)S.LmDamping);
				Posture->SetTargetFromConfiguration(Config);
				Built.AsPosture = Posture.Get();
				Built.Task = MoveTemp(Posture);
				break;
			}
			case EMinkTaskKind::Damping:
			{
				TArray<int32> JointIds;
				ResolveJointIds(m, S.Joints, S.JointNames, JointIds);
				Built.Task = MakeUnique<FMinkDampingTask>(m, SubsetCost(m, JointIds, (double)S.Cost));
				break;
			}
			case EMinkTaskKind::TwistFollow:
			{
				// A posture task over the base DOFs whose target the per-step
				// integrator advances from the twist bus (see ComputeAndApply).
				// Joints must be EXACTLY the 3 base joints in x, y, th order —
				// ResolveJointIds preserves the spec order.
				TArray<int32> JointIds;
				ResolveJointIds(m, S.Joints, S.JointNames, JointIds);
				if (JointIds.Num() != 3)
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] Tasks[%d]: TwistFollow needs exactly 3 joints (x, y, th in order), got %d — skipped."),
						i, JointIds.Num());
					continue;
				}
				if (!TwistSource)
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] Tasks[%d]: TwistFollow has no UMjTwistController sibling — base will hold its seed pose."),
						i);
				}
				auto Twist = MakeUnique<FMinkPostureTask>(m, SubsetCost(m, JointIds, (double)S.Cost),
					(double)S.Gain, (double)S.LmDamping);
				Twist->SetTargetFromConfiguration(Config);
				for (int32 k = 0; k < 3; ++k)
				{
					Built.TwistJointId[k] = JointIds[k];
					Built.TwistQposAdr[k] = m->jnt_qposadr[JointIds[k]];
				}
				Built.bTwistFollow = true;
				Built.bTwistSeeded = false; // seeded from the reference on first step
				Built.AsPosture = Twist.Get();
				Built.Task = MoveTemp(Twist);
				break;
			}
		}

		if (Built.Task.IsValid())
		{
			Mink->BuiltTasks.Add(MoveTemp(Built));
		}
	}

	// --- limits ------------------------------------------------------------------
	for (int32 li = 0; li < Limits.Num(); ++li)
	{
		FMinkLimitSpec& L = Limits[li];
		if (L.Kind == EMinkLimitKind::Configuration)
		{
			Mink->BuiltLimits.Add(MakeUnique<FMinkConfigurationLimit>(m, (double)L.Gain, (double)L.MinDistance));
		}
		else if (L.Kind == EMinkLimitKind::CollisionAvoidance)
		{
			// One pair of geom groups per spec entry (A x B, cross-producted,
			// filtered and deduped inside the limit). Names resolve here with
			// the suffix-tolerant geom-or-body rules; the limit gets raw ids.
			FMinkGeomGroup GroupA, GroupB;
			ResolveGeomGroup(m, L.GeomsA, GroupA.Ids, li, TEXT("GeomsA"));
			ResolveGeomGroup(m, L.GeomsB, GroupB.Ids, li, TEXT("GeomsB"));
			if (GroupA.Ids.Num() == 0 || GroupB.Ids.Num() == 0)
			{
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] Limits[%d]: CollisionAvoidance needs geoms on BOTH sides (A=%d, B=%d) — skipped."),
					li, GroupA.Ids.Num(), GroupB.Ids.Num());
				continue;
			}
			TArray<FMinkCollisionPair> Pairs;
			Pairs.Emplace(GroupA, GroupB);
			auto Lim = MakeUnique<FMinkCollisionAvoidanceLimit>(m, Pairs, (double)L.Gain,
				(double)L.MinDistance, (double)L.DetectionDistance, (double)L.BoundRelaxation);
			if (!Lim->bIsValid)
			{
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] Limits[%d]: CollisionAvoidance construction failed — skipped."), li);
				continue;
			}
			UE_LOG(LogURLabRuntime, Log,
				TEXT("[MinkIK] Limits[%d]: CollisionAvoidance active with %d geom pair(s) "
					 "(min_dist=%.3f, detect=%.3f, gain=%.2f)."),
				li, Lim->MaxNumContacts(), L.MinDistance, L.DetectionDistance, L.Gain);
			Mink->BuiltLimits.Add(MoveTemp(Lim));
		}
		else if (L.Kind == EMinkLimitKind::Velocity)
		{
			// (joint name, per-DOF cap) pairs; empty Joints => every non-free joint.
			TArray<TPair<FString, FMinkVec>> Caps;
			auto AddCap = [&](int32 JntId) {
				const int32 T = m->jnt_type[JntId];
				if (T == mjJNT_FREE)
					return;
				const int32 W = (T == mjJNT_BALL) ? 3 : 1;
				const char* Nm = mj_id2name(m, mjOBJ_JOINT, JntId);
				if (Nm)
					Caps.Emplace(FString(UTF8_TO_TCHAR(Nm)), FMinkVec::Constant(W, (double)L.MaxVelocity));
			};
			// Refs win; captured names are the duplication-safe fallback.
			TArray<int32> LimitJointIds;
			ResolveJointIds(m, L.Joints, L.JointNames, LimitJointIds);
			if (LimitJointIds.Num() == 0)
			{
				for (int32 j = 0; j < m->njnt; ++j)
					AddCap(j);
			}
			else
			{
				for (int32 Jid : LimitJointIds)
					AddCap(Jid);
			}
			Mink->BuiltLimits.Add(MakeUnique<FMinkVelocityLimit>(m, Caps));
		}
	}

	// --- resolve driven actuators --------------------------------------------------
	// Refs win; captured names are the duplication-safe fallback (after a
	// PIE/Simulate world copy the DriveJoints refs are all null). Empty both =>
	// drive every bound actuator, as before.
	TArray<int32> DriveJointIds;
	ResolveJointIds(m, DriveJoints, DriveJointNames, DriveJointIds);
	if (DriveJointIds.Num() == 0)
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
		for (int32 JointId : DriveJointIds)
		{
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
				const char* Nm = mj_id2name(m, mjOBJ_JOINT, JointId);
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] DriveJoints: no joint actuator found for '%s' — it will not be commanded."),
					Nm ? UTF8_TO_TCHAR(Nm) : TEXT("<unnamed>"));
			}
		}
	}
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

void UMjMinkIKController::ComputeAndApply(mjModel* m, mjData* d, uint8 Source)
{
	// Throttled diagnostics: first calls + every 2000th tell us this ran, what
	// target it saw, and what it wrote — ground truth for remote debugging.
	const bool bDiag = (DiagCounter < 3) || (DiagCounter % 2000 == 0);
	++DiagCounter;

	// Live reconfigure: if the specs changed (MarkSpecsChanged bumped the counter
	// on the game thread), rebuild the solver stack here on the physics thread so
	// all BuiltTasks mutation stays single-threaded vs ApplyControls — no restart
	// needed to retune costs/damping/joints.
	const int32 CurGen = SpecGeneration.GetValue();
	if (CurGen != BuiltGeneration && Mink.IsValid() && Mink->Config.IsValid())
	{
		RebuildFromSpecs(m, d);
		BuiltGeneration = CurGen;
		UE_LOG(LogURLabRuntime, Log,
			TEXT("[MinkIK] hot-reconfigured (gen %d): %d task(s), %d limit(s), %d driven actuator(s)."),
			CurGen, Mink->BuiltTasks.Num(), Mink->BuiltLimits.Num(), DriveCtrlIds.Num());
	}

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

	// Snapshot the ApplyConfig-written surface under ConfigMutex (base-class
	// thread model): copy scalars + per-spec enables to locals and release, so
	// the solve never overlaps a config write and never holds the lock.
	int32 CfgMaxIters;
	float CfgQpDamping, CfgIntegrateDtOverride, CfgPosThreshold, CfgOriThreshold;
	bool bCfgSyncFromLiveState, bCfgDrawTarget;
	TArray<bool, TInlineAllocator<8>> CfgSpecEnabled;
	{
		FScopeLock ConfigLock(&ConfigMutex);
		CfgMaxIters = MaxIters;
		CfgQpDamping = QpDamping;
		CfgIntegrateDtOverride = IntegrateDtOverride;
		CfgPosThreshold = PosThreshold;
		CfgOriThreshold = OriThreshold;
		bCfgSyncFromLiveState = bSyncFromLiveState;
		bCfgDrawTarget = bDrawTarget;
		CfgSpecEnabled.SetNum(Tasks.Num());
		for (int32 i = 0; i < Tasks.Num(); ++i)
		{
			CfgSpecEnabled[i] = Tasks[i].bEnabled;
		}
	}

	if (bCfgSyncFromLiveState)
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
	// Debug-draw snapshot for TickComponent (game thread); only populated when
	// bDrawTarget is set so this whole path costs nothing in production.
	TArray<FMinkTargetSnapshot> NewDebugTargets;
	for (FMinkIKState::FBuiltTask& B : Mink->BuiltTasks)
	{
		const bool bTaskEnabled = !CfgSpecEnabled.IsValidIndex(B.SpecIndex) || CfgSpecEnabled[B.SpecIndex];
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
			// Whatever the source above (mocap / manual / held bind pose), the
			// resolved target now lives in TransformTargetToWorld — snapshot it
			// for the debug draw regardless of why it did or didn't change.
			if (bCfgDrawTarget && B.AsFrame->TransformTargetToWorld.IsSet())
			{
				const FMinkSE3& Tgt = B.AsFrame->TransformTargetToWorld.GetValue();
				FMinkTargetSnapshot Snap;
				Snap.Pos[0] = Tgt.WxyzXyz[4];
				Snap.Pos[1] = Tgt.WxyzXyz[5];
				Snap.Pos[2] = Tgt.WxyzXyz[6];
				Snap.Quat[0] = Tgt.WxyzXyz[0];
				Snap.Quat[1] = Tgt.WxyzXyz[1];
				Snap.Quat[2] = Tgt.WxyzXyz[2];
				Snap.Quat[3] = Tgt.WxyzXyz[3];
				NewDebugTargets.Add(Snap);
			}
		}
		Active.Add(B.Task.Get());
	}
	if (bCfgDrawTarget)
	{
		FScopeLock Lock(&DebugTargetMutex);
		DebugTargets = MoveTemp(NewDebugTargets);
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

	// Integrate exactly once per real physics step: the engine also invokes us
	// on idle physics-thread iterations, which must not advance the reference.
	const double Now = d->time;
	// Reset detection: every reset/restore site bumps the global sim-reset
	// epoch. Sim time is NOT a reliable signal — a reset can land exactly on
	// the time we last integrated at (equal-time blind spot: bind at t=0, one
	// solve, reset back to t=0 looks "idle", then t advances "normally" and
	// the stale reference yanks the robot through kp·(ctrl−qpos). The
	// backwards-time check stays only as a fallback for state writers that
	// don't call NotifySimReset().
	const uint64 EpochNow = GetSimResetEpoch();
	if (EpochNow != LastSeenResetEpoch || (LastSimTime >= 0.0 && Now < LastSimTime))
	{
		LastSeenResetEpoch = EpochNow;
		// Re-enter the first-integration path below: a full re-base of the
		// open-loop reference AND every passive target from the live state
		// (the old backwards-time guard re-based only the reference, leaving
		// posture/frame targets pointing at the pre-reset pose).
		LastSimTime = -1.0;
	}
	if (LastSimTime >= 0.0 && Now == LastSimTime)
	{
		return; // idle invocation — sim didn't advance; hold ctrl as-is
	}
	const bool bFirstIntegration = (LastSimTime < 0.0);
	double Dt;
	if (CfgIntegrateDtOverride > 0.0f)
	{
		Dt = (double)CfgIntegrateDtOverride;
	}
	else if (bFirstIntegration)
	{
		Dt = m->opt.timestep;
	}
	else
	{
		Dt = FMath::Min(Now - LastSimTime, 10.0 * m->opt.timestep);
	}

	// First integration after Bind OR after a detected sim reset: the open-loop
	// reference was seeded at some earlier pose (Bind: the SPAWN pose; reset:
	// the pre-reset trajectory) that no longer matches the live state. Solving
	// from that stale reference would make the first ctrl frame command the old
	// pose — a violent yank through kp·(ctrl−qpos). Re-base the reference and
	// every passive target on the live state now, at the moment stepping
	// actually (re)begins (Kevin's configuration.update(data.qpos)-after-reset
	// semantics, deferred to first use).
	if (bFirstIntegration)
	{
		Config.Update(d->qpos);
		for (FMinkIKState::FBuiltTask& B : Mink->BuiltTasks)
		{
			if (B.AsPosture)
			{
				// Kevin sets the posture target after the home reset; ours was
				// captured at Bind (spawn) — refresh it from the re-based reference.
				B.AsPosture->SetTargetFromConfiguration(Config);
				// TwistFollow integrators re-seed from the re-based reference too.
				B.bTwistSeeded = false;
			}
			else if (B.AsFrame)
			{
				// Only frames with NO target source this step: a latched manual
				// target is intentional and preserved; a mocap-driven frame is
				// refreshed every step anyway. Without this, the held build-time
				// target (spawn EE pose) would deliberately drive back to spawn.
				const FManualTarget* T = Manual.Find(B.SpecIndex);
				if ((!T || !T->bSet) && B.MocapIndex < 0)
				{
					B.AsFrame->SetTargetFromConfiguration(Config);
				}
			}
		}
		UE_LOG(LogURLabRuntime, Log,
			TEXT("[MinkIK] first integration (bind or sim reset): re-based reference + passive targets from live state"));
	}
	LastSimTime = Now;

	// TwistFollow: advance each follower's base target from the twist bus —
	// ONCE per physics step (this step's Dt), before the solve iterations.
	// Everything runs in REFERENCE space (the IK's open-loop config), not live
	// qpos: if the QP starves the base to serve another task, the reference
	// stops advancing and the leash self-limits the target — no windup. The
	// twist -> world rotation + integrate/leash/reseed math is shared with
	// UMjBaseDriveController (MjBaseIntegrate).
	for (FMinkIKState::FBuiltTask& B : Mink->BuiltTasks)
	{
		if (!B.bTwistFollow || !B.AsPosture)
		{
			continue;
		}
		const bool bTaskOn = !CfgSpecEnabled.IsValidIndex(B.SpecIndex) || CfgSpecEnabled[B.SpecIndex];
		if (!bTaskOn)
		{
			B.bTwistSeeded = false; // re-seed cleanly on re-enable
			continue;
		}
		FMinkVec TargetQ = Config.GetQ();
		if (!B.bTwistSeeded)
		{
			for (int32 k = 0; k < 3; ++k)
			{
				B.TwistTarget[k] = TargetQ[B.TwistQposAdr[k]];
			}
			B.bTwistSeeded = true;
		}
		const FVector Tw = TwistSource ? TwistSource->GetTwist() : FVector::ZeroVector;
		double VWorld[3];
		MjBaseIntegrate::TwistToWorld(Tw.X, Tw.Y, Tw.Z, TargetQ[B.TwistQposAdr[2]], VWorld);
		const MjBaseIntegrate::FLeashParams Leash; // BaseDrive defaults
		for (int32 k = 0; k < 3; ++k)
		{
			const int32 Jid = B.TwistJointId[k];
			const bool bLimited = Jid >= 0 && m->jnt_limited[Jid];
			B.TwistTarget[k] = MjBaseIntegrate::IntegrateLeashed(B.TwistTarget[k], VWorld[k], Dt,
				/*Measured*/ TargetQ[B.TwistQposAdr[k]], /*bAngular*/ k == 2, Leash, bLimited,
				bLimited ? (double)m->jnt_range[Jid * 2] : 0.0,
				bLimited ? (double)m->jnt_range[Jid * 2 + 1] : 0.0);
			TargetQ[B.TwistQposAdr[k]] = B.TwistTarget[k];
		}
		// Non-base entries carry the current reference q — their cost is zero
		// (SubsetCost), so only the 3 base entries shape the QP objective.
		B.AsPosture->SetTarget(TargetQ);
	}

	for (int32 It = 0; It < CfgMaxIters; ++It)
	{
		const FMinkIKResult R = MinkSolveIK(Config, Active, Dt, (double)CfgQpDamping,
			/*bSafetyBreak*/ false, LimitsArg);
		if (!R.IsSuccess())
		{
			if (ErrorLogBudget-- > 0)
			{
				UE_LOG(LogURLabRuntime, Warning, TEXT("[MinkIK] solve failed: %s — holding last command."),
					IkStatusName(R.Status));
			}
			return; // hold the previous ctrl; never write a bad solution
		}

		// Kevin's reference applies no clamp — differential-IK convergence
		// velocities are legitimately large per inner iteration (Newton-style
		// steps that shrink as the reference converges over max_iters). The ONLY
		// guard is against a non-finite solve, purely so we never write NaN/inf to
		// d->ctrl; any finite velocity is integrated, exactly like the demo.
		const double VMax = R.Velocity.size() > 0 ? R.Velocity.cwiseAbs().maxCoeff() : 0.0;
		if (!FMath::IsFinite(VMax))
		{
			++BadSolveCount;
			if (BadSolveCount <= 10 || BadSolveCount % 100 == 0)
			{
				Eigen::Index Worst = 0;
				R.Velocity.cwiseAbs().maxCoeff(&Worst);
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] BAD SOLVE #%d discarded: |v|max=%.1f at dof %d"),
					BadSolveCount, VMax, (int32)Worst);
			}
			return;
		}
		Config.IntegrateInplace(R.Velocity, Dt);

		if (FirstFrame)
		{
			FMinkVec Err;
			if (FirstFrame->ComputeError(Config, Err) && Err.size() >= 6
				&& Err.head(3).norm() <= (double)CfgPosThreshold
				&& Err.tail(3).norm() <= (double)CfgOriThreshold)
			{
				break;
			}
		}
	}

	// Pass-through for every non-drive actuator: a bound controller replaces
	// the articulation's default ctrl path entirely (BaseDrive contract), so
	// non-drive actuators (gripper, suction) must keep receiving their
	// UI/network values through us — otherwise SetNetworkControl is a silent
	// no-op while the mink is bound. Iterates AllActuatorIdMap (captured at
	// Bind, unfiltered by transmission type) rather than the inherited
	// Bindings array, which only covers joint-transmission actuators — a
	// tendon-driven gripper (e.g. fingers_actuator) never appears in Bindings.
	for (const TPair<int32, UMjActuator*>& Elem : AllActuatorIdMap)
	{
		const int32 ActId = Elem.Key;
		UMjActuator* Comp = Elem.Value;
		if (ActId < 0 || !Comp || DriveCtrlIds.Contains(ActId))
		{
			continue;
		}
		d->ctrl[ActId] = Comp->ResolveDesiredControl(Source);
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

	// Dense per-joint trace of the first ~60 steps: for each driven joint print
	// the target qpos we command (= ctrl), the actual qpos, and the servo error
	// between them. This is the ground truth for "is the port sending sane ctrl,
	// and is the sim diverging from it" — the real numbers to eyeball with Buzz.
	if (DiagCounter <= 60)
	{
		FString Line;
		for (int32 i = 0; i < DriveCtrlIds.Num(); ++i)
		{
			const int32 QA = DriveQposAddrs[i];
			const double Target = (QA < Q.size()) ? (double)Q[QA] : 0.0; // = ctrl written
			const double Live = d->qpos[QA];
			Line += FString::Printf(TEXT(" a%d[tgt=%.4f q=%.4f err=%+.4f]"),
				DriveCtrlIds[i], Target, Live, Target - Live);
		}
		// Cartesian error the frame task actually sees (target vs current EE) —
		// tells us if the huge velocity is a real target/pose mismatch or a
		// singularity blowing up a tiny error.
		double FramePos = -1.0, FrameOri = -1.0;
		if (FirstFrame)
		{
			FMinkVec Err;
			if (FirstFrame->ComputeError(Config, Err) && Err.size() >= 6)
			{
				FramePos = Err.head(3).norm();
				FrameOri = Err.tail(3).norm();
			}
		}
		UE_LOG(LogURLabRuntime, Log,
			TEXT("[MinkIK] trace #%d t=%.4f sync=%d frameErr(pos=%.4f ori=%.4f)%s"),
			DiagCounter, d->time, bCfgSyncFromLiveState ? 1 : 0, FramePos, FrameOri, *Line);
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

void UMjMinkIKController::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

#if ENABLE_DRAW_DEBUG
	bool bDraw;
	{
		FScopeLock ConfigLock(&ConfigMutex);
		bDraw = bDrawTarget;
	}
	if (!bDraw)
	{
		return;
	}
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	TArray<FMinkTargetSnapshot> Targets;
	{
		FScopeLock Lock(&DebugTargetMutex);
		Targets = DebugTargets;
	}
	for (const FMinkTargetSnapshot& T : Targets)
	{
		const FVector WorldPos = MjUtils::MjToUEPosition(T.Pos);
		const FQuat WorldRot = MjUtils::MjToUERotation(T.Quat);
		// Thin, non-persistent — redrawn every tick, so a single frame's
		// lifetime (the codebase's established idiom for per-tick debug
		// shapes; see MjPerturbation.cpp / MjNavComponent.cpp) is enough.
		DrawDebugSphere(World, WorldPos, DrawTargetSize, 12, FColor::Red, false, -1.0f, 0, 1.0f);
		DrawDebugCoordinateSystem(World, WorldPos, WorldRot.Rotator(), DrawTargetSize * 2.0f, false, -1.0f, 0, 1.5f);
	}
#endif // ENABLE_DRAW_DEBUG
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
	OutSchema->SetStringField(TEXT("integrate_dt_override"), TEXT("number"));
	OutSchema->SetStringField(TEXT("pos_threshold"), TEXT("number"));
	OutSchema->SetStringField(TEXT("ori_threshold"), TEXT("number"));
	OutSchema->SetStringField(TEXT("sync_from_live_state"), TEXT("bool"));
	OutSchema->SetStringField(TEXT("draw_target"), TEXT("bool"));
	OutSchema->SetStringField(TEXT("task_enabled"), TEXT("array<bool>"));
	OutSchema->SetStringField(TEXT("task_costs"),
		TEXT("map<spec index, {position_cost, orientation_cost | cost}>"));
}

void UMjMinkIKController::GetCurrentConfigInternal(TSharedPtr<FJsonObject>& OutParams) const
{
	OutParams = MakeShared<FJsonObject>();
	OutParams->SetNumberField(TEXT("max_iters"), MaxIters);
	OutParams->SetNumberField(TEXT("qp_damping"), QpDamping);
	OutParams->SetNumberField(TEXT("integrate_dt_override"), IntegrateDtOverride);
	OutParams->SetNumberField(TEXT("pos_threshold"), PosThreshold);
	OutParams->SetNumberField(TEXT("ori_threshold"), OriThreshold);
	OutParams->SetBoolField(TEXT("sync_from_live_state"), bSyncFromLiveState);
	OutParams->SetBoolField(TEXT("draw_target"), bDrawTarget);
	TArray<TSharedPtr<FJsonValue>> Enabled;
	for (const FMinkTaskSpec& S : Tasks)
	{
		Enabled.Add(MakeShared<FJsonValueBoolean>(S.bEnabled));
	}
	OutParams->SetArrayField(TEXT("task_enabled"), Enabled);
	// Dense per-spec cost report (write side is the sparse "task_costs" map).
	TArray<TSharedPtr<FJsonValue>> Costs;
	for (const FMinkTaskSpec& S : Tasks)
	{
		TSharedPtr<FJsonObject> O = MakeShared<FJsonObject>();
		if (S.Kind == EMinkTaskKind::Frame)
		{
			O->SetNumberField(TEXT("position_cost"), S.PositionCost);
			O->SetNumberField(TEXT("orientation_cost"), S.OrientationCost);
		}
		else
		{
			O->SetNumberField(TEXT("cost"), S.Cost);
		}
		Costs.Add(MakeShared<FJsonValueObject>(O));
	}
	OutParams->SetArrayField(TEXT("task_costs"), Costs);
}

void UMjMinkIKController::ApplyConfigInternal(const TSharedPtr<FJsonObject>& InParams)
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
	if (InParams->TryGetNumberField(TEXT("integrate_dt_override"), V))
	{
		IntegrateDtOverride = (float)FMath::Max(0.0, V);
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
	if (InParams->TryGetBoolField(TEXT("draw_target"), B))
	{
		bDrawTarget = B;
	}
	const TArray<TSharedPtr<FJsonValue>>* Enabled = nullptr;
	if (InParams->TryGetArrayField(TEXT("task_enabled"), Enabled) && Enabled)
	{
		for (int32 i = 0; i < Enabled->Num() && i < Tasks.Num(); ++i)
		{
			Tasks[i].bEnabled = (*Enabled)[i]->AsBool();
		}
	}

	// Live per-task cost updates. Sparse map keyed by spec index; only the
	// fields present change:
	//   "task_costs": { "<idx>": {"position_cost": f, "orientation_cost": f},  (Frame)
	//                   "<idx>": {"cost": f} }                                 (Posture/Damping)
	// Costs clamp to >= 0; kind-mismatched fields warn and are ignored. Unlike
	// task_enabled/targets (read live each solve), costs are BAKED into the
	// built solver stack — so any accepted change bumps the spec generation and
	// the physics thread rebuilds from specs on its next step (MarkSpecsChanged,
	// the same path add_controller reconfigure uses).
	const TSharedPtr<FJsonObject>* CostsObj = nullptr;
	if (InParams->TryGetObjectField(TEXT("task_costs"), CostsObj) && CostsObj && CostsObj->IsValid())
	{
		bool bAnyCostChanged = false;
		for (const auto& Pair : (*CostsObj)->Values)
		{
			// Strict index parse: Atoi maps junk to 0, so require a round-trip.
			const int32 Idx = FCString::Atoi(*Pair.Key);
			if (FString::FromInt(Idx) != Pair.Key || !Tasks.IsValidIndex(Idx))
			{
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] task_costs: key '%s' is not a valid spec index (have %d specs) — entry ignored."),
					*Pair.Key, Tasks.Num());
				continue;
			}
			const TSharedPtr<FJsonObject>* Fields = nullptr;
			if (!Pair.Value.IsValid() || !Pair.Value->TryGetObject(Fields) || !Fields || !Fields->IsValid())
			{
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] task_costs[%s]: expected an object of cost fields — entry ignored."),
					*Pair.Key);
				continue;
			}
			FMinkTaskSpec& Spec = Tasks[Idx];
			const bool bFrame = Spec.Kind == EMinkTaskKind::Frame;
			double CV = 0.0;
			if ((*Fields)->TryGetNumberField(TEXT("position_cost"), CV))
			{
				if (bFrame)
				{
					Spec.PositionCost = (float)FMath::Max(0.0, CV);
					bAnyCostChanged = true;
				}
				else
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] task_costs[%d]: position_cost only applies to Frame tasks — ignored."), Idx);
				}
			}
			if ((*Fields)->TryGetNumberField(TEXT("orientation_cost"), CV))
			{
				if (bFrame)
				{
					Spec.OrientationCost = (float)FMath::Max(0.0, CV);
					bAnyCostChanged = true;
				}
				else
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] task_costs[%d]: orientation_cost only applies to Frame tasks — ignored."), Idx);
				}
			}
			if ((*Fields)->TryGetNumberField(TEXT("cost"), CV))
			{
				if (!bFrame)
				{
					Spec.Cost = (float)FMath::Max(0.0, CV);
					bAnyCostChanged = true;
				}
				else
				{
					UE_LOG(LogURLabRuntime, Warning,
						TEXT("[MinkIK] task_costs[%d]: cost applies to Posture/Damping tasks — use position_cost/orientation_cost for Frame."),
						Idx);
				}
			}
		}
		if (bAnyCostChanged)
		{
			MarkSpecsChanged();
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
