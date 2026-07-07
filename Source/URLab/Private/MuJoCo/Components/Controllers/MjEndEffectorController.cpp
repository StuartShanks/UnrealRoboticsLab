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

#include "MuJoCo/Components/Controllers/MjEndEffectorController.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"
#include "EngineUtils.h"

namespace
{
// Mirrors the resolver in MjBody.cpp / MjQuickConvertComponent.cpp: the manager
// owns the (single) physics engine the mocap command queue lives on.
UMjPhysicsEngine* GetEngine(const UObject* WorldCtx)
{
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
			return Manager->PhysicsEngine;
	}
	if (WorldCtx)
	{
		if (UWorld* World = WorldCtx->GetWorld())
		{
			for (TActorIterator<AAMjManager> It(World); It; ++It)
			{
				if (It->PhysicsEngine)
					return It->PhysicsEngine;
			}
		}
	}
	return nullptr;
}

// The registered MuJoCo name of a body component: MjName if set, else its UE name.
FString ResolveBodyName(const UMjBody* Body)
{
	return Body->MjName.IsEmpty() ? Body->GetName() : Body->MjName;
}

// Find the sibling UMjBody whose registered name matches TargetBodyName.
UMjBody* FindTargetBody(AActor* Owner, const FString& TargetBodyName)
{
	if (!Owner)
		return nullptr;

	TArray<UMjBody*> Bodies;
	Owner->GetComponents<UMjBody>(Bodies);
	for (UMjBody* Body : Bodies)
	{
		if (Body && !Body->bIsDefault && ResolveBodyName(Body) == TargetBodyName)
			return Body;
	}
	return nullptr;
}
} // namespace

UMjEndEffectorController::UMjEndEffectorController()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UMjEndEffectorController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bDriveTarget || MocapBodyId < 0)
		return;

	if (bUseGizmo)
		SetEndEffectorTarget(GetComponentLocation(), GetComponentQuat());
	else
		SetEndEffectorTarget(TargetPosition, TargetRotation.Quaternion());
}

void UMjEndEffectorController::InjectMocapAndWeld(FMujocoSpecWrapper& Wrapper)
{
	if (!bEnabled)
		return;

	if (TargetBodyName.IsEmpty())
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjEndEffectorController] '%s' has no TargetBodyName set — skipping."),
			*GetName());
		return;
	}

	UMjBody* TargetBody = FindTargetBody(GetOwner(), TargetBodyName);
	if (!TargetBody)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjEndEffectorController] '%s' could not find target body '%s' on owner."),
			*GetName(), *TargetBodyName);
		return;
	}

	// Place the mocap body coincident with the target body's initial world pose.
	// CreateBody parents to the spec's world body and converts the UE world
	// transform to MuJoCo — the same convention root bodies use when attaching
	// to world, so the mocap body lands exactly where the EE body compiles to.
	// With the weld's relpose left at zero, MuJoCo bakes "coincident" as the held
	// relative pose, so the constraint is satisfied at t=0 (no startup snap).
	MocapBodyName = GetName() + TEXT("_mocap");

	mjsBody* World = mjs_findBody(Wrapper.Spec, "world");
	mjsBody* Mocap = Wrapper.CreateBody(MocapBodyName, World, TargetBody->GetComponentTransform());
	if (!Mocap)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjEndEffectorController] '%s' failed to create mocap body."), *GetName());
		return;
	}
	Mocap->mocap = 1;

	// Weld the mocap body (name1) to the end-effector body (name2). Names are the
	// unprefixed child-spec names; mjs_attach prefixes both consistently so the
	// references resolve at compile time (same path UMjEquality relies on).
	mjsEquality* Weld = mjs_addEquality(Wrapper.Spec, nullptr);
	if (!Weld)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjEndEffectorController] '%s' failed to add weld equality."), *GetName());
		return;
	}
	const FString WeldName = GetName() + TEXT("_weld");
	mjs_setName(Weld->element, TCHAR_TO_UTF8(*WeldName));

	Weld->type = mjEQ_WELD;
	Weld->objtype = mjOBJ_BODY;
	MjSetString(Weld->name1, MocapBodyName);
	MjSetString(Weld->name2, TargetBodyName);

	Weld->solref[0] = (mjtNum)Solref.X;
	Weld->solref[1] = (mjtNum)Solref.Y;
	for (int32 i = 0; i < Solimp.Num() && i < 5; ++i)
		Weld->solimp[i] = (mjtNum)Solimp[i];

	// Weld data layout: [0:3] anchor, [3:10] relpose (pos+quat), [10] torquescale.
	// Leave anchor + relpose zero (coincident); only set torquescale.
	Weld->data[10] = (mjtNum)TorqueScale;

	UE_LOG(LogURLab, Log, TEXT("[MjEndEffectorController] '%s' injected mocap '%s' welded to '%s'."),
		*GetName(), *MocapBodyName, *TargetBodyName);
}

void UMjEndEffectorController::ResolveAfterCompile(mjModel* Model, mjData* Data, const FString& Prefix)
{
	MocapBodyId = -1;
	if (!Model || !bEnabled || MocapBodyName.IsEmpty())
		return;

	const FString PrefixedName = Prefix + MocapBodyName;
	MocapBodyId = mj_name2id(Model, mjOBJ_BODY, TCHAR_TO_UTF8(*PrefixedName));
	if (MocapBodyId < 0)
	{
		// Fall back to the unprefixed name in case attach ran without a prefix.
		MocapBodyId = mj_name2id(Model, mjOBJ_BODY, TCHAR_TO_UTF8(*MocapBodyName));
	}

	if (MocapBodyId < 0 || Model->body_mocapid[MocapBodyId] < 0)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjEndEffectorController] '%s' could not resolve mocap body '%s' (id=%d) in compiled model."),
			*GetName(), *PrefixedName, MocapBodyId);
		MocapBodyId = -1;
		return;
	}

	UE_LOG(LogURLab, Log, TEXT("[MjEndEffectorController] '%s' resolved mocap body '%s' -> id %d (mocapid %d)."),
		*GetName(), *PrefixedName, MocapBodyId, Model->body_mocapid[MocapBodyId]);

	// Seed the gizmo and the explicit target on the end-effector's initial pose so
	// flipping on bDriveTarget doesn't yank the arm from wherever the gizmo happened
	// to sit. Uses the target body's editor-time transform (== the initial pose the
	// mocap body was welded at).
	if (UMjBody* TargetBody = FindTargetBody(GetOwner(), TargetBodyName))
	{
		const FTransform EePose = TargetBody->GetComponentTransform();
		SetWorldLocationAndRotation(EePose.GetLocation(), EePose.GetRotation());
		TargetPosition = EePose.GetLocation();
		TargetRotation = EePose.Rotator();
	}
}

void UMjEndEffectorController::SetEndEffectorTarget(FVector WorldPos, FQuat WorldRot)
{
	if (!bEnabled || MocapBodyId < 0)
		return;

	UMjPhysicsEngine* Engine = GetEngine(this);
	if (!Engine)
		return;

	double MjPos[3];
	double MjQuat[4];
	MjUtils::UEToMjPosition(WorldPos, MjPos);
	MjUtils::UEToMjRotation(WorldRot, MjQuat);
	Engine->SubmitMocapPose(MocapBodyId, MjPos, MjQuat);
}

void UMjEndEffectorController::SnapTargetToEndEffector()
{
	if (MocapBodyId < 0)
		return;

	UMjBody* TargetBody = FindTargetBody(GetOwner(), TargetBodyName);
	if (!TargetBody)
		return;

	// Reads the EE pose from the latest render snapshot (xpos/xquat mapped to UE)
	// and re-commands the mocap target to match it.
	SetEndEffectorTarget(TargetBody->GetWorldPosition(), TargetBody->GetWorldRotation());
}

#if WITH_EDITOR
TArray<FString> UMjEndEffectorController::GetBodyOptions() const
{
	return UMjComponent::GetSiblingComponentOptions(this, UMjBody::StaticClass());
}
#endif
