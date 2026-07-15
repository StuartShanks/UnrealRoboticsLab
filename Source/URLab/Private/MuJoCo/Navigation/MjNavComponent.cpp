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

#include "MuJoCo/Navigation/MjNavComponent.h"
#include "Utils/URLabLogging.h" // LogURLab — nav warnings must be visible

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Bodies/MjWorldBody.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "NavigationSystem.h"
#include "NavigationPath.h"
#include "DrawDebugHelpers.h"

UMjNavComponent::UMjNavComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

UMjTwistController* UMjNavComponent::FindTwist() const
{
	return GetOwner() ? GetOwner()->FindComponentByClass<UMjTwistController>() : nullptr;
}

UMjBody* UMjNavComponent::ResolveBaseBody() const
{
	const AMjArticulation* Art = Cast<AMjArticulation>(GetOwner());
	if (!Art)
		return nullptr;
	if (!BaseBodyName.IsEmpty())
	{
		// Exact, then suffix match (import prefixes).
		if (UMjBody* B = Art->GetBody(BaseBodyName))
			return B;
		for (UMjBody* B : Art->GetBodies())
			if (B && B->GetName().EndsWith(BaseBodyName))
				return B;
		return nullptr;
	}
	// Default: the body that OWNS the planar base joints. "First body attached
	// to the world body" is unreliable — a scene may declare an IK mocap marker
	// (e.g. the TidyBot's pinch_site_target) as the first world child, which is
	// static, so nav would read a frozen pose and never close its feedback loop.
	// Identify the base by its joints instead: find a joint named joint_x/_y/_th
	// (suffix-tolerant for import prefixes) and return its owning UMjBody.
	//
	// NOTE: these three joint names are HARD-CODED — they are the fixed base
	// convention shared with UMjBaseDriveController's default BaseJointNames.
	// If the base joints are ever renamed, update both places (or set the
	// component's BaseBodyName override above).
	auto IsBaseJoint = [](const FString& N) {
		for (const TCHAR* Suf : {TEXT("joint_x"), TEXT("joint_y"), TEXT("joint_th")})
		{
			const FString S(Suf);
			if (N == S || N.EndsWith(FString(TEXT("_")) + S) || N.EndsWith(FString(TEXT("/")) + S))
				return true;
		}
		return false;
	};
	for (UMjJoint* J : Art->GetJoints())
	{
		if (!J || !IsBaseJoint(J->GetMjName()))
			continue;
		for (USceneComponent* P = J->GetAttachParent(); P; P = P->GetAttachParent())
			if (UMjBody* B = Cast<UMjBody>(P))
				return B;
	}
	// Fallback: no body owns joint_x/_y/_th (e.g. a rig without the standard
	// planar base joints) — use the first body attached to the world body (the
	// pre-existing default). Only reached when the joint match above finds
	// nothing, so it never re-introduces the mocap-marker pick for real tidybots.
	for (UMjBody* B : Art->GetBodies())
		if (B && Cast<UMjWorldBody>(B->GetAttachParent()))
			return B;
	return nullptr;
}

bool UMjNavComponent::GetBasePose(FVector& OutPos, float& OutYawRad) const
{
	const UMjBody* B = ResolveBaseBody();
	if (!B)
		return false;
	// UE component transform: snapshot-synced by the manager pump; safe on
	// the game thread. (GetWorldPosition() reads live mjData — never here.)
	OutPos = B->GetComponentLocation();
	OutYawRad = FMath::DegreesToRadians(B->GetComponentRotation().Yaw);
	return true;
}

bool UMjNavComponent::SetNavGoal(FVector WorldGoal)
{
	UNavigationSystemV1* Nav = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!Nav)
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: no navigation system in world."));
		return false;
	}
	FVector Start;
	float Yaw;
	if (!GetBasePose(Start, Yaw))
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: could not resolve base body pose."));
		return false;
	}
	FNavLocation Projected;
	if (!Nav->ProjectPointToNavigation(WorldGoal, Projected, FVector(100, 100, 500)))
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: goal %s is off the navmesh."), *WorldGoal.ToString());
		return false;
	}
	UNavigationPath* P = Nav->FindPathToLocationSynchronously(GetWorld(), Start, Projected.Location);
	if (!P || !P->IsValid() || P->PathPoints.Num() < 1)
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: no path to %s."), *WorldGoal.ToString());
		return false;
	}
	Path = P->PathPoints;
	BestDistToGoalCm = TNumericLimits<float>::Max();
	TimeSinceProgress = 0.f;
	DistToGoalM.store(FVector::Dist2D(Start, Path.Last()) / 100.f, std::memory_order_release);
	SetState(EMjNavState::Navigating);
	return true;
}

void UMjNavComponent::SetPathForTesting(const TArray<FVector>& PathPoints)
{
	Path = PathPoints;
	BestDistToGoalCm = TNumericLimits<float>::Max();
	TimeSinceProgress = 0.f;
	DistToGoalM.store(0.f, std::memory_order_release);
	SetState(EMjNavState::Navigating);
}

void UMjNavComponent::StopWithState(EMjNavState S)
{
	if (UMjTwistController* T = FindTwist())
		T->SetTwist(0.f, 0.f, 0.f);
	DistToGoalM.store(S == EMjNavState::Arrived ? 0.f : -1.f, std::memory_order_release);
	SetState(S);
	if (S == EMjNavState::Arrived)
		OnNavGoalReached.Broadcast();
	else if (S == EMjNavState::Failed)
		OnNavGoalFailed.Broadcast();
}

void UMjNavComponent::ClearNavGoal()
{
	Path.Reset();
	StopWithState(EMjNavState::Idle);
}

void UMjNavComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (GetNavState() != EMjNavState::Navigating || Path.Num() == 0)
		return;

	MjNavPursuit::FPursuitState S;
	if (!GetBasePose(S.Position, S.YawRad))
	{
		StopWithState(EMjNavState::Failed);
		return;
	}

	MjNavPursuit::FPursuitParams P;
	P.MaxSpeed = MaxSpeed;
	P.MaxYawRate = MaxYawRate;
	P.LookaheadDist = LookaheadDist;
	P.AcceptanceRadius = AcceptanceRadius;
	P.DecelRadius = DecelRadius;
	P.YawGain = YawGain;
	P.MinSpeedForHeading = MinSpeedForHeading;

	const MjNavPursuit::FPursuitResult R = MjNavPursuit::ComputeTwist(Path, S, P);

	const float DistCm = FVector::Dist2D(S.Position, Path.Last());
	DistToGoalM.store(DistCm / 100.f, std::memory_order_release);

	if (R.bArrived)
	{
		StopWithState(EMjNavState::Arrived);
		return;
	}

	// Progress watchdog.
	if (DistCm < BestDistToGoalCm - MinProgress)
	{
		BestDistToGoalCm = DistCm;
		TimeSinceProgress = 0.f;
	}
	else
	{
		TimeSinceProgress += DeltaTime;
		if (TimeSinceProgress >= StuckTimeout)
		{
			StopWithState(EMjNavState::Failed);
			return;
		}
	}

	if (UMjTwistController* T = FindTwist())
		T->SetTwist(R.Vx, R.Vy, R.YawRate);

	if (bDebugDraw)
	{
		for (int32 i = 0; i + 1 < Path.Num(); ++i)
			DrawDebugLine(GetWorld(), Path[i], Path[i + 1], FColor::Green, false, -1.f, 0, 2.f);
		DrawDebugSphere(GetWorld(), R.LookaheadPoint, 8.f, 8, FColor::Yellow, false, -1.f);
	}
}

void UMjNavComponent::EndPlay(const EEndPlayReason::Type Reason)
{
	if (UMjTwistController* T = FindTwist())
		T->SetTwist(0.f, 0.f, 0.f);
	Super::EndPlay(Reason);
}
