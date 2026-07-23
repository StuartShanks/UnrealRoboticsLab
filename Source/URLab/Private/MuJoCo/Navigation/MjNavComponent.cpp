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

FMjNavGoalDecision UMjNavComponent::DecideNavGoal(UWorld* World, const FVector& StartUE,
	const FVector& WorldGoal, float MaxProjectionCm, float AcceptanceRadiusCm)
{
	FMjNavGoalDecision D;
	D.ProjectedUE = WorldGoal;

	UNavigationSystemV1* Nav = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	if (!Nav)
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: no navigation system in world."));
		D.Reject = EMjNavGoalReject::OffNavmesh;
		return D;
	}
	FNavLocation Projected;
	if (!Nav->ProjectPointToNavigation(WorldGoal, Projected, FVector(100, 100, 500)))
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: goal %s is off the navmesh."), *WorldGoal.ToString());
		D.Reject = EMjNavGoalReject::OffNavmesh;
		return D;
	}
	D.ProjectedUE = Projected.Location;
	D.ProjectionCm = FVector::Dist2D(WorldGoal, Projected.Location);

	// Strict mode: the caller bounded how far projection may move their goal.
	if (MaxProjectionCm >= 0.f && D.ProjectionCm > MaxProjectionCm)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("UMjNavComponent: goal %s projected %.0f cm onto the navmesh (max %.0f) — rejected."),
			*WorldGoal.ToString(), D.ProjectionCm, MaxProjectionCm);
		D.Reject = EMjNavGoalReject::ProjectionExceedsMax;
		return D;
	}
	// Historical mode: accept, but never silently — a displaced goal changes
	// what 'arrived' means, so say so at least in the log.
	if (D.ProjectionCm > AcceptanceRadiusCm)
	{
		UE_LOG(LogURLab, Log,
			TEXT("UMjNavComponent: goal %s SUBSTITUTED — projected %.0f cm to %s (arrival is measured against the projected point)."),
			*WorldGoal.ToString(), D.ProjectionCm, *Projected.Location.ToString());
	}

	UNavigationPath* P = Nav->FindPathToLocationSynchronously(World, StartUE, Projected.Location);
	if (!P || !P->IsValid() || P->PathPoints.Num() < 1)
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: no path to %s."), *WorldGoal.ToString());
		D.Reject = EMjNavGoalReject::NoPath;
		return D;
	}
	// Partial paths end short of even the projected goal — the second silent
	// shortfall. Strict mode rejects; historical mode reports.
	D.bPartial = P->IsPartial();
	if (MaxProjectionCm >= 0.f && D.bPartial)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("UMjNavComponent: path to %s is PARTIAL (ends short of the goal) — rejected in strict mode."),
			*WorldGoal.ToString());
		D.Reject = EMjNavGoalReject::PartialPath;
		return D;
	}
	D.bAccepted = true;
	D.PathPoints = P->PathPoints;
	return D;
}

bool UMjNavComponent::SetNavGoal(FVector WorldGoal, float MaxProjectionCm)
{
	// Goal-substitution report: reset per attempt so GetLastGoalInfo always
	// describes THIS call, including rejections.
	LastRequestedGoalUE = WorldGoal;
	LastProjectedGoalUE = WorldGoal;
	LastProjectionCm = -1.f;
	bLastPathPartial = false;
	LastRejectReason = EMjNavGoalReject::None;
	bHasGoalInfo = true;

	FVector Start;
	float Yaw;
	if (!GetBasePose(Start, Yaw))
	{
		UE_LOG(LogURLab, Warning, TEXT("UMjNavComponent: could not resolve base body pose."));
		return false;
	}

	const FMjNavGoalDecision D =
		DecideNavGoal(GetWorld(), Start, WorldGoal, MaxProjectionCm, AcceptanceRadius);
	LastProjectedGoalUE = D.ProjectedUE;
	LastProjectionCm = D.ProjectionCm;
	bLastPathPartial = D.bPartial;
	LastRejectReason = D.Reject;
	if (!D.bAccepted)
		return false;

	Path = D.PathPoints;
	BestDistToGoalCm = TNumericLimits<float>::Max();
	TimeSinceProgress = 0.f;
	DistToGoalM.store(FVector::Dist2D(Start, Path.Last()) / 100.f, std::memory_order_release);
	SetState(EMjNavState::Navigating);
	return true;
}

float UMjNavComponent::GetDistanceToRequestedM() const
{
	if (!bHasGoalInfo)
		return -1.f;
	FVector Pos;
	float Yaw;
	if (!GetBasePose(Pos, Yaw))
		return -1.f;
	return FVector::Dist2D(Pos, LastRequestedGoalUE) / 100.f;
}

void UMjNavComponent::SetPathForTesting(const TArray<FVector>& PathPoints)
{
	Path = PathPoints;
	BestDistToGoalCm = TNumericLimits<float>::Max();
	TimeSinceProgress = 0.f;
	DistToGoalM.store(0.f, std::memory_order_release);
	SetState(EMjNavState::Navigating);
}

bool UMjNavComponent::GetLookahead(FVector& OutUEPos, float& OutUEYawRad) const
{
	if (!bHasLookahead)
		return false;
	OutUEPos = LookaheadUE;
	OutUEYawRad = LookaheadYawUE;
	return true;
}

void UMjNavComponent::StopWithState(EMjNavState S)
{
	if (UMjTwistController* T = FindTwist())
		T->SetTwist(0.f, 0.f, 0.f);
	DistToGoalM.store(S == EMjNavState::Arrived ? 0.f : -1.f, std::memory_order_release);
	// Arrived keeps the final carrot (the goal pose, stored by the tick that
	// detected arrival) so a carrot consumer can hold station; Idle/Failed
	// invalidate it.
	if (S != EMjNavState::Arrived)
		bHasLookahead = false;
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

	// Publish the carrot before the arrival check: on the arrival tick the
	// result carries the goal point + held yaw, which is exactly the pose a
	// carrot consumer should hold after Arrived.
	LookaheadUE = R.LookaheadPoint;
	LookaheadYawUE = R.DesiredYawRad;
	bHasLookahead = true;

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
