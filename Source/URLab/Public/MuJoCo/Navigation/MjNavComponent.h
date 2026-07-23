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

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MuJoCo/Navigation/MjNavPursuit.h"
#include <atomic>
#include "MjNavComponent.generated.h"

class UMjTwistController;
class UMjBody;

UENUM(BlueprintType)
enum class EMjNavState : uint8
{
	Idle,
	Navigating,
	Arrived,
	Failed,
};

/** Why the last SetNavGoal returned false (None while accepted / no goal yet). */
enum class EMjNavGoalReject : uint8
{
	None,
	OffNavmesh,           // projection found no navmesh point within the search extent
	ProjectionExceedsMax, // projected goal displaced beyond the caller's MaxProjectionCm
	PartialPath,          // path ends short of the projected goal (strict mode only)
	NoPath,               // pathfinder returned nothing
};

/** Outcome of the goal projection + path decision (see DecideNavGoal). */
struct FMjNavGoalDecision
{
	bool bAccepted = false;
	FVector ProjectedUE = FVector::ZeroVector; // requested goal when projection failed
	float ProjectionCm = -1.f;                 // requested->projected 2D displacement; -1 = no projection
	bool bPartial = false;                     // path ends short of the projected goal
	EMjNavGoalReject Reject = EMjNavGoalReject::None;
	TArray<FVector> PathPoints;                // valid when bAccepted
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnMjNavEvent);

/**
 * Game-thread navigation for a holonomic MuJoCo base. Plans on the UE Recast
 * navmesh, follows the path with holonomic pure pursuit (MjNavPursuit), and
 * writes the resulting robot-frame twist to the sibling UMjTwistController —
 * the same bus WASD and the `set_twist` op use. Consumed physics-side by
 * UMjBaseDriveController. Knows nothing about actuators or mjData.
 *
 * Pose source: the base UMjBody's UE component transform (snapshot-synced by
 * the manager pump). Never reads live mjData (physics-thread data).
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Nav Component"))
class URLAB_API UMjNavComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UMjNavComponent();

	/** Base body for pose. Empty ⇒ first UMjBody attached to the world body. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav")
	FString BaseBodyName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Motion")
	float MaxSpeed = 0.6f; // m/s
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Motion")
	float MaxYawRate = 1.2f; // rad/s
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Motion")
	float LookaheadDist = 60.f; // cm
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Motion")
	float AcceptanceRadius = 15.f; // cm
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Motion")
	float DecelRadius = 75.f; // cm
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Motion")
	float YawGain = 2.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Motion")
	float MinSpeedForHeading = 0.05f; // m/s

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Watchdog")
	float StuckTimeout = 5.f; // s
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Watchdog")
	float MinProgress = 5.f; // cm

	/** Draw the path polyline + lookahead point each tick while navigating. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav|Debug")
	bool bDebugDraw = false;

	UPROPERTY(BlueprintAssignable, Category = "Nav")
	FOnMjNavEvent OnNavGoalReached;
	UPROPERTY(BlueprintAssignable, Category = "Nav")
	FOnMjNavEvent OnNavGoalFailed;

	/** Plan a path to WorldGoal (UE cm) and start following it. Returns false
	 *  (no state change) if the goal can't be projected to the navmesh or no
	 *  path exists. Game thread only.
	 *
	 *  Projection is NOT identity: goals inside an obstacle's agent-radius
	 *  erosion band get silently relocated up to the projection extent (~1 m)
	 *  to the nearest navigable point, and the follower then "arrives" at the
	 *  SUBSTITUTE. Query GetLastGoalInfo() for requested vs projected.
	 *  MaxProjectionCm >= 0 opts into strict mode: reject when the projection
	 *  displaces beyond it, and reject partial paths (which end short of even
	 *  the projected goal). Default -1 keeps the historical accept-anything
	 *  behavior. */
	UFUNCTION(BlueprintCallable, Category = "Nav")
	bool SetNavGoal(FVector WorldGoal, float MaxProjectionCm = -1.f);

	/** Requested vs projected goal of the most recent SetNavGoal (UE cm), the
	 *  2D displacement between them (cm; -1 before any successful projection),
	 *  whether the accepted path was partial, and the reject reason (None when
	 *  accepted). Only meaningful after SetNavGoal ran at least once — check
	 *  HasGoalInfo(). Game thread only. */
	void GetLastGoalInfo(FVector& OutRequestedUE, FVector& OutProjectedUE,
		float& OutProjectionCm, bool& bOutPartial, EMjNavGoalReject& OutReject) const
	{
		OutRequestedUE = LastRequestedGoalUE;
		OutProjectedUE = LastProjectedGoalUE;
		OutProjectionCm = LastProjectionCm;
		bOutPartial = bLastPathPartial;
		OutReject = LastRejectReason;
	}

	bool HasGoalInfo() const { return bHasGoalInfo; }

	/** 2D distance (metres) from the base body to the REQUESTED (pre-projection)
	 *  goal — the honest "did I get where the caller asked" number, as opposed
	 *  to GetDistanceToGoal() which measures against the projected path end.
	 *  Returns -1 with no goal or unresolvable base pose. Game thread only. */
	float GetDistanceToRequestedM() const;

	/** The projection + path decision behind SetNavGoal, world/start explicit
	 *  so tests can exercise it against a real navmesh without an articulation
	 *  rig (whose FMjUESession world has no navigation system). Pure query —
	 *  mutates nothing. Game thread only (navmesh queries). */
	static FMjNavGoalDecision DecideNavGoal(UWorld* World, const FVector& StartUE,
		const FVector& WorldGoal, float MaxProjectionCm, float AcceptanceRadiusCm);

	UFUNCTION(BlueprintCallable, Category = "Nav")
	void ClearNavGoal();

	UFUNCTION(BlueprintCallable, Category = "Nav")
	EMjNavState GetNavState() const { return (EMjNavState)StateAtomic.load(std::memory_order_acquire); }

	/** Straight-line distance to goal in metres; -1 when no goal. Thread-safe
	 *  (read by the get_nav_status bridge op off the game thread). */
	UFUNCTION(BlueprintCallable, Category = "Nav")
	float GetDistanceToGoal() const { return DistToGoalM.load(std::memory_order_acquire); }

	/** Latest pursuit lookahead ("carrot") pose: position in UE cm plus the
	 *  desired UE yaw (rad, +CW from above — heading toward the direction of
	 *  travel). Valid while Navigating (from the first tick after a goal is
	 *  set) and after Arrived (holds the goal point); returns false when
	 *  Idle/Failed. Game thread only — written by TickComponent, and the
	 *  get_nav_status bridge op marshals its read to the game thread. This is
	 *  the pose a whole-body IK consumer streams as a base Frame-task target
	 *  instead of consuming the twist. */
	bool GetLookahead(FVector& OutUEPos, float& OutUEYawRad) const;

	/** TEST SEAM: start following an injected polyline (UE cm) without a
	 *  navmesh. Same follower path as SetNavGoal. */
	void SetPathForTesting(const TArray<FVector>& PathPoints);

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	TArray<FVector> Path; // UE cm, game thread only
	std::atomic<uint8> StateAtomic{(uint8)EMjNavState::Idle};
	std::atomic<float> DistToGoalM{-1.f};

	// Watchdog bookkeeping (game thread).
	float BestDistToGoalCm = TNumericLimits<float>::Max();
	float TimeSinceProgress = 0.f;

	// Latest pursuit carrot (game thread; see GetLookahead).
	FVector LookaheadUE = FVector::ZeroVector;
	float LookaheadYawUE = 0.f;
	bool bHasLookahead = false;

	// Last SetNavGoal bookkeeping for the goal-substitution report (game
	// thread; the get_nav_status/set_nav_goal ops marshal their reads).
	FVector LastRequestedGoalUE = FVector::ZeroVector;
	FVector LastProjectedGoalUE = FVector::ZeroVector;
	float LastProjectionCm = -1.f;
	bool bLastPathPartial = false;
	bool bHasGoalInfo = false;
	EMjNavGoalReject LastRejectReason = EMjNavGoalReject::None;

	void SetState(EMjNavState S) { StateAtomic.store((uint8)S, std::memory_order_release); }
	void StopWithState(EMjNavState S); // zero twist + set state (+ fire delegate)
	bool GetBasePose(FVector& OutPos, float& OutYawRad) const;
	UMjTwistController* FindTwist() const;
	UMjBody* ResolveBaseBody() const;
};
