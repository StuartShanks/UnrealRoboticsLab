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
#include "MuJoCo/Components/Controllers/MjArticulationController.h"
#include "MjBaseDriveController.generated.h"

class UMjTwistController;

UENUM(BlueprintType)
enum class EMjBaseDriveActuatorMode : uint8
{
	/** Base actuators are position actuators: integrate a position target
	 *  from the twist each step (tidybot). */
	PositionIntegrate,
	/** Base actuators are velocity actuators: write the world-frame
	 *  velocity directly to ctrl. */
	VelocityDirect,
};

/**
 * Drives a planar holonomic base (x-slide, y-slide, z-hinge) from the twist
 * stored in a sibling UMjTwistController — the missing in-engine consumer of
 * the twist bus. WASD/gamepad possession, the `set_twist` bridge op, and
 * UMjNavComponent all drive the base through this one controller.
 *
 * IMPORTANT: a bound UMjArticulationController replaces the ENTIRE default
 * ctrl path for its articulation (ApplyControls returns after ComputeAndApply)
 * — so this controller writes the three base actuators from the twist and
 * passes every other binding through via ResolveDesiredControl, keeping arm
 * actuators live.
 *
 * Twist convention (bus): robot-frame (vx fwd, vy LEFT, yaw_rate CCW),
 * m/s + rad/s — MuJoCo/ROS handedness. All math here is MuJoCo-side.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Base Drive Controller"))
class URLAB_API UMjBaseDriveController : public UMjArticulationController
{
	GENERATED_BODY()

public:
	/** Base joints in order X-slide, Y-slide, yaw-hinge. Resolved against the
	 *  compiled model by suffix (import prefixes tolerated). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Base Drive")
	TArray<FString> BaseJointNames = {TEXT("joint_x"), TEXT("joint_y"), TEXT("joint_th")};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Base Drive")
	EMjBaseDriveActuatorMode ActuatorMode = EMjBaseDriveActuatorMode::PositionIntegrate;

	/** Anti-windup: integrated position target may lead the live joint by at
	 *  most this much (metres, slides). Bounds force when blocked. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Base Drive")
	float MaxLeashLinear = 0.05f;

	/** Anti-windup leash for the yaw hinge (radians). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Base Drive")
	float MaxLeashAngular = 0.1f;

	/** If |target − qpos| exceeds this (m or rad), re-seed target = qpos
	 *  (catches keyframe resets / teleports). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Base Drive")
	float ReseedThreshold = 0.25f;

	// --- UMjArticulationController ---
	virtual void Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& ActuatorIdMap) override;
	virtual void ComputeAndApply(mjModel* m, mjData* d, uint8 Source) override;

private:
	/** Index into Bindings for base X / Y / TH, or -1 if unresolved. */
	int32 BaseBindingIdx[3] = {-1, -1, -1};
	/** Integrated position targets for X / Y / TH (MuJoCo units). */
	double Target[3] = {0, 0, 0};
	bool bBaseResolved = false;

	/** Sibling twist source; resolved in Bind (game thread), read on the
	 *  physics thread via its thread-safe GetTwist(). */
	UPROPERTY()
	UMjTwistController* TwistSource = nullptr;
};
