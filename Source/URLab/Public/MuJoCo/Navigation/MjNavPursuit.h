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

/**
 * Holonomic pure-pursuit math for the mobile-base nav stack. Pure functions,
 * no UObject/world dependencies — unit-tested directly (MjNavPursuitTests).
 *
 * Inputs are UE space (cm, LH, UE yaw radians where +yaw is clockwise from
 * above). The OUTPUT twist is in the twist-bus convention used by
 * UMjTwistController / UMjBaseDriveController: robot-frame, metres/sec,
 * x-forward / y-LEFT, yaw_rate positive CCW (geometry_msgs/Twist-aligned).
 * The UE→bus conversion happens in exactly one place: ComputeTwist step 5.
 */
namespace MjNavPursuit
{
struct FPursuitParams
{
	float MaxSpeed = 0.6f;            // m/s
	float MaxYawRate = 1.2f;          // rad/s
	float LookaheadDist = 60.f;       // cm (UE)
	float AcceptanceRadius = 15.f;    // cm (UE)
	float DecelRadius = 75.f;         // cm (UE)
	float YawGain = 2.0f;             // 1/s
	float MinSpeedForHeading = 0.05f; // m/s — below this, hold yaw
};

struct FPursuitState
{
	FVector Position = FVector::ZeroVector; // UE cm (Z ignored)
	float YawRad = 0.f;                     // UE yaw, radians
};

struct FPursuitResult
{
	float Vx = 0.f;      // m/s, robot forward (bus convention)
	float Vy = 0.f;      // m/s, robot LEFT (bus convention)
	float YawRate = 0.f; // rad/s, CCW positive (bus convention)
	bool bArrived = false;
	FVector LookaheadPoint = FVector::ZeroVector; // UE cm, for debug draw
	/** Desired heading toward the direction of travel, UE yaw rad (+CW from
	 *  above). Always computed (the yaw-RATE command stays gated on
	 *  MinSpeedForHeading); on arrival / empty path it holds State.YawRad.
	 *  LookaheadPoint + this is the "carrot" pose a whole-body IK consumer
	 *  can track instead of the twist. */
	float DesiredYawRad = 0.f;
};

/** Signed shortest angular error FromRad→ToRad in (-PI, PI]. */
URLAB_API float ShortestAngleRad(float FromRad, float ToRad);

/**
 * One pursuit step. PathPoints is the navmesh polyline in UE cm (>= 1 point;
 * last point is the goal). Returns zero twist + bArrived when within
 * AcceptanceRadius of the goal. Z is ignored throughout (planar base).
 */
URLAB_API FPursuitResult ComputeTwist(const TArray<FVector>& PathPoints,
	const FPursuitState& State, const FPursuitParams& P);
} // namespace MjNavPursuit
