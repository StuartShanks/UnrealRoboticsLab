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
 * Shared twist -> base-target math for the twist-bus consumers:
 * UMjBaseDriveController (integrates into actuator position targets) and the
 * mink TwistFollow task (integrates into a posture-task target). Pure
 * functions, no UObject/mjModel dependencies — unit-tested directly
 * (MjBaseIntegrateTests).
 *
 * Conventions: the twist is the bus convention (robot frame, m/s, x-forward /
 * y-LEFT, yaw_rate +CCW — geometry_msgs/Twist-aligned, see UMjTwistController);
 * Theta, Measured, and the targets are MuJoCo joint space (joint_x / joint_y
 * metres, joint_th rad, CCW).
 */
namespace MjBaseIntegrate
{
struct FLeashParams
{
	/** Max distance the integrated target may lead the measured pose (m). */
	float MaxLeashLinear = 0.05f;
	/** Max angle the integrated target may lead the measured yaw (rad). */
	float MaxLeashAngular = 0.1f;
	/** |target - measured| beyond this snaps the target back to measured
	 *  (discontinuities: teleports, resets). */
	float ReseedThreshold = 0.25f;
};

/** Rotate a robot-frame twist into world-frame planar velocity
 *  [vx, vy, yaw_rate] at base yaw Theta (rad, CCW). */
URLAB_API void TwistToWorld(double Vx, double Vy, double YawRate, double Theta,
	double OutVWorld[3]);

/**
 * One integrate-with-leash step for a single base DOF: integrate Target by
 * V*Dt; reseed to Measured on discontinuity (ReseedThreshold); leash-clamp to
 * Measured +/- (angular or linear) leash; clamp to [RangeLo, RangeHi] when
 * bRangeLimited. Returns the new target. Exact semantics lifted from
 * UMjBaseDriveController's PositionIntegrate path.
 */
URLAB_API double IntegrateLeashed(double Target, double V, double Dt, double Measured,
	bool bAngular, const FLeashParams& P, bool bRangeLimited = false,
	double RangeLo = 0.0, double RangeHi = 0.0);
} // namespace MjBaseIntegrate
