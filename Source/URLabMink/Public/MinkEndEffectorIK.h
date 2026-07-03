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
#include "MinkTypes.h"
#include "MinkConfiguration.h"
#include "MinkSolveIK.h"
#include "Lie/MinkSE3.h"

class FMinkFrameTask;
class FMinkPostureTask;

/**
 * Configuration for FMinkEndEffectorIK: which frame to drive to a Cartesian target, which
 * joints the solver may write back, and the mink task/solver gains.
 */
struct URLABMINK_API FMinkEndEffectorIKConfig
{
	/** Name of the frame (body / site / geom) whose pose is driven to the target. */
	FString FrameName = TEXT("hand");
	EMinkFrameType FrameType = EMinkFrameType::Body;

	/**
	 * Joints whose qpos the solver writes back into the live data each step (single-DOF
	 * hinge/slide joints). Empty => every joint in the model. Names are resolved via
	 * mj_name2id against the (already prefixed) compiled model.
	 */
	TArray<FString> DriveJointNames;

	/** FrameTask position error weight (broadcast scalar). */
	double PositionCost = 1.0;
	/** FrameTask orientation error weight. Set 0 for position-only IK (e.g. under-actuated arms). */
	double OrientationCost = 1.0;
	/** PostureTask regularizer weight biasing the redundant DOFs toward the posture target. */
	double PostureCost = 1e-2;
	/** FrameTask proportional gain. */
	double Gain = 1.0;
	/** FrameTask Levenberg-Marquardt damping. */
	double LmDamping = 1e-3;
	/** QP regularization damping passed to MinkSolveIK. */
	double Damping = 1e-3;
	/** Forwarded to MinkSolveIK: abort with NotWithinConfigurationLimits on a limit violation. */
	bool bSafetyBreak = false;
};

/**
 * Differential end-effector IK built on the URLabMink solver, decoupled from Unreal/the sim.
 *
 * Given a compiled mjModel and a live mjData (owned by the caller — the sim), each SolveStep
 * reads the current qpos, builds a FrameTask toward the requested target pose (in MuJoCo
 * world coordinates) plus a PostureTask regularizer, calls MinkSolveIK, integrates the
 * returned tangent velocity, and writes the solved drive-joint qpos back into the live data
 * (zeroing those joints' qvel — kinematic apply). It never steps the sim or calls mj_forward
 * on the caller's data; the caller owns stepping/forwarding.
 *
 * This is the pure, headless-testable core of the live IK demo; the UE runtime glue (bridge
 * op + per-step callback) merely feeds it a target and calls SolveStep.
 */
class URLABMINK_API FMinkEndEffectorIK
{
public:
	FMinkEndEffectorIK(const mjModel* InModel, const FMinkEndEffectorIKConfig& InConfig);
	~FMinkEndEffectorIK();

	FMinkEndEffectorIK(const FMinkEndEffectorIK&) = delete;
	FMinkEndEffectorIK& operator=(const FMinkEndEffectorIK&) = delete;

	/** True once the target frame and every drive joint resolved in the model. */
	bool IsValid() const { return bValid; }

	/** Override the posture-regularizer target (size nq). Defaults to the model's initial qpos. */
	void SetPostureTarget(const FMinkVec& TargetQ);

	/**
	 * One differential-IK step toward TargetInMjWorld (MuJoCo world frame). Reads LiveData's
	 * qpos, writes the solved drive-joint qpos back (and zeroes their qvel). Returns the solver
	 * status; on any non-Success status LiveData is left unmodified.
	 */
	EMinkIKStatus SolveStep(mjData* LiveData, const FMinkSE3& TargetInMjWorld, double Dt);

	/** World transform of the driven frame for LiveData's current qpos. */
	bool GetFrameTransform(const mjData* LiveData, FMinkSE3& Out) const;

private:
	const mjModel* Model = nullptr;
	FMinkEndEffectorIKConfig Config;
	TUniquePtr<FMinkConfiguration> Configuration;
	TUniquePtr<FMinkFrameTask> FrameTask;
	TUniquePtr<FMinkPostureTask> PostureTask;
	TArray<int32> DriveQposAdr;
	TArray<int32> DriveDofAdr;
	bool bValid = false;
};
