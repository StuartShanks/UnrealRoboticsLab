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
#include "Lie/MinkSE3.h"
#include "Tasks/MinkTask.h"

class FMinkConfiguration;

/**
 * Port of mink tasks/frame_task.py FrameTask — regulates the position and orientation of a
 * frame of interest (body, geom or site) on the robot towards a target pose in the world frame.
 */
class URLABMINK_API FMinkFrameTask : public FMinkTask
{
public:
	FMinkFrameTask(const FString& FrameName, EMinkFrameType FrameType, const FMinkVec& PositionCost /*1 or 3*/,
		const FMinkVec& OrientationCost /*1 or 3*/, double Gain = 1.0, double LmDamping = 0.0);

	/** Cost.head(3). Must be a vector of shape (1,) (broadcast) or (3,), all entries >= 0. Returns false
	 * + error log (Python TaskDefinitionError message text) otherwise. */
	bool SetPositionCost(const FMinkVec& PositionCost);

	/** Cost.tail(3). Must be a vector of shape (1,) (broadcast) or (3,), all entries >= 0. Returns false
	 * + error log (Python TaskDefinitionError message text) otherwise. NOTE: mirrors mink's own
	 * copy-paste quirk — the negativity error text says "position cost should be >= 0" even here. */
	bool SetOrientationCost(const FMinkVec& OrientationCost);

	/** Set the target pose directly. */
	void SetTarget(const FMinkSE3& TransformTargetToWorld);

	/** Set the target pose from the frame's current pose in Configuration. False + error log if the
	 * frame cannot be resolved. */
	bool SetTargetFromConfiguration(const FMinkConfiguration& Configuration);

	/** e(q) = target.RMinus(frame) = (frame^-1 * target).Log(). False + error log
	 * ("No target set for FMinkFrameTask") if no target has been set. */
	bool ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const override;

	/** J(q) = -T_tb.Jlog() * frameJac, with T_tb = target^-1 * frame. */
	bool ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const override;

	// Overridden to share the frame transform between error & jacobian (mirrors _error_and_jacobian):
	bool ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const override;
	EMinkTaskStatus ComputeQpResidual(const FMinkConfiguration& Configuration, FMinkResidual& Out) const override;

	FString FrameName;
	EMinkFrameType FrameType;
	TOptional<FMinkSE3> TransformTargetToWorld;
	static constexpr int32 K = 6;

private:
	/** Port of FrameTask._error_and_jacobian — fetches the frame transform once and shares it. */
	bool ErrorAndJacobian(const FMinkConfiguration& Configuration, FMinkVec& Error, FMinkMat& Jacobian) const;
};
