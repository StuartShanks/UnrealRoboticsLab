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
#include "Tasks/MinkTask.h"

class FMinkConfiguration;

/**
 * Port of mink tasks/com_task.py ComTask — regulates the center of mass of the robot (the
 * subtree rooted at body 1, i.e. everything except the world body) towards a target CoM
 * position in the world frame.
 */
class URLABMINK_API FMinkComTask : public FMinkTask
{
public:
	explicit FMinkComTask(const FMinkVec& Cost /*1 or 3*/, double Gain = 1.0, double LmDamping = 0.0);

	/** Must be a vector of shape (1,) (broadcast to all 3 coordinates) or (3,), all entries >= 0.
	 * Returns false + error log (Python TaskDefinitionError message text) otherwise. */
	bool SetCost(const FMinkVec& Cost);

	/** Set the target CoM position in the world frame directly. */
	bool SetTarget(const FMinkVec3& TargetCom);

	/** Set the target CoM from Data->subtree_com[1] (the subtree rooted at body 1) at the current
	 * configuration. */
	void SetTargetFromConfiguration(const FMinkConfiguration& Configuration);

	/** e(q) = subtree_com[1] - target. False + error log ("No target set for FMinkComTask") if no
	 * target has been set. */
	bool ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const override;

	/** J(q) = mj_jacSubtreeCom(Model, Data, ..., 1); body index 1 hardcoded (mirrors Python's own
	 * TODO not to hardcode the subtree index). Does not require a target to be set. */
	bool ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const override;

	TOptional<FMinkVec3> TargetCom;
	static constexpr int32 K = 3;
};
