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
 * Port of mink tasks/posture_task.py PostureTask — regulates joint angles towards a target
 * posture q*. Often used as a low-priority regularizer biasing the IK solution.
 */
class URLABMINK_API FMinkPostureTask : public FMinkTask
{
public:
	FMinkPostureTask(const mjModel* Model, const FMinkVec& Cost, double Gain = 1.0, double LmDamping = 0.0);

	/** Cost must be a vector of shape (1,) (broadcast to all dofs) or (K,). Returns false + error
	 * log (Python TaskDefinitionError message text) on shape mismatch or a negative entry. */
	bool SetCost(const FMinkVec& Cost);

	/** TargetQ must have size Nq. Returns false + error log (Python InvalidTarget message) otherwise. */
	bool SetTarget(const FMinkVec& TargetQ);

	void SetTargetFromConfiguration(const FMinkConfiguration& Configuration);

	/** e(q) = q - target_q via mj_differentiatePos; freejoint dofs zeroed. False + error log
	 * ("No target set for FMinkPostureTask") if no target has been set. */
	bool ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const override;

	/** J(q) = I(nv) with freejoint columns zeroed. */
	bool ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const override;

	TOptional<FMinkVec> TargetQ;
	int32 K = 0;
	int32 NqCached = 0;

protected:
	const mjModel* ModelRef;
	TArray<int32> FreeVIds;
};
