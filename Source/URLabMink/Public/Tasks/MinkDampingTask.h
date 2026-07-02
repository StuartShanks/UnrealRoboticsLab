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
#include "Tasks/MinkPostureTask.h"

class FMinkConfiguration;

/**
 * Port of mink tasks/damping_task.py DampingTask — L2-regularization on joint velocities
 * (a.k.a. velocity damping). Contributes 1/2 dq^T diag(cost)^2 dq to the QP objective,
 * favoring minimum-norm joint velocities in redundant or near-singular situations. Unlike
 * PostureTask, this task does not chase a target posture — it never needs one set, since its
 * error is identically zero. gain and lm_damping are fixed to 0.0 (see ctor).
 */
class URLABMINK_API FMinkDampingTask : public FMinkPostureTask
{
public:
	FMinkDampingTask(const mjModel* Model, const FMinkVec& Cost);

	/** e(q) = zeros(nv) unconditionally — the damping task has no reference to chase. */
	bool ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const override;
};
