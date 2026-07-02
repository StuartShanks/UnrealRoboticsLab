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
 * Port of mink tasks/dof_freezing_task.py DofFreezingTask — freezes specific degrees of
 * freedom to zero velocity. Typically used as an equality constraint to prevent selected
 * joints from moving. Since neither the error nor the Jacobian depend on the configuration,
 * both are computed once at construction and cached.
 */
class URLABMINK_API FMinkDofFreezingTask : public FMinkTask
{
public:
	/** Validates InDofIndices is non-empty, every entry lies in [0, Model->nv), and there are no
	 * duplicates; on failure logs the exact Python TaskDefinitionError message text and sets
	 * bIsValid = false instead of raising (see global constraints deviation #1). On success,
	 * DofIndices is InDofIndices sorted ascending, and the (k x nv) selector Jacobian plus the
	 * zero (k,) error are cached. */
	FMinkDofFreezingTask(const mjModel* Model, const TArray<int32>& InDofIndices, double Gain = 1.0);

	/** e(q) = zeros(k) — the error is always zero since we're constraining velocity, not position. */
	bool ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const override;

	/** J(q) = cached selector matrix (k x nv); row i has a 1.0 at column DofIndices[i]. */
	bool ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const override;

	TArray<int32> DofIndices;

private:
	FMinkVec CachedError;
	FMinkMat CachedJacobian;
};
