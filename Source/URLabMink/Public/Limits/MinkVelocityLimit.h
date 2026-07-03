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
#include "Limits/MinkLimit.h"

class FMinkConfiguration;

/**
 * Port of mink limits/velocity_limit.py VelocityLimit — inequality constraint on joint
 * velocities in a robot model. Floating base joints are not supported.
 *
 * Velocities uses an ordered TArray<TPair<FString, FMinkVec>> rather than a map (see global
 * constraints deviation #4) so that the G/h row order matches the Python dict insertion order
 * exactly.
 */
class URLABMINK_API FMinkVelocityLimit : public FMinkLimit
{
public:
	/**
	 * For each (joint name, max velocity) pair, in order: a free joint name sets bIsValid =
	 * false and logs "Free joint <name> is not supported"; a limit vector whose size does not
	 * match dof_width(jnt_type) sets bIsValid = false and logs the exact Python shape-mismatch
	 * message (see global constraints deviation #1).
	 */
	FMinkVelocityLimit(const mjModel* Model, const TArray<TPair<FString, FMinkVec>>& Velocities);

	bool bIsValid = true;

	/** Port of VelocityLimit.compute_qp_inequalities. */
	bool ComputeQpInequalities(const FMinkConfiguration& Configuration, double Dt, FMinkInequality& Out) const override;

	TArray<int32> Indices;
	FMinkVec Limit;

private:
	TOptional<FMinkMat> ProjectionMatrix; // eye(nv)[indices]
};
