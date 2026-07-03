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
 * Port of mink limits/configuration_limit.py ConfigurationLimit — inequality constraint on
 * joint positions in a robot model. Floating base joints are ignored.
 */
class URLABMINK_API FMinkConfigurationLimit : public FMinkLimit
{
public:
	/**
	 * Validates gain is in (0, 1]; on failure logs the exact Python LimitDefinitionError
	 * message text and sets bIsValid = false instead of raising (see global constraints
	 * deviation #1). Free joints and unlimited joints are skipped when building Lower/Upper
	 * and Indices, mirroring the Python __init__.
	 */
	FMinkConfigurationLimit(const mjModel* Model, double Gain = 0.95, double MinDistanceFromLimits = 0.0);

	bool bIsValid = true;

	/** Port of ConfigurationLimit.compute_qp_inequalities. */
	bool ComputeQpInequalities(const FMinkConfiguration& Configuration, double Dt, FMinkInequality& Out) const override;

private:
	const mjModel* ModelRef = nullptr;
	double Gain = 0.95;
	TArray<int32> Indices;                // limited dof indices
	FMinkVec Lower, Upper;                // nq-length vectors (±mjMAXVAL padding outside limited joints)
	TOptional<FMinkMat> ProjectionMatrix; // eye(nv)[indices]
};
