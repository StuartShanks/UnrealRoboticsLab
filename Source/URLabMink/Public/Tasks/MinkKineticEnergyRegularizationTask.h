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
 * Port of mink tasks/kinetic_energy_regularization_task.py KineticEnergyRegularizationTask —
 * penalizes the system's kinetic energy T ~= 1/2 dq^T (cost * M(q) / dt^2) dq, where M(q) is
 * the joint-space inertia matrix. Often used with a low priority in the task stack as an
 * inertia-weighted version of DampingTask. Unlike FMinkTask subclasses, this is a direct
 * FMinkBaseTask subclass: it has no error/Jacobian/residual form (it exercises the
 * EMinkTaskStatus::NoResidualForm dense fallback inherited from FMinkBaseTask). The
 * integration timestep must be set via SetDt before ComputeQpObjective is called.
 */
class URLABMINK_API FMinkKineticEnergyRegularizationTask : public FMinkBaseTask
{
public:
	/** cost must be >= 0. On failure logs the exact Python TaskDefinitionError message text and
	 * sets bIsValid = false instead of raising (see global constraints deviation #1). */
	explicit FMinkKineticEnergyRegularizationTask(double Cost);

	/** Sets the integration timestep in seconds, storing InvDtSq = 1 / dt^2. */
	void SetDt(double Dt);

	/** H = cost * InvDtSq * Configuration.GetInertiaMatrix(); C = zeros(nv). False + error log
	 * ("No integration timestep set for FMinkKineticEnergyRegularizationTask") if SetDt has not
	 * been called yet. */
	bool ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const override;

	bool bIsValid = true;
	double Cost = 0.0;
	TOptional<double> InvDtSq;
};
