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

#include "Tasks/MinkKineticEnergyRegularizationTask.h"

#include "MinkConfiguration.h"

FMinkKineticEnergyRegularizationTask::FMinkKineticEnergyRegularizationTask(double InCost)
	: Cost(InCost)
{
	if (Cost < 0.0)
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkKineticEnergyRegularizationTask cost should be >= 0"));
		bIsValid = false;
	}
}

void FMinkKineticEnergyRegularizationTask::SetDt(double Dt)
{
	InvDtSq = 1.0 / (Dt * Dt);
}

bool FMinkKineticEnergyRegularizationTask::ComputeQpObjective(
	const FMinkConfiguration& Configuration, FMinkObjective& Out) const
{
	if (!InvDtSq.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No integration timestep set for FMinkKineticEnergyRegularizationTask"));
		return false;
	}

	Out.H = Cost * InvDtSq.GetValue() * Configuration.GetInertiaMatrix();
	Out.C = FMinkVec::Zero(Configuration.Nv());
	return true;
}
