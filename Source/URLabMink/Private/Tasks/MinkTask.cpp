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

#include "Tasks/MinkTask.h"

#include "MinkConfiguration.h"

double FMinkObjective::Value(const FMinkVec& X) const
{
	// Mirrors Python Objective.value: float(x.T @ H @ x + c @ x) — NOT halved.
	return X.dot(H * X) + C.dot(X);
}

FMinkTask::FMinkTask(const FMinkVec& InCost, double InGain, double InLmDamping)
	: Cost(InCost)
	, Gain(InGain)
	, LmDamping(InLmDamping)
{
	if (!(Gain >= 0.0 && Gain <= 1.0))
	{
		UE_LOG(LogURLabMink, Error, TEXT("`gain` must be in the range [0, 1]"));
		bIsValid = false;
	}
	if (LmDamping < 0.0)
	{
		UE_LOG(LogURLabMink, Error, TEXT("`lm_damping` must be >= 0"));
		bIsValid = false;
	}
}

void FMinkTask::WeightedResidual(const FMinkVec& Error, const FMinkMat& Jacobian, FMinkResidual& Out) const
{
	Out.WeightedError = Cost.cwiseProduct(-Gain * Error);
	Out.WeightedJacobian = Cost.asDiagonal() * Jacobian;
	Out.Mu = LmDamping * Out.WeightedError.squaredNorm();
}

void FMinkTask::AssembleQp(
	const FMinkVec& Error, const FMinkMat& Jacobian, const FMinkMat& EyeNv, FMinkObjective& Out) const
{
	FMinkResidual Residual;
	WeightedResidual(Error, Jacobian, Residual);

	Out.H = Residual.WeightedJacobian.transpose() * Residual.WeightedJacobian;
	if (Residual.Mu > 0.0)
	{
		Out.H += Residual.Mu * EyeNv;
	}
	Out.C = -(Residual.WeightedJacobian.transpose() * Residual.WeightedError);
}

bool FMinkTask::ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const
{
	FMinkVec Error;
	FMinkMat Jacobian;
	if (!ComputeError(Configuration, Error) || !ComputeJacobian(Configuration, Jacobian))
	{
		return false;
	}
	AssembleQp(Error, Jacobian, Configuration.EyeNv, Out);
	return true;
}

EMinkTaskStatus FMinkTask::ComputeQpResidual(const FMinkConfiguration& Configuration, FMinkResidual& Out) const
{
	FMinkVec Error;
	FMinkMat Jacobian;
	if (!ComputeError(Configuration, Error) || !ComputeJacobian(Configuration, Jacobian))
	{
		return EMinkTaskStatus::Error;
	}
	WeightedResidual(Error, Jacobian, Out);
	return EMinkTaskStatus::Ok;
}
