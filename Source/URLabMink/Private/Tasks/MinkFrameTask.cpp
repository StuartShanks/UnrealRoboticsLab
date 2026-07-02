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

#include "Tasks/MinkFrameTask.h"

#include "MinkConfiguration.h"

FMinkFrameTask::FMinkFrameTask(const FString& InFrameName, EMinkFrameType InFrameType,
	const FMinkVec& PositionCost, const FMinkVec& OrientationCost, double Gain, double LmDamping)
	: FMinkTask(FMinkVec::Zero(K), Gain, LmDamping)
	, FrameName(InFrameName)
	, FrameType(InFrameType)
{
	SetPositionCost(PositionCost);
	SetOrientationCost(OrientationCost);
}

bool FMinkFrameTask::SetPositionCost(const FMinkVec& NewCost)
{
	if (!(NewCost.size() == 1 || NewCost.size() == 3))
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("FMinkFrameTask position cost should be a vector of shape 1 (aka identical cost for all "
				 "coordinates) or (3,) but got (%lld,)"),
			(long long)NewCost.size());
		return false;
	}
	if (!(NewCost.array() >= 0.0).all())
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkFrameTask position cost should be >= 0"));
		return false;
	}

	if (NewCost.size() == 1)
	{
		Cost.head(3).setConstant(NewCost(0));
	}
	else
	{
		Cost.head(3) = NewCost;
	}
	return true;
}

bool FMinkFrameTask::SetOrientationCost(const FMinkVec& NewCost)
{
	if (!(NewCost.size() == 1 || NewCost.size() == 3))
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("FMinkFrameTask orientation cost should be a vector of shape 1 (aka identical cost for all "
				 "coordinates) or (3,) but got (%lld,)"),
			(long long)NewCost.size());
		return false;
	}
	if (!(NewCost.array() >= 0.0).all())
	{
		// NOTE: mirrors mink's own copy-paste quirk — the orientation setter's non-negativity
		// error text says "position cost should be >= 0" in the pinned Python reference.
		UE_LOG(LogURLabMink, Error, TEXT("FMinkFrameTask position cost should be >= 0"));
		return false;
	}

	if (NewCost.size() == 1)
	{
		Cost.tail(3).setConstant(NewCost(0));
	}
	else
	{
		Cost.tail(3) = NewCost;
	}
	return true;
}

void FMinkFrameTask::SetTarget(const FMinkSE3& NewTransformTargetToWorld)
{
	TransformTargetToWorld = NewTransformTargetToWorld;
}

bool FMinkFrameTask::SetTargetFromConfiguration(const FMinkConfiguration& Configuration)
{
	FMinkSE3 Frame;
	if (!Configuration.GetTransformFrameToWorld(FrameName, FrameType, Frame))
	{
		return false;
	}
	SetTarget(Frame);
	return true;
}

bool FMinkFrameTask::ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const
{
	if (!TransformTargetToWorld.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkFrameTask"));
		return false;
	}

	FMinkSE3 Frame;
	if (!Configuration.GetTransformFrameToWorld(FrameName, FrameType, Frame))
	{
		return false;
	}
	Out = TransformTargetToWorld->RMinus(Frame);
	return true;
}

bool FMinkFrameTask::ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const
{
	if (!TransformTargetToWorld.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkFrameTask"));
		return false;
	}

	FMinkMat Jac;
	if (!Configuration.GetFrameJacobian(FrameName, FrameType, Jac))
	{
		return false;
	}
	FMinkSE3 Frame;
	if (!Configuration.GetTransformFrameToWorld(FrameName, FrameType, Frame))
	{
		return false;
	}

	const FMinkSE3 TTb = TransformTargetToWorld->Inverse().Multiply(Frame);
	Out = -TTb.Jlog() * Jac;
	return true;
}

bool FMinkFrameTask::ErrorAndJacobian(const FMinkConfiguration& Configuration, FMinkVec& Error, FMinkMat& Jacobian) const
{
	if (!TransformTargetToWorld.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkFrameTask"));
		return false;
	}

	FMinkSE3 Frame;
	if (!Configuration.GetTransformFrameToWorld(FrameName, FrameType, Frame))
	{
		return false;
	}
	FMinkMat Jac;
	if (!Configuration.GetFrameJacobian(FrameName, FrameType, Jac))
	{
		return false;
	}

	Error = TransformTargetToWorld->RMinus(Frame);
	const FMinkSE3 TTb = TransformTargetToWorld->Inverse().Multiply(Frame);
	Jacobian = -TTb.Jlog() * Jac;
	return true;
}

bool FMinkFrameTask::ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const
{
	FMinkVec Error;
	FMinkMat Jacobian;
	if (!ErrorAndJacobian(Configuration, Error, Jacobian))
	{
		return false;
	}
	AssembleQp(Error, Jacobian, Configuration.EyeNv, Out);
	return true;
}

EMinkTaskStatus FMinkFrameTask::ComputeQpResidual(const FMinkConfiguration& Configuration, FMinkResidual& Out) const
{
	FMinkVec Error;
	FMinkMat Jacobian;
	if (!ErrorAndJacobian(Configuration, Error, Jacobian))
	{
		return EMinkTaskStatus::Error;
	}
	WeightedResidual(Error, Jacobian, Out);
	return EMinkTaskStatus::Ok;
}
