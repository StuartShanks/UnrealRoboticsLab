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

#include "Tasks/MinkRelativeFrameTask.h"

#include "MinkConfiguration.h"

FMinkRelativeFrameTask::FMinkRelativeFrameTask(const FString& InFrameName, EMinkFrameType InFrameType,
	const FString& InRootName, EMinkFrameType InRootType, const FMinkVec& PositionCost,
	const FMinkVec& OrientationCost, double Gain, double LmDamping)
	: FMinkTask(FMinkVec::Zero(K), Gain, LmDamping)
	, FrameName(InFrameName)
	, RootName(InRootName)
	, FrameType(InFrameType)
	, RootType(InRootType)
{
	SetPositionCost(PositionCost);
	SetOrientationCost(OrientationCost);
}

bool FMinkRelativeFrameTask::SetPositionCost(const FMinkVec& NewCost)
{
	if (!(NewCost.size() == 1 || NewCost.size() == 3))
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("FMinkRelativeFrameTask position cost should be a vector of shape 1 (aka identical cost for all "
				 "coordinates) or (3,) but got (%lld,)"),
			(long long)NewCost.size());
		return false;
	}
	if (!(NewCost.array() >= 0.0).all())
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkRelativeFrameTask position cost should be >= 0"));
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

bool FMinkRelativeFrameTask::SetOrientationCost(const FMinkVec& NewCost)
{
	if (!(NewCost.size() == 1 || NewCost.size() == 3))
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("FMinkRelativeFrameTask orientation cost should be a vector of shape 1 (aka identical cost for "
				 "all coordinates) or (3,) but got (%lld,)"),
			(long long)NewCost.size());
		return false;
	}
	if (!(NewCost.array() >= 0.0).all())
	{
		// NOTE: mirrors mink's own copy-paste quirk — the orientation setter's non-negativity
		// error text says "position cost should be >= 0" in the pinned Python reference.
		UE_LOG(LogURLabMink, Error, TEXT("FMinkRelativeFrameTask position cost should be >= 0"));
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

void FMinkRelativeFrameTask::SetTarget(const FMinkSE3& NewTransformTargetToRoot)
{
	TransformTargetToRoot = NewTransformTargetToRoot;
}

bool FMinkRelativeFrameTask::SetTargetFromConfiguration(const FMinkConfiguration& Configuration)
{
	FMinkSE3 Transform;
	if (!Configuration.GetTransform(FrameName, FrameType, RootName, RootType, Transform))
	{
		return false;
	}
	SetTarget(Transform);
	return true;
}

bool FMinkRelativeFrameTask::ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const
{
	if (!TransformTargetToRoot.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkRelativeFrameTask"));
		return false;
	}

	FMinkSE3 Frame;
	if (!Configuration.GetTransform(FrameName, FrameType, RootName, RootType, Frame))
	{
		return false;
	}
	Out = Frame.RMinus(*TransformTargetToRoot);
	return true;
}

bool FMinkRelativeFrameTask::ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const
{
	if (!TransformTargetToRoot.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkRelativeFrameTask"));
		return false;
	}

	FMinkMat JacFrame;
	if (!Configuration.GetFrameJacobian(FrameName, FrameType, JacFrame))
	{
		return false;
	}
	FMinkMat JacRoot;
	if (!Configuration.GetFrameJacobian(RootName, RootType, JacRoot))
	{
		return false;
	}
	FMinkSE3 Frame;
	if (!Configuration.GetTransform(FrameName, FrameType, RootName, RootType, Frame))
	{
		return false;
	}

	const FMinkSE3 TFt = TransformTargetToRoot->Inverse().Multiply(Frame);
	Out = TFt.Jlog() * (JacFrame - Frame.Inverse().Adjoint() * JacRoot);
	return true;
}

bool FMinkRelativeFrameTask::ErrorAndJacobian(
	const FMinkConfiguration& Configuration, FMinkVec& Error, FMinkMat& Jacobian) const
{
	if (!TransformTargetToRoot.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkRelativeFrameTask"));
		return false;
	}

	FMinkMat JacFrame;
	if (!Configuration.GetFrameJacobian(FrameName, FrameType, JacFrame))
	{
		return false;
	}
	FMinkMat JacRoot;
	if (!Configuration.GetFrameJacobian(RootName, RootType, JacRoot))
	{
		return false;
	}
	FMinkSE3 Frame;
	if (!Configuration.GetTransform(FrameName, FrameType, RootName, RootType, Frame))
	{
		return false;
	}

	Error = Frame.RMinus(*TransformTargetToRoot);
	const FMinkSE3 TFt = TransformTargetToRoot->Inverse().Multiply(Frame);
	Jacobian = TFt.Jlog() * (JacFrame - Frame.Inverse().Adjoint() * JacRoot);
	return true;
}

bool FMinkRelativeFrameTask::ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const
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

EMinkTaskStatus FMinkRelativeFrameTask::ComputeQpResidual(
	const FMinkConfiguration& Configuration, FMinkResidual& Out) const
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
