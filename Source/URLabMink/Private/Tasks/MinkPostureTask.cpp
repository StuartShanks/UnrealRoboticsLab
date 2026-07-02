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

#include "Tasks/MinkPostureTask.h"

#include "MinkConfiguration.h"
#include "MinkUtils.h"

FMinkPostureTask::FMinkPostureTask(const mjModel* Model, const FMinkVec& Cost, double Gain, double LmDamping)
	: FMinkTask(FMinkVec::Zero(Model->nv), Gain, LmDamping)
	, ModelRef(Model)
{
	TArray<int32> QIds;
	TArray<int32> VIds;
	MinkGetFreejointDims(Model, QIds, VIds);
	FreeVIds = VIds;

	K = Model->nv;
	NqCached = Model->nq;
	SetCost(Cost);
}

bool FMinkPostureTask::SetCost(const FMinkVec& NewCost)
{
	if (!(NewCost.size() == 1 || NewCost.size() == K))
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("FMinkPostureTask cost must be a vector of shape (1,) (aka identical cost for all dofs) or "
				 "(%d,). Got (%lld,)"),
			K, (long long)NewCost.size());
		return false;
	}
	if (!(NewCost.array() >= 0.0).all())
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkPostureTask cost should be >= 0"));
		return false;
	}

	if (NewCost.size() == 1)
	{
		Cost.setConstant(K, NewCost(0));
	}
	else
	{
		Cost.head(K) = NewCost;
	}
	return true;
}

bool FMinkPostureTask::SetTarget(const FMinkVec& NewTargetQ)
{
	if (NewTargetQ.size() != NqCached)
	{
		UE_LOG(LogURLabMink, Error, TEXT("Expected target posture to have shape (%d,) but got (%lld,)"), NqCached,
			(long long)NewTargetQ.size());
		return false;
	}
	TargetQ = NewTargetQ;
	return true;
}

void FMinkPostureTask::SetTargetFromConfiguration(const FMinkConfiguration& Configuration)
{
	SetTarget(Configuration.GetQ());
}

bool FMinkPostureTask::ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const
{
	if (!TargetQ.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkPostureTask"));
		return false;
	}

	const FMinkVec Q = Configuration.GetQ();
	Out.resize(Configuration.Nv());
	// NOTE: mj_differentiatePos computes qpos2 (-) qpos1 == q (-) target_q, so target is qpos1
	// (Python mink's compute_error passes qpos1=target_q, qpos2=configuration.q).
	mj_differentiatePos(Configuration.Model, Out.data(), 1.0, TargetQ->data(), Q.data());

	for (const int32 VId : FreeVIds)
	{
		Out(VId) = 0.0;
	}
	return true;
}

bool FMinkPostureTask::ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const
{
	Out = FMinkMat::Identity(Configuration.Nv(), Configuration.Nv());
	for (const int32 VId : FreeVIds)
	{
		Out.col(VId).setZero();
	}
	return true;
}
