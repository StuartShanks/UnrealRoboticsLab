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

#include "Tasks/MinkComTask.h"

#include "MinkConfiguration.h"

FMinkComTask::FMinkComTask(const FMinkVec& Cost, double Gain, double LmDamping)
	: FMinkTask(FMinkVec::Zero(K), Gain, LmDamping)
{
	SetCost(Cost);
}

bool FMinkComTask::SetCost(const FMinkVec& NewCost)
{
	if (!(NewCost.size() == 1 || NewCost.size() == K))
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("FMinkComTask cost must be a vector of shape (1,) (aka identical cost for all coordinates) or "
				 "(%d,). Got (%lld,)"),
			K, (long long)NewCost.size());
		return false;
	}
	if (!(NewCost.array() >= 0.0).all())
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkComTask cost must be >= 0"));
		return false;
	}

	if (NewCost.size() == 1)
	{
		Cost.setConstant(K, NewCost(0));
	}
	else
	{
		Cost = NewCost;
	}
	return true;
}

bool FMinkComTask::SetTarget(const FMinkVec3& NewTargetCom)
{
	TargetCom = NewTargetCom;
	return true;
}

void FMinkComTask::SetTargetFromConfiguration(const FMinkConfiguration& Configuration)
{
	// Body 0 is the world body, so we start from body 1 (the robot) — mirrors Python's TODO not
	// to hardcode the subtree index.
	const double* P = Configuration.Data->subtree_com + 3;
	SetTarget(FMinkVec3(P[0], P[1], P[2]));
}

bool FMinkComTask::ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const
{
	if (!TargetCom.IsSet())
	{
		UE_LOG(LogURLabMink, Error, TEXT("No target set for FMinkComTask"));
		return false;
	}

	const double* P = Configuration.Data->subtree_com + 3;
	const FMinkVec3 Com(P[0], P[1], P[2]);
	Out = Com - *TargetCom;
	return true;
}

bool FMinkComTask::ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const
{
	FMinkRowMat Jac(K, Configuration.Nv());
	mj_jacSubtreeCom(Configuration.Model, Configuration.Data, Jac.data(), 1);
	Out = Jac;
	return true;
}
