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

#include "Tasks/MinkDofFreezingTask.h"

#include "MinkConfiguration.h"

namespace
{
/** Renders a TArray<int32> the way Python's list.__repr__ would, e.g. "[0, 2, 0]". */
FString PyListRepr(const TArray<int32>& Values)
{
	FString Out = TEXT("[");
	for (int32 Index = 0; Index < Values.Num(); ++Index)
	{
		if (Index > 0)
		{
			Out += TEXT(", ");
		}
		Out += FString::FromInt(Values[Index]);
	}
	Out += TEXT("]");
	return Out;
}
} // namespace

FMinkDofFreezingTask::FMinkDofFreezingTask(const mjModel* Model, const TArray<int32>& InDofIndices, double Gain)
	: FMinkTask(FMinkVec::Ones(InDofIndices.Num()), Gain, /*LmDamping=*/0.0)
{
	if (InDofIndices.Num() == 0)
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkDofFreezingTask requires at least one DOF index."));
		bIsValid = false;
		return;
	}

	for (const int32 DofIdx : InDofIndices)
	{
		if (DofIdx < 0 || DofIdx >= Model->nv)
		{
			UE_LOG(LogURLabMink, Error, TEXT("DOF index %d is out of range [0, %d)."), DofIdx, Model->nv);
			bIsValid = false;
			return;
		}
	}

	const TSet<int32> Unique(InDofIndices);
	if (Unique.Num() != InDofIndices.Num())
	{
		UE_LOG(LogURLabMink, Error, TEXT("Duplicate DOF indices found: %s."), *PyListRepr(InDofIndices));
		bIsValid = false;
		return;
	}

	DofIndices = InDofIndices;
	DofIndices.Sort();

	const int32 K = DofIndices.Num();
	const int32 Nv = Model->nv;
	CachedError = FMinkVec::Zero(K);
	CachedJacobian = FMinkMat::Zero(K, Nv);
	for (int32 Index = 0; Index < K; ++Index)
	{
		CachedJacobian(Index, DofIndices[Index]) = 1.0;
	}
}

bool FMinkDofFreezingTask::ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const
{
	Out = CachedError;
	return true;
}

bool FMinkDofFreezingTask::ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const
{
	Out = CachedJacobian;
	return true;
}
