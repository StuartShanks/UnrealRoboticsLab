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

#include "Limits/MinkVelocityLimit.h"

#include "MinkConfiguration.h"

FMinkVelocityLimit::FMinkVelocityLimit(const mjModel* Model, const TArray<TPair<FString, FMinkVec>>& Velocities)
{
	TArray<int32> IndexList;
	TArray<double> LimitList;

	for (const TPair<FString, FMinkVec>& Entry : Velocities)
	{
		const FString& JointName = Entry.Key;
		const int32 Jid = mj_name2id(Model, mjOBJ_JOINT, TCHAR_TO_UTF8(*JointName));
		if (Jid == -1)
		{
			UE_LOG(LogURLabMink, Error, TEXT("Joint %s does not exist in the model."), *JointName);
			bIsValid = false;
			return;
		}

		const int32 JntType = Model->jnt_type[Jid];
		if (JntType == mjJNT_FREE)
		{
			UE_LOG(LogURLabMink, Error, TEXT("Free joint %s is not supported"), *JointName);
			bIsValid = false;
			return;
		}

		const int32 Vadr = Model->jnt_dofadr[Jid];
		const int32 Vdim = MinkDofWidth(JntType);
		const FMinkVec& MaxVel = Entry.Value;
		if (MaxVel.size() != Vdim)
		{
			UE_LOG(LogURLabMink, Error,
				TEXT("Joint %s must have a limit of shape (%d,). Got: (%lld,)"), *JointName, Vdim,
				(long long)MaxVel.size());
			bIsValid = false;
			return;
		}

		for (int32 K = 0; K < Vdim; ++K)
		{
			IndexList.Add(Vadr + K);
			LimitList.Add(MaxVel(K));
		}
	}

	Indices = IndexList;
	Limit = FMinkVec(LimitList.Num());
	for (int32 Index = 0; Index < LimitList.Num(); ++Index)
	{
		Limit(Index) = LimitList[Index];
	}

	const int32 Nb = Indices.Num();
	if (Nb > 0)
	{
		const FMinkMat Eye = FMinkMat::Identity(Model->nv, Model->nv);
		FMinkMat Proj(Nb, Model->nv);
		for (int32 Index = 0; Index < Nb; ++Index)
		{
			Proj.row(Index) = Eye.row(Indices[Index]);
		}
		ProjectionMatrix = MoveTemp(Proj);
	}
}

bool FMinkVelocityLimit::ComputeQpInequalities(
	const FMinkConfiguration& Configuration, double Dt, FMinkInequality& Out) const
{
	Out = FMinkInequality();
	if (!ProjectionMatrix.IsSet())
	{
		return true;
	}

	const int32 Nb = Indices.Num();
	const int32 Nv = static_cast<int32>(ProjectionMatrix->cols());

	FMinkMat G(2 * Nb, Nv);
	G.topRows(Nb) = *ProjectionMatrix;
	G.bottomRows(Nb) = -(*ProjectionMatrix);

	FMinkVec H(2 * Nb);
	H.head(Nb) = Dt * Limit;
	H.tail(Nb) = Dt * Limit;

	Out.G = MoveTemp(G);
	Out.H = MoveTemp(H);
	return true;
}
