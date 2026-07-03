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

#include "Limits/MinkConfigurationLimit.h"

#include "MinkConfiguration.h"

FMinkConfigurationLimit::FMinkConfigurationLimit(const mjModel* Model, double InGain, double MinDistanceFromLimits)
	: ModelRef(Model)
	, Gain(InGain)
{
	if (!(0.0 < InGain && InGain <= 1.0))
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkConfigurationLimit gain must be in the range (0, 1]"));
		bIsValid = false;
		return;
	}

	TArray<int32> IndexList;
	Lower = FMinkVec::Constant(Model->nq, -mjMAXVAL);
	Upper = FMinkVec::Constant(Model->nq, mjMAXVAL);
	for (int32 Jnt = 0; Jnt < Model->njnt; ++Jnt)
	{
		const int32 JntType = Model->jnt_type[Jnt];
		// Skip free joints and joints without limits.
		if (JntType == mjJNT_FREE || !Model->jnt_limited[Jnt])
		{
			continue;
		}

		const int32 QposDim = MinkQposWidth(JntType);
		const int32 Padr = Model->jnt_qposadr[Jnt];
		Lower.segment(Padr, QposDim).setConstant(Model->jnt_range[2 * Jnt + 0] + MinDistanceFromLimits);
		Upper.segment(Padr, QposDim).setConstant(Model->jnt_range[2 * Jnt + 1] - MinDistanceFromLimits);

		const int32 JntDim = MinkDofWidth(JntType);
		const int32 JntId = Model->jnt_dofadr[Jnt];
		for (int32 K = 0; K < JntDim; ++K)
		{
			IndexList.Add(JntId + K);
		}
	}

	Indices = IndexList;

	const int32 Dim = Indices.Num();
	if (Dim > 0)
	{
		const FMinkMat Eye = FMinkMat::Identity(Model->nv, Model->nv);
		FMinkMat Proj(Dim, Model->nv);
		for (int32 Index = 0; Index < Dim; ++Index)
		{
			Proj.row(Index) = Eye.row(Indices[Index]);
		}
		ProjectionMatrix = MoveTemp(Proj);
	}
}

bool FMinkConfigurationLimit::ComputeQpInequalities(
	const FMinkConfiguration& Configuration, double Dt, FMinkInequality& Out) const
{
	Out = FMinkInequality();
	if (!ProjectionMatrix.IsSet())
	{
		return true;
	}

	const FMinkVec Q = Configuration.GetQ();
	const int32 Nv = ModelRef->nv;

	// Upper.
	FMinkVec DeltaQMax(Nv);
	mj_differentiatePos(ModelRef, DeltaQMax.data(), 1.0, Q.data(), Upper.data());

	// Lower. NOTE: mj_differentiatePos computes qpos2 - qpos1, so the argument order is
	// swapped compared to the upper computation above (mirrors the Python NOTE comment).
	FMinkVec DeltaQMin(Nv);
	mj_differentiatePos(ModelRef, DeltaQMin.data(), 1.0, Lower.data(), Q.data());

	const int32 Dim = Indices.Num();
	FMinkVec PMin(Dim);
	FMinkVec PMax(Dim);
	for (int32 Index = 0; Index < Dim; ++Index)
	{
		PMin(Index) = Gain * DeltaQMin(Indices[Index]);
		PMax(Index) = Gain * DeltaQMax(Indices[Index]);
	}

	FMinkMat G(2 * Dim, Nv);
	G.topRows(Dim) = *ProjectionMatrix;
	G.bottomRows(Dim) = -(*ProjectionMatrix);

	FMinkVec H(2 * Dim);
	H.head(Dim) = PMax;
	H.tail(Dim) = PMin;

	Out.G = MoveTemp(G);
	Out.H = MoveTemp(H);
	return true;
}
