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

#include "Tasks/MinkEqualityConstraintTask.h"

#include "MinkConfiguration.h"

namespace
{
/** Renders a TArray<int32> the way Python's list.__repr__ would, e.g. "[0, 2, 0]". */
FString EqPyListRepr(const TArray<int32>& Values)
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

FMinkEqualityConstraintTask::FMinkEqualityConstraintTask(const mjModel* Model, const FMinkVec& Cost,
	const TArray<FString>& EqualityNames, const TArray<int32>& EqualityIds, double Gain, double LmDamping)
	: FMinkTask(FMinkVec::Zero(1), Gain, LmDamping)
	, ModelRef(Model)
{
	TArray<int32> ResolvedIds;

	if (EqualityNames.Num() > 0 || EqualityIds.Num() > 0)
	{
		for (const FString& Name : EqualityNames)
		{
			const int32 EqId = mj_name2id(Model, mjOBJ_EQUALITY, TCHAR_TO_UTF8(*Name));
			if (EqId == -1)
			{
				UE_LOG(LogURLabMink, Error, TEXT("Equality constraint '%s' not found."), *Name);
				bIsValid = false;
				return;
			}
			if (!Model->eq_active0[EqId])
			{
				UE_LOG(LogURLabMink, Error, TEXT("Equality constraint %d is not active at initial configuration."),
					EqId);
				bIsValid = false;
				return;
			}
			ResolvedIds.Add(EqId);
		}

		for (const int32 EqId : EqualityIds)
		{
			if (EqId < 0 || EqId >= Model->neq)
			{
				UE_LOG(LogURLabMink, Error,
					TEXT("Equality constraint index %d out of range. Must be in range [0, %lld)."), EqId,
					(long long)Model->neq);
				bIsValid = false;
				return;
			}
			if (!Model->eq_active0[EqId])
			{
				UE_LOG(LogURLabMink, Error, TEXT("Equality constraint %d is not active at initial configuration."),
					EqId);
				bIsValid = false;
				return;
			}
			ResolvedIds.Add(EqId);
		}

		const TSet<int32> Unique(ResolvedIds);
		if (Unique.Num() != ResolvedIds.Num())
		{
			UE_LOG(LogURLabMink, Error, TEXT("Duplicate equality constraint IDs provided: %s."),
				*EqPyListRepr(ResolvedIds));
			bIsValid = false;
			return;
		}
	}
	else
	{
		for (int32 EqId = 0; EqId < Model->neq; ++EqId)
		{
			ResolvedIds.Add(EqId);
		}
	}

	if (ResolvedIds.Num() == 0)
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkEqualityConstraintTask no equality constraints found in this model."));
		bIsValid = false;
		return;
	}

	EqIds = ResolvedIds;
	SetCost(Cost);
}

bool FMinkEqualityConstraintTask::SetCost(const FMinkVec& NewCost)
{
	const int32 NeqTotal = EqIds.Num();
	if (!(NewCost.size() == 1 || NewCost.size() == NeqTotal))
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("FMinkEqualityConstraintTask cost must be a vector of shape (1,) or (%d,). Got (%lld,)."), NeqTotal,
			(long long)NewCost.size());
		return false;
	}
	if (!(NewCost.array() >= 0.0).all())
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkEqualityConstraintTask cost must be >= 0"));
		return false;
	}

	// Per-equality cost, positionally aligned with EqIds (mirrors python self._cost).
	CostPerEq = (NewCost.size() == 1) ? FMinkVec::Constant(NeqTotal, NewCost(0)) : NewCost;

	// Expanded per-row cost assuming every resolved equality is currently active (mirrors python
	// self.cost = np.repeat(self._cost, repeats)); UpdateActiveConstraints overwrites this per
	// configuration once rows are known to actually be active.
	TArray<double> Repeated;
	for (int32 Index = 0; Index < NeqTotal; ++Index)
	{
		const int32 EqId = EqIds[Index];
		const int32 Width = MinkConstraintWidth(ModelRef->eq_type[EqId]);
		for (int32 W = 0; W < Width; ++W)
		{
			Repeated.Add(CostPerEq(Index));
		}
	}
	Cost.resize(Repeated.Num());
	for (int32 Index = 0; Index < Repeated.Num(); ++Index)
	{
		Cost(Index) = Repeated[Index];
	}
	return true;
}

bool FMinkEqualityConstraintTask::UpdateActiveConstraints(const FMinkConfiguration& Configuration) const
{
	const mjData* Data = Configuration.Data;

	ActiveRows.Reset();
	TArray<double> RowCosts;
	for (int32 Row = 0; Row < Data->nefc; ++Row)
	{
		if (Data->efc_type[Row] == mjCNSTR_EQUALITY && EqIds.Contains(Data->efc_id[Row]))
		{
			const int32 EqId = Data->efc_id[Row];
			// Upstream mink v1.2.0 raises IndexError here (cost indexed by raw eq id);
			// we convert to a controlled failure per the port's error model.
			const int32 IdIndex = EqIds.Find(EqId);
			if (IdIndex >= CostPerEq.size())
			{
				UE_LOG(LogURLabMink, Error,
					TEXT("[FMinkEqualityConstraintTask] equality id %d exceeds cost table size %d (upstream mink indexes cost by raw eq id — non-prefix selections are unsupported)"),
					EqId, (int32)CostPerEq.size());
				return false;
			}

			ActiveRows.Add(Row);
			RowCosts.Add(CostPerEq(IdIndex));
		}
	}

	Cost.resize(RowCosts.Num());
	for (int32 Index = 0; Index < RowCosts.Num(); ++Index)
	{
		Cost(Index) = RowCosts[Index];
	}
	return true;
}

bool FMinkEqualityConstraintTask::ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const
{
	if (!UpdateActiveConstraints(Configuration))
	{
		return false;
	}

	const int32 K = ActiveRows.Num();
	Out.resize(K);
	for (int32 Index = 0; Index < K; ++Index)
	{
		Out(Index) = Configuration.Data->efc_pos[ActiveRows[Index]];
	}
	return true;
}

bool FMinkEqualityConstraintTask::ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const
{
	if (!UpdateActiveConstraints(Configuration))
	{
		return false;
	}

	const mjModel* Model = Configuration.Model;
	const mjData* Data = Configuration.Data;
	const int32 Nefc = Data->nefc;
	const int32 Nv = Model->nv;

	FMinkRowMat DenseJ = FMinkRowMat::Zero(Nefc, Nv);
	if (Nefc > 0)
	{
		if (mj_isSparse(Model))
		{
			mju_sparse2dense(
				DenseJ.data(), Data->efc_J, Nefc, Nv, Data->efc_J_rownnz, Data->efc_J_rowadr, Data->efc_J_colind);
		}
		else
		{
			DenseJ = Eigen::Map<const FMinkRowMat>(Data->efc_J, Nefc, Nv);
		}
	}

	const int32 K = ActiveRows.Num();
	Out.resize(K, Nv);
	for (int32 Index = 0; Index < K; ++Index)
	{
		Out.row(Index) = DenseJ.row(ActiveRows[Index]);
	}
	return true;
}
