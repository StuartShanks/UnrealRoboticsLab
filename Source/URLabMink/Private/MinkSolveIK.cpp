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

#include "MinkSolveIK.h"

#include "MinkConfiguration.h"
#include "Tasks/MinkTask.h"
#include "Limits/MinkLimit.h"
#include "Limits/MinkConfigurationLimit.h"

namespace
{
/** Finer-grained internal build status; MinkBuildIK adapts this to a bare bool, while
 * MinkSolveIK uses it directly to pick between EMinkIKStatus::TaskError/LimitError. */
enum class EMinkBuildStatus : uint8
{
	Ok,
	TaskError,
	LimitError
};

/** Port of solve_ik.py _compute_qp_objective. */
EMinkBuildStatus ComputeQpObjective(const FMinkConfiguration& Configuration,
	const TArray<const FMinkBaseTask*>& Tasks, double Damping, FMinkMat& OutH, FMinkVec& OutC)
{
	const int32 Nv = Configuration.Nv();

	TArray<FMinkMat> WeightedJacobians;
	TArray<FMinkVec> WeightedErrors;
	double MuTotal = 0.0;
	TOptional<FMinkMat> HDense;
	TOptional<FMinkVec> CDense;

	for (const FMinkBaseTask* Task : Tasks)
	{
		FMinkResidual Residual;
		const EMinkTaskStatus Status = Task->ComputeQpResidual(Configuration, Residual);
		if (Status == EMinkTaskStatus::Ok)
		{
			WeightedJacobians.Add(Residual.WeightedJacobian);
			WeightedErrors.Add(Residual.WeightedError);
			MuTotal += Residual.Mu;
		}
		else if (Status == EMinkTaskStatus::NoResidualForm)
		{
			FMinkObjective Objective;
			if (!Task->ComputeQpObjective(Configuration, Objective))
			{
				return EMinkBuildStatus::TaskError;
			}
			if (HDense.IsSet())
			{
				*HDense += Objective.H;
				*CDense += Objective.C;
			}
			else
			{
				HDense = Objective.H;
				CDense = Objective.C;
			}
		}
		else // EMinkTaskStatus::Error
		{
			return EMinkBuildStatus::TaskError;
		}
	}

	if (WeightedJacobians.Num() > 0)
	{
		int32 TotalRows = 0;
		for (const FMinkMat& W : WeightedJacobians)
		{
			TotalRows += static_cast<int32>(W.rows());
		}
		FMinkMat W(TotalRows, Nv);
		FMinkVec Errs(TotalRows);
		int32 Row = 0;
		for (int32 i = 0; i < WeightedJacobians.Num(); ++i)
		{
			const int32 Rows = static_cast<int32>(WeightedJacobians[i].rows());
			W.middleRows(Row, Rows) = WeightedJacobians[i];
			Errs.segment(Row, Rows) = WeightedErrors[i];
			Row += Rows;
		}
		OutH = W.transpose() * W;
		OutC = -(W.transpose() * Errs);
	}
	else
	{
		OutH = FMinkMat::Zero(Nv, Nv);
		OutC = FMinkVec::Zero(Nv);
	}

	// Global LM damping plus the summed per-task LM terms on the diagonal — added BEFORE the
	// dense fallback H/c, exactly mirroring solve_ik.py's order of operations.
	OutH.diagonal().array() += Damping + MuTotal;

	if (HDense.IsSet())
	{
		OutH += *HDense;
		OutC += *CDense;
	}

	return EMinkBuildStatus::Ok;
}

/** Port of solve_ik.py _compute_qp_inequalities. Returns false iff a limit itself reports an
 * internal failure (distinct from IsInactive(), which just means "skip this limit"). */
bool ComputeQpInequalities(const FMinkConfiguration& Configuration, double Dt,
	const TArray<const FMinkLimit*>& Limits, TOptional<FMinkMat>& OutG, TOptional<FMinkVec>& OutH)
{
	TArray<FMinkMat> GList;
	TArray<FMinkVec> HList;
	for (const FMinkLimit* Limit : Limits)
	{
		FMinkInequality Inequality;
		if (!Limit->ComputeQpInequalities(Configuration, Dt, Inequality))
		{
			return false;
		}
		if (Inequality.IsInactive())
		{
			continue;
		}
		GList.Add(*Inequality.G);
		HList.Add(*Inequality.H);
	}

	if (GList.Num() == 0)
	{
		OutG.Reset();
		OutH.Reset();
		return true;
	}

	const int32 Nv = static_cast<int32>(GList[0].cols());
	int32 TotalRows = 0;
	for (const FMinkMat& G : GList)
	{
		TotalRows += static_cast<int32>(G.rows());
	}
	FMinkMat G(TotalRows, Nv);
	FMinkVec H(TotalRows);
	int32 Row = 0;
	for (int32 i = 0; i < GList.Num(); ++i)
	{
		const int32 Rows = static_cast<int32>(GList[i].rows());
		G.middleRows(Row, Rows) = GList[i];
		H.segment(Row, Rows) = HList[i];
		Row += Rows;
	}
	OutG = MoveTemp(G);
	OutH = MoveTemp(H);
	return true;
}

/** Port of solve_ik.py _compute_qp_equalities. Returns false iff a constraint task reports an
 * error while computing its Jacobian or error (mirrors the Jacobian-then-error call order). */
bool ComputeQpEqualities(const FMinkConfiguration& Configuration, const TArray<const FMinkTask*>* Constraints,
	TOptional<FMinkMat>& OutA, TOptional<FMinkVec>& OutB)
{
	if (Constraints == nullptr || Constraints->Num() == 0)
	{
		OutA.Reset();
		OutB.Reset();
		return true;
	}

	TArray<FMinkMat> AList;
	TArray<FMinkVec> BList;
	for (const FMinkTask* Task : *Constraints)
	{
		FMinkMat Jacobian;
		FMinkVec Error;
		if (!Task->ComputeJacobian(Configuration, Jacobian) || !Task->ComputeError(Configuration, Error))
		{
			return false;
		}
		AList.Add(Jacobian);
		BList.Add(-Task->Gain * Error);
	}

	const int32 Nv = static_cast<int32>(AList[0].cols());
	int32 TotalRows = 0;
	for (const FMinkMat& A : AList)
	{
		TotalRows += static_cast<int32>(A.rows());
	}
	FMinkMat A(TotalRows, Nv);
	FMinkVec B(TotalRows);
	int32 Row = 0;
	for (int32 i = 0; i < AList.Num(); ++i)
	{
		const int32 Rows = static_cast<int32>(AList[i].rows());
		A.middleRows(Row, Rows) = AList[i];
		B.segment(Row, Rows) = BList[i];
		Row += Rows;
	}
	OutA = MoveTemp(A);
	OutB = MoveTemp(B);
	return true;
}

/** Port of solve_ik.py build_ik, returning the fine-grained status. */
EMinkBuildStatus MinkBuildIKInternal(const FMinkConfiguration& Configuration,
	const TArray<const FMinkBaseTask*>& Tasks, double Dt, FMinkQpProblem& OutProblem, double Damping,
	const TArray<const FMinkLimit*>* Limits, const TArray<const FMinkTask*>* Constraints)
{
	const EMinkBuildStatus ObjectiveStatus =
		ComputeQpObjective(Configuration, Tasks, Damping, OutProblem.H, OutProblem.C);
	if (ObjectiveStatus != EMinkBuildStatus::Ok)
	{
		return ObjectiveStatus;
	}

	// limits == nullptr defaults to a single ConfigurationLimit(model), mirroring Python.
	TOptional<FMinkConfigurationLimit> DefaultLimit;
	TArray<const FMinkLimit*> DefaultLimitsArray;
	const TArray<const FMinkLimit*>* EffectiveLimits = Limits;
	if (Limits == nullptr)
	{
		DefaultLimit.Emplace(Configuration.Model);
		DefaultLimitsArray.Add(&DefaultLimit.GetValue());
		EffectiveLimits = &DefaultLimitsArray;
	}

	if (!ComputeQpInequalities(Configuration, Dt, *EffectiveLimits, OutProblem.G, OutProblem.HIneq))
	{
		return EMinkBuildStatus::LimitError;
	}

	if (!ComputeQpEqualities(Configuration, Constraints, OutProblem.A, OutProblem.B))
	{
		return EMinkBuildStatus::TaskError;
	}

	return EMinkBuildStatus::Ok;
}
} // namespace

bool MinkBuildIK(const FMinkConfiguration& Configuration, const TArray<const FMinkBaseTask*>& Tasks, double Dt,
	FMinkQpProblem& OutProblem, double Damping, const TArray<const FMinkLimit*>* Limits,
	const TArray<const FMinkTask*>* Constraints)
{
	return MinkBuildIKInternal(Configuration, Tasks, Dt, OutProblem, Damping, Limits, Constraints)
		== EMinkBuildStatus::Ok;
}

FMinkIKResult MinkSolveIK(const FMinkConfiguration& Configuration, const TArray<const FMinkBaseTask*>& Tasks,
	double Dt, double Damping, bool bSafetyBreak, const TArray<const FMinkLimit*>* Limits,
	const TArray<const FMinkTask*>* Constraints)
{
	FMinkIKResult Result;

	// Mirrors solve_ik.py: check_limits runs BEFORE the QP is built at all.
	if (!Configuration.CheckLimits(1e-6, bSafetyBreak))
	{
		Result.Status = EMinkIKStatus::NotWithinConfigurationLimits;
		return Result;
	}

	FMinkQpProblem Problem;
	const EMinkBuildStatus BuildStatus =
		MinkBuildIKInternal(Configuration, Tasks, Dt, Problem, Damping, Limits, Constraints);
	if (BuildStatus == EMinkBuildStatus::TaskError)
	{
		Result.Status = EMinkIKStatus::TaskError;
		return Result;
	}
	if (BuildStatus == EMinkBuildStatus::LimitError)
	{
		Result.Status = EMinkIKStatus::LimitError;
		return Result;
	}

	FMinkVec X;
	if (!MinkSolveQp(Problem, X))
	{
		UE_LOG(LogURLabMink, Warning, TEXT("QP solver qpmad failed to find a solution."));
		Result.Status = EMinkIKStatus::NoSolutionFound;
		return Result;
	}

	Result.Status = EMinkIKStatus::Success;
	Result.Velocity = X / Dt;
	return Result;
}
