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

#include "MinkQp.h"

#include <limits>

THIRD_PARTY_INCLUDES_START
#include "ThirdParty/qpmad/solver.h"
THIRD_PARTY_INCLUDES_END

bool MinkSolveQp(const FMinkQpProblem& Problem, FMinkVec& OutX)
{
	const int32 N = static_cast<int32>(Problem.H.rows());
	const int32 NumEq = Problem.A ? static_cast<int32>(Problem.A->rows()) : 0;
	const int32 NumIneq = Problem.G ? static_cast<int32>(Problem.G->rows()) : 0;

	// qpmad's general-constraint form takes a single matrix A with two-sided bounds [Lb, Ub].
	// Equalities are stacked as Lb == Ub rows; inequalities (G x <= h) as (-inf, h] rows.
	FMinkMat A(NumEq + NumIneq, N);
	FMinkVec Lb(NumEq + NumIneq);
	FMinkVec Ub(NumEq + NumIneq);
	if (NumEq)
	{
		A.topRows(NumEq) = *Problem.A;
		Lb.head(NumEq) = *Problem.B;
		Ub.head(NumEq) = *Problem.B;
	}
	if (NumIneq)
	{
		A.bottomRows(NumIneq) = *Problem.G;
		Lb.tail(NumIneq).setConstant(-std::numeric_limits<double>::infinity());
		Ub.tail(NumIneq) = *Problem.HIneq;
	}

	FMinkMat HCopy = Problem.H; // qpmad factorizes the Hessian in place.
	qpmad::Solver Solver;
	try
	{
		// qpmad::Solver::solve has no 3-arg (primal, H, h) overload for the fully-unconstrained
		// case (checked against the vendored solver.h template chain), so an empty (0-row) A/Lb/Ub
		// is always passed; this dispatches identically for constrained and unconstrained problems.
		const qpmad::Solver::ReturnStatus Status = Solver.solve(OutX, HCopy, Problem.C, A, Lb, Ub);
		if (Status != qpmad::Solver::OK)
		{
			UE_LOG(LogURLabMink, Warning, TEXT("MinkSolveQp: qpmad returned status %d"), (int32)Status);
			return false;
		}
	}
	catch (const std::exception& E)
	{
		UE_LOG(LogURLabMink, Warning, TEXT("MinkSolveQp: qpmad threw: %hs"), E.what());
		return false;
	}
	return true;
}
