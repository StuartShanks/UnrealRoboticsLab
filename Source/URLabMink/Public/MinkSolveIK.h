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

#pragma once

#include "CoreMinimal.h"
#include "MinkTypes.h"
#include "MinkQp.h"

class FMinkConfiguration;
class FMinkBaseTask;
class FMinkLimit;
class FMinkTask;

/** Port of the mink solve_ik.py failure taxonomy (exceptions turned into a status enum, see
 * global constraints deviation #1). */
enum class EMinkIKStatus : uint8
{
	Success,
	NoSolutionFound,              // QP solver (qpmad) failed to find a solution.
	NotWithinConfigurationLimits, // configuration.check_limits raised with safety_break=True.
	TaskError,                    // a task or constraint reported EMinkTaskStatus::Error / false.
	LimitError                    // a limit's ComputeQpInequalities itself reported failure.
};

/** Port of the np.ndarray returned by solve_ik.py solve_ik, wrapped with a status. */
struct URLABMINK_API FMinkIKResult
{
	EMinkIKStatus Status = EMinkIKStatus::TaskError;
	FMinkVec Velocity; // tangent-space v (dq/dt), valid iff Success
	bool IsSuccess() const { return Status == EMinkIKStatus::Success; }
};

/**
 * Port of mink solve_ik.py build_ik — assembles (but does not solve) the QP for the current
 * configuration, tasks, limits and equality constraints.
 *
 * Limits semantics mirror Python's `limits: Sequence[Limit] | None`:
 *   Limits == nullptr            -> defaults to a single FMinkConfigurationLimit(Configuration.Model)
 *   Limits == pointer to []      -> no limits at all (inequality constraints disabled)
 *   Limits == pointer to [...]   -> exactly those limits, in order
 *
 * Constraints mirror Python's `constraints: Sequence[Task] | None`:
 *   Constraints == nullptr or pointer to [] -> no equality constraints
 *   Constraints == pointer to [...]         -> those tasks enforced as A dq = b equalities
 *
 * Returns false iff a task's residual/objective computation reported an error (mirrors the only
 * way build_ik's Python callees can themselves fail; a limit's own internal failure is reported
 * dropping through the same bool for MinkBuildIK, but MinkSolveIK surfaces it as its own
 * EMinkIKStatus::LimitError so that failure mode is distinguishable from EMinkIKStatus::TaskError).
 */
URLABMINK_API bool MinkBuildIK(const FMinkConfiguration& Configuration,
	const TArray<const FMinkBaseTask*>& Tasks, double Dt,
	FMinkQpProblem& OutProblem, double Damping = 1e-12,
	const TArray<const FMinkLimit*>* Limits = nullptr,
	const TArray<const FMinkTask*>* Constraints = nullptr);

/**
 * Port of mink solve_ik.py solve_ik — computes a velocity tangent to the current configuration
 * that satisfies, at (weighted) best, the given set of kinematic tasks.
 *
 * Order of operations mirrors Python exactly: Configuration.CheckLimits(1e-6, bSafetyBreak) is
 * checked BEFORE the QP is built at all; a violation (only when bSafetyBreak is true) returns
 * EMinkIKStatus::NotWithinConfigurationLimits with a default-constructed Velocity, without
 * touching Tasks/Limits/Constraints. See MinkBuildIK for the Limits/Constraints nullptr
 * semantics, which are forwarded unchanged.
 */
URLABMINK_API FMinkIKResult MinkSolveIK(const FMinkConfiguration& Configuration,
	const TArray<const FMinkBaseTask*>& Tasks, double Dt,
	double Damping = 1e-12, bool bSafetyBreak = false,
	const TArray<const FMinkLimit*>* Limits = nullptr,
	const TArray<const FMinkTask*>* Constraints = nullptr);
