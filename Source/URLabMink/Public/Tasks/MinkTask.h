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

class FMinkConfiguration;

/**
 * Port of mink tasks/task.py Objective — quadratic objective 1/2 dq^T H dq + c^T dq.
 * NOTE: Value() mirrors Python's Objective.value exactly (x^T H x + c.x, NOT halved).
 */
struct URLABMINK_API FMinkObjective
{
	FMinkMat H;
	FMinkVec C;

	double Value(const FMinkVec& X) const;
};

/**
 * Port of the tuple returned by tasks/task.py BaseTask.compute_qp_residual —
 * the weighted least-squares residual (weighted_jacobian, weighted_error, mu).
 */
struct URLABMINK_API FMinkResidual
{
	FMinkMat WeightedJacobian; // cost[:, None] * J
	FMinkVec WeightedError;    // cost * (-gain * error)
	double Mu = 0.0;           // lm_damping * ||weighted_error||^2
};

/** Mirrors the tri-state return of tasks/task.py BaseTask.compute_qp_residual (tuple | None). */
enum class EMinkTaskStatus : uint8
{
	Ok,
	NoResidualForm,
	Error
};

/** Port of mink tasks/task.py BaseTask. */
class URLABMINK_API FMinkBaseTask
{
public:
	virtual ~FMinkBaseTask() = default;

	virtual bool ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const = 0;

	/** Dense fallback: mirrors BaseTask.compute_qp_residual returning None. */
	virtual EMinkTaskStatus ComputeQpResidual(const FMinkConfiguration& Configuration, FMinkResidual& Out) const
	{
		return EMinkTaskStatus::NoResidualForm;
	}
};

/** Port of mink tasks/task.py Task. */
class URLABMINK_API FMinkTask : public FMinkBaseTask
{
public:
	/** Validates gain in [0, 1] and lm_damping >= 0; on failure logs the exact Python exception
	 * message and sets bIsValid = false instead of raising (see global constraints deviation #1). */
	FMinkTask(const FMinkVec& InCost, double InGain, double InLmDamping);

	bool bIsValid = true;
	// mutable: FMinkEqualityConstraintTask mutates this per-configuration inside its const
	// ComputeError/ComputeJacobian (mirrors python EqualityConstraintTask._update_active_constraints
	// setting self.cost from what is conceptually a read-only compute call).
	mutable FMinkVec Cost;
	double Gain = 1.0;
	double LmDamping = 0.0;

	virtual bool ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const = 0;
	virtual bool ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const = 0;

	bool ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const override;
	EMinkTaskStatus ComputeQpResidual(const FMinkConfiguration& Configuration, FMinkResidual& Out) const override;

protected:
	/** Port of Task._weighted_residual. */
	void WeightedResidual(const FMinkVec& Error, const FMinkMat& Jacobian, FMinkResidual& Out) const;

	/** Port of Task._assemble_qp. */
	void AssembleQp(const FMinkVec& Error, const FMinkMat& Jacobian, const FMinkMat& EyeNv, FMinkObjective& Out) const;
};
