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
#include "Tasks/MinkTask.h"

class FMinkConfiguration;

/**
 * Port of mink tasks/equality_constraint_task.py EqualityConstraintTask — regulates equality
 * constraints (mjEQ_CONNECT, mjEQ_WELD, mjEQ_JOINT, mjEQ_TENDON) in a model. Useful for modeling
 * "loop joints" such as four-bar linkages. Can regulate all equality constraints in the model or
 * a specific subset identified by name or ID.
 *
 * MuJoCo computes the constraint residual and its Jacobian and stores them in Data->efc_pos and
 * Data->efc_J (potentially in sparse format). ComputeError/ComputeJacobian simply extract the
 * rows corresponding to the active equality constraints specified for this task. Requires
 * mj_makeConstraint to have run (FMinkConfiguration::Update already does this when Model->neq > 0).
 */
class URLABMINK_API FMinkEqualityConstraintTask : public FMinkTask
{
public:
	// Empty EqualityNamesOrIds = regulate ALL equality constraints (mink default).
	FMinkEqualityConstraintTask(const mjModel* Model, const FMinkVec& Cost,
		const TArray<FString>& EqualityNames = {}, const TArray<int32>& EqualityIds = {}, double Gain = 1.0,
		double LmDamping = 0.0);

	/** Cost must be a vector of shape (1,) (broadcast to all resolved equalities) or (neq_total,), all
	 * entries >= 0. Returns false + error log (Python TaskDefinitionError message text) otherwise. On
	 * success, also expands the per-row base Cost via MinkConstraintWidth(eq_type) repeats (python
	 * np.repeat), assuming every resolved row is initially active. */
	bool SetCost(const FMinkVec& Cost); // scalar or per-equality (neq_total)

	/** e(q) = Data->efc_pos rows of the active equality constraints (mirrors compute_error). */
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override;

	/** J(q) = Data->efc_J rows (dense-ified via mju_sparse2dense if the model is sparse) of the active
	 * equality constraints. */
	bool ComputeJacobian(const FMinkConfiguration&, FMinkMat&) const override;

private:
	/** Scans Data->efc_type / Data->efc_id for rows belonging to our resolved EqIds, mirrors Python's
	 * _update_active_constraints. Populates ActiveRows and mutates the base Cost to
	 * CostPerEq[Data->efc_id[row]] per active row (mirrors self.cost = self._cost[active_eq_ids]).
	 * Returns false where upstream mink v1.2.0 would raise IndexError: a row's raw equality id
	 * exceeding the cost table size (non-prefix selections are unsupported). */
	bool UpdateActiveConstraints(const FMinkConfiguration&) const;

	const mjModel* ModelRef;
	TArray<int32> EqIds;              // resolved, validated (active at qpos0)
	FMinkVec CostPerEq;               // per-equality cost
	mutable TArray<int32> ActiveRows; // efc row indices for our eq ids
									  // NOTE: FMinkTask::Cost is mutated per-configuration (mirrors python); mark mutable.
};
