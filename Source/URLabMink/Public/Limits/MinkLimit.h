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
 * Port of mink limits/limit.py Constraint — a linear inequality constraint of the form
 * G(q) * Δq <= h(q). Inactive iff both G and h are unset (mirrors Python's G is None and
 * h is None).
 */
struct URLABMINK_API FMinkInequality
{
	TOptional<FMinkMat> G;
	TOptional<FMinkVec> H;

	bool IsInactive() const { return !G.IsSet() && !H.IsSet(); }
};

/**
 * Port of mink limits/limit.py Limit — abstract base class for kinematic limits. Subclasses
 * compute a linearized QP inequality (G, h) given the current robot configuration and the
 * integration timestep.
 */
class URLABMINK_API FMinkLimit
{
public:
	virtual ~FMinkLimit() = default;

	/** Port of Limit.compute_qp_inequalities. */
	virtual bool ComputeQpInequalities(const FMinkConfiguration& Configuration, double Dt, FMinkInequality& Out) const = 0;
};
