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

/**
 * Mirrors qpsolvers.Problem(H, c, G, h, A, b):
 *   min 1/2 xT H x + cT x   s.t.  G x <= h,  A x = b
 */
struct URLABMINK_API FMinkQpProblem
{
	FMinkMat H;
	FMinkVec C;
	TOptional<FMinkMat> G;
	TOptional<FMinkVec> HIneq;
	TOptional<FMinkMat> A;
	TOptional<FMinkVec> B;
};

/**
 * Solves Problem via the vendored qpmad (Goldfarb-Idnani active-set) backend.
 * Returns false (+ Warning log) when the solver reports no solution (including
 * infeasible or ill-formed problems, which qpmad reports by throwing).
 */
URLABMINK_API bool MinkSolveQp(const FMinkQpProblem& Problem, FMinkVec& OutX);
