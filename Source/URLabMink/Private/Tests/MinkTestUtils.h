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
#include "Dom/JsonObject.h"
#include "Misc/AutomationTest.h"
#include "MinkTypes.h"

/** Absolute path to <plugin>/Scripts/mink_golden. */
URLABMINK_API FString MinkGoldenDir();

/** Loads Scripts/mink_golden/fixtures/<LayerName>.json into Out. Returns false on I/O or parse failure. */
URLABMINK_API bool MinkLoadFixture(const FString& LayerName, TSharedPtr<FJsonObject>& Out);

/** Loads an MJCF model from Scripts/mink_golden/models/<ModelName> via mj_loadXML. Returns nullptr on failure. */
URLABMINK_API mjModel* MinkLoadModel(const FString& ModelName);

/** Converts a flat JSON number array into an Eigen dynamic vector. */
URLABMINK_API FMinkVec MinkJsonVec(const TArray<TSharedPtr<FJsonValue>>& A);

/** Converts a JSON array-of-row-arrays into an Eigen dynamic matrix. */
URLABMINK_API FMinkMat MinkJsonMat(const TArray<TSharedPtr<FJsonValue>>& A);

/**
 * Flattened max-abs-diff compare with the rule diff <= Tol * max(1.0, maxAbs(Expected)).
 * On failure, adds a test error naming What, the max diff, and the first offending flat index.
 */
URLABMINK_API bool MinkExpectNear(FAutomationTestBase& Test, const TCHAR* What, const FMinkMat& Actual,
	const FMinkMat& Expected, double Tol);
