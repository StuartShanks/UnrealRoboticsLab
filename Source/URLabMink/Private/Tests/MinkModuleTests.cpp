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

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Modules/ModuleManager.h"
#include "MinkTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkModuleLoadsTest,
	"URLab.Mink.Module.Loads",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkModuleLoadsTest::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("URLabMink module is loaded"),
		FModuleManager::Get().IsModuleLoaded(TEXT("URLabMink")));

	// MuJoCo links and answers.
	TestTrue(TEXT("mj_versionString non-null"), mj_versionString() != nullptr);

	// Constants parity spot-checks (mink constants.py).
	TestEqual(TEXT("dof_width(free)"), MinkDofWidth(mjJNT_FREE), 6);
	TestEqual(TEXT("qpos_width(ball)"), MinkQposWidth(mjJNT_BALL), 4);
	TestEqual(TEXT("constraint_width(weld)"), MinkConstraintWidth(mjEQ_WELD), 6);

	// Skew is antisymmetric with the right entries.
	const FMinkMat3 S = MinkSkew(FMinkVec3(1.0, 2.0, 3.0));
	TestEqual(TEXT("skew(0,1)"), S(0, 1), -3.0);
	TestEqual(TEXT("skew(2,0)"), S(2, 0), -2.0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
