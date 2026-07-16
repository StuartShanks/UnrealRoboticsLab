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
#include "MuJoCo/Navigation/MjBaseIntegrate.h"

using namespace MjBaseIntegrate;

// URLab.Nav.BaseIntegrate.TwistToWorld — golden values for the robot->world
// rotation shared by base-drive and the mink TwistFollow task.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseIntegrateTwistToWorld,
	"URLab.Nav.BaseIntegrate.TwistToWorld",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseIntegrateTwistToWorld::RunTest(const FString&)
{
	double V[3];
	// Theta 0: passthrough.
	TwistToWorld(0.6, 0.0, 0.3, 0.0, V);
	TestEqual(TEXT("t0 vx"), V[0], 0.6, 1e-6);
	TestEqual(TEXT("t0 vy"), V[1], 0.0, 1e-6);
	TestEqual(TEXT("t0 w"), V[2], 0.3, 1e-6);
	// Theta PI/2 (facing +Y): forward becomes world +Y.
	TwistToWorld(0.6, 0.0, 0.0, HALF_PI, V);
	TestEqual(TEXT("t90 vx"), V[0], 0.0, 1e-6);
	TestEqual(TEXT("t90 vy"), V[1], 0.6, 1e-6);
	// Theta PI/2 with left twist: left (robot +y) becomes world -X.
	TwistToWorld(0.0, 0.5, 0.0, HALF_PI, V);
	TestEqual(TEXT("t90 left vx"), V[0], -0.5, 1e-6);
	TestEqual(TEXT("t90 left vy"), V[1], 0.0, 1e-6);
	return true;
}

// URLab.Nav.BaseIntegrate.Leash — integrate, leash clamp, reseed, range clamp:
// exact PositionIntegrate semantics lifted from UMjBaseDriveController.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseIntegrateLeash,
	"URLab.Nav.BaseIntegrate.Leash",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseIntegrateLeash::RunTest(const FString&)
{
	FLeashParams P; // 0.05 linear / 0.1 angular / 0.25 reseed

	// Plain integration: small step, no clamps engage.
	TestEqual(TEXT("integrates V*Dt"),
		IntegrateLeashed(1.0, 0.5, 0.002, 1.0, /*bAngular*/ false, P), 1.001, 1e-6);

	// Leash: target may lead measured by at most MaxLeashLinear.
	TestEqual(TEXT("linear leash clamps"),
		IntegrateLeashed(1.04, 10.0, 0.002, 1.0, false, P), 1.05, 1e-6);
	TestEqual(TEXT("angular leash clamps wider"),
		IntegrateLeashed(0.09, 10.0, 0.002, 0.0, /*bAngular*/ true, P), 0.1, 1e-6);

	// Reseed: a discontinuity (measured jumped) snaps the target back.
	TestEqual(TEXT("reseeds on discontinuity"),
		IntegrateLeashed(2.0, 0.0, 0.002, 1.0, false, P), 1.0, 1e-6);

	// Joint-range clamp caps after the leash: 1.0 + 30*0.002 = 1.06 -> leash
	// clamps to measured+0.05 = 1.05 -> range Hi caps at 1.02.
	TestEqual(TEXT("range clamp"),
		IntegrateLeashed(1.0, 30.0, 0.002, 1.0, false, P, /*bRangeLimited*/ true,
			/*Lo*/ -1.0, /*Hi*/ 1.02),
		1.02, 1e-6);
	return true;
}
