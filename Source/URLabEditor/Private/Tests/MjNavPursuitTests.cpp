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
#include "MuJoCo/Navigation/MjNavPursuit.h"

using namespace MjNavPursuit;

// URLab.Nav.Pursuit.StraightAhead — robot at origin, yaw 0, path straight +X:
// full speed forward, no strafe, no rotation.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavPursuitStraightAhead,
	"URLab.Nav.Pursuit.StraightAhead",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavPursuitStraightAhead::RunTest(const FString&)
{
	FPursuitParams P;
	FPursuitState S;                                                   // origin, yaw 0
	TArray<FVector> Path = {FVector::ZeroVector, FVector(1000, 0, 0)}; // 10 m +X
	FPursuitResult R = ComputeTwist(Path, S, P);
	TestEqual(TEXT("vx = MaxSpeed"), R.Vx, P.MaxSpeed, 1e-3f);
	TestEqual(TEXT("vy = 0"), R.Vy, 0.f, 1e-3f);
	TestEqual(TEXT("yaw_rate = 0"), R.YawRate, 0.f, 1e-3f);
	TestFalse(TEXT("not arrived"), R.bArrived);
	return true;
}

// URLab.Nav.Pursuit.FrameConversion — path along UE −Y (= MuJoCo/bus LEFT).
// Golden values for the single most bug-prone conversion in the stack:
//   robot yaw 0, dir (0,-1,0)_UE → vy = +MaxSpeed (left), and the yaw
//   command toward UE heading −90° (CW error → negative UE yaw rate)
//   must come out POSITIVE on the bus (CCW toward MuJoCo left).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavPursuitFrameConversion,
	"URLab.Nav.Pursuit.FrameConversion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavPursuitFrameConversion::RunTest(const FString&)
{
	FPursuitParams P;
	P.YawGain = 1.0f;
	FPursuitState S; // origin, yaw 0
	TArray<FVector> Path = {FVector::ZeroVector, FVector(0, -1000, 0)};
	FPursuitResult R = ComputeTwist(Path, S, P);
	TestEqual(TEXT("vx = 0"), R.Vx, 0.f, 1e-3f);
	TestEqual(TEXT("vy = +MaxSpeed (left)"), R.Vy, P.MaxSpeed, 1e-3f);
	TestTrue(TEXT("bus yaw rate positive (CCW)"), R.YawRate > 0.1f);
	TestTrue(TEXT("yaw rate clamped"), R.YawRate <= P.MaxYawRate + 1e-3f);
	return true;
}

// URLab.Nav.Pursuit.DecelAndArrive — half DecelRadius → half speed;
// inside AcceptanceRadius → zero twist + arrived.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavPursuitDecelAndArrive,
	"URLab.Nav.Pursuit.DecelAndArrive",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavPursuitDecelAndArrive::RunTest(const FString&)
{
	FPursuitParams P;
	FPursuitState S;
	// Goal at half the decel radius, straight ahead:
	TArray<FVector> Path = {FVector::ZeroVector, FVector(P.DecelRadius * 0.5f, 0, 0)};
	FPursuitResult R = ComputeTwist(Path, S, P);
	TestEqual(TEXT("vx = MaxSpeed/2"), R.Vx, P.MaxSpeed * 0.5f, 5e-2f);

	// Goal inside acceptance radius:
	TArray<FVector> Path2 = {FVector::ZeroVector, FVector(P.AcceptanceRadius * 0.5f, 0, 0)};
	FPursuitResult R2 = ComputeTwist(Path2, S, P);
	TestTrue(TEXT("arrived"), R2.bArrived);
	TestEqual(TEXT("vx zero"), R2.Vx, 0.f, 1e-4f);
	TestEqual(TEXT("vy zero"), R2.Vy, 0.f, 1e-4f);
	TestEqual(TEXT("yaw zero"), R2.YawRate, 0.f, 1e-4f);
	return true;
}

// URLab.Nav.Pursuit.LookaheadCutsCorner — L-shaped path; from the elbow
// region the commanded direction blends toward the second leg (the
// lookahead point is on the second segment, not the elbow).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavPursuitLookahead,
	"URLab.Nav.Pursuit.LookaheadCutsCorner",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavPursuitLookahead::RunTest(const FString&)
{
	FPursuitParams P; // LookaheadDist 60
	// Robot 20 cm before the elbow at (100,0); path turns +Y... (UE) after it.
	FPursuitState S;
	S.Position = FVector(80, 0, 0);
	TArray<FVector> Path = {FVector::ZeroVector, FVector(100, 0, 0), FVector(100, 500, 0)};
	FPursuitResult R = ComputeTwist(Path, S, P);
	// Lookahead lands 40 cm up the second leg → point (100, 40, 0).
	TestEqual(TEXT("lookahead X"), R.LookaheadPoint.X, 100.0, 1.0);
	TestEqual(TEXT("lookahead Y"), R.LookaheadPoint.Y, 40.0, 1.0);
	// Direction has a −vy (UE +Y = robot RIGHT = bus −vy) component.
	TestTrue(TEXT("strafing right on the bus"), R.Vy < -0.05f);
	TestTrue(TEXT("still moving forward"), R.Vx > 0.05f);
	return true;
}

// URLab.Nav.Pursuit.ShortestAngle — wrap-around cases.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavPursuitShortestAngle,
	"URLab.Nav.Pursuit.ShortestAngle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavPursuitShortestAngle::RunTest(const FString&)
{
	TestEqual(TEXT("0→PI/2"), ShortestAngleRad(0.f, HALF_PI), HALF_PI, 1e-4f);
	TestEqual(TEXT("PI-0.1 → -PI+0.1 wraps forward"),
		ShortestAngleRad(PI - 0.1f, -PI + 0.1f), 0.2f, 1e-4f);
	TestEqual(TEXT("-PI+0.1 → PI-0.1 wraps backward"),
		ShortestAngleRad(-PI + 0.1f, PI - 0.1f), -0.2f, 1e-4f);
	return true;
}
