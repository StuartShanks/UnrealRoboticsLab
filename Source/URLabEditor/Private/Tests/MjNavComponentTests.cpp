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
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Navigation/MjNavComponent.h"
#include "MuJoCo/Input/MjTwistController.h"

namespace
{
struct FNavRig
{
	UMjNavComponent* Nav = nullptr;
	UMjTwistController* Twist = nullptr;
	void Configure(FMjUESession& Sess)
	{
		Twist = NewObject<UMjTwistController>(Sess.Robot, TEXT("TwistCtrl"));
		Twist->RegisterComponent();
		Nav = NewObject<UMjNavComponent>(Sess.Robot, TEXT("NavComp"));
		Nav->RegisterComponent();
	}
};
} // namespace

// URLab.Nav.Component.NoNavmeshGraceful — SetNavGoal with no navmesh in the
// world returns false and stays Idle (no crash).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavCompNoNavmesh,
	"URLab.Nav.Component.NoNavmeshGraceful",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavCompNoNavmesh::RunTest(const FString&)
{
	FNavRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	TestFalse(TEXT("rejects goal"), Rig.Nav->SetNavGoal(FVector(500, 0, 0)));
	TestEqual(TEXT("still idle"), (int)Rig.Nav->GetNavState(), (int)EMjNavState::Idle);
	S.Cleanup();
	return true;
}

// URLab.Nav.Component.FollowsInjectedPath — inject a path ahead of the robot;
// one tick writes a forward twist to the bus and reports Navigating.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavCompFollows,
	"URLab.Nav.Component.FollowsInjectedPath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavCompFollows::RunTest(const FString&)
{
	FNavRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	// Robot spawns at origin; path 5 m straight +X (UE).
	Rig.Nav->SetPathForTesting({FVector::ZeroVector, FVector(500, 0, 0)});
	TestEqual(TEXT("navigating"), (int)Rig.Nav->GetNavState(), (int)EMjNavState::Navigating);
	Rig.Nav->TickComponent(0.016f, LEVELTICK_All, nullptr);
	const FVector T = Rig.Twist->GetTwist();
	TestTrue(TEXT("forward twist on bus"), T.X > 0.1f);
	TestTrue(TEXT("distance reported"), Rig.Nav->GetDistanceToGoal() > 4.0f);
	S.Cleanup();
	return true;
}

// URLab.Nav.Component.ArrivesWhenClose — path whose goal is within
// AcceptanceRadius of the robot: first tick → Arrived, twist zeroed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavCompArrives,
	"URLab.Nav.Component.ArrivesWhenClose",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavCompArrives::RunTest(const FString&)
{
	FNavRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	Rig.Twist->SetTwist(0.3f, 0.f, 0.f);                                 // pre-load a stale twist
	Rig.Nav->SetPathForTesting({FVector::ZeroVector, FVector(5, 0, 0)}); // 5 cm
	Rig.Nav->TickComponent(0.016f, LEVELTICK_All, nullptr);
	TestEqual(TEXT("arrived"), (int)Rig.Nav->GetNavState(), (int)EMjNavState::Arrived);
	const FVector T = Rig.Twist->GetTwist();
	TestEqual(TEXT("twist zeroed"), (float)T.Size(), 0.f, 1e-4f);
	S.Cleanup();
	return true;
}

// URLab.Nav.Component.WatchdogFails — no pump runs in test worlds, so the
// pose never changes: after StuckTimeout of ticks the watchdog must fire
// Failed and zero the twist.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavCompWatchdog,
	"URLab.Nav.Component.WatchdogFails",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavCompWatchdog::RunTest(const FString&)
{
	FNavRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	Rig.Nav->StuckTimeout = 0.5f;
	Rig.Nav->SetPathForTesting({FVector::ZeroVector, FVector(500, 0, 0)});
	for (int i = 0; i < 40; ++i) // 40 × 0.016 s ≈ 0.64 s > StuckTimeout
		Rig.Nav->TickComponent(0.016f, LEVELTICK_All, nullptr);
	TestEqual(TEXT("failed"), (int)Rig.Nav->GetNavState(), (int)EMjNavState::Failed);
	TestEqual(TEXT("twist zeroed"), (float)Rig.Twist->GetTwist().Size(), 0.f, 1e-4f);
	S.Cleanup();
	return true;
}

// URLab.Nav.Component.ClearZeroesTwist — ClearNavGoal mid-drive zeroes the
// bus and returns to Idle.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavCompClear,
	"URLab.Nav.Component.ClearZeroesTwist",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavCompClear::RunTest(const FString&)
{
	FNavRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	Rig.Nav->SetPathForTesting({FVector::ZeroVector, FVector(500, 0, 0)});
	Rig.Nav->TickComponent(0.016f, LEVELTICK_All, nullptr);
	TestTrue(TEXT("moving"), Rig.Twist->GetTwist().X > 0.1f);
	Rig.Nav->ClearNavGoal();
	TestEqual(TEXT("idle"), (int)Rig.Nav->GetNavState(), (int)EMjNavState::Idle);
	TestEqual(TEXT("twist zeroed"), (float)Rig.Twist->GetTwist().Size(), 0.f, 1e-4f);
	S.Cleanup();
	return true;
}

// URLab.Nav.Component.LookaheadPose — the carrot accessor (nav-through-mink
// step 1): invalid until the first navigating tick; tracks the pursuit
// lookahead while navigating; holds the goal after arrival; invalidated by
// ClearNavGoal.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavCompLookahead,
	"URLab.Nav.Component.LookaheadPose",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavCompLookahead::RunTest(const FString&)
{
	FNavRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	FVector Look;
	float LookYaw = 0.f;
	TestFalse(TEXT("no carrot before a goal"), Rig.Nav->GetLookahead(Look, LookYaw));

	// Path 5 m straight +X from the origin-spawned robot: after one tick the
	// carrot sits LookaheadDist ahead on the path, heading 0.
	Rig.Nav->SetPathForTesting({FVector::ZeroVector, FVector(500, 0, 0)});
	TestFalse(TEXT("no carrot until the first tick"), Rig.Nav->GetLookahead(Look, LookYaw));
	Rig.Nav->TickComponent(0.016f, LEVELTICK_All, nullptr);
	if (TestTrue(TEXT("carrot valid while navigating"), Rig.Nav->GetLookahead(Look, LookYaw)))
	{
		TestEqual(TEXT("carrot X = LookaheadDist along path"), (float)Look.X, Rig.Nav->LookaheadDist, 1.f);
		TestEqual(TEXT("carrot Y on path"), (float)Look.Y, 0.f, 1.f);
		TestEqual(TEXT("carrot yaw = path heading"), LookYaw, 0.f, 1e-3f);
	}

	// Goal within AcceptanceRadius: the arrival tick stores the goal pose and
	// Arrived keeps it (a carrot consumer holds station).
	Rig.Nav->SetPathForTesting({FVector::ZeroVector, FVector(5, 0, 0)});
	Rig.Nav->TickComponent(0.016f, LEVELTICK_All, nullptr);
	TestEqual(TEXT("arrived"), (int)Rig.Nav->GetNavState(), (int)EMjNavState::Arrived);
	if (TestTrue(TEXT("carrot survives arrival"), Rig.Nav->GetLookahead(Look, LookYaw)))
	{
		TestEqual(TEXT("carrot = goal after arrival"), (float)Look.X, 5.f, 1e-2f);
	}

	Rig.Nav->ClearNavGoal();
	TestFalse(TEXT("carrot invalidated by ClearNavGoal"), Rig.Nav->GetLookahead(Look, LookYaw));
	S.Cleanup();
	return true;
}
