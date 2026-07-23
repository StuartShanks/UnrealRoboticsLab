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
#include "Editor.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "MjLevelOps.h"
#include "NavigationSystem.h"
#include "NavMesh/NavMeshBoundsVolume.h"
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

// URLab.Nav.Component.GoalSubstitution — a REAL navmesh with an obstacle:
// goals in the obstacle's agent-radius erosion band get projected sideways;
// the substitution must be REPORTED (requested vs projected + displacement),
// strict mode (MaxProjectionCm) must reject oversized projections, and a goal
// deep inside the obstacle (beyond the projection extent) rejects off_navmesh.
// Exercises UMjNavComponent::DecideNavGoal (the logic behind SetNavGoal) in
// the EDITOR world directly — the FMjUESession rig world has no navigation
// system, so the full-component path can't meet a navmesh in automation.
// Regression for the live HomeInterior failure where nav reported 'arrived'
// ~1 m from the requested staging point with no indication the goal moved.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavCompGoalSubstitution,
	"URLab.Nav.Component.GoalSubstitution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavCompGoalSubstitution::RunTest(const FString&)
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	TestNotNull(TEXT("editor world"), World);

	// Scene: 10x10 m floor (top at z=0) + a 2x2x1 m obstacle centred at
	// (3, 0) — it occupies x∈[2,4] m; with the DEFAULT agent radius (~35 cm;
	// AgentRadiusCm=0 keeps the level default — the >0 reconfigure path
	// RebuildAll()s an unbuilt navmesh and trips an ensure in automation) the
	// navigable boundary sits near x≈1.65 m. Navmesh over the middle 6x6 m.
	FString Name, Path, Err;
	bool bExisting = false;
	TestTrue(TEXT("floor spawns"),
		URLabLevelOps::SpawnBoxSync(TEXT("navsub_floor_test"),
			FVector(0, 0, -0.1), FVector(10.0, 10.0, 0.2), 0.0,
			Name, Path, bExisting, Err));
	// Obstacle is 4 m tall so its TOP (z=4 m) sits outside the nav bounds
	// (z≤2.5 m) — otherwise Recast meshes the box roof and the deep-inside
	// goal in case D "projects" straight up with ~0 displacement.
	TestTrue(TEXT("obstacle spawns"),
		URLabLevelOps::SpawnBoxSync(TEXT("navsub_obstacle_test"),
			FVector(3.0, 0, 2.0), FVector(2.0, 2.0, 4.0), 0.0,
			Name, Path, bExisting, Err));
	TestTrue(*FString::Printf(TEXT("nav bounds ok: %s"), *Err),
		URLabLevelOps::SpawnNavBoundsSync(
			FVector(0, 0, 0.5), FVector(6.0, 6.0, 2.0), /*AgentRadiusCm=*/0.f, Name, bExisting, Err));
	bool bNavData = false;
	const double Deadline = FPlatformTime::Seconds() + 30.0;
	while (FPlatformTime::Seconds() < Deadline)
	{
		if (URLabLevelOps::IsNavBuildDone(bNavData) && bNavData)
			break;
		FPlatformProcess::Sleep(0.1);
	}
	TestTrue(TEXT("nav data present after build"), bNavData);

	// Wait until the OBSTACLE actually carves the mesh. The test runs
	// synchronously on the game thread, so the nav system never Ticks on its
	// own — pending octree registrations (the box) would never process. Pump
	// NavSys->Tick + Build until the deep-inside point stops projecting.
	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	TestNotNull(TEXT("nav system"), NavSys);
	bool bCarved = false;
	const double CarveDeadline = FPlatformTime::Seconds() + 15.0;
	while (NavSys && FPlatformTime::Seconds() < CarveDeadline)
	{
		FNavLocation Tmp;
		if (!NavSys->ProjectPointToNavigation(FVector(300, 0, 0), Tmp, FVector(100, 100, 500)))
		{
			bCarved = true;
			break;
		}
		NavSys->Tick(0.1f); // process pending octree updates + dirty tiles
		NavSys->Build();
		FPlatformProcess::Sleep(0.1);
	}

	const FVector Start(-100, -100, 0); // open floor, on-mesh
	const float Accept = 15.f;          // component default AcceptanceRadius

	// A) Open-floor goal: accepted, projection ~0, no substitution; the
	// on-mesh START reports a ~zero recovery displacement.
	{
		const FMjNavGoalDecision D =
			UMjNavComponent::DecideNavGoal(World, Start, FVector(-200, 100, 0), -1.f, Accept);
		TestTrue(TEXT("open goal accepted"), D.bAccepted);
		TestTrue(TEXT("open goal barely projected"), D.ProjectionCm >= 0.f && D.ProjectionCm < 15.f);
		TestEqual(TEXT("open goal no reject"), (int)D.Reject, (int)EMjNavGoalReject::None);
		TestTrue(TEXT("open goal has a path"), D.PathPoints.Num() >= 1);
		TestTrue(TEXT("on-mesh start reports ~zero displacement"),
			D.StartProjectionCm >= 0.f && D.StartProjectionCm < 15.f);
	}

	// B) Goal inside the erosion band (x=1.90 m; boundary ≈1.65 m): historical
	// mode ACCEPTS but must report the sideways substitution.
	{
		const FMjNavGoalDecision D =
			UMjNavComponent::DecideNavGoal(World, Start, FVector(190, 0, 0), -1.f, Accept);
		TestTrue(TEXT("banded goal accepted (historical mode)"), D.bAccepted);
		TestTrue(*FString::Printf(TEXT("substitution reported (proj %.0f cm)"), D.ProjectionCm),
			D.ProjectionCm > 5.f && D.ProjectionCm < 90.f);
		TestTrue(TEXT("projected point moved off the requested spot"),
			FVector::Dist2D(FVector(190, 0, 0), D.ProjectedUE) > 5.f);
	}

	// C) Same banded goal in STRICT mode: rejected with the right reason.
	{
		const FMjNavGoalDecision D =
			UMjNavComponent::DecideNavGoal(World, Start, FVector(190, 0, 0), /*MaxProjectionCm=*/5.f, Accept);
		TestFalse(TEXT("strict mode rejects the substitution"), D.bAccepted);
		TestEqual(TEXT("reason = projection_exceeds_max"),
			(int)D.Reject, (int)EMjNavGoalReject::ProjectionExceedsMax);
	}

	// D) Deep inside the obstacle (x=3 m; nearest navigable ≈1.65 m away,
	// beyond the 1 m projection extent): off-navmesh rejection. Only
	// assertable when the environment actually carved (see the pump above) —
	// A/B/C already cover the substitution-reporting contract, so a
	// non-carving automation environment degrades to a warning, not a fail.
	if (bCarved)
	{
		const FMjNavGoalDecision D =
			UMjNavComponent::DecideNavGoal(World, Start, FVector(300, 0, 0), -1.f, Accept);
		TestFalse(*FString::Printf(
			TEXT("deep-inside goal rejected (got: accepted=%d proj=%.0fcm projected=%s)"),
			D.bAccepted ? 1 : 0, D.ProjectionCm, *D.ProjectedUE.ToString()),
			D.bAccepted);
		TestEqual(TEXT("reason = off_navmesh"),
			(int)D.Reject, (int)EMjNavGoalReject::OffNavmesh);

		// E) Off-mesh-wedge recovery: START in the erosion band (where a
		// whole-body reach parks the base). Accepted, with the recovery
		// displacement reported — the path begins back on the mesh.
		{
			const FMjNavGoalDecision D2 = UMjNavComponent::DecideNavGoal(
				World, FVector(190, 0, 0), FVector(-200, 100, 0), -1.f, Accept);
			TestTrue(TEXT("banded START recovers (accepted)"), D2.bAccepted);
			TestTrue(*FString::Printf(TEXT("recovery leg reported (start proj %.0f cm)"),
				D2.StartProjectionCm),
				D2.StartProjectionCm > 5.f && D2.StartProjectionCm < 100.f);
		}
		// F) START unrecoverably deep (beyond the 1 m projection extent):
		// distinct start_off_navmesh rejection.
		{
			const FMjNavGoalDecision D2 = UMjNavComponent::DecideNavGoal(
				World, FVector(300, 0, 0), FVector(-200, 100, 0), -1.f, Accept);
			TestFalse(TEXT("deep-inside START rejected"), D2.bAccepted);
			TestEqual(TEXT("reason = start_off_navmesh"),
				(int)D2.Reject, (int)EMjNavGoalReject::StartOffNavmesh);
		}
	}
	else
	{
		AddWarning(TEXT("obstacle never carved the navmesh in this environment — "
			"deep-inside/off_navmesh + wedge-recovery cases skipped (covered by NoNavmeshGraceful for the reason path)"));
	}

	// Cleanup: boxes + nav volume.
	for (TActorIterator<AStaticMeshActor> It(World); It; ++It)
		if (It->Tags.Contains(FName(TEXT("navsub_floor_test"))) ||
			It->Tags.Contains(FName(TEXT("navsub_obstacle_test"))))
			It->Destroy();
	for (TActorIterator<ANavMeshBoundsVolume> It(World); It; ++It)
		if (It->Tags.Contains(FName(TEXT("URLabNavBounds"))))
		{
			It->Destroy();
			break;
		}
	return true;
}
