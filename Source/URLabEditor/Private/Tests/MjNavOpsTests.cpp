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
#include "Bridge/RpcDispatcher.h"

namespace
{
TSharedPtr<FJsonObject> NavReq(const TCHAR* Op, const FString& Art)
{
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("op"), Op);
	R->SetStringField(TEXT("session_id"), TEXT("test-session"));
	R->SetStringField(TEXT("articulation"), Art);
	return R;
}
} // namespace

// URLab.Nav.Ops.SetGoalNoComponent — articulation without a nav component
// → error no_nav_component.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavOpsNoComponent,
	"URLab.Nav.Ops.SetGoalNoComponent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavOpsNoComponent::RunTest(const FString&)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));
	TSharedPtr<FJsonObject> Req = NavReq(TEXT("set_nav_goal"), S.Robot->GetName());
	Req->SetNumberField(TEXT("x"), 1.0);
	Req->SetNumberField(TEXT("y"), 0.0);
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString Err;
	TestTrue(TEXT("code field present"), Reply->TryGetStringField(TEXT("code"), Err));
	TestEqual(TEXT("no_nav_component"), Err, TEXT("no_nav_component"));
	S.Cleanup();
	return true;
}

// URLab.Nav.Ops.SetGoalNoNavmesh — nav component present, no navmesh →
// set_nav_goal_ok with accepted=false (graceful, not an error).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavOpsNoNavmesh,
	"URLab.Nav.Ops.SetGoalNoNavmesh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavOpsNoNavmesh::RunTest(const FString&)
{
	UMjNavComponent* Nav = nullptr;
	FMjUESession S;
	if (!S.Init([&Nav](FMjUESession& Sess) {
			Nav = NewObject<UMjNavComponent>(Sess.Robot, TEXT("NavComp"));
			Nav->RegisterComponent();
		}))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));
	TSharedPtr<FJsonObject> Req = NavReq(TEXT("set_nav_goal"), S.Robot->GetName());
	Req->SetNumberField(TEXT("x"), 1.0);
	Req->SetNumberField(TEXT("y"), 0.5);
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString OpOut;
	TestTrue(TEXT("ok reply"), Reply->TryGetStringField(TEXT("op"), OpOut));
	TestEqual(TEXT("set_nav_goal_ok"), OpOut, TEXT("set_nav_goal_ok"));
	bool bAccepted = true;
	TestTrue(TEXT("accepted field"), Reply->TryGetBoolField(TEXT("accepted"), bAccepted));
	TestFalse(TEXT("not accepted (no navmesh)"), bAccepted);
	S.Cleanup();
	return true;
}

// URLab.Nav.Ops.Status — get_nav_status reflects the component state
// (idle initially; navigating after an injected path).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavOpsStatus,
	"URLab.Nav.Ops.Status",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavOpsStatus::RunTest(const FString&)
{
	UMjNavComponent* Nav = nullptr;
	FMjUESession S;
	if (!S.Init([&Nav](FMjUESession& Sess) {
			Nav = NewObject<UMjNavComponent>(Sess.Robot, TEXT("NavComp"));
			Nav->RegisterComponent();
		}))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(NavReq(TEXT("get_nav_status"), S.Robot->GetName()));
	FString State;
	TestTrue(TEXT("state field"), Reply->TryGetStringField(TEXT("state"), State));
	TestEqual(TEXT("idle"), State, TEXT("idle"));

	Nav->SetPathForTesting({FVector::ZeroVector, FVector(500, 0, 0)});
	Reply = Disp->Dispatch(NavReq(TEXT("get_nav_status"), S.Robot->GetName()));
	Reply->TryGetStringField(TEXT("state"), State);
	TestEqual(TEXT("navigating"), State, TEXT("navigating"));
	S.Cleanup();
	return true;
}

// URLab.Nav.Ops.StatusLookahead — get_nav_status carries the pursuit carrot in
// MuJoCo convention (nav-through-mink step 1): absent before the first
// navigating tick, then lookahead_pos (metres, y = −UE_Y/100) + lookahead_yaw
// (rad, CCW = −UE yaw) — the inverse of set_nav_goal's MJ→UE mapping, so a
// script can stream it straight into a base Frame-task target.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavOpsStatusLookahead,
	"URLab.Nav.Ops.StatusLookahead",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavOpsStatusLookahead::RunTest(const FString&)
{
	UMjNavComponent* Nav = nullptr;
	FMjUESession S;
	if (!S.Init([&Nav](FMjUESession& Sess) {
			Nav = NewObject<UMjNavComponent>(Sess.Robot, TEXT("NavComp"));
			Nav->RegisterComponent();
		}))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	// No goal yet: has_lookahead false, pose fields omitted.
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(NavReq(TEXT("get_nav_status"), S.Robot->GetName()));
	bool bHas = true;
	TestTrue(TEXT("has_lookahead field present"), Reply->TryGetBoolField(TEXT("has_lookahead"), bHas));
	TestFalse(TEXT("no carrot before a goal"), bHas);
	TestFalse(TEXT("pose omitted when invalid"), Reply->HasField(TEXT("lookahead_pos")));

	// Path 5 m along UE −Y from the origin-spawned robot, one tick: carrot UE
	// (0, −LookaheadDist) → MJ [0, +LookaheadDist/100]; UE heading −PI/2 → MJ
	// yaw +PI/2 (CCW toward MuJoCo left, same convention as the twist bus).
	Nav->SetPathForTesting({FVector::ZeroVector, FVector(0, -500, 0)});
	Nav->TickComponent(0.016f, LEVELTICK_All, nullptr);
	Reply = Disp->Dispatch(NavReq(TEXT("get_nav_status"), S.Robot->GetName()));
	Reply->TryGetBoolField(TEXT("has_lookahead"), bHas);
	if (TestTrue(TEXT("carrot valid while navigating"), bHas))
	{
		const TArray<TSharedPtr<FJsonValue>>* Pos = nullptr;
		if (TestTrue(TEXT("lookahead_pos [x, y]"),
				Reply->TryGetArrayField(TEXT("lookahead_pos"), Pos) && Pos && Pos->Num() == 2))
		{
			TestEqual(TEXT("x (MJ m)"), (*Pos)[0]->AsNumber(), 0.0, 1e-2);
			TestEqual(TEXT("y = -UE_Y/100 (MJ m)"), (*Pos)[1]->AsNumber(),
				(double)Nav->LookaheadDist / 100.0, 1e-2);
		}
		double Yaw = 0.0;
		TestTrue(TEXT("lookahead_yaw field"), Reply->TryGetNumberField(TEXT("lookahead_yaw"), Yaw));
		TestEqual(TEXT("yaw = -UE yaw (MJ CCW)"), Yaw, (double)HALF_PI, 1e-3);
	}
	S.Cleanup();
	return true;
}

// URLab.Nav.Ops.SetSuction — set_suction stages the value on the articulation's
// adhesion actuator; robots without one error cleanly.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavOpsSetSuction,
	"URLab.Nav.Ops.SetSuction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavOpsSetSuction::RunTest(const FString&)
{
	// Error path: the plain test-rig robot has no adhesion actuator.
	{
		FMjUESession S;
		if (!S.Init([](FMjUESession&) {}))
		{
			AddError(S.LastError);
			S.Cleanup();
			return false;
		}
		FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
		Disp->SetActiveSessionIdForTest(TEXT("test-session"));
		TSharedPtr<FJsonObject> Req = NavReq(TEXT("set_suction"), S.Robot->GetName());
		Req->SetNumberField(TEXT("value"), 1.0);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		FString Err;
		// MakeError() puts the error string in the "code" field (see
		// FMjNavOpsNoComponent above) — not "error"; the brief's draft test
		// checked "error", which doesn't match the dispatcher's actual error
		// contract anywhere else in this file.
		TestTrue(TEXT("code field"), Reply->TryGetStringField(TEXT("code"), Err));
		TestEqual(TEXT("no_adhesion_actuator"), Err, TEXT("no_adhesion_actuator"));
		S.Cleanup();
	}
	// Happy path: the suction variant has exactly one adhesion actuator.
	//
	// FMjXmlImportSession::Compile() spawns the Manager but — unlike
	// FMjUESession::Init(), which stands up Manager->BridgeServer right after
	// Compile() because BeginPlay never fires in headless test worlds — it
	// does not construct a BridgeServer at all (none of its other tests need
	// a dispatcher). MjNavDemoOpsTests.cpp's imported-robot-style ops tests
	// all actually use FMjUESession, not FMjXmlImportSession, so there's no
	// existing "imported robot + dispatcher" example to mirror verbatim.
	// Closest faithful option: apply FMjUESession::Init's own
	// BridgeServer-standup lines (and Cleanup's teardown-before-DestroyWorld
	// lines) here, since that IS the established pattern for bringing up a
	// dispatcher in a headless test world — just applied to the import
	// session instead of the minimal rig.
	{
		const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
			TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml"));
		FMjXmlImportSession S;
		if (!S.InitFromFile(XmlPath) || !S.Compile())
		{
			AddError(S.LastError);
			S.Cleanup();
			return false;
		}
		S.Manager->BridgeServer = NewObject<UURLabBridgeServer>(S.Manager, TEXT("BridgeServer"));
		S.Manager->BridgeServer->SetOwnedByManager(true);
		S.Manager->BridgeServer->Start(TEXT(""));
		S.Manager->BridgeServer->RegisterManager(S.Manager);
		FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
		Disp->SetActiveSessionIdForTest(TEXT("test-session"));
		TSharedPtr<FJsonObject> Req = NavReq(TEXT("set_suction"), S.Robot->GetName());
		Req->SetNumberField(TEXT("value"), 0.7);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		double V = 0.0;
		TestTrue(TEXT("value echoed"), Reply->TryGetNumberField(TEXT("value"), V));
		TestEqual(TEXT("value"), V, 0.7, 1e-6);
		FString ActName;
		TestTrue(TEXT("actuator named"), Reply->TryGetStringField(TEXT("actuator"), ActName));
		TestTrue(TEXT("named 'suction'"), ActName.Contains(TEXT("suction")));
		// Mirror FMjUESession::Cleanup()'s teardown order: unregister + stop
		// the bridge before the world (and its PhysicsEngine) go away.
		S.Manager->BridgeServer->UnregisterManager(S.Manager);
		S.Manager->BridgeServer->Stop();
		S.Manager->BridgeServer = nullptr;
		S.Cleanup();
	}
	return true;
}
