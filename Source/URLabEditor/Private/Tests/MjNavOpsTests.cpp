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
