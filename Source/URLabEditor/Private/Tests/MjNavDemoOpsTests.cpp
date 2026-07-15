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
#include "Dom/JsonObject.h"
#include "Editor.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "MjLevelOps.h"
#include "NavigationSystem.h"
#include "NavMesh/NavMeshBoundsVolume.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "MuJoCo/Components/Controllers/MjBaseDriveController.h"
#include "MuJoCo/Components/Controllers/MjPassthroughController.h"
#include "MuJoCo/Navigation/MjNavComponent.h"
#include "Bridge/RpcDispatcher.h"

namespace
{
// Unity build: names must be unique module-wide.
AStaticMeshActor* NavDemoFindBoxByTag(UWorld* World, const FString& ActorId)
{
	const FName Tag(*FString::Printf(TEXT("URLab.ActorId=%s"), *ActorId));
	for (TActorIterator<AStaticMeshActor> It(World); It; ++It)
	{
		if (It->Tags.Contains(Tag))
			return *It;
	}
	return nullptr;
}
} // namespace

// URLab.Nav.Ops.SpawnBox — spawns a static, blocking, nav-relevant cube at the
// converted MuJoCo location; idempotent per actor_id.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavDemoSpawnBox,
	"URLab.Nav.Ops.SpawnBox",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavDemoSpawnBox::RunTest(const FString&)
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	TestNotNull(TEXT("editor world"), World);

	FString Name, Path, Err;
	bool bExisting = false;
	const bool bOk = URLabLevelOps::SpawnBoxSync(
		TEXT("navdemo_box_test"), FVector(1.0, 2.0, 0.5), FVector(2.0, 1.0, 0.5),
		/*YawDeg=*/0.0, Name, Path, bExisting, Err);
	TestTrue(*FString::Printf(TEXT("SpawnBoxSync ok: %s"), *Err), bOk);
	TestFalse(TEXT("fresh spawn"), bExisting);

	AStaticMeshActor* Box = NavDemoFindBoxByTag(World, TEXT("navdemo_box_test"));
	TestNotNull(TEXT("box found by actor-id tag"), Box);
	if (!Box)
		return false;

	UStaticMeshComponent* SMC = Box->GetStaticMeshComponent();
	TestNotNull(TEXT("mesh set"), SMC->GetStaticMesh().Get());
	TestTrue(TEXT("static mobility"), SMC->Mobility == EComponentMobility::Static);
	TestTrue(TEXT("blocking collision"),
		SMC->GetCollisionEnabled() == ECollisionEnabled::QueryAndPhysics);
	TestTrue(TEXT("affects navmesh"), SMC->CanEverAffectNavigation());

	// MJ (1, 2, 0.5) m -> UE (100, -200, 50) cm.
	const FVector Loc = Box->GetActorLocation();
	TestEqual(TEXT("loc X"), Loc.X, 100.0, 0.5);
	TestEqual(TEXT("loc Y"), Loc.Y, -200.0, 0.5);
	TestEqual(TEXT("loc Z"), Loc.Z, 50.0, 0.5);
	// Engine cube is 100 cm; scale == size in metres.
	TestEqual(TEXT("scale X"), Box->GetActorScale3D().X, 2.0, 0.01);

	// Idempotent re-call: same actor updated, not duplicated.
	const bool bOk2 = URLabLevelOps::SpawnBoxSync(
		TEXT("navdemo_box_test"), FVector(1.5, 2.0, 0.5), FVector(2.0, 1.0, 0.5),
		0.0, Name, Path, bExisting, Err);
	TestTrue(TEXT("re-call ok"), bOk2);
	TestTrue(TEXT("was existing"), bExisting);
	int32 Count = 0;
	for (TActorIterator<AStaticMeshActor> It(World); It; ++It)
		if (It->Tags.Contains(FName(TEXT("URLab.ActorId=navdemo_box_test"))))
			++Count;
	TestEqual(TEXT("no duplicate"), Count, 1);

	Box = NavDemoFindBoxByTag(World, TEXT("navdemo_box_test"));
	if (Box)
		Box->Destroy();
	return true;
}

// URLab.Nav.Ops.NavBounds — a floor box + spawn_nav_bounds produce nav data;
// a point on the floor projects onto the navmesh; re-call reuses the volume.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavDemoNavBounds,
	"URLab.Nav.Ops.NavBounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavDemoNavBounds::RunTest(const FString&)
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	TestNotNull(TEXT("editor world"), World);

	// Floor: 10x10x0.2 m, top at MJ z=0.
	FString Name, Path, Err;
	bool bExisting = false;
	TestTrue(TEXT("floor spawns"),
		URLabLevelOps::SpawnBoxSync(TEXT("navdemo_floor_test"),
			FVector(0, 0, -0.1), FVector(10.0, 10.0, 0.2), 0.0,
			Name, Path, bExisting, Err));

	TestTrue(*FString::Printf(TEXT("nav bounds ok: %s"), *Err),
		URLabLevelOps::SpawnNavBoundsSync(
			FVector(0, 0, 0.5), FVector(6.0, 6.0, 2.0), /*AgentRadiusCm=*/0.f, Name, bExisting, Err));
	TestFalse(TEXT("fresh volume"), bExisting);

	// Wait for the async build (editor tests may not tick the world; poll).
	bool bNavData = false;
	const double Deadline = FPlatformTime::Seconds() + 30.0;
	while (FPlatformTime::Seconds() < Deadline)
	{
		if (URLabLevelOps::IsNavBuildDone(bNavData) && bNavData)
			break;
		FPlatformProcess::Sleep(0.1);
	}
	TestTrue(TEXT("nav data present after build"), bNavData);

	UNavigationSystemV1* NavSys =
		FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	TestNotNull(TEXT("nav system exists"), NavSys);
	if (NavSys)
	{
		FNavLocation Projected;
		// UE cm: a point 1 m above the floor centre projects down onto it.
		const bool bOnMesh = NavSys->ProjectPointToNavigation(
			FVector(0, 0, 100.0), Projected, FVector(200, 200, 500));
		TestTrue(TEXT("floor point is on the navmesh"), bOnMesh);
	}

	// Idempotent: second call reuses the tagged volume.
	TestTrue(TEXT("re-call ok"),
		URLabLevelOps::SpawnNavBoundsSync(
			FVector(0, 0, 0.5), FVector(6.0, 6.0, 2.0), /*AgentRadiusCm=*/0.f, Name, bExisting, Err));
	TestTrue(TEXT("volume reused"), bExisting);

	// Cleanup: floor + volume.
	if (AStaticMeshActor* Floor = NavDemoFindBoxByTag(World, TEXT("navdemo_floor_test")))
		Floor->Destroy();
	for (TActorIterator<ANavMeshBoundsVolume> It(World); It; ++It)
		if (It->Tags.Contains(FName(TEXT("URLabNavBounds"))))
		{
			It->Destroy();
			break;
		}
	return true;
}

// URLab.Nav.Ops.AddNavStack — attaches twist + base-drive + nav components with
// params applied; idempotent; bad joint name warns instead of failing.
//
// NOTE: FMjUESession's rig has exactly ONE UMjJoint, whose UE object name is
// "TestJoint" — this is also its compiled MuJoCo name (no prefix, no
// collision, since it's the sole joint in the model). The brief's original
// {"joint_x", "joint_y", "not_a_joint"} doesn't match this rig at all (0 of 3
// would resolve, not 2 of 3). Adjusted here to {"TestJoint", "TestJoint",
// "not_a_joint"} so exactly two entries genuinely resolve against the real
// joint and exactly one is bogus — keeping the warning-count assertion a real
// test of the advisory-warning path.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavDemoAddNavStack,
	"URLab.Nav.Ops.AddNavStack",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavDemoAddNavStack::RunTest(const FString&)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	URLabLevelOps::FNavStackParams P;
	P.MaxSpeed = 1.25f;
	P.AcceptanceRadiusM = 0.30f;
	P.ActuatorMode = TEXT("velocity_direct");
	P.BaseJoints = {TEXT("TestJoint"), TEXT("TestJoint"), TEXT("not_a_joint")};
	P.bDebugDraw = true;

	TArray<FString> Created, Existing, Warnings;
	FString Err;
	const bool bOk = URLabLevelOps::AddNavStackSync(
		S.Robot, P, Created, Existing, Warnings, Err);
	TestTrue(*FString::Printf(TEXT("AddNavStackSync ok: %s"), *Err), bOk);
	TestEqual(TEXT("three created"), Created.Num(), 3);
	TestEqual(TEXT("none existing"), Existing.Num(), 0);
	TestEqual(TEXT("bad joint warned"), Warnings.Num(), 1);

	UMjBaseDriveController* Drive =
		S.Robot->FindComponentByClass<UMjBaseDriveController>();
	UMjNavComponent* Nav = S.Robot->FindComponentByClass<UMjNavComponent>();
	TestNotNull(TEXT("twist attached"),
		S.Robot->FindComponentByClass<UMjTwistController>());
	TestNotNull(TEXT("drive attached"), Drive);
	TestNotNull(TEXT("nav attached"), Nav);
	if (!Drive || !Nav)
	{
		S.Cleanup();
		return false;
	}
	TestTrue(TEXT("actuator mode applied"),
		Drive->ActuatorMode == EMjBaseDriveActuatorMode::VelocityDirect);
	TestEqual(TEXT("base joints applied"),
		Drive->BaseJointNames[2], FString(TEXT("not_a_joint")));
	TestEqual(TEXT("max speed applied"), Nav->MaxSpeed, 1.25f);
	TestEqual(TEXT("acceptance m->cm"), Nav->AcceptanceRadius, 30.f);
	TestTrue(TEXT("debug draw applied"), Nav->bDebugDraw);

	// Idempotent re-call.
	Created.Reset();
	Existing.Reset();
	Warnings.Reset();
	TestTrue(TEXT("re-call ok"), URLabLevelOps::AddNavStackSync(
									 S.Robot, P, Created, Existing, Warnings, Err));
	TestEqual(TEXT("none created on re-call"), Created.Num(), 0);
	TestEqual(TEXT("three existing"), Existing.Num(), 3);

	S.Cleanup();
	return true;
}

// URLab.Nav.Ops.SetActiveController — repoints the bound controller between
// two attached UMjArticulationController components; idempotent; errors on
// unknown/ambiguous tokens.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavDemoSetActiveController,
	"URLab.Nav.Ops.SetActiveController",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavDemoSetActiveController::RunTest(const FString&)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	// Attach two controllers. PostSetup already bound one (whichever
	// FindComponentByClass found); the op must be able to select either.
	UMjBaseDriveController* Drive =
		NewObject<UMjBaseDriveController>(S.Robot, TEXT("DriveCtrl"));
	S.Robot->AddInstanceComponent(Drive);
	Drive->RegisterComponent();
	UMjPassthroughController* Pass =
		NewObject<UMjPassthroughController>(S.Robot, TEXT("PassCtrl"));
	S.Robot->AddInstanceComponent(Pass);
	Pass->RegisterComponent();

	FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));
	auto Req = [&](const TCHAR* Token) {
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), TEXT("set_active_controller"));
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		R->SetStringField(TEXT("articulation"), S.Robot->GetName());
		R->SetStringField(TEXT("controller"), Token);
		return R;
	};

	// Select the passthrough.
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req(TEXT("passthrough")));
	FString Active;
	TestTrue(TEXT("active field"), Reply->TryGetStringField(TEXT("active"), Active));
	TestEqual(TEXT("passthrough active"), Active, TEXT("MjPassthroughController"));
	TestTrue(TEXT("bound repointed"),
		S.Robot->GetActiveController() == (UMjArticulationController*)Pass);

	// Idempotent repeat.
	Reply = Disp->Dispatch(Req(TEXT("passthrough")));
	bool bWasActive = false;
	TestTrue(TEXT("was_active field"), Reply->TryGetBoolField(TEXT("was_active"), bWasActive));
	TestTrue(TEXT("was already active"), bWasActive);

	// Switch to the base drive. Adopting it calls Bind(), which logs an
	// Error because the one-joint test rig doesn't have "joint_x" — that's
	// expected and harmless here: the op's contract is repointing, not
	// successful base resolution (see MjTestHelpers.h FMjUESession docs).
	AddExpectedError(TEXT("not found in compiled model"),
		EAutomationExpectedErrorFlags::Contains, 1);
	Reply = Disp->Dispatch(Req(TEXT("base_drive")));
	Reply->TryGetStringField(TEXT("active"), Active);
	TestEqual(TEXT("base drive active"), Active, TEXT("MjBaseDriveController"));
	TestTrue(TEXT("bound repointed again"),
		S.Robot->GetActiveController() == (UMjArticulationController*)Drive);

	// Unknown token.
	Reply = Disp->Dispatch(Req(TEXT("warp_drive")));
	FString Code;
	TestTrue(TEXT("error code"), Reply->TryGetStringField(TEXT("code"), Code));
	TestEqual(TEXT("unknown_controller"), Code, TEXT("unknown_controller"));

	// Ambiguous token ("controller" is a substring of both class names).
	Reply = Disp->Dispatch(Req(TEXT("controller")));
	Reply->TryGetStringField(TEXT("code"), Code);
	TestEqual(TEXT("ambiguous"), Code, TEXT("ambiguous"));

	S.Cleanup();
	return true;
}
