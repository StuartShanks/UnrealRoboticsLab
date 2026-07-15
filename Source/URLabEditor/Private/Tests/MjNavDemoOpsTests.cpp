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
