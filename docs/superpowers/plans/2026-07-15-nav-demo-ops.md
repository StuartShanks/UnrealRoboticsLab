# Nav Demo Bridge Ops Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Three typed editor bridge ops (`spawn_box`, `spawn_nav_bounds`, `add_nav_stack`) plus a reproducible live demo script (`tidybot_nav_demo.py`) that proves the nav stack drives a tidybot around an obstacle.

**Architecture:** Each op follows the house pattern: a testable `URLabLevelOps::*Sync` function in `MjLevelOps.cpp` + a thin JSON handler in `MjEditorOpHandlers.cpp` + a `RegEditor` registration under the `scene` namespace (the Python client synthesizes `client.scene.<op>` from server meta — zero client changes). The demo script mirrors `Scripts/demos/tidybot_mink_demo.py`.

**Tech Stack:** UE 5.7 editor C++ (URLabEditor module), UE automation tests, Python 3 + `urlab_client` (bridge venv).

**Spec:** `docs/superpowers/specs/2026-07-15-nav-demo-ops-design.md` — read it first.

## Global Constraints

- Branch: `feat/nav-stack` (main checkout, no worktree). Commit ONLY explicit file paths, never `git add -A` (uncommitted third-party WIP exists in the tree: `third_party/build_all.sh`, `Scripts/mink_golden/*` — NEVER stage these).
- Compile command (editor-safe): `"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex` — success line `Result: Succeeded`.
- Test command (REQUIRES the editor closed; check `pgrep UnrealEditor` first): `./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "URLab.Nav"` — pass token `Result={Success}`. If an editor is open, compile-verify only and note tests as deferred for the controller to batch.
- Format ONLY files you created/edited: `/home/stuart/miniconda3/bin/clang-format -i <files>`. NEVER run `Scripts/format.sh`.
- Unity build is ON: file-scope statics/helpers in test .cpp files must have unique names across the whole module (prefix with `NavDemo`).
- Coordinates on the wire are MuJoCo world metres; conversion happens inside the Sync functions via `MjUtils::MjToUEPosition` (see `SpawnActorSync`, `MjLevelOps.cpp:400`). UE-side distances in components are cm.
- All new ops are editor-only (`RegEditor`), namespace `scene`.

---

### Task 1: `spawn_box`

**Files:**
- Modify: `Source/URLabEditor/Public/MjLevelOps.h` (declare `SpawnBoxSync` next to `SpawnActorSync`, ~line 131)
- Modify: `Source/URLabEditor/Private/MjLevelOps.cpp` (implement, directly below `SpawnActorSync`)
- Modify: `Source/URLabEditor/Private/MjEditorOpHandlers.cpp` (handler + registration)
- Create: `Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp`

**Interfaces:**
- Consumes: `MjUtils::MjToUEPosition` (already used in MjLevelOps.cpp), file-local `FindByActorIdOnly(UWorld*, const FString&)` and `MakeActorIdTag(const FString&)` in MjLevelOps.cpp.
- Produces: `URLabLevelOps::SpawnBoxSync(const FString& ActorId, const FVector& LocationMeters, const FVector& SizeMeters, double YawDeg, FString& OutActorName, FString& OutActorPath, bool& OutWasExisting, FString& OutError) -> bool` — Task 2's test and Task 4's script rely on the op; wire op name `spawn_box`, reply `spawn_box_ok`.

- [ ] **Step 1: Write the failing test**

Create `Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp`:

```cpp
// (standard copyright/disclaimer header — copy verbatim from MjNavOpsTests.cpp lines 1-21)

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
```

(Tag format verified against `MakeActorIdTag`: `URLab.ActorId=<id>`.)

- [ ] **Step 2: Compile to verify the test fails to link**

Run the compile command. Expected: FAIL — `SpawnBoxSync` undeclared.

- [ ] **Step 3: Implement**

`MjLevelOps.h`, next to `SpawnActorSync` (~line 131), same doc-comment style:

```cpp
/** Spawn (or update, per ActorId) a static blocking AStaticMeshActor cube.
 *  Location in MuJoCo world metres; Size = full extents in metres; YawDeg
 *  CCW-positive about MuJoCo Z. UE-side only (no MuJoCo geom) — callers add
 *  add_quick_convert(static=true) when MuJoCo needs it too. */
URLABEDITOR_API bool SpawnBoxSync(
	const FString& ActorId,
	const FVector& LocationMeters,
	const FVector& SizeMeters,
	double YawDeg,
	FString& OutActorName,
	FString& OutActorPath,
	bool& OutWasExisting,
	FString& OutError);
```

`MjLevelOps.cpp`, directly below `SpawnActorSync` (helpers `FindByActorIdOnly` / `MakeActorIdTag` are in scope there):

```cpp
bool SpawnBoxSync(
	const FString& ActorId,
	const FVector& LocationMeters,
	const FVector& SizeMeters,
	double YawDeg,
	FString& OutActorName,
	FString& OutActorPath,
	bool& OutWasExisting,
	FString& OutError)
{
	OutActorName.Empty();
	OutActorPath.Empty();
	OutWasExisting = false;
	OutError.Empty();

	if (!GEditor)
	{
		OutError = TEXT("GEditor null");
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World)
	{
		OutError = TEXT("editor world unavailable");
		return false;
	}
	if (ActorId.IsEmpty())
	{
		OutError = TEXT("actor_id required");
		return false;
	}
	if (SizeMeters.GetMin() <= 0.0)
	{
		OutError = TEXT("size components must be > 0");
		return false;
	}

	double MjPos[3] = {LocationMeters.X, LocationMeters.Y, LocationMeters.Z};
	const FVector UELoc = MjUtils::MjToUEPosition(MjPos);
	// MJ yaw is CCW about Z (RH); UE yaw is CW (LH Y-flip) -> negate.
	const FRotator UERot(0.0, -YawDeg, 0.0);
	// Engine cube asset is 100 cm -> component scale == size in metres.
	const FVector UEScale(SizeMeters.X, SizeMeters.Y, SizeMeters.Z);

	if (AActor* Existing = FindByActorIdOnly(World, ActorId))
	{
		AStaticMeshActor* Box = Cast<AStaticMeshActor>(Existing);
		if (!Box)
		{
			OutError = FString::Printf(
				TEXT("actor_id '%s' already in world with class %s; not a spawn_box actor"),
				*ActorId, *Existing->GetClass()->GetPathName());
			return false;
		}
		Box->SetActorLocationAndRotation(UELoc, UERot);
		Box->SetActorScale3D(UEScale);
		OutActorName = Box->GetName();
		OutActorPath = Box->GetPathName();
		OutWasExisting = true;
		return true;
	}

	UStaticMesh* Cube = LoadObject<UStaticMesh>(
		nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!Cube)
	{
		OutError = TEXT("engine cube mesh not found (/Engine/BasicShapes/Cube)");
		return false;
	}

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride =
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AStaticMeshActor* Box =
		World->SpawnActor<AStaticMeshActor>(UELoc, UERot, Params);
	if (!Box)
	{
		OutError = TEXT("SpawnActor returned null for AStaticMeshActor");
		return false;
	}
	UStaticMeshComponent* SMC = Box->GetStaticMeshComponent();
	// Editor world: setting the mesh on a Static-mobility component is the
	// same thing the editor's drag-drop placement does.
	SMC->SetStaticMesh(Cube);
	if (UMaterial* Mat = LoadObject<UMaterial>(nullptr,
			TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial")))
	{
		SMC->SetMaterial(0, Mat);
	}
	SMC->SetMobility(EComponentMobility::Static);
	Box->SetActorScale3D(UEScale);
	Box->Tags.AddUnique(FName(*MakeActorIdTag(ActorId)));

	OutActorName = Box->GetName();
	OutActorPath = Box->GetPathName();
	return true;
}
```

Add `#include "Engine/StaticMeshActor.h"` and `#include "Materials/Material.h"` to MjLevelOps.cpp includes if not present.

Handler in `MjEditorOpHandlers.cpp`, after `HandleSpawnGrid`:

```cpp
TSharedPtr<FJsonObject> HandleSpawnBox(const TSharedPtr<FJsonObject>& Req)
{
	FString ActorId;
	if (!Req->TryGetStringField(TEXT("actor_id"), ActorId) || ActorId.IsEmpty())
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_box requires non-empty 'actor_id'"));

	FVector Loc, Size;
	if (!ReadVec3(Req, TEXT("location"), Loc, FVector::ZeroVector))
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_box requires 'location' [3] (MuJoCo metres)"));
	if (!ReadVec3(Req, TEXT("size"), Size, FVector::OneVector))
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_box requires 'size' [3] full extents (metres)"));
	double YawDeg = 0.0;
	Req->TryGetNumberField(TEXT("yaw_deg"), YawDeg);

	FString ActorName, ActorPath, Err;
	bool bWasExisting = false;
	if (!URLabLevelOps::SpawnBoxSync(ActorId, Loc, Size, YawDeg,
			ActorName, ActorPath, bWasExisting, Err))
		return MakeJsonError(TEXT("spawn_failed"), Err);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("spawn_box_ok"));
	Reply->SetStringField(TEXT("actor_id"), ActorId);
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	Reply->SetStringField(TEXT("actor_path"), ActorPath);
	Reply->SetBoolField(TEXT("was_existing"), bWasExisting);
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}
```

Registration, in the `scene` block next to `spawn_grid`:

```cpp
	RegEditor(TEXT("spawn_box"), TEXT("scene"),
		GameThreadHandler(&HandleSpawnBox),
		/*Reply=*/{TEXT("op:string"), TEXT("actor_id:string"), TEXT("actor_name:string"), TEXT("actor_path:string"), TEXT("was_existing:bool"), TEXT("requires_pie_restart:bool")},
		/*Required=*/{TEXT("actor_id"), TEXT("location"), TEXT("size")});
```

Also add the matching `URLabOpRegistry::UnregisterHandler(TEXT("spawn_box"));` in the unregister block (~line 2146).

- [ ] **Step 4: Compile; run the test if no editor is open**

Compile: expect `Result: Succeeded`. If `pgrep UnrealEditor` is empty, run the test command with `--filter "URLab.Nav.Ops.SpawnBox"`; expect `Result={Success}`. Otherwise mark tests deferred.

- [ ] **Step 5: Format + commit**

```bash
/home/stuart/miniconda3/bin/clang-format -i Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp Source/URLabEditor/Private/MjLevelOps.cpp Source/URLabEditor/Public/MjLevelOps.h Source/URLabEditor/Private/MjEditorOpHandlers.cpp
git add Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp Source/URLabEditor/Private/MjLevelOps.cpp Source/URLabEditor/Public/MjLevelOps.h Source/URLabEditor/Private/MjEditorOpHandlers.cpp
git commit -m "feat(bridge): spawn_box editor op — static blocking cube for navmesh scenes"
```

---

### Task 2: `spawn_nav_bounds`

**Files:**
- Modify: `Source/URLabEditor/URLabEditor.Build.cs` (add `"NavigationSystem"` to PrivateDependencyModuleNames)
- Modify: `Source/URLabEditor/Public/MjLevelOps.h`
- Modify: `Source/URLabEditor/Private/MjLevelOps.cpp`
- Modify: `Source/URLabEditor/Private/MjEditorOpHandlers.cpp`
- Modify: `Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp`

**Interfaces:**
- Consumes: `URLabLevelOps::SpawnBoxSync` (Task 1) in the test; `RunOnGameThreadSync` (file-local, MjEditorOpHandlers.cpp) in the handler.
- Produces: `URLabLevelOps::SpawnNavBoundsSync(const FVector& CenterMeters, const FVector& ExtentMeters, FString& OutActorName, bool& OutWasExisting, FString& OutError) -> bool` (spawn/update + notify + trigger build; does NOT wait); `URLabLevelOps::IsNavBuildDone(bool& bOutNavDataPresent) -> bool` (true when no build in progress). Wire op `spawn_nav_bounds`, reply `spawn_nav_bounds_ok` with `nav_data_present:bool`, `build_seconds:float`.

- [ ] **Step 1: Write the failing test**

Append to `MjNavDemoOpsTests.cpp`:

```cpp
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
			FVector(0, 0, 0.5), FVector(6.0, 6.0, 2.0), Name, bExisting, Err));
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
			FVector(0, 0, 0.5), FVector(6.0, 6.0, 2.0), Name, bExisting, Err));
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
```

Add includes to the test file: `#include "NavigationSystem.h"`, `#include "NavMesh/NavMeshBoundsVolume.h"`.

- [ ] **Step 2: Compile to verify failure** — expect link failure on `SpawnNavBoundsSync` / `IsNavBuildDone`.

- [ ] **Step 3: Implement**

`URLabEditor.Build.cs`: add `"NavigationSystem"` to the PrivateDependencyModuleNames list (same list containing `"UnrealEd"`).

`MjLevelOps.h`:

```cpp
/** Spawn (or update — a single volume tagged 'URLabNavBounds' is reused) a
 *  NavMeshBoundsVolume spanning center ± extent (MuJoCo metres), notify the
 *  navigation system and trigger a rebuild. Does not wait for the build —
 *  poll IsNavBuildDone. */
URLABEDITOR_API bool SpawnNavBoundsSync(
	const FVector& CenterMeters,
	const FVector& ExtentMeters,
	FString& OutActorName,
	bool& OutWasExisting,
	FString& OutError);

/** True when no navmesh build is in progress. bOutNavDataPresent reports
 *  whether nav data (a RecastNavMesh) exists in the editor world. */
URLABEDITOR_API bool IsNavBuildDone(bool& bOutNavDataPresent);
```

`MjLevelOps.cpp` (includes: `"NavigationSystem.h"`, `"NavMesh/NavMeshBoundsVolume.h"`, `"Builders/CubeBuilder.h"`, `"ActorFactories/ActorFactory.h"`):

```cpp
bool SpawnNavBoundsSync(
	const FVector& CenterMeters,
	const FVector& ExtentMeters,
	FString& OutActorName,
	bool& OutWasExisting,
	FString& OutError)
{
	OutActorName.Empty();
	OutWasExisting = false;
	OutError.Empty();

	if (!GEditor)
	{
		OutError = TEXT("GEditor null");
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World)
	{
		OutError = TEXT("editor world unavailable");
		return false;
	}
	if (ExtentMeters.GetMin() <= 0.0)
	{
		OutError = TEXT("extent components must be > 0");
		return false;
	}

	double MjPos[3] = {CenterMeters.X, CenterMeters.Y, CenterMeters.Z};
	const FVector UECenter = MjUtils::MjToUEPosition(MjPos);

	static const FName NavBoundsTag(TEXT("URLabNavBounds"));
	ANavMeshBoundsVolume* Vol = nullptr;
	for (TActorIterator<ANavMeshBoundsVolume> It(World); It; ++It)
	{
		if (It->Tags.Contains(NavBoundsTag))
		{
			Vol = *It;
			OutWasExisting = true;
			break;
		}
	}
	if (!Vol)
	{
		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Vol = World->SpawnActor<ANavMeshBoundsVolume>(
			UECenter, FRotator::ZeroRotator, Params);
		if (!Vol)
		{
			OutError = TEXT("SpawnActor returned null for ANavMeshBoundsVolume");
			return false;
		}
		Vol->Tags.AddUnique(NavBoundsTag);
	}
	else
	{
		Vol->SetActorLocation(UECenter);
	}

	// Size the brush exactly the way the editor's Place tool does.
	UCubeBuilder* Builder = NewObject<UCubeBuilder>();
	Builder->X = ExtentMeters.X * 200.0; // full size, cm
	Builder->Y = ExtentMeters.Y * 200.0;
	Builder->Z = ExtentMeters.Z * 200.0;
	UActorFactory::CreateBrushForVolumeActor(Vol, Builder);

	OutActorName = Vol->GetName();

	if (UNavigationSystemV1* NavSys =
			FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
	{
		NavSys->OnNavigationBoundsUpdated(Vol);
		NavSys->Build();
	}
	else
	{
		OutError = TEXT("no navigation system in editor world");
		return false;
	}
	return true;
}

bool IsNavBuildDone(bool& bOutNavDataPresent)
{
	bOutNavDataPresent = false;
	if (!GEditor)
		return true;
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World)
		return true;
	UNavigationSystemV1* NavSys =
		FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	if (!NavSys)
		return true;
	bOutNavDataPresent =
		(NavSys->GetDefaultNavDataInstance(FNavigationSystem::DontCreate) != nullptr);
	return !NavSys->IsNavigationBuildInProgress();
}
```

Handler — self-marshalled (registered WITHOUT `GameThreadHandler`, like `begin_pie`): the spawn hops to the game thread; the wait polls from the worker so the game thread keeps ticking the async build.

```cpp
// spawn_nav_bounds self-marshals (begin_pie pattern): spawn + build-trigger on
// the game thread, then poll from the worker so the editor keeps ticking.
TSharedPtr<FJsonObject> HandleSpawnNavBounds(const TSharedPtr<FJsonObject>& Req)
{
	FVector Center, Extent;
	if (!ReadVec3(Req, TEXT("center"), Center, FVector::ZeroVector))
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_nav_bounds requires 'center' [3] (MuJoCo metres)"));
	if (!ReadVec3(Req, TEXT("extent"), Extent, FVector::OneVector))
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_nav_bounds requires 'extent' [3] half-extents (metres)"));
	double TimeoutS = 10.0;
	Req->TryGetNumberField(TEXT("timeout_s"), TimeoutS);

	const double T0 = FPlatformTime::Seconds();

	FString ActorName, Err;
	bool bWasExisting = false, bSpawnOk = false;
	RunOnGameThreadSync([&]() -> TSharedPtr<FJsonObject> {
		bSpawnOk = URLabLevelOps::SpawnNavBoundsSync(
			Center, Extent, ActorName, bWasExisting, Err);
		return nullptr;
	});
	if (!bSpawnOk)
		return MakeJsonError(TEXT("spawn_failed"), Err);

	bool bNavData = false, bDone = false;
	const double Deadline = FPlatformTime::Seconds() + TimeoutS;
	while (FPlatformTime::Seconds() < Deadline)
	{
		RunOnGameThreadSync([&]() -> TSharedPtr<FJsonObject> {
			bDone = URLabLevelOps::IsNavBuildDone(bNavData);
			return nullptr;
		});
		if (bDone && bNavData)
			break;
		FPlatformProcess::Sleep(0.2);
	}
	if (!(bDone && bNavData))
		return MakeJsonError(TEXT("nav_build_timeout"),
			FString::Printf(TEXT("navmesh not ready after %.1fs "
								 "(nav_data_present=%d, build_done=%d)"),
				TimeoutS, bNavData ? 1 : 0, bDone ? 1 : 0));

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("spawn_nav_bounds_ok"));
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	Reply->SetBoolField(TEXT("was_existing"), bWasExisting);
	Reply->SetBoolField(TEXT("nav_data_present"), bNavData);
	Reply->SetNumberField(TEXT("build_seconds"), FPlatformTime::Seconds() - T0);
	return Reply;
}
```

(`RunOnGameThreadSync` verified: template over a lambda returning `TSharedPtr<FJsonObject>`, with an `IsInGameThread()` fast-path — the `return nullptr;` usage above is correct.)

Registration (no `GameThreadHandler` wrapper — the handler self-marshals):

```cpp
	RegEditor(TEXT("spawn_nav_bounds"), TEXT("scene"), &HandleSpawnNavBounds,
		/*Reply=*/{TEXT("op:string"), TEXT("actor_name:string"), TEXT("was_existing:bool"), TEXT("nav_data_present:bool"), TEXT("build_seconds:float")},
		/*Required=*/{TEXT("center"), TEXT("extent")});
```

Plus `UnregisterHandler(TEXT("spawn_nav_bounds"))` in the unregister block.

- [ ] **Step 4: Compile; run `--filter "URLab.Nav.Ops.NavBounds"` if editor closed.** Expected `Result={Success}`. If `ProjectPointToNavigation` fails because the editor test world never ticks the async build, switch the Sync function's build call to the synchronous editor path (`NavSys->Build()` already is the blocking editor build; if it proves insufficient, note findings and consult the controller).

- [ ] **Step 5: Format (same 4 files + Build.cs is C# — do NOT clang-format Build.cs) + commit**

```bash
git add Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp Source/URLabEditor/Private/MjLevelOps.cpp Source/URLabEditor/Public/MjLevelOps.h Source/URLabEditor/Private/MjEditorOpHandlers.cpp Source/URLabEditor/URLabEditor.Build.cs
git commit -m "feat(bridge): spawn_nav_bounds editor op — NavMeshBoundsVolume + navmesh build"
```

---

### Task 3: `add_nav_stack`

**Files:**
- Modify: `Source/URLabEditor/Public/MjLevelOps.h`
- Modify: `Source/URLabEditor/Private/MjLevelOps.cpp`
- Modify: `Source/URLabEditor/Private/MjEditorOpHandlers.cpp`
- Modify: `Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp`

**Interfaces:**
- Consumes: `AMjArticulation` (`MuJoCo/MjArticulation.h`), `UMjTwistController` (`MuJoCo/Components/Controllers/MjTwistController.h`), `UMjBaseDriveController` (`.../MjBaseDriveController.h`, props: `TArray<FString> BaseJointNames`, `EMjBaseDriveActuatorMode ActuatorMode` with values `PositionIntegrate`/`VelocityDirect`), `UMjNavComponent` (`MuJoCo/Navigation/MjNavComponent.h`, props: `float MaxSpeed` m/s, `float MaxYawRate` rad/s, `float LookaheadDist` cm, `float AcceptanceRadius` cm, `float DecelRadius` cm, `float StuckTimeout` s, `bool bDebugDraw`). Articulation resolution loop copied from `HandleAddController` (MjEditorOpHandlers.cpp:1653).
- Produces: `URLabLevelOps::FNavStackParams` struct + `URLabLevelOps::AddNavStackSync(AMjArticulation* Art, const FNavStackParams& P, TArray<FString>& OutCreated, TArray<FString>& OutExisting, TArray<FString>& OutWarnings, FString& OutError) -> bool`. Wire op `add_nav_stack`, reply `add_nav_stack_ok` with `created:array`, `existing:array`, `warnings:array`.

- [ ] **Step 1: Write the failing test**

Append to `MjNavDemoOpsTests.cpp` (include `"Tests/MjTestHelpers.h"`, `"MuJoCo/MjArticulation.h"`, `"MuJoCo/Components/Controllers/MjTwistController.h"`, `"MuJoCo/Components/Controllers/MjBaseDriveController.h"`, `"MuJoCo/Navigation/MjNavComponent.h"`):

```cpp
// URLab.Nav.Ops.AddNavStack — attaches twist + base-drive + nav components with
// params applied; idempotent; bad joint name warns instead of failing.
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
	P.BaseJoints = {TEXT("joint_x"), TEXT("joint_y"), TEXT("not_a_joint")};
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
```

NOTE: `FMjUESession` (MjTestHelpers.h) provides `S.Robot` (an `AMjArticulation*` with `UMjJoint` components named per its rig). Read the helper first: if the rig's joints are NOT named `joint_x`/`joint_y`, adjust `P.BaseJoints` so exactly one entry is bogus and the others match real joints — the warning-count assertion must stay meaningful.

- [ ] **Step 2: Compile to verify failure** — expect `FNavStackParams` / `AddNavStackSync` undeclared.

- [ ] **Step 3: Implement**

`MjLevelOps.h` (forward-declare `class AMjArticulation;` at the top):

```cpp
/** Optional tuning for add_nav_stack. Unset optionals keep component defaults.
 *  Distances are MuJoCo metres (converted to component cm internally). */
struct FNavStackParams
{
	TArray<FString> BaseJoints; // empty = component default (joint_x/y/th)
	FString ActuatorMode;       // "", "position_integrate", "velocity_direct"
	TOptional<float> MaxSpeed;
	TOptional<float> MaxYawRate;
	TOptional<float> LookaheadM;
	TOptional<float> AcceptanceRadiusM;
	TOptional<float> DecelRadiusM;
	TOptional<float> StuckTimeout;
	TOptional<bool> bDebugDraw;
};

/** Attach (or reuse) UMjTwistController + UMjBaseDriveController +
 *  UMjNavComponent on the articulation and apply params. Unresolvable base
 *  joint names are warnings, not failures (resolution is suffix-tolerant at
 *  Bind; the warning is advisory). */
URLABEDITOR_API bool AddNavStackSync(
	AMjArticulation* Art,
	const FNavStackParams& Params,
	TArray<FString>& OutCreated,
	TArray<FString>& OutExisting,
	TArray<FString>& OutWarnings,
	FString& OutError);
```

`MjLevelOps.cpp` (includes: the three component headers + `"MuJoCo/MjArticulation.h"` + `"MuJoCo/Components/Joints/MjJoint.h"`):

```cpp
namespace
{
// Advisory check mirroring the runtime's suffix-tolerant resolution
// (MjBaseDriveController.cpp ResolveJointByName): exact, "_Name" or "/Name".
bool NavStackJointNameKnown(AMjArticulation* Art, const FString& Name)
{
	TInlineComponentArray<UMjJoint*> Joints(Art);
	const FString US = TEXT("_") + Name, SL = TEXT("/") + Name;
	for (UMjJoint* J : Joints)
	{
		if (!J)
			continue;
		const FString S = J->GetMjName();
		if (S == Name || S.EndsWith(US) || S.EndsWith(SL))
			return true;
	}
	return false;
}

template <typename T>
T* NavStackFindOrCreate(AMjArticulation* Art, const TCHAR* CompName,
	TArray<FString>& OutCreated, TArray<FString>& OutExisting)
{
	if (T* Existing = Art->FindComponentByClass<T>())
	{
		OutExisting.Add(Existing->GetName());
		return Existing;
	}
	T* C = NewObject<T>(Art, CompName);
	if (!C)
		return nullptr;
	Art->AddInstanceComponent(C);
	C->RegisterComponent();
	OutCreated.Add(C->GetName());
	return C;
}
} // namespace

bool AddNavStackSync(
	AMjArticulation* Art,
	const FNavStackParams& Params,
	TArray<FString>& OutCreated,
	TArray<FString>& OutExisting,
	TArray<FString>& OutWarnings,
	FString& OutError)
{
	OutCreated.Reset();
	OutExisting.Reset();
	OutWarnings.Reset();
	OutError.Empty();

	if (!Art)
	{
		OutError = TEXT("null articulation");
		return false;
	}

	UMjTwistController* Twist = NavStackFindOrCreate<UMjTwistController>(
		Art, TEXT("TwistController"), OutCreated, OutExisting);
	UMjBaseDriveController* Drive = NavStackFindOrCreate<UMjBaseDriveController>(
		Art, TEXT("BaseDriveController"), OutCreated, OutExisting);
	UMjNavComponent* Nav = NavStackFindOrCreate<UMjNavComponent>(
		Art, TEXT("NavComponent"), OutCreated, OutExisting);
	if (!Twist || !Drive || !Nav)
	{
		OutError = TEXT("NewObject returned null");
		return false;
	}

	if (Params.BaseJoints.Num() > 0)
	{
		if (Params.BaseJoints.Num() != 3)
		{
			OutError = TEXT("base_joints must have exactly 3 entries (X, Y, TH)");
			return false;
		}
		Drive->BaseJointNames = Params.BaseJoints;
	}
	for (const FString& N : Drive->BaseJointNames)
	{
		if (!NavStackJointNameKnown(Art, N))
			OutWarnings.Add(FString::Printf(
				TEXT("base joint '%s' not found on actor (advisory — Bind "
					 "resolves against the compiled model)"), *N));
	}

	if (!Params.ActuatorMode.IsEmpty())
	{
		if (Params.ActuatorMode == TEXT("position_integrate"))
			Drive->ActuatorMode = EMjBaseDriveActuatorMode::PositionIntegrate;
		else if (Params.ActuatorMode == TEXT("velocity_direct"))
			Drive->ActuatorMode = EMjBaseDriveActuatorMode::VelocityDirect;
		else
		{
			OutError = FString::Printf(
				TEXT("unknown actuator_mode '%s' (position_integrate | "
					 "velocity_direct)"), *Params.ActuatorMode);
			return false;
		}
	}

	if (Params.MaxSpeed.IsSet())
		Nav->MaxSpeed = Params.MaxSpeed.GetValue();
	if (Params.MaxYawRate.IsSet())
		Nav->MaxYawRate = Params.MaxYawRate.GetValue();
	if (Params.LookaheadM.IsSet())
		Nav->LookaheadDist = Params.LookaheadM.GetValue() * 100.f;
	if (Params.AcceptanceRadiusM.IsSet())
		Nav->AcceptanceRadius = Params.AcceptanceRadiusM.GetValue() * 100.f;
	if (Params.DecelRadiusM.IsSet())
		Nav->DecelRadius = Params.DecelRadiusM.GetValue() * 100.f;
	if (Params.StuckTimeout.IsSet())
		Nav->StuckTimeout = Params.StuckTimeout.GetValue();
	if (Params.bDebugDraw.IsSet())
		Nav->bDebugDraw = Params.bDebugDraw.GetValue();

	return true;
}
```

(`UMjJoint::GetMjName()` verified to exist — `MuJoCo/Components/Joints/MjJoint.h:256`; `FindMjComponentByName` in MjEditorOpHandlers.cpp uses the same accessor.)

Handler in `MjEditorOpHandlers.cpp`, after `HandleAddController` (reuse its articulation-resolution block verbatim):

```cpp
TSharedPtr<FJsonObject> HandleAddNavStack(const TSharedPtr<FJsonObject>& Req)
{
	if (!GEditor)
		return MakeJsonError(TEXT("not_in_editor"), TEXT("GEditor null"));
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("editor world unavailable"));

	// Same key-or-sole resolution as add_controller.
	FString Key, Err;
	bool bByName = false;
	AMjArticulation* Art = nullptr;
	if (ResolveActorKey(Req, Key, bByName, Err))
	{
		for (TActorIterator<AMjArticulation> It(World); It; ++It)
		{
			AMjArticulation* A = *It;
			if (A && (A->ActorId.Equals(Key) || A->GetName().Equals(Key) || A->GetActorLabel().Equals(Key)))
			{
				Art = A;
				break;
			}
		}
		if (!Art)
			return MakeJsonError(TEXT("unknown_articulation"), Key);
	}
	else
	{
		int32 N = 0;
		for (TActorIterator<AMjArticulation> It(World); It; ++It)
		{
			Art = *It;
			++N;
		}
		if (N == 0)
			return MakeJsonError(TEXT("no_articulation"), TEXT("no AMjArticulation in level"));
		if (N > 1)
			return MakeJsonError(TEXT("ambiguous"), TEXT("multiple articulations; pass 'target'"));
	}

	URLabLevelOps::FNavStackParams P;
	const TArray<TSharedPtr<FJsonValue>>* JointsArr = nullptr;
	if (Req->TryGetArrayField(TEXT("base_joints"), JointsArr) && JointsArr)
		for (const TSharedPtr<FJsonValue>& V : *JointsArr)
			P.BaseJoints.Add(V->AsString());
	Req->TryGetStringField(TEXT("actuator_mode"), P.ActuatorMode);
	double D = 0.0;
	if (Req->TryGetNumberField(TEXT("max_speed"), D))
		P.MaxSpeed = (float)D;
	if (Req->TryGetNumberField(TEXT("max_yaw_rate"), D))
		P.MaxYawRate = (float)D;
	if (Req->TryGetNumberField(TEXT("lookahead"), D))
		P.LookaheadM = (float)D;
	if (Req->TryGetNumberField(TEXT("acceptance_radius"), D))
		P.AcceptanceRadiusM = (float)D;
	if (Req->TryGetNumberField(TEXT("decel_radius"), D))
		P.DecelRadiusM = (float)D;
	if (Req->TryGetNumberField(TEXT("stuck_timeout"), D))
		P.StuckTimeout = (float)D;
	bool bDraw = false;
	if (Req->TryGetBoolField(TEXT("debug_draw"), bDraw))
		P.bDebugDraw = bDraw;

	TArray<FString> Created, Existing, Warnings;
	if (!URLabLevelOps::AddNavStackSync(Art, P, Created, Existing, Warnings, Err))
		return MakeJsonError(TEXT("attach_failed"), Err);

	auto ToJsonArr = [](const TArray<FString>& In) {
		TArray<TSharedPtr<FJsonValue>> Out;
		for (const FString& S : In)
			Out.Add(MakeShared<FJsonValueString>(S));
		return Out;
	};
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("add_nav_stack_ok"));
	Reply->SetStringField(TEXT("actor_name"), Art->GetName());
	Reply->SetArrayField(TEXT("created"), ToJsonArr(Created));
	Reply->SetArrayField(TEXT("existing"), ToJsonArr(Existing));
	Reply->SetArrayField(TEXT("warnings"), ToJsonArr(Warnings));
	return Reply;
}
```

Registration (next to `add_controller`, namespace `scene` so the synthesized client binding is `client.scene.add_nav_stack` — there is no `client.nav` namespace object):

```cpp
	RegEditor(TEXT("add_nav_stack"), TEXT("scene"),
		GameThreadHandler(&HandleAddNavStack),
		/*Reply=*/{TEXT("op:string"), TEXT("actor_name:string"), TEXT("created:array"), TEXT("existing:array"), TEXT("warnings:array")},
		/*Required=*/{});
```

Plus `UnregisterHandler(TEXT("add_nav_stack"))`.

- [ ] **Step 4: Compile; run `--filter "URLab.Nav.Ops.AddNavStack"` if editor closed.** Expected `Result={Success}`.

- [ ] **Step 5: Format the 4 C++ files + commit**

```bash
git add Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp Source/URLabEditor/Private/MjLevelOps.cpp Source/URLabEditor/Public/MjLevelOps.h Source/URLabEditor/Private/MjEditorOpHandlers.cpp
git commit -m "feat(bridge): add_nav_stack editor op — attach twist/base-drive/nav components"
```

---

### Task 4: `tidybot_nav_demo.py` + guide update

**Files:**
- Create: `Scripts/demos/tidybot_nav_demo.py`
- Modify: `docs/guides/navigation-demo.md` (add "Scripted demo" section; update the validation checklist)

**Interfaces:**
- Consumes: ops from Tasks 1-3 via `client.scene.spawn_box / spawn_nav_bounds / add_nav_stack`; existing `client.scene.import_xml / spawn_actor / ensure_manager / add_quick_convert`, `client.sim.start`, `client.runtime.set_mode / set_nav_goal / get_nav_status`, `client.outliner.get_actor_bounds` (PIE-aware position readback, returns MuJoCo metres). Model: `Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml`. Venv python: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python`.
- Produces: exit 0 = all acceptance items pass; nonzero otherwise. Prereq: editor open + bridge listening (this task is compile/lint-only for the executor; the LIVE RUN is done by the controller/user session afterwards).

- [ ] **Step 1: Write the script**

Create `Scripts/demos/tidybot_nav_demo.py`:

```python
#!/usr/bin/env python3
"""TidyBot navigation bridge demo — live E2E for the mobile-base nav stack.

Authors the whole demo scene over the bridge (floor, obstacle wall, navmesh
bounds), imports + spawns the TidyBot, attaches the nav stack
(twist + base-drive + nav components), enters PIE in live mode, then:

  1. HAPPY PATH  — set_nav_goal behind an obstacle wall; accept when
     get_nav_status reaches 'arrived', final distance <= ACCEPT_DIST, and the
     sampled track detoured (max |y| >= DETOUR_MIN_Y — the straight line to the
     goal has y == 0).
  2. OFF-MESH    — set_nav_goal inside the wall; accept when accepted == false.

Run with the bridge venv's python while the editor is open:
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
        Scripts/demos/tidybot_nav_demo.py

Exits nonzero if any acceptance item fails. See also tidybot_mink_demo.py
(same orchestration idioms) and docs/guides/navigation-demo.md.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

MODEL_XML = (
    Path(__file__).resolve().parents[1]
    / "mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"
)
ACTOR_ID = "tidybot_0"

# Scene layout (MuJoCo world metres). Wall at x=2 spans y in [-1.5, 1.5];
# robot starts at origin; goal is straight through the wall.
FLOOR = dict(actor_id="nav_floor", location=(0.0, 0.0, -0.1), size=(20.0, 20.0, 0.2))
OBSTACLES = [
    dict(actor_id="nav_wall", location=(2.0, 0.0, 0.4), size=(0.4, 3.0, 0.8)),
    dict(actor_id="nav_block_a", location=(3.0, -1.6, 0.4), size=(0.6, 0.6, 0.8)),
    dict(actor_id="nav_block_b", location=(1.0, 1.8, 0.4), size=(0.6, 0.6, 0.8)),
]
NAV_BOUNDS = dict(center=(0.0, 0.0, 0.5), extent=(12.0, 12.0, 2.0))

GOAL = (4.0, 0.0)          # behind the wall
OFFMESH_GOAL = (2.0, 0.0)  # inside the wall
ACCEPT_DIST = 0.20         # m: acceptance_radius (0.15) + slack
DETOUR_MIN_Y = 1.0         # m: sampled |y| must exceed this at least once
TIMEOUT_S = 90.0
POLL_S = 0.5


def log(msg: str) -> None:
    print(f"[nav-demo] {msg}", flush=True)


def fail(msg: str) -> None:
    print(f"[nav-demo] FAIL: {msg}", flush=True)
    sys.exit(1)


def robot_xy(client) -> tuple[float, float]:
    """Robot planar position in MuJoCo metres via the PIE-aware bounds op."""
    r = client.outliner.get_actor_bounds(target=ACTOR_ID)
    c = r["center"]
    return float(c[0]), float(c[1])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="tcp://localhost")
    ap.add_argument("--keep-open", action="store_true",
                    help="leave PIE running after the demo")
    args = ap.parse_args()

    client = URLabClient(args.host)
    client.connect()
    log("connected")

    # ---- Phase 1: scene ----------------------------------------------------
    log("authoring scene (floor, wall, nav bounds)")
    client.scene.spawn_box(**FLOOR)  # UE-only: MJCF plane is the physics floor
    for ob in OBSTACLES:
        client.scene.spawn_box(**ob)
        client.scene.add_quick_convert(target=ob["actor_id"], static=True)
    r = client.scene.spawn_nav_bounds(**NAV_BOUNDS, timeout_s=30.0)
    if not r.get("nav_data_present"):
        fail(f"navmesh did not bake: {r}")
    log(f"navmesh ready in {r.get('build_seconds', 0):.1f}s")

    # ---- Phase 2: robot ----------------------------------------------------
    log(f"importing {MODEL_XML.name}")
    imp = client.scene.import_xml(path=str(MODEL_XML))
    bp = imp["blueprint_class_path"]
    client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
    client.scene.ensure_manager()
    stack = client.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
    if stack.get("warnings"):
        log(f"add_nav_stack warnings: {stack['warnings']}")
    log(f"nav stack: created={stack.get('created')} existing={stack.get('existing')}")

    # ---- Phase 3: PIE, live mode -------------------------------------------
    log("entering PIE (sim.start)")
    client.sim.start(timeout_s=180.0)
    try:
        client.runtime.set_mode("live")
    except URLabRPCError as exc:
        log(f"set_mode(live) warning: {exc}")
    time.sleep(2.0)  # let physics settle on spawn

    # ---- Phase 4: happy path ------------------------------------------------
    log(f"set_nav_goal {GOAL}")
    r = client.runtime.set_nav_goal(articulation=ACTOR_ID, x=GOAL[0], y=GOAL[1])
    if not r.get("accepted"):
        fail(f"happy-path goal rejected: {r}")

    max_abs_y = 0.0
    state = "navigating"
    deadline = time.time() + TIMEOUT_S
    while time.time() < deadline:
        s = client.runtime.get_nav_status(articulation=ACTOR_ID)
        state = s["state"]
        x, y = robot_xy(client)
        max_abs_y = max(max_abs_y, abs(y))
        log(f"  state={state:<10} dist={s['distance_to_goal']:.2f} pos=({x:.2f},{y:.2f})")
        if state in ("arrived", "failed"):
            break
        time.sleep(POLL_S)

    if state != "arrived":
        fail(f"did not arrive (state={state})")
    s = client.runtime.get_nav_status(articulation=ACTOR_ID)
    if s["distance_to_goal"] > ACCEPT_DIST:
        fail(f"arrived but distance {s['distance_to_goal']:.2f} > {ACCEPT_DIST}")
    if max_abs_y < DETOUR_MIN_Y:
        fail(f"no detour observed (max |y| = {max_abs_y:.2f} < {DETOUR_MIN_Y})"
             " — did it drive through the wall?")
    log(f"ARRIVED, detour max |y| = {max_abs_y:.2f} m")

    # ---- Phase 5: off-mesh reject -------------------------------------------
    log(f"off-mesh goal {OFFMESH_GOAL} (inside the wall)")
    r = client.runtime.set_nav_goal(
        articulation=ACTOR_ID, x=OFFMESH_GOAL[0], y=OFFMESH_GOAL[1])
    if r.get("accepted"):
        fail("off-mesh goal was accepted — expected rejection")
    log("off-mesh goal correctly rejected")

    if not args.keep_open:
        try:
            client.sim.stop()
        except URLabRPCError:
            pass
    log("ALL ACCEPTANCE ITEMS PASSED")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Lint check** — `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python -m py_compile Scripts/demos/tidybot_nav_demo.py`; expect exit 0. (The live run happens in the controller/user editor session, not in this task.)

- [ ] **Step 3: Update `docs/guides/navigation-demo.md`**

Add after the "### Autonomous (bridge)" section:

```markdown
### Scripted demo (one command)

The whole setup above — floor, obstacles, navmesh bounds, robot, components,
PIE — can be run as a single reproducible script (editor open, bridge up):

​```bash
/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
    Scripts/demos/tidybot_nav_demo.py
​```

It authors the scene with `spawn_box` / `spawn_nav_bounds`, attaches the
components with `add_nav_stack`, drives a goal behind an obstacle wall, and
asserts arrival + a real detour + off-mesh rejection. `--keep-open` leaves
PIE running for inspection.
```

(Remove the stray zero-width characters around the inner code fence when writing the file — they are only there to nest fences in this plan.)

In the "## Validation checklist" section, mark items 1-2 as done (automation suite green 2026-07-15, `joint_y` sign confirmed) and point items 3-5 at the scripted demo.

- [ ] **Step 4: Commit**

```bash
git add Scripts/demos/tidybot_nav_demo.py docs/guides/navigation-demo.md
git commit -m "demo(nav): scripted TidyBot navigation E2E over the bridge"
```

---

## Verification (whole feature, controller-run)

1. Compile green; `URLab.Nav` filter green (now 22 tests) in an editor-free window.
2. Live run: open the editor, run `tidybot_nav_demo.py`, expect `ALL ACCEPTANCE ITEMS PASSED`.
3. Update `.superpowers/sdd/progress.md` + the nav-stack memory with the outcome.
