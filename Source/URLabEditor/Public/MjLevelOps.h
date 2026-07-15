// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonValue.h"

class AMjArticulation;

/**
 * Editor-side helpers for the bridge's level / asset RPCs. Each function
 * is synchronous, returns true on success, and reports an error via
 * OutError on failure. The JSON-envelope marshalling lives in
 * URLabEditorOpHandlers — these helpers are pure UE-side primitives.
 */
namespace URLabLevelOps
{
/** Programmatically run UMujocoImportFactory on an MJCF file. Result
 *  blueprint is placed under /Game/MuJoCoImports/<stem>. Idempotent
 *  on the same path unless bForceReimport is true. */
URLABEDITOR_API bool ImportXmlSync(
	const FString& AbsXmlPath,
	bool bForceReimport,
	FString& OutBlueprintClassPath,
	FString& OutBlueprintShortName,
	bool& bOutImportedNow,
	FString& OutError);

/** Resolve a short name or full /Game path into a level asset path of
 *  the form `/Game/Levels/<name>` (no extension). Idempotent. */
URLABEDITOR_API FString ResolveLevelPath(const FString& NameOrPath);

/** Create a new empty level at the given /Game/Levels/<name> path,
 *  save it, and leave it loaded as the editor's current level. With
 *  bForceOverwrite=true, deletes any existing asset at the target
 *  path first (switching the editor off it if currently loaded). */
URLABEDITOR_API bool CreateLevelSync(
	const FString& NameOrPath,
	bool bForceOverwrite,
	FString& OutLevelPath,
	FString& OutError);

/** Force-delete an asset by object path (`/Game/Foo/Bar.Bar` form).
 *  Switches the editor off the asset first if it's a UWorld
 *  currently loaded. Returns true and sets bOutWasFound=false if
 *  no asset existed at the path (idempotent). */
URLABEDITOR_API bool DestroyAssetSync(
	const FString& ObjectPath,
	bool& bOutWasFound,
	FString& OutError);

/** Filtered actor enumeration. All filters optional; AND across set
 *  filters. Uses the same row shape as ListActorsSync. */
URLABEDITOR_API bool FindActorsSync(
	const FString& ClassFilter,
	const FString& TagFilter,
	const FString& NamePrefix,
	bool bSearchPieWorld,
	TArray<TSharedPtr<FJsonValue>>& OutActors,
	bool& bOutSearchedPie,
	FString& OutError);

/** AABB of an actor's components, MJ metres. */
URLABEDITOR_API bool GetActorBoundsSync(
	const FString& ActorKey,
	bool bComponentsOnly,
	double OutMin[3],
	double OutMax[3],
	FString& OutResolvedName,
	FString& OutError);

/** Full level snapshot — every URLab-relevant actor with its
 *  URLab-specific metadata (mj_class, joints, actuators, sensors). */
URLABEDITOR_API bool SnapshotSceneSync(
	TArray<TSharedPtr<FJsonValue>>& OutActors,
	bool& bOutInPie,
	FString& OutLevelPath,
	FString& OutError);

/** Spawn a copy of an existing actor with a fresh actor_id. */
URLABEDITOR_API bool DuplicateActorSync(
	const FString& SrcKey,
	const FString& NewActorId,
	bool bHasOverrideLocation,
	const FVector& OverrideLocationMeters,
	FString& OutActorName,
	FString& OutActorPath,
	FString& OutBlueprintClassPath,
	FString& OutError);

/** Walks the actor's attachment tree. */
URLABEDITOR_API bool ActorHierarchySync(
	const FString& ActorKey,
	TSharedPtr<FJsonObject>& OutRoot,
	FString& OutError);

/** Load an existing /Game/.../<name> level into the editor. */
URLABEDITOR_API bool LoadLevelSync(
	const FString& NameOrPath,
	FString& OutLevelPath,
	FString& OutError);

/** Save the editor's currently-loaded level. Returns the level's
 *  asset path on success. */
URLABEDITOR_API bool SaveCurrentLevelSync(
	FString& OutLevelPath,
	FString& OutError);

/**
 * Spawn an actor from a Blueprint into the editor's current level.
 *
 * BlueprintNameOrPath accepts either a short BP path
 * `/Game/MuJoCoImports/foo` or the explicit class path
 * `/Game/MuJoCoImports/foo.foo_C` — both resolve to the same class.
 * ActorId is stored on AMjArticulation::ActorId when the BP
 * derives from AMjArticulation; for non-articulation actors it's
 * recorded as a UE actor tag of the form `URLab.ActorId=<id>`.
 *
 * Location is in MJ-native metres (converted to UE cm internally).
 * RotationQuatXyzw uses the (x,y,z,w) convention — same as the
 * existing handshake / step replies.
 *
 * Idempotent on a non-empty ActorId: if an actor already exists
 * with the same id, its transform is updated and the existing
 * actor is returned (OutWasExisting=true). The caller's BP class
 * must match the existing actor's class — a mismatch is a hard
 * error so a typo doesn't silently change a robot's geometry.
 * Empty ActorId always creates a new actor.
 */
URLABEDITOR_API bool SpawnActorSync(
	const FString& BlueprintNameOrPath,
	const FString& ActorId,
	const FVector& LocationMeters,
	const FQuat& RotationQuatXyzw,
	const FVector& Scale,
	FString& OutActorName,
	FString& OutActorPath,
	FString& OutBlueprintClassPath,
	bool& OutWasExisting,
	FString& OutError);

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
 *  whether nav data (a RecastNavMesh) exists in the editor world.
 *  Caveat: this is an existence check (a RecastNavMesh instance exists and
 *  no build is running), not a guarantee of non-empty tiles — correct today
 *  only because the editor Build() call is synchronous. */
URLABEDITOR_API bool IsNavBuildDone(bool& bOutNavDataPresent);

/** Destroy a spawned actor by actor id (preferred) or actor name. */
URLABEDITOR_API bool DestroyActorSync(
	const FString& ActorIdOrActorName,
	FString& OutError);

/**
 * Edit-time SetActorTransform. Either ``LocationMeters`` or
 * ``RotationQuatXyzw`` may be provided; both nullable so callers can
 * patch one without touching the other. Doesn't trigger a recompile —
 * the change applies on the next PIE start.
 */
URLABEDITOR_API bool SetActorTransformSync(
	const FString& ActorIdOrActorName,
	const FVector* LocationMeters,
	const FQuat* RotationQuatXyzw,
	FString& OutActorName,
	FString& OutError);

/**
 * Spawn a light into the editor's current level. Kind is one of
 * ``"directional" | "point" | "spot"`` (case-insensitive). Pose is
 * in MJ-native metres / degrees; intensity and color use UE-native
 * conventions (intensity in cd / lumens depending on light kind, RGB
 * in [0,1]). ActorId is recorded as the actor tag
 * ``URLab.ActorId=<id>`` since light actors don't derive from
 * AMjArticulation.
 */
URLABEDITOR_API bool SpawnLightSync(
	const FString& Kind,
	const FString& ActorId,
	const FVector& LocationMeters,
	const FVector& RotationEulerDegrees,
	float Intensity,
	const FLinearColor& Color,
	FString& OutActorName,
	FString& OutActorPath,
	FString& OutResolvedKind,
	FString& OutError);

/** Enumerate all actors in the editor world. Each entry is a JSON
 *  object with `name`, `class`, `location` (MJ metres), `rotation_quat`
 *  (xyzw), `actor_id`, `is_articulation`, `has_quick_convert`, and
 *  (when present) the quick-convert summary (`static`, `complex_mesh`,
 *  `driven_by_unreal`). Skips engine transient / brush / hidden default
 *  actors so the outliner stays focused on user-placed scene content. */
URLABEDITOR_API bool ListActorsSync(
	TArray<TSharedPtr<FJsonValue>>& OutActors,
	FString& OutError);

/** Select an actor in the editor's outliner / viewport so the user
 *  can see what the bridge is talking about. Actor id preferred;
 *  raw actor name accepted as fallback. */
URLABEDITOR_API bool SelectActorSync(
	const FString& ActorIdOrActorName,
	FString& OutActorName,
	FString& OutError);

/** Add a UMjQuickConvertComponent to the named actor and configure
 *  its core fields. Idempotent — if the actor already has a
 *  QuickConvert component, the existing one is reconfigured rather
 *  than a second one added. Returns the configured property values
 *  through the out parameters so the caller can echo them back to
 *  the bridge UI. */
URLABEDITOR_API bool AddQuickConvertSync(
	const FString& ActorIdOrActorName,
	bool bStatic,
	bool bComplexMesh,
	float CoACDThreshold,
	bool bDrivenByUnreal,
	const FVector& Friction,
	FString& OutActorName,
	FString& OutError);

/** Strip the UMjQuickConvertComponent off an actor (if present). */
URLABEDITOR_API bool RemoveQuickConvertSync(
	const FString& ActorIdOrActorName,
	FString& OutActorName,
	FString& OutError);

/** Enumerate previously-imported MJCF blueprints under
 *  ``/Game/MuJoCoImports`` so the bridge UI can populate a
 *  spawn-actor dropdown without forcing the user to remember each
 *  BP path. Each entry: ``blueprint_class_path`` (the explicit
 *  ``..._C`` class path for spawn_actor) and ``short_name``. */
URLABEDITOR_API bool ListBlueprintsSync(
	TArray<TSharedPtr<FJsonValue>>& OutBlueprints,
	FString& OutError);

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
} // namespace URLabLevelOps
