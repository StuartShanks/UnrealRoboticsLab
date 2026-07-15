# Nav Demo Bridge Ops — Design

**Date:** 2026-07-15
**Branch:** `feat/nav-stack` (based on the ikmink tip — IK + nav share one lineage)
**Goal:** make the mobile-base navigation demo fully scriptable over the bridge, then
prove nav works live with a reproducible Python demo (`tidybot_nav_demo.py`).

## Why

The nav stack (spec `2026-07-04-nav-stack-design.md`) is implemented and its automation
suite is 19/19 green, but the live tier of its validation checklist — demo level, E2E
drive around an obstacle, off-mesh rejection — has never run. The editor-op surface can
author almost everything (`create_level`, `spawn_actor`, `add_quick_convert`,
`begin_pie`, `import_xml`, `ensure_manager`), and the mink IK demo
(`Scripts/demos/tidybot_mink_demo.py`) proves the live-editor orchestration pattern.
Three gaps block a scripted nav demo; this design fills them with three typed editor
ops, following the `add_controller` precedent.

## Verified code facts (this design relies on)

1. `spawn_actor` resolves only Blueprint class paths (`ResolveBpClassPath` +
   `LoadObject<UClass>`, `MjLevelOps.cpp`) — it cannot spawn an engine
   `AStaticMeshActor`, hence `spawn_box`.
2. Imported MJCF geoms are query/overlap-only and `UMjPlane` creates no UE geometry —
   imported scenes can never bake a navmesh; UE-side blocking geometry is required.
3. `add_quick_convert` already supports `static: true` (fixed MuJoCo geom, no free
   joint) — obstacles need no new MuJoCo-side plumbing.
4. `add_controller` (`MjEditorOpHandlers.cpp:1653`) is the template for
   articulation-targeted attach ops: `ResolveActorKey` → key-or-sole-articulation
   resolution, idempotent attach via `FindComponentByClass` + `NewObject` +
   `AddInstanceComponent` + `RegisterComponent`, non-fatal `warnings` array.
5. The nav components are PIE-duplication-safe by construction: `UMjBaseDriveController`
   binds base joints by name (`BaseJointNames`), `UMjNavComponent` finds the twist
   controller by class at runtime, `UMjTwistController` is stateless atomics. No
   `TObjectPtr` refs to lose (the failure mode that bit the IK demo's `add_controller`
   in PIE).
6. The Python client needs no changes: `namespaces/base.py::__getattr__` synthesises
   RPC callables, so `client.scene.spawn_box(...)` etc. work the moment the ops exist.
7. UE default nav agent radius is 34 cm ≈ tidybot footprint (35 cm) — no
   project-settings changes needed.
8. The dispatcher enforces sessions (`ValidateSession`); tests use
   `SetActiveSessionIdForTest` + a `session_id` field (the `MjStepServerTests` idiom,
   now also used by `MjNavOpsTests`).

## Op 1: `spawn_box`

Editor-only op. Spawns an `AStaticMeshActor` with the engine cube mesh
(`/Engine/BasicShapes/Cube`), **static** mobility, default **blocking** collision
(bakes navmesh; carves holes around obstacles). UE-side only — for obstacles the
caller follows up with `add_quick_convert(static=true)`; a floor needs no MuJoCo side
(the MJCF scene provides the physics plane).

| Field | Type | Notes |
|---|---|---|
| `actor_id` | string, required | idempotency key, same semantics as `spawn_actor` |
| `location` | [3] number, required | MuJoCo metres, world frame (X, −Y, Z → ×100 cm) |
| `size` | [3] number, required | **full extents** in metres (not half-extents) |
| `yaw_deg` | number, optional | rotation about Z, MuJoCo CCW-positive |

Reply `spawn_box_ok`: `actor_id`, `actor_name`, `actor_path`, `was_existing`.
Errors: `missing_field`, `spawn_failed`.
Implementation: `URLabLevelOps::SpawnBoxSync` (MjLevelOps.cpp) + thin handler; engine
cube is 100 cm, so component scale = size_m × 100 / 100 = size_m. Assign the shared
`BasicShapeMaterial` so it renders.

## Op 2: `spawn_nav_bounds`

Editor-only op. Spawns an `ANavMeshBoundsVolume` with a real cube brush via the
engine's own path (`UActorFactory::CreateBrushForVolumeActor` + `UCubeBuilder` — what
the editor Place tool runs), notifies the nav system
(`UNavigationSystemV1::OnNavigationBoundsUpdated`), triggers a build, and waits up to
`timeout_s` for `IsNavigationBuildInProgress()` to clear. The wait MUST use the same
pattern as `HandleBeginPie` (which polls up to 30 s inside an editor op without wedging
the editor) — the implementation task starts by reading that handler and copying its
wait mechanics.

| Field | Type | Notes |
|---|---|---|
| `center` | [3] number, required | MuJoCo metres |
| `extent` | [3] number, required | half-extents, metres (volume spans center ± extent) |
| `timeout_s` | number, optional | default 10 |

Reply `spawn_nav_bounds_ok`: `actor_name`, `nav_data_present` (bool — a
`ARecastNavMesh` exists and build finished), `build_seconds`.
Errors: `missing_field`, `spawn_failed`, `nav_build_timeout` (volume kept; caller may
retry/poll by re-issuing with the same effect — the op is idempotent per level: reuse
an existing `NavMeshBoundsVolume` spawned by a prior call rather than stacking
volumes; identify ours by actor label `URLabNavBounds`).

## Op 3: `add_nav_stack`

Editor-only op, `add_controller`'s sibling. Resolves the articulation (same
key-or-sole logic), idempotently attaches three instance components —
`UMjTwistController`, `UMjBaseDriveController`, `UMjNavComponent` — and applies
optional tuning. Attach-or-reuse per component, independently.

| Field | Applies to | Notes |
|---|---|---|
| `target` | — | actor key; optional if exactly one articulation |
| `base_joints` | BaseDrive | [3] strings, default `joint_x`/`joint_y`/`joint_th`; unmatched names → warning |
| `actuator_mode` | BaseDrive | `"position_integrate"` (default) or `"velocity_direct"` |
| `max_speed`, `max_yaw_rate` | Nav | m/s, rad/s |
| `lookahead`, `acceptance_radius`, `decel_radius` | Nav | metres (converted to cm internally) |
| `stuck_timeout` | Nav | seconds |
| `debug_draw` | Nav | bool, draws path + lookahead |

Reply `add_nav_stack_ok`: `created` [names], `existing` [names], `warnings` [].
Errors: `unknown_articulation`, `no_articulation`, `ambiguous`, `attach_failed`.
No config-lock needed: components are only mutated pre-PIE (unlike the live-solving
IK controller); binding is name/class-based at Bind (fact 5).

## Demo script: `Scripts/demos/tidybot_nav_demo.py`

Mirrors `tidybot_mink_demo.py` conventions (bridge venv, `urlab_client`, phase logs,
nonzero exit on any acceptance failure). Prereq: editor open, bridge listening.

1. **Scene** — `create_level`; floor `spawn_box` 20×20×0.2 m, top at z=0 (UE-only);
   three obstacle boxes between spawn and goal, each + `add_quick_convert(static=true)`;
   `spawn_nav_bounds` covering the floor; **assert `nav_data_present`**.
2. **Robot** — `import_xml` tidybot scene, `spawn_actor` at origin, `ensure_manager`,
   `add_nav_stack` (defaults; `debug_draw=true`).
3. **PIE** — `begin_pie`, **live mode** (pursuit runs on game-thread tick; direct-mode
   step batches don't tick the world continuously).
4. **Happy path** — `set_nav_goal` at a point whose straight line crosses an obstacle;
   poll `get_nav_status` (0.5 s) until `arrived`/`failed`/timeout. Accept: state
   `arrived`, final base qpos within `acceptance_radius` of the goal, and the robot's
   sampled track deviated from the straight line (it actually detoured).
5. **Off-mesh reject** — `set_nav_goal` at a point inside an obstacle: accept
   `accepted == false`.
6. Teardown: `stop_pie` (keep level for inspection with `--keep-open`).

The stuck/wedge failure path stays covered by `URLab.Nav.Component.WatchdogFails`
(automation) — user decision 2026-07-15.

## Testing

Automation (extend the `URLab.Nav.Ops` family, session idiom per fact 8):
- `spawn_box`: actor exists with static mobility, blocking collision, cube mesh, and
  `CanEverAffectNavigation`; idempotent per `actor_id`.
- `spawn_nav_bounds`: in the editor test world, nav data present after the op; volume
  bounds match center/extent; second call reuses the volume.
- `add_nav_stack`: all three components attached with params applied (spot-check
  `MaxSpeed`, `BaseJointNames`, `ActuatorMode`); re-call → `existing` not duplicates;
  bad joint name → warning, not failure.

Live tier = the demo script itself, run in the user's editor session.
Regression gate: existing `URLab.Nav` 19/19 stays green; compile via the standard
`Build.sh TestEditor` command.

## Non-goals

- Generic component/actor reflection ops (rejected Approach B).
- Wedge/stuck live test (covered by automation).
- Combining base-nav with arm-IK on one robot — that is the **next** feature
  (mobile manipulation), enabled by this demo infrastructure but out of scope here.
