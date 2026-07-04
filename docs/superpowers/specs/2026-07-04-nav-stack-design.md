# Mobile Base Navigation Stack — Design

**Date:** 2026-07-04
**Branch:** `feat/nav-stack` (off `main`; must not touch the IK work on `feat/urlabmink-live-ik`)
**Status:** Approved in brainstorming; verified against code before write-up.

## Goal

A mobile manipulation platform (tidybot-style: planar holonomic base + arm) whose
**base autonomously drives from A to B** on the floor to a commanded 2D goal,
avoiding static obstacles. Planning uses UE's Recast navmesh; actuation goes
through MuJoCo actuators (MuJoCo remains physics-authoritative; UE is the
visualizer). No manipulation in this phase — base navigation only.

Decisions locked during brainstorming:
- **Success criterion:** drive-to-goal-point with static obstacle avoidance.
- **Planner:** in-engine UE Recast navmesh + synchronous path query (no
  AIController needed for v1).
- **Motion style:** holonomic pure pursuit — strafe toward the lookahead point,
  yaw follows the direction of travel.
- **Goal sources:** `set_nav_goal` bridge op (primary, scriptable) + a
  BlueprintCallable `SetNavGoal(FVector)` (the op calls it).
- **Architecture (Approach A):** the existing thread-safe twist in
  `UMjTwistController` is the shared bus. Nav is "an automated twist producer";
  a new generic base-drive controller is the single twist consumer. WASD,
  gamepad, `set_twist`, and nav all drive through the same primitive.
- **Environment/navmesh source (revised after code verification):** UE-native
  floor/wall/obstacle actors with `UMjQuickConvertComponent (Static=true)` —
  they exist identically in UE (Recast bakes real blocking collision for free)
  and in MuJoCo (QuickConvert generates static geoms). Single source of truth,
  zero import-pipeline changes. The original "opt imported MJCF geoms into
  navigation" idea is **not viable for floors**: `UMjPlane` creates no UE
  geometry at all (`MjPlane.cpp` is pure spec import/export), so an MJCF plane
  floor can never provide the walkable surface.

## Non-goals

- Dynamic obstacles / periodic replanning (static world; replan only on new goal).
- Nav + mink-IK running simultaneously on one articulation. An articulation
  binds exactly **one** `UMjArticulationController`
  (`FindComponentByClass`, `MjArticulation.cpp:654`), so base-drive and IK
  controllers are mutually exclusive today. Combining them is future work that
  touches the IK controller (owned by another workstream).
- Stuck-recovery behaviors (v1 has a watchdog that aborts, nothing cleverer).
- `scene.ensure_navmesh` editor op for fully scripted level assembly (deferrable
  nice-to-have; v1 places the NavMeshBoundsVolume in the editor by hand).
- Arm control of any kind (arm actuators are passed through untouched).

## Facts verified against the code (constraints the design must respect)

1. **A bound controller replaces the entire default ctrl path.**
   `AMjArticulation::ApplyControls` (`MjArticulation.cpp:758-763`) calls
   `ComputeAndApply` and returns; the default loop feeding every actuator from
   `ResolveDesiredControl` never runs. Any controller that only manages a subset
   of actuators **must pass through the rest itself** or they freeze.
2. **Twist bus semantics are already fixed** and geometry_msgs/Twist-aligned:
   robot-frame `(vx fwd, vy, yaw_rate)` in m/s and rad/s
   (`RpcDispatcher.cpp:2570` comment, ZMQ broadcast, WASD handlers). Nav must
   produce exactly this convention.
3. **Game-thread pose reads must use the UE component transform** (synced from
   render snapshots by the manager pump). `UMjBody::GetWorldPosition()`
   dereferences live `mjData` (`MjBody.cpp:360`) and races the physics thread.
4. **Physics-thread state reads are safe inside `ComputeAndApply`** (it runs
   under the step loop; reading `d->qpos` there is the established pattern).
5. **Tidybot base chain** is world → `joint_x` (slide) → `joint_y` (slide) →
   `joint_th` (hinge): slides live in the **world** frame; twist arrives in the
   **robot** frame — the servo must rotate by live yaw.
6. **`URLab.Build.cs` has no `NavigationSystem` dependency** — it must be added.
7. **QuickConvert supports static bodies**: `Static=true` ⇒ "no free joint,
   cannot move under physics forces. Use for fixed obstacles."
   (`MjQuickConvertComponent.h:88-91`.)
8. **Coordinate conventions** (from `MjUtils.cpp:31-36`): MuJoCo RH/Z-up/metres,
   UE LH/Z-up/cm; map X→X, Y→−Y, Z→Z, ×100.

## Components

Four isolated units. New code lives under `Source/URLab/{Public,Private}/MuJoCo/`
following existing layout. Nothing under `URLabMink` or the mink IK controller
files is touched.

### 1. `UMjBaseDriveController : UMjArticulationController` (new, physics thread)

`.../Components/Controllers/MjBaseDriveController.{h,cpp}`

The reusable twist→actuator primitive (fills the gap that today is closed
off-board in Python). Consumes the sibling `UMjTwistController`'s twist and
drives the three base actuators; passes everything else through.

Properties:
- `TArray<FString> BaseJointNames` — default `{"joint_x","joint_y","joint_th"}`;
  resolved against the compiled model by suffix (same `ResolveByName`
  exact/`_name`/`/name` pattern as the mink controller) to tolerate import
  prefixes.
- `EMjBaseDriveActuatorMode ActuatorMode` — `PositionIntegrate` (default;
  position actuators, integrate a target) | `VelocityDirect`
  (`d->ctrl = v` for velocity-actuator bases).
- `float MaxLeashLinear` (default 0.05 m), `float MaxLeashAngular`
  (default 0.1 rad) — anti-windup clamp, see below.
- `float ReseedThreshold` (default 0.25 m / rad) — target re-seed trigger.

`Bind(m, d, ActuatorIdMap)`: `Super::Bind` fills `Bindings`; resolve the three
base joints → their actuator bindings + qpos addresses; seed
`Target[i] = d->qpos[addr]`. Missing base joint/actuator ⇒ log error, controller
stays disabled for the base (pass-through still runs).

`ComputeAndApply(m, d, Source)` each physics sub-step:

```
θ  = d->qpos[qposadr(joint_th)]                    // live yaw, MuJoCo frame
(vx, vy, ω) = Twist->GetTwist()                    // robot frame (bus semantics)
vx_w = cos θ · vx − sin θ · vy                     // rotate into world frame
vy_w = sin θ · vx + cos θ · vy
dt = m->opt.timestep

// PositionIntegrate mode, per base DOF (x, y, th):
Target[i] += v_w[i] · dt
Target[i]  = clamp(Target[i], qpos[i] − MaxLeash, qpos[i] + MaxLeash)   // anti-windup
if |Target[i] − qpos[i]| > ReseedThreshold: Target[i] = qpos[i]         // reset/teleport
if joint limited: Target[i] = clamp(Target[i], jnt_range)
d->ctrl[baseAct[i]] = Target[i]

// Pass-through for every non-base binding (constraint #1):
d->ctrl[B.ActuatorMjID] = B.Component->ResolveDesiredControl(Source)
```

Notes:
- Anti-windup leash: when the robot is physically blocked, the integrated
  target cannot run away, so position error — and requested force — stays
  bounded, and the base stops pushing shortly after the twist zeroes.
- Yaw: unlimited MuJoCo hinges don't wrap qpos and the target integrates
  continuously — no wrap handling needed.
- The twist source is found once at `BeginPlay` (`GetOwner()->
  FindComponentByClass<UMjTwistController>()`); absent twist controller ⇒
  base DOFs hold their seeded targets (robot stands still), pass-through
  unaffected.
- `GetTwist()` takes a lock; it is a 3-float copy under an uncontended
  `FCriticalSection`, called at physics rate — same pattern the ZMQ broadcast
  already uses from the physics thread.

### 2. `UMjNavComponent : UActorComponent` (new, game thread)

`.../Navigation/MjNavComponent.{h,cpp}`

The planner/follower. Owns the goal, queries the navmesh, runs holonomic pure
pursuit each `TickComponent`, writes the result to the twist bus. Knows nothing
about actuators or MuJoCo internals.

Properties (UE units):
- `FString BaseBodyName` — base `UMjBody` for pose; empty ⇒ articulation root
  body; suffix-resolved.
- `float MaxSpeed` (default 0.6 m/s), `float MaxYawRate` (default 1.2 rad/s).
- `float LookaheadDist` (default 60 cm), `float AcceptanceRadius` (default
  15 cm), `float DecelRadius` (default 75 cm).
- `float YawGain` (default 2.0), `float MinSpeedForHeading` (default 0.05 m/s —
  below this, hold yaw; don't chase atan2 noise).
- `float StuckTimeout` (default 5 s), `float MinProgress` (default 5 cm).

API:
- `UFUNCTION(BlueprintCallable) bool SetNavGoal(FVector WorldGoal)` — project
  goal to navmesh (`ProjectPointToNavigation`, generous Z extent); off-mesh ⇒
  return false + log. On-mesh ⇒ synchronous path query
  (`UNavigationSystemV1::FindPathToLocationSynchronously(World, BasePos, Goal)`),
  store polyline, state → `Navigating`.
- `UFUNCTION(BlueprintCallable) void ClearNavGoal()` — zero twist, state → `Idle`.
- `GetNavState()` → `EMjNavState { Idle, Navigating, Arrived, Failed }`;
  `GetDistanceToGoal()`.
- `UPROPERTY(BlueprintAssignable) OnNavGoalReached`, `OnNavGoalFailed`
  (dynamic multicast delegates).

`TickComponent` while `Navigating` (all math in UE space; convert once at the
bus boundary):
1. Base pose from the resolved `UMjBody`'s **component transform** (constraint #3).
2. Closest point on polyline → walk `LookaheadDist` ahead → lookahead point.
3. Desired world velocity = direction · `MaxSpeed` ·
   `min(1, dist_to_goal / DecelRadius)`.
4. Desired yaw = direction of travel (only above `MinSpeedForHeading`);
   `yaw_rate = clamp(YawGain · shortest_angle_error, ±MaxYawRate)`.
5. Rotate world velocity into robot frame (base yaw), then convert UE →
   bus/ROS convention in one place:
   `vx = fwd_cm/100`, `vy = −right_cm/100`, `ω = −ue_yaw_rate`
   (UE LH +yaw is CW from above; bus +yaw is CCW). **This is the single most
   bug-prone line in the stack — it gets a dedicated unit test and a
   debug-draw arrow.**
6. `Twist->SetTwist(vx, vy, ω)`.

Arrival & failure:
- `dist_to_goal < AcceptanceRadius` ⇒ zero twist, `Arrived`, fire
  `OnNavGoalReached`.
- Watchdog: if best-distance-to-goal hasn't improved by `MinProgress` within
  `StuckTimeout` ⇒ zero twist, `Failed`, fire `OnNavGoalFailed`.
- `ClearNavGoal`, `EndPlay`, component deactivation ⇒ zero twist. **Never leave
  a stale twist on the bus.**

Pure-pursuit math lives in a free-function namespace `MjNavPursuit`
(`.../Navigation/MjNavPursuit.{h,cpp}`) operating on plain structs
(`FMjPursuitInput` → `FMjPursuitOutput`) — unit-testable with no world, no sim
(mirrors how the mink math is tested).

### 3. Bridge ops (runtime namespace)

Registered in `RegisterDispatcherOps` (`RpcDispatcher.cpp`) via the existing
`Reg(...)` pattern:

- `set_nav_goal` — required: `articulation`, `x`, `y` (MuJoCo metres; world
  frame). Handler: resolve articulation → its `UMjNavComponent` (error
  `no_nav_component`), convert MuJoCo→UE (×100, Y-flip; Z from the base body's
  current height), marshal to the **game thread** (nav queries are
  game-thread-only — same marshalling pattern the editor ops use), call
  `SetNavGoal`. Reply: `set_nav_goal_ok` + `accepted:bool` (false = off-mesh).
- `get_nav_status` — required: `articulation`. Reply: `state:string`
  (`idle|navigating|arrived|failed`), `distance_to_goal:number` (metres). Lets a
  script implement `wait_until_arrived`.

### 4. Demo scene + navmesh

- Level: UE-native floor plane + a few wall/box obstacle actors, each with
  `UMjQuickConvertComponent (Static=true)` — present in both worlds by
  construction. Robot: imported tidybot MJCF (external, from
  `mujoco_menagerie/stanford_tidybot` per `docs/mink-ik-mobile-tidybot.md`),
  with `UMjTwistController` + `UMjBaseDriveController` + `UMjNavComponent`
  added, plus one `MjManager`.
- `NavMeshBoundsVolume` over the floor; RecastNavMesh agent radius ≈ 35 cm
  (tidybot footprint). Editor auto-bake covers PIE; no runtime generation
  needed for v1.
- `URLab.Build.cs`: add `"NavigationSystem"` to the dependency list
  (constraint #6).

## Data flow

```
set_nav_goal (bridge)  /  SetNavGoal() (BP)
        │  game thread
        ▼
UMjNavComponent ── navmesh path (Recast) ── pure pursuit vs base UE transform
        │  robot-frame twist (bus convention: m/s, rad/s, CCW yaw)
        ▼
UMjTwistController.SetTwist(vx, vy, ω)        ← thread-safe bus
        │                                        (also fed by WASD / set_twist)
        ▼  physics thread, per sub-step
UMjBaseDriveController.ComputeAndApply
   rotate by live yaw → integrate + leash → d->ctrl[joint_x/y/th]
   pass-through d->ctrl for all other bindings
        ▼
mj_step → robot moves → render snapshot → pump → UE transforms update
        └──────────────► (loop: nav component sees the new base pose)
```

Planning/pursuit is game-thread in UE space (where the navmesh lives); the
tight servo is physics-thread in MuJoCo space (where the sim lives); the
existing thread-safe twist is the only thing crossing the boundary.

## Error handling summary

| Failure | Behavior |
|---|---|
| Base joint/actuator not found at Bind | Log error; base drive disabled; pass-through still runs |
| No `UMjTwistController` sibling | Base holds seeded target (stands still); logged once |
| Goal off-navmesh | `SetNavGoal` returns false; op replies `accepted:false`; no state change |
| Path query fails | Same as off-mesh |
| No progress (blocked) | Watchdog: zero twist, `Failed`, `OnNavGoalFailed` |
| Keyframe reset / teleport mid-drive | Servo re-seeds targets (ReseedThreshold); nav replans naturally from new pose on next goal, or watchdog fires |
| Robot physically blocked | Anti-windup leash bounds position error and force |
| Component shutdown mid-drive | Zero twist on the bus |

## Testing

1. **Unit — `MjNavPursuit`** (automation test, no world): lookahead point
   selection on a polyline, decel scaling near goal, shortest-angle yaw error,
   and golden-value tests for the UE↔bus frame conversion (the step-5 line).
2. **Automation — servo** (existing inline-MJCF pattern from
   `MjImportTests.cpp`): compile a minimal planar-base model (2 slides + hinge +
   position actuators), bind `UMjBaseDriveController`, inject a twist, step N
   times → assert: base displaces in the commanded direction; robot-frame twist
   with nonzero yaw produces the correctly rotated world motion; a wall geom in
   the path bounds position error (leash works); non-base actuator values pass
   through.
3. **End-to-end (scripted/manual):** demo level; from Python `set_nav_goal`
   behind an obstacle; robot detours around it; `get_nav_status` reaches
   `arrived`; WASD still drives the base when nav is idle (bus reuse proof).

## Future work (explicitly deferred)

- Nav + IK simultaneously (requires multi-controller or controller-composition
  work on the articulation — coordinate with the IK workstream).
- Dynamic obstacles: runtime navmesh rebuild + periodic replan.
- `scene.ensure_navmesh` editor op for fully scripted level assembly.
- Opting imported MJCF box/mesh obstacle geoms into navigation (useful for
  MJCF-authored scenes; never sufficient for plane floors).
- Smarter recovery (backup-and-retry) on watchdog failure.
