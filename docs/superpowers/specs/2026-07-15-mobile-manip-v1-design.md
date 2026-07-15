# Mobile Manipulation v1 — Sequential Drive-then-Reach — Design

**Date:** 2026-07-15
**Branch:** `feat/nav-stack` (pushed to `fork`)
**Goal:** one tidybot that navigates around obstacles to a staging pose, then
reaches an end-effector target near a table — the first nav + IK composition.

## Scope decisions (user, 2026-07-15)

- **Behavior:** SEQUENTIAL — nav drives the base (arm holds home), then control
  switches to the mink IK for the reach. Simultaneous drive+reach and
  whole-body-QP nav are explicitly v2+ (see Non-goals).
- **Orchestration:** bridge script. The engine gains exactly ONE new primitive
  (`set_active_controller`); all sequencing lives in Python.
- **Acceptance:** reach a pose (mm/cm-level EE tracking, mink-demo style). No
  grasping in v1.
- **Sim modes:** live for the drive phase (validated 2026-07-15), DIRECT for the
  reach phase (the proven `tidybot_mink_demo.py` pattern — client streams
  targets and steps, reads synced qpos for FK).

## Verified code facts (this design relies on)

1. `UMjMinkIKController` is already whole-body: its drive joints include
   `joint_x/_y/_th` + the 7 arm joints, and a base Damping cost ("lazy base",
   `19c8d77`) makes the QP arm-first, recruiting the base only when the target
   is out of reach. `fix_base`-style task toggling works live (`eb2f25a`).
2. `AMjArticulation::AdoptRuntimeController(UMjArticulationController*)`
   (MjArticulation.h:524) already implements a safe runtime controller swap:
   it Binds the controller fully while invisible to the physics thread, then
   publishes `CachedController` behind a release fence paired with the acquire
   fence in `ApplyControls`.
3. An articulation binds exactly ONE controller (`CachedController`); with two
   attached, PIE's `PostSetup` binds whichever `FindComponentByClass` returns
   first — order is not a contract. The script must select explicitly.
4. Handoff hygiene already exists: `UMjNavComponent` zeroes the twist on
   arrival (`StopWithState`); the IK re-bases its open-loop reference at first
   integration (`1244df9` — no arm yank after Bind); `UMjBaseDriveController`
   re-seeds targets from live qpos on (re-)Bind.
5. `add_controller` (IK) and `add_nav_stack` (nav trio) both attach instance
   components pre-PIE and are PIE/Simulate-duplication-safe (`e035011`
   name-fallback; the nav components are name/class-resolved by construction).
6. Nav requires level isolation — one robot per level (duplicate
   `joint_x/_y/_th` otherwise; see navigation-demo.md warning) — so the demo
   authors its own fresh level, same as `tidybot_nav_demo.py`.
7. Direct mode + a bound controller is a proven combination (the entire mink
   demo ran that way, ctrl-parity green).

## The one new primitive: `set_active_controller` (runtime op)

Registered in `RpcDispatcher.cpp` next to `set_nav_goal` (ManagerRequired,
`runtime` namespace — the Python client synthesises
`client.runtime.set_active_controller(...)`, no client changes).

| Field | Type | Notes |
|---|---|---|
| `articulation` | string, required | UE actor name (same resolution as `set_nav_goal`) |
| `controller` | string, required | matched case-insensitively as a substring of the attached `UMjArticulationController` subclasses' class names — `"base_drive"` → `UMjBaseDriveController`, `"mink_ik"` → `UMjMinkIKController` |

Handler: marshal to the game thread (the `HandleSetNavGoal` idiom —
`IsInGameThread` fast-path + `AsyncTask` with by-value `TWeakObjectPtr` /
thread-safe `TSharedPtr` result / `FEvent`), iterate the actor's
`UMjArticulationController` components, pick the first whose class name
contains the token (case-insensitive), call `AdoptRuntimeController` on it.

Reply `set_active_controller_ok`: `active` (resolved class name, e.g.
`"MjMinkIKController"`), `was_active: bool` (already the bound controller —
idempotent no-op, `AdoptRuntimeController` is not re-invoked).
Errors: `unknown_articulation`, `unknown_controller` (no attached component
matches; message lists the attached controller class names), `ambiguous`
(token matches more than one attached controller class), `not_ready`.

The op does NOT guard against switching mid-navigation — sequencing is the
script's job. (Nav keeps ticking on the game thread if left Navigating, but
its twist lands on an unbound base-drive, harmless; the demo clears/awaits
`arrived` before switching anyway.)

## Demo: `Scripts/demos/tidybot_mobile_manip_demo.py` (new file)

`tidybot_nav_demo.py` stays untouched (its own acceptance keeps running).
The new script reuses its scene-authoring idioms:

1. **Scene** — `create_level("MobileManipDemo")`; floor 20×20 (UE-only);
   the wall + two blocks (same layout, forces the detour); a **table**:
   `spawn_box(actor_id="manip_table", ...)` + `add_quick_convert(static=True)`
   placed past the staging pose (e.g. table centered ~x=5.0, staging ~x=4.2);
   `spawn_nav_bounds(agent_radius=55)`.
2. **Robot** — `import_xml` tidybot, `spawn_actor(actor_id="mm_tidybot")`,
   `ensure_manager`, `add_nav_stack(...)`, then `add_controller(...)` with the
   mink spec copied from `tidybot_mink_demo.py` (frame `pinch_site`, 10 drive
   joints, posture + lazy-base damping tasks).
3. **Phase A: drive (live)** — `sim.start` → `set_mode("live")` →
   `set_paused(False)` → `set_active_controller("base_drive")` →
   `set_nav_goal(staging)` → poll `get_nav_status` to `arrived` (assert the
   detour like the nav demo).
4. **Switch** — `set_active_controller("mink_ik")`; assert reply
   `active == "MjMinkIKController"`.
5. **Phase B: reach (direct)** — `set_mode("direct")` → seed the first target
   at the current `pinch_site` pose (mink-demo idiom), then step-batch toward
   a target above the table surface; accept when EE-to-target error < 2 cm
   sustained over the final batches (exact tolerances/timings set in the plan
   from the mink demo's numbers). The lazy-base stays enabled: the base MAY
   legally shuffle if the target is at reach margin — that is whole-body
   behavior, not a failure.
6. Nonzero exit on any phase failure; `--keep-open` leaves the session up.

Editor workflow per [[collaborative-dev-nav]]: the user opens the editor and
presses Simulate; the script attaches to the running session (`sim.start`'s
already-ready path). Build/test happens with the editor closed.

## Error handling

House style throughout. New failure modes: `unknown_controller` / `ambiguous`
(op); demo fails fast with a labelled phase message if the switch reply's
`active` is not the requested controller, or if direct-mode stepping stalls
(the mink demo's step-timeout idiom).

## Testing

- **Automation** (`URLab.Nav.Ops.SetActiveController`, new test in
  `MjNavDemoOpsTests.cpp`): on the `FMjUESession` rig attach
  `UMjBaseDriveController` + `UMjPassthroughController` (NOT MinkIK — the
  one-joint rig can't bind it meaningfully), dispatch the op both ways,
  assert: the bound controller repoints (observable via the articulation's
  bound-controller class, or ctrl-path behavior), `was_active` on a repeat
  call, `unknown_controller` on a bogus token, `ambiguous` if applicable.
  Session idiom (`SetActiveSessionIdForTest` + `session_id`) as established.
- **Regression:** existing `URLab.Nav` (22) + `URLab.MinkIK`/`URLab.Mink`
  suites stay green — the op adds code, changes none.
- **E2E:** the demo, run collaboratively (user Simulates, controller drives
  phases, both watch phase A live; phase B asserted numerically).

## Non-goals (v2+)

- Simultaneous drive+reach (true multi-controller composition over disjoint
  actuator sets — the `ApplyControls` partitioning approach, rejected for v1).
- Whole-body nav-in-the-QP (navmesh path as a mink task).
- Grasping (gripper contact/friction/lift) and any object interaction.
- An in-engine mission/sequencing component (Blueprint-visible autonomy).
- Switching controllers automatically on nav events.
