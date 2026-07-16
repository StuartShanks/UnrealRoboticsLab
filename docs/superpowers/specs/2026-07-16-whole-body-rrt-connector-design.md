# Whole-Body RRT-Connect Free-Space Connector — Design

**Date:** 2026-07-16
**Branch:** feat/urlabmink-live-ik (or successor)
**Status:** Approved design, pending implementation plan

## Problem

Nothing in the stack takes the arm (and base) from an arbitrary configuration to a
pre-grasp pose through free space, collision-free. The straight-line Cartesian
carpet + greedy whole-body QP wedges at reach limits and drifts the base — the
live-measured instance is the suction-pick reach plateau: with the base parked at
nav staging, the cup asymptotes ~2.5 cm above the box top regardless of target
height, and when the base is left free the QP wanders it off-center in y and
sweeps the box. This is the "free-space connector" gap: the QP is a *tracker*,
not a *planner*.

The fix is a sampling-based planner that proposes a geometrically feasible
whole-body path, executed by the existing QP, with the existing collision guards
active as the insurance layer: **planner proposes / QP disposes / guards insure**.

## Goals

- Whole-body (10-DoF: 3 holonomic base + 7 arm) RRT-Connect connector planning,
  client-side, over the bridge's compiled-model replica. No planning in the engine.
- One small engine addition: a `posture_target` wire param so plans execute in
  joint space (the planned base path is *executed*, not rediscovered by the QP).
- First validation gate: the suction-pick scene. Success = `GRIPPED+LIFTED`
  (box rises > 5 cm with the cup). This simultaneously finishes suction pick v1.

## Non-Goals (v1)

- Held-object planning (post-grip retreat is a straight vertical lift; planning
  with the grasped object as part of the robot is a later milestone).
- Opening-phase machinery: trajectory-IK ribbon along handle paths, constrained
  planning, asset affordance schema formalization. Cupboard milestone.
- MPPI anywhere in this loop (roadmap keeps it as a base twist producer, later).
- Fixing the direct-mode `client.step()` NetworkValue-zeroing engine bug. v1
  plans and syncs **pre-suction only**; the bug must be fixed before held-object
  planning ever syncs mid-grip (recorded as a follow-up, not part of this spec).
- Promoting the planner into the URLab_Bridge client library. It starts in this
  repo's `Scripts/demos/`; promotion happens after the API stabilizes.

## Prior decisions (locked during brainstorm)

1. **Execution path: `posture_target` wire** (not FK-projected EE carpet, not a
   split EE-carpet + twist-bus hybrid). Rationale: a whole-body plan's value is
   largely the base path; projecting to EE-only poses discards it and asks the
   greedy QP to rediscover exactly what it fails at today.
2. **Goal IK: python-mink**, installed into the bridge `.venv`
   (`pip install mink`). It is the reference implementation the C++ mink port is
   golden-tested against — same QP semantics at plan time and track time.
3. **Validation scene: suction pick** (already authored, guards configured,
   binary success criterion, and it is the live-measured instance of the gap).
4. **Integration depth: probe-first.** Probe scripts prove the loop end-to-end;
   the `PlannedReach` py_trees skill is the last task, added only after the live
   gate passes, swapped into the pick tree where `ReachRamp` sits.

## Architecture

```
                    PLAN TIME (client-side, pure Python)
 synced qpos ──► PlanContext (client.model mjb replica + scratch MjData)
                    │
 pre-grasp pose ──► goal IK (python-mink, multi-seed) ──► goal q's (collision-free)
                    │
                    ▼
            RRT-Connect, 10-DoF (joint_x, joint_y, joint_th + 7 arm)
            base samples biased: disc around start ∪ goal base positions
            edge check: interpolate at fixed resolution, mj_kinematics + mj_collision
                    │
            shortcut smoothing ──► time-parameterized q(t) waypoints

                    TRACK TIME (existing engine, one new param)
 waypoints ──► configure_controller{posture_target: {joint: q}}   [NEW WIRE]
                    │  frame task DISABLED, twist_follow DISABLED (task_enabled)
                    ▼
            whole-body mink QP tracks full q — planned base path executed exactly
            collision guards + velocity limits stay active (insurance layer)

                    HANDOFF CONTRACT
 before execute: skill disables twist_follow  ──  after: nav re-seeds from wherever
 the base ended (set_nav_goal already reads current pose — no engine work)
```

The planner delivers the robot to **pre-grasp** (cup ~3 cm above the box top,
cup-down). The final seat + suction engage reuses the already-live-validated
frame-task descent (`pick_direct` endgame: direct target on the box top, suction
on within 5 cm, dwell, lift). The contact phase is never planned.

### Why the client can plan at all

The bridge client receives the server's **compiled binary model (mjb)** in the
handshake (`client.py:474 → _load_mjb`, `MjModel.from_binary_path`). MuJoCo
compiles the whole scene into one model, so `client.model` is an exact collision
replica — robot, table, box, obstacles, contact filtering — checkable with plain
`mujoco` calls at thousands of configs/sec, zero UE round-trips.

## Component 1: `posture_target` wire (engine)

**File:** `Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp`
(configure/get param handlers).

New `configure_controller` param:

```json
{"posture_target": {"joint_x": 4.61, "joint_y": 0.0, "joint_th": 0.0,
                    "joint_1": 1.2, "...": 0.0}}
```

- **Named-joint partial map.** Robust to joint ordering; callers send only the
  planned DoF. Unmentioned joints keep their current posture target.
- Handler resolves joint names against the articulation, builds the full
  `TargetQ`, calls `B.AsPosture->SetTarget(TargetQ)` — the identical call
  TwistFollow makes every step (`MjMinkIKController.cpp:896`), so the solve path
  is already exercised. Targets are read live each solve (cheap path, like frame
  targets — no `MarkSpecsChanged`).
- Unknown joint names: warn-and-ignore per entry (same convention as the
  `task_costs` invalid-index handling), never fail the whole call.
- **Precedence rule (documented in the op schema):** TwistFollow also writes the
  posture target internally every step — while the `twist_follow` task is
  enabled it wins (last writer per solve). Planned execution therefore requires
  `twist_follow` disabled via the existing `task_enabled`. Enforced by the skill
  layer; no new engine logic.
- **No `posture_cost` param.** The existing `task_costs` surface already tunes
  the posture task cost live.
- **Readback:** `get_controller_params` reports the current posture target as a
  dense named map (symmetric with the dense `task_costs` readback).

## Component 2: planner module (`Scripts/demos/urlab_planner.py`)

Pure client-side Python over the mjb replica. No UE round-trips during planning.

- **`PlanContext.from_client(client, articulation, planned_joints)`** — take one
  synced physics read for the start snapshot, build a scratch `MjData` over
  `client.model`, resolve qpos/dof index maps for the planned joints. Start q
  comes from the physics mirror, **never** `find_actors` (which returns the
  frozen spawn pose).
- **`context.collision_free(q) -> bool`** — write q into scratch qpos,
  `mj_kinematics` + `mj_collision`, then two mandatory filters:
  1. **Penetration test, not contact test:** a contact counts only if
     `con.dist < -1e-6`. The cup geom carries `margin=0.03 gap=0.03` (the
     adhesion fix), which makes MuJoCo report positive-distance contacts within
     3 cm — those are suction sensing, not collisions.
  2. **Allowed-pair baseline:** geom-pair contacts already penetrating at the
     start config (any resting contact) are captured once and exempted; only
     *new* penetrating pairs count.
- **`sample_goal_configs(context, pre_grasp_pose, n=8)`** — python-mink IK from
  n seeds (base seeded near the target pose, arm postures randomized); keep
  solutions that converged, respect joint limits, and pass `collision_free`;
  dedupe near-identical configs.
- **`rrt_connect(context, q_start, goal_qs, ...)`** — bidirectional RRT-Connect;
  the goal tree is seeded with *all* goal configs. Base dimensions sample from a
  disc (radius ~1 m) around the start and goal base positions; arm dimensions
  sample uniform within joint limits. Edges validated by interpolation at a
  fixed joint-space resolution, every interior config through `collision_free`.
- **`shortcut(path, attempts)`** — random-pair shortcutting with full edge
  re-validation.
- **`time_parameterize(path, v_limits)`** — per-segment dwell from the max
  joint displacement over that segment divided by a per-joint velocity limit
  (defaults derived from the model's actuator/joint limits); returns
  `Plan(waypoints=[(t_sec, {joint: q})], stats)` where stats carries node count,
  collision checks, wall time — printed by probes for tuning.

Failure reporting distinguishes stages: "goal IK found no feasible config" vs
"RRT exhausted its iteration budget" — they are fixed differently (pose/scene
problem vs planner budget problem).

## Component 3: execution (probe first, then `PlannedReach`)

Probe scripts follow the established author/drive split:

1. Author (or reuse) the suction-pick scene; press Simulate (user-driven).
2. Drive probe: synced start read → plan → execute:
   - `task_enabled`: frame OFF, twist_follow OFF (posture + damping stay on).
   - Stream each `posture_target` waypoint at its scheduled time.
   - **Watchdog:** synced tracking error vs the current waypoint above tolerance
     for longer than a timeout → abort: stop streaming, restore `task_enabled`,
     report FAILURE with the divergence measurement.
   - All synced reads happen pre-suction (v1 constraint, see Non-Goals).
3. At pre-grasp: re-enable the frame task, run the validated `pick_direct`
   endgame (direct EE target on the box top, suction within 5 cm, hold-suction
   re-asserts, dwell, vertical lift), read the box physics z, report
   `GRIPPED+LIFTED` / `no lift`.
4. Nav handoff: after execution, `set_nav_goal` re-seeds from the current base
   pose — nothing to do beyond re-enabling twist_follow when nav next runs.

Once the live gate passes, wrap plan+execute as a `PlannedReach` py_trees skill
in `Scripts/demos/urlab_skills.py` with the same interface position as
`ReachRamp` (consumes blackboard affordance pose, returns RUNNING/SUCCESS/
FAILURE), and swap it into `tidybot_suction_pick_demo.py`'s tree.

## Testing

- **Engine (URLab.MinkIK suite, editor closed):** `posture_target` parse →
  solve → drive ctrl moves toward the target; unknown-joint warn-and-ignore;
  readback round-trip; precedence (twist_follow enabled overrides it).
- **Planner offline (pure MuJoCo, no UE):** tiny synthetic MJCF (planar base +
  2-link arm + wall obstacle): plan found; endpoints exact; every interpolated
  edge config passes `collision_free`. Regression tests for both silent-failure
  filters: margin contacts ignored (cup-margin case) and baseline resting pairs
  exempted.
- **Golden guard:** `Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml`
  stays byte-identical.
- **Live gate:** suction-pick scene end-to-end → `GRIPPED+LIFTED`.

## Risks / open questions

- **Adhesion strength at pre-grasp handoff:** the shelved experiment showed the
  grip is marginal at 2.5 cm standoff. The planner removes the standoff (base
  places the cup for a true seat), which should make the existing
  margin=0.03/gain=100 adhesion sufficient — but if a seated cup still fails to
  lift, widen margin/gain (the shelved 2-line XML experiment) as a fallback.
- **QP tracking of aggressive posture steps:** waypoint spacing must respect the
  QP's velocity limits or the watchdog will trip; time parameterization uses the
  same limits, but live tuning of tolerance/timeout is expected.
- **mjb staleness:** the replica is refreshed per connect/handshake. Probes must
  connect (or refresh) *after* the scene is authored and Simulate starts, or the
  planner checks against a stale world.
