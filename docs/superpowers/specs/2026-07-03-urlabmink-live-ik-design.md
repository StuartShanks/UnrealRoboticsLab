# URLabMink Live IK Demo — Design

**Date:** 2026-07-03
**Branch:** `feat/urlabmink-live-ik` (off `feat/urlabmink`)
**Status:** Approved (core sections 1–2); API/testing sections captured here for build.

## Goal

A live, mouse-free demo that exercises the `URLabMink` differential-IK solver on
the imported Franka Panda: a Cartesian end-effector target is commanded from
Python, and the UE-side mink controller solves each physics step so the arm's
`hand` frame tracks the target. Proves the solver works end-to-end on a real
robot, not just in unit tests.

Decisions locked during brainstorming:
- **Drive path:** Python-commanded Cartesian target (consistent with the
  established mouse-free workflow; the editor viewport mouse is unusable over
  Chrome Remote Desktop).
- **Apply mode:** **kinematic** — write the solved joint angles into `qpos`.
  Isolates the solver (EE reaches target ⇔ mink correct), no controller/gravity
  confounds. Dynamic (actuator-target) mode is an explicit future phase reusing
  the same plumbing.

## Non-goals

- Dynamic / actuator-driven tracking (future phase).
- VR / teleoperation (the paused `UMjEndEffectorController` WIP path).
- Collision-aware IK, multi-arm, or gripper IK. Gripper stays on `actuator8`.

## Components

Three isolated units, each independently testable, following existing patterns:

### 1. `UMjMinkIKController` (new `USceneComponent`, `URLAB_API`)
Dropped on an `AMjArticulation`. Mirrors the shape of `UMjEndEffectorController`
but drives the real solver. Owns the mink solve loop; knows nothing about ZMQ.

- `FString TargetBodyName` (default `"hand"`) — EE frame to solve for.
- `bool bEnabled`.
- Target pose in Unreal world space; thread-safe `SetIKTarget(FVector WorldPos,
  FQuat WorldRot)` (queued, applied before the next physics step) and
  `SnapTargetToCurrentEE()` to avoid a yank on enable.
- Task gains: `PositionCost`, `OrientationCost`, `PostureCost`, `Damping`
  (defaults from the mink examples).
- Lifecycle: resolve the target frame id after the model compiles (mirrors
  `UMjEndEffectorController::ResolveAfterCompile`).

### 2. `set_ik_target` bridge op
Registered in `OpRegistry`; handler runs on the game thread, resolves the
articulation's `UMjMinkIKController`, calls `SetIKTarget`. Request carries
`{ pos:[x,y,z], quat:[w,x,y,z], enable:bool, body? }`; reply `set_ik_target_ok`.
Owns transport; knows nothing about the solver.

### 3. Python `client.ik.set_target(...)`
Small new `URLabIKAPI` namespace method that sends the op. Signature roughly
`set_target(pos, quat=(1,0,0,0), *, enable=True)`. Owns the client-side call.

## Data flow — the solve loop (kinematic)

Each physics step, in the same pre-step hook the fake-IK controller uses, when
`bEnabled` and a target is set:

1. Build/refresh `FMinkConfiguration` from the articulation's live
   `mjModel`/`mjData`.
2. Convert the target Unreal-world pose → MuJoCo-world `FMinkSE3`, reusing
   URLab's existing UE↔MuJoCo transform (cm→m, axis change) — the same
   conversion the fake-IK controller applies to its mocap target.
3. `FrameTask(hand)` toward that SE3; task set `{FrameTask, PostureTask}`; default
   config limit.
4. `result = MinkSolveIK(cfg, tasks, dt, Damping, bSafetyBreak=false)`.
5. On success: `q = cfg.Integrate(result.Velocity, dt)`; write the arm DOFs into
   `d->qpos`, zero their `qvel`, `mj_forward`. Rewriting every step keeps it
   kinematic. Gripper DOFs untouched.

Solving every step means a newly-set target is tracked smoothly (differential IK
converges over successive frames rather than snapping).

## Error handling

- `MinkSolveIK` returns `EMinkIKStatus`. On any non-`Success` status the
  controller holds the last `qpos` and logs the status (rate-limited); it never
  writes a garbage solution. `NotWithinConfigurationLimits` / `NoSolutionFound`
  are surfaced distinctly in the log.
- Unresolved target frame (bad `TargetBodyName`) → controller inert + one
  warning; the op reply reports the failure.
- `set_ik_target` before PIE / before the controller exists → op returns a
  `set_ik_target_failed` reply with a reason (mirrors existing failed-op
  conventions).

## Testing

- **C++ automation** `URLab.LiveIK.PandaConverges` (URLab or URLabMink test
  module): load the Panda model, attach the controller, set a known-reachable
  target, run N kinematic solve steps, assert the `hand` world pose converges
  within tolerance (pos + orientation). Validates controller glue + solver on the
  real robot. Also a `TargetUnreachable` case asserting graceful hold + status.
- **Python integration smoke** (demo script, not CI): with the sim running,
  `client.ik.set_target(reachable_pose)`, step, read `hand` pose back, assert
  within tolerance.
- **Demo script** `Scripts/urlab_ik_demo.py` (or scratchpad): move the target
  through a few reachable poses so the arm visibly tracks.

## Isolation / process

- New work on `feat/urlabmink-live-ik` (off `feat/urlabmink`) to keep the proven
  port pristine and avoid the uncommitted paused WIP.
- Never `git add -A` in this repo; stage only the new live-IK files. The paused
  `MjEndEffectorController` WIP + `MjArticulation.cpp` changes stay untouched in
  the working tree.
- Build/verify via `Scripts/build_and_test_linux.sh`.

## Open implementation questions (resolve during build)

- Exact per-step hook: does the mink controller reuse the same
  `MjArticulation` controller-tick path the WIP added, or a dedicated pre-step
  callback? Confirm when reading `MjArticulation.cpp` / `MjPhysicsEngine.cpp`.
- Which arm DOFs to write (joint1..7) vs. full `nv`; whether to freeze fingers
  via `MinkDofFreezingTask` or rely on the posture task.
