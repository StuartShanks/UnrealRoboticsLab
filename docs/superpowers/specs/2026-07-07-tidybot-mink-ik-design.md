# TidyBot mink IK in Unreal — design spec

**Date:** 2026-07-07
**Branch:** `feat/urlabmink-live-ik`
**Goal:** Full parity with `mink/examples/mobile_tidybot.py` (Kevin Zakka's mink, v1.2.0)
running in Unreal via URLab: the TidyBot (planar base + 7-DoF Kinova arm) tracks a
movable `pinch_site_target`, base drives toward far targets, and a fix-base toggle
pins the base — all through real physics (`d->ctrl` on position actuators, never
`qpos` writes).

## Context: what exists, what's broken

- **`URLabMink`** (committed): 1:1 C++ port of the mink library — Configuration,
  Frame/Posture/Damping (+ more) tasks, Configuration/Velocity/CollisionAvoidance
  limits, `MinkSolveIK` — with golden-parity tests against Python mink 1.2.0
  (`Scripts/mink_golden`, venv with mink 1.2.0 + mujoco 3.9.0). Treated as
  **parity-locked**: changed only with golden-trace evidence.
- **`UMjMinkIKController`** (committed + uncommitted polish): data-driven
  `UMjArticulationController` that composes the mink task stack from
  `FMinkTaskSpec`/`FMinkLimitSpec` arrays at Bind and runs the example's loop each
  physics step: route targets → `MinkSolveIK` × MaxIters → `IntegrateInplace` →
  `d->ctrl[act] = q[qposadr]`. The exact tidybot stack is expressible as data — no
  new controller class is needed.
- **The blocker** (`docs/mink-ik-buzz-eod-report.md`): with the frame task enabled
  on TidyBot in the editor, the sim NaNs (`QACC`) within ~0.1 s and auto-resets.
  Posture-only is stable. Root cause never established.
- **Known infra landmines** from the same report: importer fatal-crashes on the
  include-style multi-`<worldbody>` `scene.xml`; model sim options
  (`integrator="implicitfast"`, `cone=elliptic`, `impratio`) not applied
  automatically on import/spawn (TidyBot base actuators are kp=1e6 — Euler
  diverges); mocap bodies are UE-authoritative (`UMjBody` tick re-stamps
  `mocap_pos` from the UE transform, so RPC `set_mocap_pose` is overwritten);
  client `ControllerKind` enum closed → unknown kinds silently bypass the
  controller.
- **Abandoned pivot** (uncommitted): `MjEndEffectorController`, a mocap+weld
  "fake IK" that zeroes all actuator forces at PostSetup. Not part of this design;
  parked on a side branch because its actuator neutralization would sabotage mink
  IK testing if left in-tree.

## Design decision: where the "1:1 port" lives

The literal line-for-line port of `mobile_tidybot.py` becomes a **native C++
automation test** (Rung B below) — the isolation instrument and permanent
regression gate. The user-facing demo runs through the existing data-driven
`UMjMinkIKController` configured with the example's exact task stack. Nothing that
exists is rewritten.

## Phase 0 — Untangle the working tree

The uncommitted diff mixes two efforts. Split at hunk level:

- **Commit to this branch** (explicit paths only — shared-tree discipline):
  `MjMinkIKController.{h,cpp}` polish (no-clamp finite guard, diagnostics, live
  spec rebuild), `AdoptRuntimeController` in `MjArticulation.{h,cpp}`, and the
  `add_controller` MarkSpecsChanged/adopt wiring in `MjEditorOpHandlers.cpp`.
- **Park on `exp/mocap-weld-teleop`**: `MjEndEffectorController.{h,cpp}` and the
  EE-injection + actuator-neutralization hunks in `MjArticulation.cpp`.
- Compile-verify after the split (editor-safe Build.sh; see
  `urlab-build-test-workflow` memory).

## Phase 1 — Ground-truth repro ladder

Vendor `mink/examples/stanford_tidybot/` (scene.xml, tidybot.xml, mesh assets)
pinned to the **v1.2.0 tag** (matching the venv's mink) into
`Scripts/mink_golden/models/stanford_tidybot/`, with license notes (mink is
Apache-2.0; the TidyBot model derives from mujoco_menagerie — carry its license
file).

Three rungs, each adding one URLab layer. A shared, deterministic target script is
used by all rungs: hold at initial EE pose → small offset step (arm-only reach) →
slow far circle (forces base motion). No randomness, no wall-clock.

- **Rung A — Python ground truth.** Headless variant of `mobile_tidybot.py` in
  `Scripts/mink_golden` (no viewer, no keyboard; the scripted trajectory drives
  `pinch_site_target` mocap directly). Dumps a per-step JSONL/CSV trace: sim time,
  q, ctrl, EE pose, target pose. Proves algorithm+model stability; produces the
  golden trace.
- **Rung B — native C++ 1:1 test** (`URLab.Mink.TidyBot.*`, in URLabMink tests):
  `mj_loadXML` the same scene.xml; replicate the Python loop exactly —
  `mj_resetDataKeyframe(home)`, `configuration.update(qpos)`,
  `posture.SetTargetFromConfiguration`, move-mocap-to-frame, then per step: set
  frame target from mocap → up-to-20-iteration solve/integrate with 1e-4
  thresholds → `d->ctrl[actuator_ids] = q[dof_ids]` (exactly the example's 10
  named actuators) → `mj_step`. Asserts: all of qpos/qvel/qacc finite every step;
  EE within 2 cm position / 0.1 rad orientation of the target at trajectory
  checkpoints (after a settle window); optional per-step trace comparison against
  Rung A within tolerance.
  **Decision bit:** NaN here ⇒ port math bug (fix in URLabMink with golden
  evidence). Stable here ⇒ bug is URLab integration.
- **Rung C — URLab-integrated test** (`FMjTestSession`, URLabEditor tests): import
  the merged single-worldbody tidybot scene through URLab's importer, spawn,
  attach `UMjMinkIKController` with the example task stack (below), apply the home
  keyframe, stream the same target script, run ≥10 sim-seconds. Same assertions
  as Rung B. Expected to reproduce the NaN at first; becomes the regression gate once
  fixed.

## Phase 2 — Root-cause and fix (differential debugging)

Rung B vs Rung C differ only in URLab's layers. Diff them systematically:

1. **Compiled model** — field-by-field: `opt.integrator/timestep/cone/impratio`,
   `actuator_gainprm/biasprm/ctrlrange/forcerange`, `dof_armature/damping`, body
   masses/inertias, keyframes. The import merge (`mjs_attach` + UE-added floor)
   may alter any of these.
2. **Initial state** — is `home` actually applied? Start qpos identical?
3. **Per-step traces** — first ~50 steps of ctrl/qpos/qacc side by side.

Ordered suspects (cheapest first), from the EOD report plus fresh code reading:

1. Sim options from the model not applied on the import path (implicitfast /
   elliptic / impratio).
2. Home keyframe not applied → pathological start pose.
3. `DriveJoints` empty ⇒ controller writes **all** bound actuators; the example
   writes exactly the 10 named ones.
4. Something else stamping `d->ctrl` concurrently (raw-mode actuator ticks,
   `ApplyStepCtrl` staging — the EOD's own suspicion).
5. UE-cm vs MuJoCo-m unit mismatch anywhere on the slide-joint (`joint_x/y`) path.
6. Import-merge model deltas (collision pairs with the UE floor, solver settings).

Each confirmed root cause → minimal fix + regression assertion in the rung tests.
Fixes land in URLab integration code (importer/spawn/sim-options/controller), not
in URLabMink.

## Phase 3 — Demo parity in the editor

- **Scene asset:** merged single-worldbody `scene.xml` fixture (workaround for the
  importer multi-worldbody crash; the crash itself is ticketed, fixed here only if
  it proves to be the NaN culprit).
- **Task stack (the example, as data):**
  - Frame: `pinch_site` (site), mocap target `pinch_site_target`,
    PositionCost=1, OrientationCost=1, LmDamping=1, Gain=1
  - Posture: Joints = arm `joint_1..joint_7`, Cost=1e-3 (base DOFs get 0 — the
    example's `cost[3:]=1e-3`)
  - Damping: Joints = `joint_x, joint_y, joint_th`, Cost=100, **bEnabled=false**
    (toggling it = the example's Enter-key `fix_base`)
  - Limits: ConfigurationLimit (default gain)
  - Solver: MaxIters=20, PosThreshold=OriThreshold=1e-4, QpDamping=1e-3,
    open-loop reference (`bSyncFromLiveState=false`, like the example)
- **Setup is scripted over the bridge** (reproducible): import → `add_controller`
  with the stack above → reset to `home` keyframe → move the mocap body's UE
  actor to the current EE pose (`move_mocap_to_frame` equivalent; done game-side
  because mocap is UE-authoritative).
- **Driving:** gizmo-drag of the mocap body in Simulate (native analogue of the
  viewer drag) and bridge-streamed `target_pos`/`target_quat` for scripted runs.
  `fix_base` via the Details checkbox or `task_enabled` over
  `configure_controller`.
- **Acceptance (run end-to-end over the bridge, screenshots captured):** near
  target → arm-only reach; far target → base drives over; fix_base on → base
  pinned while the arm stretches; multi-minute scripted sweep with zero
  NaN/auto-resets and tracking error within tolerance.

## Testing

- Rung B and Rung C join the automation suite as permanent regression gates
  (`URLab.Mink.TidyBot.*`; editor must be closed to run — see workflow memory).
- Rung A + trace comparison live in `Scripts/mink_golden` beside the existing
  golden generator, documented in its README.
- Compile via editor-safe `Build.sh` after every change; format only files touched
  by this effort (never `Scripts/format.sh` on a shared tree).

## Error handling (existing behavior, kept deliberately)

- QP solve failure → hold last ctrl, log throttled.
- Non-finite solve velocity → discard the step (never write NaN to ctrl).
- Sim time going backwards (auto-reset) → re-base the open-loop reference from
  live qpos.
- Idle physics-thread invocations (sim time unchanged) → no integration.

## Documented divergences from the Python example

- **Integration dt:** the example integrates the IK reference with wall-clock
  `rate.dt` (5 ms at 200 Hz); the controller uses elapsed sim time (clamped to
  10× timestep). Equivalent under position control; stated, not hidden.
- **Target source:** viewer mouse-drag → UE gizmo drag / bridge streaming.
- **fix_base:** Enter key → Damping spec `bEnabled` / `task_enabled` RPC.
- **Pause (Space):** not ported — UE Simulate has its own pause.

## Out of scope

- Fixing the importer's multi-worldbody crash (ticketed; workaround: merged
  fixture).
- The mocap+weld `MjEndEffectorController` (parked on `exp/mocap-weld-teleop`).
- Other mink examples (humanoids, hands) — the generalisation notes in
  `docs/mink-ik-generalisation.md` stand, but only TidyBot is in scope.
- Client-side Python `ControllerKind` enum fallback (ticketed if it obstructs the
  bridge-driven demo, fixed minimally if so).
