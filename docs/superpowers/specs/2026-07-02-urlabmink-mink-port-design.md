# URLabMink — 1:1 C++ Port of mink (Design)

**Date:** 2026-07-02
**Status:** Draft — awaiting user review
**Author:** Brainstormed with Claude

## Summary

Add a new Unreal Engine runtime module, `URLabMink`, that ports
[mink](https://github.com/kevinzakka/mink) (Kevin Zakka's differential
inverse-kinematics library) to C++ **1:1**, for **real-time, in-engine IK**
during simulation and teleoperation. The port is built on MuJoCo's C API
(already linked by URLab) and Eigen (already a URLab dependency), plus one
header-only QP solver. Correctness is defined by **numerical parity** against
Python mink, enforced by a golden-vector test harness.

## Decisions (locked during brainstorming)

- **Primary goal:** real-time in-engine IK (candidate replacement for the
  mocap+weld "fake IK" used by the paused VR-teleop work).
- **Scope:** full 1:1 port — every task, every limit, the lie library, and
  `solve_ik`. Nothing left out.
- **Fidelity bar:** numerical parity — Python mink is the spec; the C++ port
  must reproduce its outputs within tolerance on shared inputs.
- **Approach:** **A — native C++ port** (see Alternatives for why not Python).

## Goals / Non-goals

**Goals**
- A self-contained C++ IK module mirroring mink's structure and behaviour.
- Zero per-tick latency, runs natively (physics thread friendly).
- Golden-vector parity tests against a pinned Python mink version.

**Non-goals (this spec)**
- Wiring IK into the live sim / actuators / VR input. `URLabMink` is a pure
  solver; the consuming integration (where it meets the VR-teleop branch) is a
  separate effort.
- An editor module (`URLabMinkEditor`). mink has no editor concept; Details-panel
  task authoring can be a later addition.

## Alternatives considered

| Option | Verdict |
|---|---|
| **A — Native C++ port** (chosen) | Self-contained, zero-latency, a true in-engine UE module. Cost: the port + a parity harness. |
| **Python mink over the existing ZMQ bridge** (`urlab_bridge`) | ~No work, always parity-correct, tracks upstream. But adds control-loop latency, needs a Python env at deploy, and is not an in-engine module. **Kept on record as a fast-prototype path** if priorities shift toward speed. |
| **Embed CPython in-process** | Literally 1:1, but GIL stalls the physics thread and packaging a Python runtime into a UE game is painful. Rejected. |

> **Fast-path note:** because `urlab_bridge` already exists, real mink can drive
> the sim from Python in ~a day with no port. If validating the teleop UX
> quickly becomes more important than shipping self-contained, prototype there
> first and treat this C++ port as the productionization step.

## Module architecture

A new **runtime** module, depending only on MuJoCo + Eigen + a QP header — **not**
on `URLab`. This keeps it an independently testable IK library. `URLab` (and the
future EE controller) depends on `URLabMink` to consume it.

```
Source/URLabMink/
  URLabMink.Build.cs
  Public/URLabMink/…
  Private/URLabMink/…
  Private/Tests/…            # UE automation parity tests

Layering:
  URLabMink → { MuJoCo C API, Eigen, QP header }   (no URLab dependency)
  URLab     → URLabMink                              (consumer)
```

Registered in `UnrealRoboticsLab.uplugin` as a Runtime module, Default loading
phase, listed before `URLab` so `URLab` can depend on it.

## Ported component map (mirrors mink 1:1)

| mink | URLabMink |
|---|---|
| `lie/` (base, so3, se3, utils) | `Lie/` — `FMinkSO3`, `FMinkSE3` (Eigen-backed; exp/log/adjoint/multiply/apply/inverse, wxyz quaternions) |
| `configuration.py` | `FMinkConfiguration` — wraps `mjModel*`/`mjData*`; FK, `GetFrameJacobian`, frame transforms, `Integrate`, limit checks |
| `tasks/task.py` (base) | `FMinkTask` — `ComputeError`, `ComputeJacobian`, `ComputeQPObjective` (→ `P`, `q`), gain/cost/lm_damping |
| `frame_task.py` | `FMinkFrameTask` |
| `relative_frame_task.py` | `FMinkRelativeFrameTask` |
| `posture_task.py` | `FMinkPostureTask` |
| `com_task.py` | `FMinkComTask` |
| `damping_task.py` | `FMinkDampingTask` |
| `limits/limit.py` (base) | `FMinkLimit` — `ComputeQPInequalities` (→ `G`, `h`) |
| `configuration_limit.py` | `FMinkConfigurationLimit` |
| `velocity_limit.py` | `FMinkVelocityLimit` |
| `collision_avoidance_limit.py` | `FMinkCollisionAvoidanceLimit` (heaviest; sequenced last) |
| `solve_ik.py` | `MinkSolveIK(config, tasks, dt, limits, damping)` → joint velocity `dq` |
| `utils.py`, `constants.py` | `FMinkUtils`, frame-type enums |

Naming/idioms follow UE conventions (`F`-prefixed structs, `UE_LOG`), but the
class decomposition, method boundaries, and math stay faithful to mink so the
parity harness maps layer-for-layer.

## QP solver

mink's default backend is `quadprog` — a **Goldfarb–Idnani** dual active-set
solver. Using the *same algorithm* in C++ is what makes tight numerical parity
achievable (a different method like OSQP's ADMM would diverge on
redundant/degenerate poses). Plan: vendor a **permissively-licensed
(MIT/BSD/Apache) header-only Goldfarb–Idnani solver** built on Eigen.

- **Open item:** pin the exact library + confirm its license is Apache-2.0
  compatible, then add it to `ThirdPartyNotices.txt`. `eiquadprog` is the obvious
  candidate but is **LGPL** — likely swap for an MIT Goldfarb–Idnani
  implementation (e.g. a QuadProg++-derived header) instead.
- Header-only ⇒ **no new `third_party/` submodule** and no drift-check wiring.

QP form (mink `solve_ik`): minimize `0.5 · dqᵀ P dq + qᵀ dq` s.t. `G dq ≤ h`,
where `P`,`q` are summed task objectives (with Levenberg/​lm damping) and `G`,`h`
are stacked limit inequalities.

## Data flow (real-time use, informative)

Per control tick, on the physics thread:
1. Update targets on tasks (e.g. `FrameTask.SetTarget(SE3)` from a VR pose).
2. `MinkSolveIK(config, tasks, dt, limits, damping)` builds the QP (`P`,`q` from
   tasks; `G`,`h` from limits) and solves for joint velocity `dq`.
3. Consumer integrates/applies `dq` to the sim.

`URLabMink` is a pure solver over a `Configuration`; steps 1 and 3 belong to the
consuming integration (out of scope here).

## Error model

mink raises exceptions; the UE hot path avoids them.
- Constructors/setup validate inputs and `UE_LOG(LogURLabMink, Warning/Error)`.
- `MinkSolveIK` returns `{ EMinkSolveStatus, TOptional<VelocityVector> }` with
  statuses like `Success`, `NoFeasibleSolution`, `InvalidFrame`,
  `NotWithinLimits`.
- No exceptions inside the solve loop.

## Testing — numerical parity (core requirement)

A layered golden-vector harness, with Python mink as the reference spec.

- **Generator:** `Scripts/mink_golden/gen_golden.py` imports a **pinned**
  `mink` + `mujoco`, loads canonical MJCF models, seeds inputs, and dumps JSON
  fixtures. Mirrors the existing `Scripts/codegen/snapshots/` convention.
- **Committed fixtures:** MJCF models + JSON golden vectors at four layers:
  1. **Lie ops** — SO3/SE3 exp/log/adjoint/multiply/apply.
  2. **Configuration** — frame Jacobians, transforms, integrate.
  3. **Per-task / per-limit** — `P`,`q` for each task; `G`,`h` for each limit.
  4. **End-to-end** — `solve_ik` `dq` for `{model, tasks, limits, q}`.
- **Assertions:** UE automation tests in `Source/URLabMink/Private/Tests/` load
  fixtures and check parity within per-layer tolerances (tight ~1e-9 for
  lie/FK; looser ~1e-5 for the full QP solve).
- **Logistics open item:** the generator needs `pip install mink mujoco` to
  (re)generate fixtures. Committed fixtures mean the C++ build/test does not need
  Python.

## Build wiring

`URLabMink.Build.cs`:
- Adds the existing `third_party/install/MuJoCo/include` include path and links
  the same MuJoCo lib URLab uses. The submodule/install **drift-check logic stays
  owned by `URLab.Build.cs`**; `URLabMink` only consumes the already-installed
  MuJoCo artifacts.
- Adds `Eigen` (UE ships an Eigen third-party module; already a URLab dep).
- Includes the vendored header-only QP solver.
- `PCHUsage = UseExplicitOrSharedPCHs`, matching sibling modules.

`URLab.Build.cs` gains `"URLabMink"` in its dependency list (consumer side).

## Proposed sequencing (for the implementation plan)

1. Module scaffold + build wiring + empty automation test that loads a fixture.
2. `Lie/` (SO3, SE3) + layer-1 golden tests.
3. `FMinkConfiguration` + layer-2 golden tests.
4. QP solver vendored + a tiny standalone QP sanity test.
5. Tasks (frame → posture → relative-frame → com → damping) + layer-3 tests.
6. Limits (configuration → velocity) + layer-3 tests.
7. `MinkSolveIK` + layer-4 end-to-end parity tests.
8. `CollisionAvoidanceLimit` (heaviest) + its tests — sequenced last.
9. `URLab` consumes `URLabMink`; docs page.

## Open questions

1. **QP solver library + license** — confirm the specific MIT/BSD Goldfarb–Idnani
   header to vendor (default: swap `eiquadprog` for an MIT QuadProg++-derived
   solver). Add to `ThirdPartyNotices.txt`.
2. **Collision-avoidance limit** — included in scope, sequenced last. Confirm ok
   to land it after the rest is parity-passing.
3. **Fixture generation env** — confirm a Python env with `mink` + `mujoco` is
   available to generate golden vectors, and pin the mink version.

## References

- mink: https://github.com/kevinzakka/mink
- MuJoCo C API (Jacobians/FK): already linked via `third_party/install/MuJoCo`
- Existing snapshot/codegen convention: `Scripts/codegen/snapshots/`
- Related paused work: VR teleop `MjEndEffectorController` (mocap+weld fake IK)
