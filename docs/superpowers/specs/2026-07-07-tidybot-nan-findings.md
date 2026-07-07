# TidyBot mink-IK instability — root-cause findings (Task 5)

**Date:** 2026-07-07 · **Branch:** `feat/urlabmink-live-ik` · **Method:** differential
(Rung B green vs Rung C red), cheapest-first, on the headless
`URLab.MinkIK.TidyBot.*` harness.

## TL;DR

The TidyBot instability is **not** caused by the dropped `<option>` (cone/impratio)
that Task 4 flagged. It is caused by a **separate, previously-undiscovered importer
bug: default-class actuator gains are not applied**, so the seven arm position
actuators (`joint_1..joint_7`) compile with **`kp = -1`** (a positive-feedback,
destabilizing gain) instead of the intended `kp = 2000 / 500`. When the mink
controller drives `ctrl = q_ref`, the negative gain makes `joint_1` (DOF 3) diverge
at t ≈ 0.67 s — exactly the observed symptom.

Two discriminating experiments prove it:

| Experiment (imported model, only this changed) | cone/impratio | arm `kp` | Result |
|---|---|---|---|
| Baseline (as imported) | 0 / 1 (wrong) | −1 (wrong) | **UNSTABLE** — NaN-warn DOF 3 @ t=0.674s, qacc→9e9, track 0.33–2.32 m |
| **Force cone=elliptic + impratio=10** | 1 / 10 (correct) | −1 (wrong) | **STILL UNSTABLE** — NaN-warn DOF 3 @ t=0.666s, qacc→9e9, track 0.40–1.69 m |
| **Copy native actuator gains** (options left wrong) | 0 / 1 (wrong) | 2000/500 (correct) | **STABLE & TRACKING** — no warn, qacc≤6e3, track 0.0073/0.0078/0.2382/0.050/0.050/0.052 m — **matches Rung B exactly** |

Options are a red herring for the NaN. Gains are the root cause.

---

## Root cause 1 (PRIMARY) — default-class actuator gains dropped on import

### Discriminating experiment + numbers

`URLAB_FIX_GAINS=1` copies the native `actuator_gaintype/biastype/gainprm/biasprm`
(from MuJoCo's own compile of `tidybot.xml`) onto the imported model, leaving cone
(pyramidal) and impratio (1) at their wrong imported values. Result:

```
[TASK5] copied native actuator gains onto imported model
[TASK5] pre-loop opt: cone=0 impratio=1.0 integrator=3 timestep=0.0020 (forced=0)
checkpoint 299:  trackErr=0.0073  tol=0.02  maxAbsQacc=31.6      PASS
checkpoint 899:  trackErr=0.0078  tol=0.02  maxAbsQacc=3.15e+03  PASS
checkpoint 999:  trackErr=0.2382  tol=0.30  maxAbsQacc=3.15e+03  PASS
checkpoint 1600: trackErr=0.0504  tol=0.10  maxAbsQacc=5.99e+03  PASS
checkpoint 2200: trackErr=0.0505  tol=0.10  maxAbsQacc=5.99e+03  PASS
checkpoint 2499: trackErr=0.0525  tol=0.10  maxAbsQacc=5.99e+03  PASS
Result: 1/1 GREEN
```

These numbers are identical (to 4 dp) to Rung B's native run — proving the *only*
material Rung B→Rung C dynamics delta is the actuator gains, and that the pyramidal
cone / impratio=1 the importer produces is perfectly stable for this scenario once
gains are correct.

### Mechanism (file:line)

TidyBot's arm actuators inherit their servo gains from `<default>` classes, e.g.
`tidybot.xml:24-29`:

```xml
<default class="large_actuator"><position kp="2000" kv="100" forcerange="-105 105"/></default>
<default class="small_actuator"><position kp="500"  kv="50"  forcerange="-52 52"/></default>
```
and the actuators reference them by `class="large_actuator"` etc. — with **no `kp`
on the actuator element itself**.

Import path:

1. `UMjActuator::RegisterToSpec` — `Source/URLab/Private/MuJoCo/Components/Actuators/MjActuator.cpp:513,519`
   resolves the class default and calls `mjs_addActuator(Wrapper.Spec, effectiveDefault)`.
   At this point MuJoCo would inherit `kp = 2000` from the class. **This part works.**
2. `UMjActuator::RegisterToSpec:526` then calls `ExportTo(act, effectiveDefault)`.
3. `UMjPositionActuator::ExportTo` — `Source/URLab/Private/MuJoCo/Components/Actuators/MjPositionActuator.cpp:44-47`:
   ```cpp
   double kvBuf[1] = {bOverride_kv ? (double)kv : -1.0};
   ...
   mjs_setToPosition(Element, bOverride_kp ? (double)kp : -1.0, kvBuf, dampratioBuf, timeconstBuf, ...);
   ```
   `UMjPositionActuator::ImportFromXml` (`MjPositionActuator.cpp:62-63`) only reads
   `kp`/`kv` from the **actuator element's own attributes**, never from the resolved
   class default. For the arm actuators there is no element `kp`, so
   `bOverride_kp == false` and the call passes the **`-1.0` "unset" sentinel**.
   MuJoCo's `mjs_setToPosition` writes it **literally**: compiled
   `actuator_gainprm[0] = -1`, `biasprm[1] = +1` (= −kp), `biasprm[2] = 0`. The
   `-1.0` sentinel **overwrites** the class-inherited `kp = 2000`.

Net compiled result (verified by the model diff): arm actuators get `kp = -1, kv = 0`.
A position actuator's force is `kp·(ctrl − q) − kv·q̇`. With `kp = -1` this is
`(q − ctrl)` — positive feedback: any deviation grows without bound. Once the frame
task commands the arm to move (ctrl ≠ q), `joint_1`/DOF 3 runs away → the t≈0.67 s NaN.

### Secondary facet — `kv` dropped on *every* position actuator (incl. explicit)

The base actuators `joint_x/joint_y/joint_th` carry **explicit** `kp` (1e6/1e6/5e4)
*and* `kv` (5e4/5e4/1e3). Their `kp` and `biasprm[1]` import correctly, but their
`biasprm[2]` (= −kv) compiles to **0** (native −50000/−50000/−1000). So the velocity
(damping) term is lost for all position actuators, likely because
`ExportTo` always passes **non-null** `dampratioBuf`/`timeconstBuf` sentinel pointers
(`= -1.0`) to `mjs_setToPosition`, which take precedence over `kv`
(`MjPositionActuator.cpp:44-47`). This is a real fidelity bug but is **not**
independently destabilizing here — the base actuators (correct high `kp`, `kv = 0`)
stayed stable; it is the arm `kp = -1` that diverges.

### Proposed minimal fix for Task 6

- **Primary:** do not clobber class-inherited gains with the `-1` sentinel. Either
  (a) have the importer fold the resolved default-class `kp`/`kv` into the component
  (set `bOverride_kp/kv` with the class values during import), **or** (b) in
  `UMjPositionActuator::ExportTo`, skip the `mjs_setToPosition` gain write when the
  param is unauthored and an `effectiveDefault` exists — leaving MuJoCo's class
  inheritance intact. This is codegen (`MjPositionActuator.cpp`), so the same pattern
  applies to `IntVelocity`/`Velocity`/`Damper` actuators — audit them too.
- **kv facet:** pass `nullptr` for `dampratio`/`timeconst` when not overridden so
  `kv` is honored; confirm against `mjs_setToPosition` semantics.
- **Regression assert:** extend `ImportCompiles` to check
  `arm gainprm[0] == 2000/500`, `biasprm[2] == -100/-50`, and
  `joint_x biasprm[2] == -50000`. (These were *never* asserted before — Task 4 only
  checked `joint_x` `kp`, which happens to be explicit and correct.)

---

## Root cause 2 (REAL bug, but NON-CAUSAL for the NaN) — second `<option>` dropped

### Discriminating experiment + numbers

`URLAB_FORCE_ELLIPTIC=1` forces `M->opt.cone = elliptic`, `M->opt.impratio = 10`
after compile, gains left wrong. Result:

```
[TASK5] pre-loop opt: cone=1 impratio=10.0 integrator=3 timestep=0.0020 (forced=1)
[TASK5] first |qacc|>1e6 at step 328 dof 7 (|qacc|=1.2e6)
[MuJoCo warn] Nan, Inf or huge value in QACC at DOF 3 ... Time = 0.6660
checkpoint 299=0.3961  899=0.9769  999=1.6134  1600=0.6802  2200=1.6876  2499=1.3888  (all FAIL)
maxAbsQacc → 8.98e9
```

Restoring the correct contact-solver options changes essentially nothing — still
unstable at the same DOF and time. **The option-drop is not the instability cause.**

### Mechanism (file:line)

`tidybot.xml:4-5` declares options across **two** `<option>` elements:
```xml
<option integrator="implicitfast"/>
<option cone="elliptic" impratio="10"/>
```
`UMujocoGenerationAction::ParseSimOptions` loop —
`Source/URLabEditor/Private/MujocoGenerationAction.cpp:243-387` — iterates the root's
children, parses the **first** `<option>` into `CDO->SimOptions`, then **`break`s at
line 385**. The second `<option>` (cone/impratio) is never parsed. At compile,
`AMjArticulation::Setup` → `SimOptions.ApplyToSpec` (`MjArticulation.cpp:285`) only
applies attributes whose `bOverride_*` flag is set, so cone/impratio fall back to
MuJoCo defaults (pyramidal, 1).

**Compounding latent bug:** even with the `break` removed, `impratio` would still not
apply — `MujocoGenerationAction.cpp:282-284` sets `Opts.Impratio` but **never sets
`Opts.bOverride_Impratio = true`**, so `ApplyToSpec`/`ApplyOverridesToModel`
(`MjOptionGenerated.cpp:31-38, 130-133`) skip it. `cone` parsing *does* set its flag
(`MujocoGenerationAction.cpp:337`), so cone needs only the `break` fix. The same
missing-flag defect affects `timestep, gravity, wind, magnetic, density, viscosity,
tolerance, iterations, ls_iterations` (lines 264-293) — none set their override flag.

### Proposed minimal fix for Task 6

- Remove the `break;` at `MujocoGenerationAction.cpp:385` and let all `<option>`
  elements accumulate (MuJoCo semantics: later attributes override earlier).
- Add `Opts.bOverride_Impratio = true;` at line 284 and audit the other unflagged
  attributes listed above.
- Fix for **fidelity** (correct contact model), not for the TidyBot NaN — the
  forced-elliptic experiment proves it will not stabilize the sim on its own.

---

## Full model-diff mismatch list (imported vs native `mj_loadXML(tidybot.xml)`)

Dumped by the `[TASK5 diff]` block added to `ImportCompiles`. Match by name suffix.

**Dynamics-relevant (real):**

| Field | Native | Imported | Note |
|---|---|---|---|
| `opt.cone` | 1 (elliptic) | 0 (pyramidal) | Root cause 2 |
| `opt.impratio` | 10 | 1 | Root cause 2 |
| `act[joint_1..7].gainprm[0]` | 2000 / 500 | **-1** | Root cause 1 (arm kp) |
| `act[joint_1..7].biasprm[1]` | -2000 / -500 | **1** | = −kp, consistent with kp=-1 |
| `act[joint_1..7].biasprm[2]` | -100 / -50 | **0** | arm kv dropped |
| `act[joint_x/y/th].biasprm[2]` | -50000/-50000/-1000 | **0** | base kv dropped (kp OK) |
| `sum(body_mass)` | 69.0905 | 70.0905 | +1.0 kg from the added scene/mocap target body; not robot-dynamics-relevant |

**Agree to precision (flagged only by the 1e-9 diff threshold; differ below display
precision from deg→rad rounding — NOT real mismatches):** all `actuator_ctrlrange`,
`jnt_range`, `key_qpos` (home), `fingers_actuator.gainprm[0]` (0.313725 vs 0.313726),
gripper `dof_damping`. `nq/nv/nu`, joint types, home keyframe presence: identical.

---

## Disposition of every brief hypothesis

| # | Hypothesis | Disposition | Why |
|---|---|---|---|
| 1 | Sim options (cone/impratio) | **CONFIRMED dropped, EXCLUDED as cause** | Force-elliptic experiment: still unstable (step 328, qacc 9e9). Real bug (RC2), not the NaN. |
| 2 | Home keyframe | **EXCLUDED** | Survives (ImportCompiles PASS; diff key_qpos agrees). Test resets to home (`MjMinkIKControllerTests.cpp:305`) before Bind. |
| 3 | Actuator set (gripper tendon) | **EXCLUDED** | Test drives the explicit 10; `fingers_actuator` (tendon) correctly skipped. With correct gains on the 10, stable. |
| 4 | Ctrl contention | **EXCLUDED (this harness)** | Only the mink controller writes `d->ctrl` (test calls `ComputeAndApply` then `mj_step`; no raw-mode `ApplyControls` runs). Live-loop risk noted below. |
| 5 | Units (cm/m leakage) | **EXCLUDED** | With correct gains, ctrl are joint-space targets tracking to 0.007 m; no scale error. |
| 6 | Posture-target / Bind order | **EXCLUDED (this harness)** | Test binds (`AdoptRuntimeController`, line 317) **after** `mj_resetDataKeyframe(home)` (line 305), so posture target = home, as in the example. Live-loop risk noted below. |
| 7 | dt divergence (0.002 vs 0.005) | **EXCLUDED as significant** | Fix-gains run at dt=0.002 reproduced Rung B's 0.005 checkpoints exactly. Negligible effect. |
| — | **Default-class actuator gains (NOT in brief)** | **CONFIRMED PRIMARY ROOT CAUSE** | Found by the model diff; proven by the fix-gains experiment. |

---

## Residual-risk list — what this headless harness cannot see (live editor)

The harness bypasses the physics thread, bridge, and mocap staging. These remain
possible contributors to the live-editor NaN in `docs/mink-ik-buzz-eod-report.md`
that Rung C does not exercise — **but note the actuator-gain bug (RC1) is present in
the live model too** (same importer), so once the items below are resolved, the live
"QACC at DOF 3/5" NaN is the same `kp = -1` runaway proven here:

- **Raw control-mode bypass (EOD #2).** An unknown `ControllerKind` (`mink_ik`) makes
  the articulation default to `control_mode=raw`; `AMjArticulation::ApplyControls`
  (`MjArticulation.cpp:732-747`) then sets `bSkipController` and writes `d->ctrl` from
  each actuator's `ResolveDesiredControl` — the mink solve never runs live. The
  harness calls `ComputeAndApply` directly and never hits this.
- **Physics-thread staging / cadence.** The engine calls `ApplyControls` every physics
  iteration (`MjPhysicsEngine.cpp:484`); the controller integrates once per sim-time
  advance and re-bases on backwards time (`MjMinkIKController.cpp:423-442`). Idle-tick
  and reset-race behavior is live-only.
- **Mocap re-stamping (EOD #3).** `UMjBody::TickComponent` re-stamps `mocap_pos` from
  the UE transform each tick. The harness streams targets via `ApplyConfig` (manual
  target precedence) and uses no mocap body, so never sees the race.
- **Step-request drain / micro-batching (EOD #5).** ~95% of 50 Hz single-step requests
  are dropped live; the harness steps in a tight deterministic loop.

**Reconciliation with the EOD "open bug":** the report's puzzle — "at a 1 rad/s vel
cap nothing can move enough in 0.1 s to legitimately diverge" — is resolved. The
divergence is **not** from the controller's ctrl magnitude/velocity; it is the
actuator gains themselves being destabilizing (`kp = -1`). Force = `(q − ctrl)` grows
from any perturbation regardless of how gently ctrl is ramped, which is why the vel
cap didn't help and why "posture-only / ctrl≈hold" survived longest (arm near
equilibrium, not commanded away).

---

## Experiment reproduction & tree state

- All experiments were done in an **uncommitted** local variant of
  `Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp` (env-gated overrides
  `URLAB_FORCE_ELLIPTIC` / `URLAB_FIX_GAINS`, a `[TASK5 diff]` model-diff block in
  `ImportCompiles`, and `[TASK5]` checkpoint/qacc logging). The test file was reverted
  to a clean state after capturing numbers.
- The full instrumentation is preserved for Task 6 to adopt as a regression harness:
  **`.superpowers/sdd/task-5-model-diff-experiment.patch`** (git-excluded; not part of
  the tree). Apply with `git apply` against the reverted test file.
- Build: `TestEditor` Development, `Result: Succeeded`. Runs via
  `Scripts/build_and_test_linux.sh --filter URLab.MinkIK.TidyBot`.
- Only production-code *reads* were done; **no production code was modified**. The only
  committed artifact from Task 5 is this findings document.
