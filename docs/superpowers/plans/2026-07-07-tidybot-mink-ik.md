# TidyBot mink IK Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `mink/examples/mobile_tidybot.py` run with full parity in Unreal via URLab — proven by a golden-trace comparison ladder (Python → native C++ → URLab-integrated) that also root-causes the known NaN divergence.

**Architecture:** The mink library port (`URLabMink`) and the data-driven `UMjMinkIKController` already exist. This plan adds: a vendored TidyBot fixture pinned to mink v1.2.0; a headless Python trace generator (Rung A); a native C++ 1:1 replica test with two-level golden comparison (Rung B); a URLab-integrated test (Rung C); differential root-cause + fix of the NaN; and a bridge-driven editor demo (Phase 3).

**Tech Stack:** UE 5.7 Linux, MuJoCo 3.9.0 (vendored), mink v1.2.0 (Python venv at `Scripts/mink_golden/.venv`), Eigen (URLabMink), UE Automation Tests.

**Spec:** `docs/superpowers/specs/2026-07-07-tidybot-mink-ik-design.md`

## Global Constraints

- Parity pin: **mink v1.2.0 + mujoco 3.9.0** (never regenerate fixtures with other versions — see `Scripts/mink_golden/README.md`).
- `URLabMink` is **parity-locked**: modify only with golden-trace evidence of a math bug.
- Shared working tree: commit **explicit file paths only** (never `git add -A`); never run `Scripts/format.sh` (format only your own files with `/home/stuart/miniconda3/bin/clang-format -i <files>`).
- Unity build is ON for URLab: file-scope statics in test .cpp files need unique names.
- Compile (editor-safe): `"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex` — success line `Result: Succeeded`.
- Automation tests (editor must be CLOSED): `./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "<Filter>"` — pass token `Result={Success}`.
- The controller writes `d->ctrl` only — **never `qpos`**.
- All paths below are relative to the plugin root `/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab` unless absolute.

## Shared reference: the deterministic target script

Used identically by Rung A (Python), Rung B (C++), Rung C (C++), and the Phase-3 demo script. Pure function of the outer-step index `k`; `p0`/`q0` are the EE (`pinch_site`) world position/orientation after resetting to the `home` keyframe.

```
N_STEPS      = 2500          # outer iterations
INTEGRATE_DT = 0.005         # Kevin's rate.dt (200 Hz) — used for solve/integrate
# sim advances by model timestep per mj_step (expected 0.002), exactly like the example

REACH = (0.0, 0.35, 0.15); FAR = (0.9, 0.0, 0.0)
k in [0, 300):     target pos = p0                                       # settle/hold
k in [300, 400):   target pos = p0 + s*REACH, s=(k-300)/100              # linear ramp
k in [400, 900):   target pos = p0 + REACH                               # hold (arm reach)
k in [900, 1000):  target pos = p0 + (1-s)*REACH + s*FAR, s=(k-900)/100  # ramp to far point
k in [1000, 2500): theta = 2*pi*(k-1000)/1200                            # far circle (forces base)
                   target pos = p0 + FAR + (0.4*(cos(theta)-1), 0.4*sin(theta), 0.0)
target quat = q0 throughout
```

(The circle starts continuously at `p0 + FAR` and has radius 0.4 centred at
`p0 + FAR - (0.4, 0, 0)`; this matches the Python/C++ code below exactly.)

Checkpoints for tracking asserts: **settled** k = {299, 899} (tolerance 0.02 m);
**moving-target** k = {999, 1600, 2200, 2499} (tolerance 0.10 m — the robot is
chasing a moving target there, lag is expected and fine).

## Trace fixture schema (`Scripts/mink_golden/fixtures/tidybot_trace.json`)

```json
{
  "meta": {"mink": "1.2.0", "mujoco": "3.9.0", "model": "stanford_tidybot/scene.xml",
            "n_steps": 2500, "integrate_dt": 0.005, "sim_timestep": 0.002,
            "joint_names": ["joint_x","joint_y","joint_th","joint_1","...","joint_7"]},
  "steps": [
    {"q_in": [18 floats],        // configuration.q before the inner loop
     "target_pos": [3], "target_quat": [4],   // wxyz
     "v0": [18],                 // solve_ik velocity of the FIRST inner iteration
     "q_out": [18],              // configuration.q after the inner loop
     "n_iters": 3,
     "ctrl": [10],               // data.ctrl values written (joint_names order)
     "q_sim": [18],              // data.qpos AFTER mj_step
     "ee_sim_pos": [3], "ee_sim_quat": [4]    // pinch_site from data after mj_step
    }, ...
  ]
}
```

`v0` is the exactness anchor (pure function of `q_in`+target — no chaos compounding). `q_out` is compared loosely (threshold-adjacent iteration-count wobble between qpmad and daqp). `q_sim`/`ee_sim_*` support the closed-loop similarity check.

---

### Task 1: Untangle the working tree (Phase 0)

**Files:**
- Branch `exp/mocap-weld-teleop` (new): snapshot of ALL current uncommitted work
- Commit to `feat/urlabmink-live-ik`: `Source/URLab/Public/MuJoCo/Components/Controllers/MjMinkIKController.h`, `Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp`, `Source/URLab/Public/MuJoCo/Core/MjArticulation.h`, `Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp` (EE hunks removed), `Source/URLabEditor/Private/MjEditorOpHandlers.cpp`, `docs/mink-ik-*.md` (4 files)
- Removed from working tree (live on exp branch only): `Source/URLab/Public/MuJoCo/Components/Controllers/MjEndEffectorController.h`, `Source/URLab/Private/MuJoCo/Components/Controllers/MjEndEffectorController.cpp`

**Interfaces:**
- Produces: a clean `feat/urlabmink-live-ik` tree where `UMjMinkIKController::MarkSpecsChanged()` + live `RebuildFromSpecs` and `AMjArticulation::AdoptRuntimeController(UMjArticulationController*)` are committed, and no `MjEndEffectorController` exists in-tree.

- [ ] **Step 1: Snapshot everything on the side branch**

```bash
cd "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab"
git checkout -b exp/mocap-weld-teleop
git add Source/URLab/Public/MuJoCo/Components/Controllers/MjEndEffectorController.h \
        Source/URLab/Private/MuJoCo/Components/Controllers/MjEndEffectorController.cpp \
        Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp \
        Source/URLab/Public/MuJoCo/Core/MjArticulation.h \
        Source/URLabEditor/Private/MjEditorOpHandlers.cpp \
        Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp \
        Source/URLab/Public/MuJoCo/Components/Controllers/MjMinkIKController.h
git commit -m "wip(teleop): snapshot mocap+weld EE-controller pivot (parked)"
git checkout feat/urlabmink-live-ik
```

Expected: `git status` on feat branch shows the EE controller files GONE from the working tree and no modifications to the five tracked files (except the untracked `docs/mink-ik-*.md` and the `third_party/build_all.sh` mode-bit change, which stays uncommitted).

- [ ] **Step 2: Bring back the mink-relevant versions**

```bash
git checkout exp/mocap-weld-teleop -- \
  Source/URLab/Public/MuJoCo/Components/Controllers/MjMinkIKController.h \
  Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp \
  Source/URLab/Public/MuJoCo/Core/MjArticulation.h \
  Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp \
  Source/URLabEditor/Private/MjEditorOpHandlers.cpp
```

- [ ] **Step 3: Strip the EE-only hunks from MjArticulation.cpp**

Three edits (Edit tool, exact strings from the current diff):
1. Remove the include line `#include "MuJoCo/Components/Controllers/MjEndEffectorController.h"`.
2. In `Setup(...)`, remove the whole block starting with the comment `// 7b. End-effector (mocap + weld) teleop controllers.` through the closing brace of the `{ TArray<UMjEndEffectorController*> ... }` scope.
3. In `PostSetup(...)`, remove the whole block starting with the comment `// Resolve any end-effector (mocap + weld) teleop controllers` through the closing brace of its scope (the block ending after the `neutralized %d actuator(s)` log).

Keep: `AdoptRuntimeController(...)` in full (it is mink-relevant — used by `add_controller`). Keep `MjArticulation.h` and `MjEditorOpHandlers.cpp` unmodified (their diffs are mink-only).

- [ ] **Step 4: Compile**

```bash
"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex
```
Expected: `Result: Succeeded`. If it fails on a dangling `UMjEndEffectorController` reference somewhere, remove that reference too (grep first: `grep -rn "MjEndEffectorController" Source/`).

- [ ] **Step 5: Commit (explicit paths)**

```bash
git add Source/URLab/Public/MuJoCo/Components/Controllers/MjMinkIKController.h \
        Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp \
        Source/URLab/Public/MuJoCo/Core/MjArticulation.h \
        Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp \
        Source/URLabEditor/Private/MjEditorOpHandlers.cpp
git commit -m "feat(MinkIK): live spec rebuild (MarkSpecsChanged) + runtime controller adoption for add_controller"
git add docs/mink-ik-buzz-eod-report.md docs/mink-ik-data-driven.md docs/mink-ik-generalisation.md docs/mink-ik-mobile-tidybot.md
git commit -m "docs(mink-ik): historical notes from the first integration attempt"
```

---

### Task 2: Vendor the TidyBot fixture + Rung A Python trace generator

**Files:**
- Create: `Scripts/mink_golden/models/stanford_tidybot/` (vendored from mink **v1.2.0 tag**: `scene.xml`, `tidybot.xml`, `README.md`, `assets/**` — 21 STL files)
- Create: `Scripts/mink_golden/gen_tidybot_trace.py`
- Create: `Scripts/mink_golden/fixtures/tidybot_trace.json` (generated, committed)
- Modify: `Scripts/mink_golden/README.md` (document the trace generator + model provenance/licenses)

**Interfaces:**
- Produces: `MinkLoadModel(TEXT("stanford_tidybot/scene.xml"))` works from C++ tests; `tidybot_trace.json` in the schema above; the target-script constants (documented in the plan header) that Tasks 3/4/7 re-implement.

- [ ] **Step 1: Vendor the model at the v1.2.0 tag**

```bash
cd "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Scripts/mink_golden"
SCRATCH="$(mktemp -d)"   # or the session scratchpad dir
curl -sL https://github.com/kevinzakka/mink/archive/refs/tags/v1.2.0.tar.gz -o "$SCRATCH/mink.tgz"
tar -xzf "$SCRATCH/mink.tgz" -C "$SCRATCH" \
    mink-1.2.0/examples/stanford_tidybot mink-1.2.0/examples/mobile_tidybot.py mink-1.2.0/LICENSE
cp -r "$SCRATCH/mink-1.2.0/examples/stanford_tidybot" models/stanford_tidybot
cp "$SCRATCH/mink-1.2.0/examples/mobile_tidybot.py" models/stanford_tidybot/mobile_tidybot.py.reference
cp "$SCRATCH/mink-1.2.0/LICENSE" models/stanford_tidybot/LICENSE.mink
```

Sanity: `grep -c "position name" models/stanford_tidybot/tidybot.xml` → expect 10 position actuators (+1 `general` fingers_actuator; total `<actuator>` children = 11). Verify the venv can load it:

```bash
./.venv/bin/python -c "import mujoco; m = mujoco.MjModel.from_xml_path('models/stanford_tidybot/scene.xml'); print(m.nq, m.nv, m.nu, m.opt.timestep)"
```
Expected: `18 18 11 0.002` (if timestep differs, update the plan constant everywhere and the fixture meta records it).

- [ ] **Step 2: Check the QP solver used by the venv**

```bash
./.venv/bin/pip show daqp || ./.venv/bin/pip list | grep -iE "daqp|qpsolvers"
```
Kevin's example uses `solver="daqp"`. If daqp is absent, check what `gen_golden.py` passes to `mink.solve_ik` (grep `solver=` in it) and use that same solver in the trace generator — consistency with the existing golden fixtures matters more than matching the example's string.

- [ ] **Step 3: Write `gen_tidybot_trace.py`**

```python
"""Headless mobile_tidybot.py: deterministic target script, per-step trace dump.

Replicates mink v1.2.0 examples/mobile_tidybot.py exactly (fix_base=False,
no pause), minus the viewer/keyboard. See docs/superpowers/plans/
2026-07-07-tidybot-mink-ik.md for the target script + schema.
"""
import json
from pathlib import Path

import mujoco
import numpy as np

import mink

_HERE = Path(__file__).parent
_XML = _HERE / "models" / "stanford_tidybot" / "scene.xml"
_OUT = _HERE / "fixtures" / "tidybot_trace.json"

N_STEPS = 2500
INTEGRATE_DT = 0.005  # the example's rate.dt at 200 Hz
SOLVER = "daqp"       # adjust per Step 2 if unavailable
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20

JOINT_NAMES = ["joint_x", "joint_y", "joint_th"] + [f"joint_{i}" for i in range(1, 8)]


def target_pos(k: int, p0: np.ndarray) -> np.ndarray:
    reach = np.array([0.0, 0.35, 0.15])
    far = np.array([0.9, 0.0, 0.0])
    if k < 300:
        return p0
    if k < 400:
        return p0 + (k - 300) / 100.0 * reach
    if k < 900:
        return p0 + reach
    if k < 1000:
        s = (k - 900) / 100.0
        return p0 + (1 - s) * reach + s * far
    theta = 2.0 * np.pi * (k - 1000) / 1200.0
    return p0 + far + np.array([0.4 * (np.cos(theta) - 1.0), 0.4 * np.sin(theta), 0.0])


def main() -> None:
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    dof_ids = np.array([model.joint(name).id for name in JOINT_NAMES])
    actuator_ids = np.array([model.actuator(name).id for name in JOINT_NAMES])

    configuration = mink.Configuration(model)

    end_effector_task = mink.FrameTask(
        frame_name="pinch_site", frame_type="site",
        position_cost=1.0, orientation_cost=1.0, lm_damping=1.0,
    )
    posture_cost = np.zeros((model.nv,))
    posture_cost[3:] = 1e-3
    posture_task = mink.PostureTask(model, cost=posture_cost)
    tasks = [end_effector_task, posture_task]
    limits = [mink.ConfigurationLimit(model)]

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    configuration.update(data.qpos)
    posture_task.set_target_from_configuration(configuration)
    mujoco.mj_forward(model, data)
    mink.move_mocap_to_frame(model, data, "pinch_site_target", "pinch_site", "site")

    site_id = model.site("pinch_site").id
    mocap_id = model.body("pinch_site_target").mocapid[0]
    p0 = data.mocap_pos[mocap_id].copy()
    q0 = data.mocap_quat[mocap_id].copy()  # wxyz

    steps = []
    for k in range(N_STEPS):
        data.mocap_pos[mocap_id] = target_pos(k, p0)
        data.mocap_quat[mocap_id] = q0
        T_wt = mink.SE3.from_mocap_name(model, data, "pinch_site_target")
        end_effector_task.set_target(T_wt)

        rec = {"q_in": configuration.q.tolist(),
               "target_pos": data.mocap_pos[mocap_id].tolist(),
               "target_quat": q0.tolist()}

        n_iters = 0
        v0 = None
        for i in range(MAX_ITERS):
            vel = mink.solve_ik(configuration, tasks, INTEGRATE_DT, SOLVER, damping=1e-3)
            if i == 0:
                v0 = vel.copy()
            configuration.integrate_inplace(vel, INTEGRATE_DT)
            n_iters = i + 1
            err = end_effector_task.compute_error(configuration)
            if (np.linalg.norm(err[:3]) <= POS_THRESHOLD
                    and np.linalg.norm(err[3:]) <= ORI_THRESHOLD):
                break

        data.ctrl[actuator_ids] = configuration.q[dof_ids]
        mujoco.mj_step(model, data)

        assert np.all(np.isfinite(data.qpos)), f"NaN qpos at step {k}"
        assert np.all(np.isfinite(data.qacc)), f"NaN qacc at step {k}"

        rec.update({
            "v0": v0.tolist(),
            "q_out": configuration.q.tolist(),
            "n_iters": n_iters,
            "ctrl": data.ctrl[actuator_ids].tolist(),
            "q_sim": data.qpos.tolist(),
            "ee_sim_pos": data.site_xpos[site_id].tolist(),
            "ee_sim_quat": _site_quat(data, site_id),
        })
        steps.append(rec)

    out = {"meta": {"mink": mink.__version__ if hasattr(mink, "__version__") else "1.2.0",
                    "mujoco": mujoco.__version__, "model": "stanford_tidybot/scene.xml",
                    "n_steps": N_STEPS, "integrate_dt": INTEGRATE_DT,
                    "sim_timestep": model.opt.timestep, "solver": SOLVER,
                    "joint_names": JOINT_NAMES},
           "steps": steps}
    _OUT.write_text(json.dumps(out))
    print(f"wrote {_OUT} ({len(steps)} steps, {_OUT.stat().st_size/1e6:.1f} MB)")


def _site_quat(data, site_id):
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
    return quat.tolist()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run it**

```bash
cd "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Scripts/mink_golden"
./.venv/bin/python gen_tidybot_trace.py
```
Expected: `wrote .../tidybot_trace.json (2500 steps, ~4-8 MB)`, **no assertion failures** — this is the proof that Kevin's algorithm on this model with this target script is stable. If it NaNs here, the target script is too aggressive: halve the circle speed (2400-step period) and re-run; update the constant in the plan header record via commit message.

- [ ] **Step 5: Eyeball the trace for physical sanity**

```bash
./.venv/bin/python - <<'EOF'
import json
t = json.load(open("fixtures/tidybot_trace.json"))
s = t["steps"]
for k in (299, 899, 999, 1600, 2200, 2499):
    import numpy as np
    err = np.linalg.norm(np.array(s[k]["ee_sim_pos"]) - np.array(s[k]["target_pos"]))
    print(k, "track_err_m=%.4f" % err, "base_xy=(%.2f, %.2f)" % (s[k]["q_sim"][0], s[k]["q_sim"][1]))
EOF
```
Expected: track errors ≤ ~0.02 at settled checkpoints (299, 899) and ≤ ~0.10 at
moving-target checkpoints (999, 1600, 2200, 2499); base_xy clearly nonzero by
k=1600 (the base drove). Record the actual values in the commit message — they
calibrate the C++ assertions. If the base never moves, the circle is within arm reach — increase `far` to `[1.2, 0, 0]`, re-run, and use the new value everywhere.

- [ ] **Step 6: Update README + commit**

Append to `Scripts/mink_golden/README.md`: a `## TidyBot trace` section stating the model provenance (mink v1.2.0 `examples/stanford_tidybot`, Apache-2.0, LICENSE.mink alongside; model originally derived from mujoco_menagerie stanford_tidybot), the generator invocation, and that `tidybot_trace.json` is committed like other fixtures.

```bash
git add Scripts/mink_golden/models/stanford_tidybot Scripts/mink_golden/gen_tidybot_trace.py \
        Scripts/mink_golden/fixtures/tidybot_trace.json Scripts/mink_golden/README.md
git commit -m "test(mink-golden): vendor stanford_tidybot @ mink v1.2.0 + headless trace generator (Rung A)"
```

---

### Task 3: Rung B — native C++ 1:1 replica with two-level golden comparison

**Files:**
- Create: `Source/URLabMink/Private/Tests/MinkTidybotTests.cpp`
- Test names: `URLab.Mink.TidyBot.SolverParity`, `URLab.Mink.TidyBot.ClosedLoopStable`

**Interfaces:**
- Consumes: `MinkLoadModel`, `MinkLoadFixture`, `MinkExpectNear`, `MinkJsonVec` (from `MinkTestUtils.h`); `FMinkConfiguration`, `FMinkFrameTask`, `FMinkPostureTask`, `FMinkConfigurationLimit`, `MinkSolveIK` (public URLabMink API, same usage as `MjMinkIKController.cpp`); `tidybot_trace.json` from Task 2.
- Produces: the decision bit — port math OK/broken. A shared C++ target-script helper `TidybotTargetPos(int32 K, const FMinkVec3& P0)` that Task 4 copies (unity-build: keep it in an anonymous namespace in each file).

- [ ] **Step 1: Write the test file**

```cpp
// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MinkTestUtils.h"
#include "MinkConfiguration.h"
#include "MinkSolveIK.h"
#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkPostureTask.h"
#include "Limits/MinkConfigurationLimit.h"
#include "Lie/MinkSE3.h"
#include "Lie/MinkSO3.h"
#include "mujoco/mujoco.h"

namespace MinkTidybotTestsLocal   // unity build: unique namespace name
{
constexpr int32 NSteps = 2500;
constexpr double IntegrateDt = 0.005;
constexpr double PosThreshold = 1e-4;
constexpr double OriThreshold = 1e-4;
constexpr int32 MaxIters = 20;

const TCHAR* JointNames[10] = {TEXT("joint_x"), TEXT("joint_y"), TEXT("joint_th"),
	TEXT("joint_1"), TEXT("joint_2"), TEXT("joint_3"), TEXT("joint_4"),
	TEXT("joint_5"), TEXT("joint_6"), TEXT("joint_7")};

/** The shared deterministic target script — MUST match gen_tidybot_trace.py. */
FMinkVec3 TidybotTargetPos(int32 K, const FMinkVec3& P0)
{
	const FMinkVec3 Reach(0.0, 0.35, 0.15);
	const FMinkVec3 Far(0.9, 0.0, 0.0);
	if (K < 300) return P0;
	if (K < 400) return P0 + (double(K - 300) / 100.0) * Reach;
	if (K < 900) return P0 + Reach;
	if (K < 1000)
	{
		const double S = double(K - 900) / 100.0;
		return P0 + (1.0 - S) * Reach + S * Far;
	}
	const double Theta = 2.0 * PI * double(K - 1000) / 1200.0;
	return P0 + Far + FMinkVec3(0.4 * (FMath::Cos(Theta) - 1.0), 0.4 * FMath::Sin(Theta), 0.0);
}

struct FTidybotRig
{
	mjModel* m = nullptr;
	mjData* d = nullptr;
	TUniquePtr<FMinkConfiguration> Config;
	TUniquePtr<FMinkFrameTask> Ee;
	TUniquePtr<FMinkPostureTask> Posture;
	TUniquePtr<FMinkConfigurationLimit> Limit;
	TArray<int32> DofIds, QposAdrs, ActIds;
	int32 SiteId = -1, MocapBodyId = -1;

	bool Init(FAutomationTestBase& T)
	{
		m = MinkLoadModel(TEXT("stanford_tidybot/scene.xml"));
		if (!m) { T.AddError(TEXT("failed to load stanford_tidybot/scene.xml")); return false; }
		d = mj_makeData(m);

		for (const TCHAR* Nm : JointNames)
		{
			const int32 J = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_UTF8(Nm));
			const int32 A = mj_name2id(m, mjOBJ_ACTUATOR, TCHAR_TO_UTF8(Nm));
			if (J < 0 || A < 0) { T.AddError(FString::Printf(TEXT("missing joint/actuator %s"), Nm)); return false; }
			DofIds.Add(m->jnt_dofadr[J]);
			QposAdrs.Add(m->jnt_qposadr[J]);
			ActIds.Add(A);
		}
		SiteId = mj_name2id(m, mjOBJ_SITE, "pinch_site");
		MocapBodyId = mj_name2id(m, mjOBJ_BODY, "pinch_site_target");
		if (SiteId < 0 || MocapBodyId < 0) { T.AddError(TEXT("missing site/mocap body")); return false; }

		const int32 KeyId = mj_name2id(m, mjOBJ_KEY, "home");
		if (KeyId < 0) { T.AddError(TEXT("no home keyframe")); return false; }
		mj_resetDataKeyframe(m, d, KeyId);

		Config = MakeUnique<FMinkConfiguration>(m);
		Config->Update(d->qpos);

		FMinkVec PostureCost = FMinkVec::Constant(m->nv, 1e-3);
		PostureCost.head(3).setZero();
		Posture = MakeUnique<FMinkPostureTask>(m, PostureCost);
		Posture->SetTargetFromConfiguration(*Config);

		Ee = MakeUnique<FMinkFrameTask>(TEXT("pinch_site"), EMinkFrameType::Site,
			FMinkVec::Constant(1, 1.0), FMinkVec::Constant(1, 1.0), /*gain*/ 1.0, /*lm*/ 1.0);
		Limit = MakeUnique<FMinkConfigurationLimit>(m);
		mj_forward(m, d);
		return true;
	}

	~FTidybotRig()
	{
		if (d) mj_deleteData(d);
		if (m) mj_deleteModel(m);
	}
};
} // namespace MinkTidybotTestsLocal

// ============================================================================
// URLab.Mink.TidyBot.SolverParity — replay recorded per-step solve inputs
// through the port; compare first-iteration velocity tightly and the
// integrated q loosely against Python mink v1.2.0.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTidybotSolverParity,
	"URLab.Mink.TidyBot.SolverParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMinkTidybotSolverParity::RunTest(const FString&)
{
	using namespace MinkTidybotTestsLocal;
	TSharedPtr<FJsonObject> Trace;
	if (!MinkLoadFixture(TEXT("tidybot_trace"), Trace))
	{
		AddError(TEXT("fixtures/tidybot_trace.json missing — run gen_tidybot_trace.py"));
		return false;
	}
	FTidybotRig R;
	if (!R.Init(*this)) return false;

	const TArray<TSharedPtr<FJsonValue>>* Steps = nullptr;
	Trace->TryGetArrayField(TEXT("steps"), Steps);
	if (!Steps || Steps->Num() == 0) { AddError(TEXT("empty trace")); return false; }

	TArray<const FMinkBaseTask*> Tasks = {R.Ee.Get(), R.Posture.Get()};
	TArray<const FMinkLimit*> Limits = {R.Limit.Get()};

	int32 NChecked = 0;
	for (int32 K = 0; K < Steps->Num(); ++K)
	{
		const TSharedPtr<FJsonObject> S = (*Steps)[K]->AsObject();
		const FMinkVec QIn = MinkJsonVec(S->GetArrayField(TEXT("q_in")));
		const FMinkVec TPos = MinkJsonVec(S->GetArrayField(TEXT("target_pos")));
		const FMinkVec TQuat = MinkJsonVec(S->GetArrayField(TEXT("target_quat")));
		const FMinkVec V0Exp = MinkJsonVec(S->GetArrayField(TEXT("v0")));
		const FMinkVec QOutExp = MinkJsonVec(S->GetArrayField(TEXT("q_out")));

		R.Config->Update(QIn);
		const double Wxyz[4] = {TQuat[0], TQuat[1], TQuat[2], TQuat[3]};
		R.Ee->SetTarget(FMinkSE3::FromRotationAndTranslation(
			FMinkSO3::FromWxyz(Wxyz), FMinkVec3(TPos[0], TPos[1], TPos[2])));

		// Level 1 (tight): first-iteration velocity — pure function of (q_in, target).
		const FMinkIKResult R0 = MinkSolveIK(*R.Config, Tasks, IntegrateDt, 1e-3, false, &Limits);
		if (!R0.IsSuccess()) { AddError(FString::Printf(TEXT("step %d: solve failed"), K)); return false; }
		if (!MinkExpectNear(*this, *FString::Printf(TEXT("v0 @ step %d"), K),
				R0.Velocity, V0Exp, 1e-6))
			return false; // stop at first divergence — everything after compounds

		// Full inner loop for q_out (loose: iteration-count wobble near thresholds).
		R.Config->Update(QIn);
		for (int32 It = 0; It < MaxIters; ++It)
		{
			const FMinkIKResult Rs = MinkSolveIK(*R.Config, Tasks, IntegrateDt, 1e-3, false, &Limits);
			if (!Rs.IsSuccess()) break;
			R.Config->IntegrateInplace(Rs.Velocity, IntegrateDt);
			FMinkVec Err;
			if (R.Ee->ComputeError(*R.Config, Err) && Err.size() >= 6
				&& Err.head(3).norm() <= PosThreshold && Err.tail(3).norm() <= OriThreshold)
				break;
		}
		if (!MinkExpectNear(*this, *FString::Printf(TEXT("q_out @ step %d"), K),
				FMinkMat(R.Config->GetQ()), FMinkMat(QOutExp), 1e-3))
			return false;
		++NChecked;
	}
	AddInfo(FString::Printf(TEXT("solver parity over %d recorded steps"), NChecked));
	return true;
}

// ============================================================================
// URLab.Mink.TidyBot.ClosedLoopStable — the literal 1:1 port of
// mobile_tidybot.py: full closed loop (solve -> ctrl -> mj_step) on raw
// MuJoCo. Finiteness every step; tracking + Python-trace agreement at
// checkpoints.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTidybotClosedLoop,
	"URLab.Mink.TidyBot.ClosedLoopStable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMinkTidybotClosedLoop::RunTest(const FString&)
{
	using namespace MinkTidybotTestsLocal;
	TSharedPtr<FJsonObject> Trace;
	MinkLoadFixture(TEXT("tidybot_trace"), Trace); // optional here; checkpoints compared if present

	FTidybotRig R;
	if (!R.Init(*this)) return false;

	// move_mocap_to_frame equivalent: target starts at the EE site.
	FMinkVec3 P0(R.d->site_xpos[3 * R.SiteId + 0], R.d->site_xpos[3 * R.SiteId + 1],
		R.d->site_xpos[3 * R.SiteId + 2]);
	double Q0[4];
	mju_mat2Quat(Q0, R.d->site_xmat + 9 * R.SiteId);

	TArray<const FMinkBaseTask*> Tasks = {R.Ee.Get(), R.Posture.Get()};
	TArray<const FMinkLimit*> Limits = {R.Limit.Get()};

	const TSet<int32> Checkpoints = {299, 899, 999, 1600, 2200, 2499};
	for (int32 K = 0; K < NSteps; ++K)
	{
		const FMinkVec3 TPos = TidybotTargetPos(K, P0);
		R.Ee->SetTarget(FMinkSE3::FromRotationAndTranslation(
			FMinkSO3::FromWxyz(Q0), TPos));

		for (int32 It = 0; It < MaxIters; ++It)
		{
			const FMinkIKResult Rs = MinkSolveIK(*R.Config, Tasks, IntegrateDt, 1e-3, false, &Limits);
			if (!Rs.IsSuccess()) { AddError(FString::Printf(TEXT("step %d: solve failed"), K)); return false; }
			R.Config->IntegrateInplace(Rs.Velocity, IntegrateDt);
			FMinkVec Err;
			if (R.Ee->ComputeError(*R.Config, Err) && Err.size() >= 6
				&& Err.head(3).norm() <= PosThreshold && Err.tail(3).norm() <= OriThreshold)
				break;
		}

		const FMinkVec Q = R.Config->GetQ();
		for (int32 i = 0; i < 10; ++i)
			R.d->ctrl[R.ActIds[i]] = Q[R.QposAdrs[i]];
		mj_step(R.m, R.d);

		for (int32 v = 0; v < R.m->nv; ++v)
			if (!FMath::IsFinite(R.d->qacc[v]) || !FMath::IsFinite(R.d->qvel[v]))
			{
				AddError(FString::Printf(TEXT("NON-FINITE qacc/qvel at step %d dof %d"), K, v));
				return false;
			}

		if (Checkpoints.Contains(K))
		{
			// Settled checkpoints hold a fixed target; moving ones chase the circle.
			const double Tol = (K == 299 || K == 899) ? 0.02 : 0.10;
			const FMinkVec3 Ee(R.d->site_xpos[3 * R.SiteId + 0],
				R.d->site_xpos[3 * R.SiteId + 1], R.d->site_xpos[3 * R.SiteId + 2]);
			const double TrackErr = (Ee - TPos).norm();
			TestTrue(FString::Printf(TEXT("checkpoint %d: track err %.4f <= %.2f m"), K, TrackErr, Tol),
				TrackErr <= Tol);

			if (Trace.IsValid()) // closed-loop similarity vs Python (loose)
			{
				const TArray<TSharedPtr<FJsonValue>>* Steps = nullptr;
				Trace->TryGetArrayField(TEXT("steps"), Steps);
				if (Steps && Steps->IsValidIndex(K))
				{
					const FMinkVec PyEe = MinkJsonVec(
						(*Steps)[K]->AsObject()->GetArrayField(TEXT("ee_sim_pos")));
					const double Diff = (Ee - FMinkVec3(PyEe[0], PyEe[1], PyEe[2])).norm();
					TestTrue(FString::Printf(TEXT("checkpoint %d: EE within 0.05 m of Python (%.4f)"), K, Diff),
						Diff <= 0.05);
				}
			}
		}
	}
	// The base must actually have driven (far-circle phase).
	TestTrue(TEXT("base moved > 0.2 m"),
		FMath::Sqrt(FMath::Square(R.d->qpos[R.QposAdrs[0]]) + FMath::Square(R.d->qpos[R.QposAdrs[1]])) > 0.2);
	return true;
}
```

- [ ] **Step 2: Compile, then run the tests (editor closed)**

```bash
"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex
./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "URLab.Mink.TidyBot"
```
Expected: `Result={Success}` for both tests. (Runtime note: SolverParity replays
2500 steps × up to ~40 QP solves ≈ tens of seconds on nv=18 — acceptable; if it
exceeds a few minutes, profile before sampling steps down, and if sampling is
ever introduced, log what fraction was skipped.)

- [ ] **Step 3: DECISION GATE — interpret the outcome**

- **Both pass** → the port math and the raw-MuJoCo loop are correct. The NaN is in URLab integration. Proceed to Task 4.
- **SolverParity fails** → genuine port math divergence. STOP the ladder; this becomes the bug. Use the failing step's `q_in`/`target` as a minimal fixture, bisect through the task/limit computations against Python (extend `gen_tidybot_trace.py` to dump intermediates for that step), fix in URLabMink, note the fix in the commit. Only then continue.
- **ClosedLoopStable fails but SolverParity passes** → stepping/actuation divergence in the raw loop (check: solver used, `IntegrateDt` constant, ctrl indices). Fix the test or the trace generator until they agree — they run the same algorithm and must match.

- [ ] **Step 4: Adjust tolerances only with evidence**

If `v0 @ step k` fails at 1e-6 but the max diff is ~1e-9–1e-8 scaled: qpmad/daqp round-off — tighten/keep. If it fails at ~1e-4+ on specific steps only: check whether those steps are at a joint limit or singularity (degenerate QP, multiple valid solutions); if verified degenerate, compare the *objective value* instead for those steps or skip-list them **with a comment naming the step and reason**. Do not blanket-loosen.

- [ ] **Step 5: Commit**

```bash
git add Source/URLabMink/Private/Tests/MinkTidybotTests.cpp
git commit -m "test(URLabMink): TidyBot 1:1 replica — golden solver parity + closed-loop stability vs mink v1.2.0 (Rungs A/B)"
```

---

### Task 4: Rung C — merged scene fixture + URLab-integrated test

**Files:**
- Create: `Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml` (single-worldbody merge of scene.xml — the importer fatal-crashes on the include/multi-worldbody pattern)
- Create: `Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp`
- Test names: `URLab.MinkIK.TidyBot.ImportCompiles`, `URLab.MinkIK.TidyBot.ClosedLoopStable`

**Interfaces:**
- Consumes: `FMjXmlImportSession` (from `Source/URLabEditor/Private/Tests/MjTestHelpers.h`: `InitFromFile(XmlPath)`, `Compile()`, `Model()`, `Data()`, `CountTemplates<T>()`, `Cleanup()` — see `MjMenagerieImportTests.cpp` for the exact usage pattern); `UMjMinkIKController` public API (`Tasks`, `Limits`, `DriveJoints`, `MaxIters`, `PosThreshold`, `OriThreshold`, `Bind`, `ComputeAndApply`, `ApplyConfig`); the target-script function (copy of Task 3's, in its own uniquely-named namespace).
- Produces: the in-URLab repro (expected RED at first — it documents the live bug) and, post-Task-6, the permanent regression gate.

- [ ] **Step 1: Create the merged scene**

Copy `tidybot.xml` → `tidybot_scene_ue.xml`, then apply these exact insertions (no `<include>` element anywhere in the result):

Into `<asset>` (after the existing entries):
```xml
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
```

Into `<worldbody>` (at the top):
```xml
    <light pos="0 0 1.5" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <body name="pinch_site_target" pos="0.5 0 .5" quat="0 1 0 0" mocap="true">
      <geom type="box" size=".05 .05 .05" contype="0" conaffinity="0" rgba=".6 .3 .3 .2"/>
    </body>
```

Verify it is equivalent to the include-form scene:
```bash
cd Scripts/mink_golden
./.venv/bin/python -c "
import mujoco
a = mujoco.MjModel.from_xml_path('models/stanford_tidybot/scene.xml')
b = mujoco.MjModel.from_xml_path('models/stanford_tidybot/tidybot_scene_ue.xml')
assert (a.nq, a.nv, a.nu, a.nbody, a.ngeom) == (b.nq, b.nv, b.nu, b.nbody, b.ngeom), 'count mismatch'
assert a.opt.integrator == b.opt.integrator and a.opt.cone == b.opt.cone
print('merged scene equivalent:', b.nq, b.nv, b.nu)
"
```
Expected: `merged scene equivalent: 18 18 11`.

- [ ] **Step 2: Write `URLab.MinkIK.TidyBot.ImportCompiles`**

Follows `MjMenagerieImportTests.cpp` exactly, plus **assertions that turn the top NaN suspects into pass/fail facts**:

```cpp
// (headers: CoreMinimal, AutomationTest, Tests/MjTestHelpers.h,
//  MuJoCo/Components/Controllers/MjMinkIKController.h, MjJoint/MjSite/MjBody/MjActuator, mujoco.h)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotImport,
	"URLab.MinkIK.TidyBot.ImportCompiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotImport::RunTest(const FString&)
{
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"));
	if (!FPaths::FileExists(XmlPath)) { AddError(TEXT("fixture missing — run Task 4 Step 1")); return false; }

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath)) { AddError(S.LastError); S.Cleanup(); return false; }
	if (!S.Compile()) { AddError(S.LastError); S.Cleanup(); return false; }

	mjModel* M = S.Model();
	TestEqual(TEXT("nq"), (int32)M->nq, 18);
	TestEqual(TEXT("nu"), (int32)M->nu, 11);
	// Suspect 1: the model's sim options must survive the import pipeline.
	TestEqual(TEXT("integrator == implicitfast"), (int32)M->opt.integrator, (int32)mjINT_IMPLICITFAST);
	TestEqual(TEXT("cone == elliptic"), (int32)M->opt.cone, (int32)mjCONE_ELLIPTIC);
	TestEqual(TEXT("impratio == 10"), M->opt.impratio, 10.0);
	// Suspect 2: home keyframe must survive with all 18 qpos.
	const int32 KeyId = mj_name2id(M, mjOBJ_KEY, "home"); // may be prefixed — fall back to suffix scan
	TestTrue(TEXT("home keyframe exists"), KeyId >= 0 || [&] {
		for (int32 i = 0; i < M->nkey; ++i)
		{
			const char* Nm = mj_id2name(M, mjOBJ_KEY, i);
			if (Nm && FString(UTF8_TO_TCHAR(Nm)).EndsWith(TEXT("home"))) return true;
		}
		return false;
	}());
	// Base actuator gains survived (kp=1e6 on joint_x — find by name suffix).
	{
		int32 JxAct = -1;
		for (int32 a = 0; a < M->nu; ++a)
		{
			const char* Nm = mj_id2name(M, mjOBJ_ACTUATOR, a);
			if (Nm && FString(UTF8_TO_TCHAR(Nm)).EndsWith(TEXT("joint_x"))) { JxAct = a; break; }
		}
		TestTrue(TEXT("joint_x actuator found"), JxAct >= 0);
		if (JxAct >= 0)
		{
			TestEqual(TEXT("joint_x kp == 1e6"), M->actuator_gainprm[JxAct * mjNGAIN + 0], 1000000.0);
		}
	}
	S.Cleanup();
	return true;
}
```

- [ ] **Step 3: Write `URLab.MinkIK.TidyBot.ClosedLoopStable`**

Same import session; then configure the actual `UMjMinkIKController` exactly as the example, drive the same target script, step directly (`ComputeAndApply` + `mj_step` — this exercises importer+model+controller; the physics-thread staging layer is exercised in Task 7's live run):

```cpp
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotClosedLoop,
	"URLab.MinkIK.TidyBot.ClosedLoopStable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotClosedLoop::RunTest(const FString&)
{
	// 1. Import + compile (as above). FMjXmlImportSession exposes `Robot`
	//    (AMjArticulation*) and `Blueprint` directly — use S.Robot; collect its
	//    components via GetComponents<T>() (see MjBaseDriveControllerTests.cpp
	//    for the component-collection idiom).
	// 2. Find components: UMjSite whose MjName ends with "pinch_site", UMjBody
	//    ending "pinch_site_target", UMjJoint components for the 10 example joints.
	// 3. Create the controller on the articulation actor:
	UMjMinkIKController* Ctrl = NewObject<UMjMinkIKController>(RobotActor, TEXT("MinkIK"));
	FMinkTaskSpec Frame;                       // the example's FrameTask
	Frame.Kind = EMinkTaskKind::Frame;
	Frame.Frame = PinchSite;
	Frame.TargetMocapBody = nullptr;           // targets streamed via ApplyConfig (mocap is UE-authoritative)
	Frame.PositionCost = 1.0f; Frame.OrientationCost = 1.0f; Frame.LmDamping = 1.0f;
	FMinkTaskSpec Posture;                     // cost[3:] = 1e-3 == arm joints only
	Posture.Kind = EMinkTaskKind::Posture;
	Posture.Cost = 1e-3f;
	Posture.Joints = ArmJoints;                // joint_1..joint_7 components
	FMinkTaskSpec Damping;                     // fix_base task, disabled (fix_base=False)
	Damping.Kind = EMinkTaskKind::Damping;
	Damping.Cost = 100.0f;
	Damping.Joints = BaseJoints;               // joint_x/joint_y/joint_th
	Damping.bEnabled = false;
	Ctrl->Tasks = {Frame, Posture, Damping};
	FMinkLimitSpec ConfLimit;                  // mink.ConfigurationLimit(model)
	Ctrl->Limits = {ConfLimit};
	Ctrl->DriveJoints = TenJoints;             // EXACTLY the example's 10 (not the gripper)
	Ctrl->MaxIters = 20;
	Ctrl->RegisterComponent();
	// 4. Reset to home keyframe (suffix-resolve key id), mj_forward.
	// 5. Ctrl->Bind(M, D, ActuatorIdMap)  — build the map from the articulation's
	//    bound actuators as PostSetup does (or call the session/articulation helper
	//    that does it; check AMjArticulation::PostSetup for the exact map source).
	// 6. Loop k = 0..2499:
	//      - target = TidybotTargetPos(k, P0) (P0/Q0 from pinch_site after reset)
	//      - stream it: build FJsonObject {"target_pos": [...], "target_quat": [...]}
	//        and Ctrl->ApplyConfig(Json)   // the supported streaming path
	//      - Ctrl->ComputeAndApply(M, D, 0);
	//      - mj_step(M, D);
	//      - assert qacc/qvel finite (same as Rung B) — on failure, AddError with
	//        step index + the last ctrl vector, and dump m->opt.integrator etc.
	//    Checkpoints: same tracking asserts as Rung B (2 cm), plus base-moved check.
	return true;
}
```

The commented steps 1/2/4/5 must be written out against the real helper APIs (read `MjTestHelpers.h` `FMjXmlImportSession` in full and `MjBaseDriveControllerTests.cpp` first; the exact component-collection idiom exists there). NOTE the dt nuance: `ComputeAndApply` computes `Dt` from `d->time` deltas — first call uses `m->opt.timestep`. That is the documented divergence from the example's 0.005; the test asserts stability and tracking, not trace-parity, at this rung.

- [ ] **Step 4: Compile + run**

```bash
"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex
./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "URLab.MinkIK.TidyBot"
```

Expected: `ImportCompiles` outcome is diagnostic gold either way (a failed option/keyframe assert = suspect confirmed cheaply). `ClosedLoopStable` is **expected to fail initially** — that's the reproduced bug, now in a headless deterministic harness instead of a live editor.

- [ ] **Step 5: Commit (including the red test — it documents the live bug)**

```bash
git add Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml \
        Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp
git commit -m "test(MinkIK): TidyBot URLab-integrated repro (Rung C) — ImportCompiles asserts sim options/keyframe; ClosedLoopStable red pending NaN root-cause"
```

---

### Task 5: Differential root-cause (investigation)

**Files:**
- Read/instrument as needed: `Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp` (`ApplyControls`, `PostSetup`), `Source/URLab/Private/MuJoCo/Core/MjPhysicsEngine.*` (who calls `ComputeAndApply`, who else writes `d->ctrl`), `Source/URLabEditor/Private/MujocoImportFactory.cpp` (option/keyframe handling)
- Create: `docs/superpowers/specs/2026-07-07-tidybot-nan-findings.md` (the deliverable: root cause + evidence)

**Interfaces:**
- Consumes: green Rung B, red (or green!) Rung C from Tasks 3–4.
- Produces: a written root cause with a reproducing assertion, consumed by Task 6.

- [ ] **Step 1: If Rung C already passed** — the bug lives in the layers Rung C bypasses (physics-thread staging, live mocap re-stamping, bridge). Skip to Step 4.

- [ ] **Step 2: Model diff.** In `ImportCompiles` (or a scratch variant), dump both models and compare field-by-field:

```cpp
// Rung B model: MinkLoadModel("stanford_tidybot/scene.xml") (native, ground truth)
// Rung C model: S.Model() (imported)
// Compare and AddInfo any mismatch in:
//   opt: timestep, integrator, cone, impratio, iterations, tolerance, solver
//   per-actuator (matched by name suffix): gainprm[0..2], biasprm[0..2],
//     ctrlrange, forcerange, trntype, trnid
//   per-joint (by suffix): type, range, armature (dof_armature), damping (dof_damping),
//     frictionloss, qposadr ordering
//   per-body (by suffix): mass, inertia diagonal
//   nkey + key_qpos content for "home"
```
Write this as a helper in the test file (`AddInfo` per mismatch) — it stays as permanent import-fidelity documentation. Any mismatch found here is a candidate root cause; fix priority goes to dynamics-relevant fields (options, gains, armature, masses).

- [ ] **Step 3: Ordered hypothesis walk (each is one cheap experiment on the red Rung C test):**

1. **Sim options** — already asserted in ImportCompiles. If red: fix importer option pass-through (Task 6).
2. **Home keyframe** — already asserted. If red or key_qpos wrong: fix keyframe transfer.
3. **Actuator set** — flip the test's `DriveJoints` between the explicit 10 and empty (= all bound actuators incl. `fingers_actuator`). If explicit-10 is stable and empty NaNs: root cause is driving the tendon-transmission gripper actuator with a qpos value (`QposAddr` for a tendon actuator binding is meaningless — check what `Super::Bind` puts in `FActuatorBinding.QposAddr` for `mjTRN_TENDON`).
4. **Ctrl contention** — grep every writer of `d->ctrl`/staged ctrl: `grep -rn "->ctrl\[" Source/URLab/ | grep -v Test`. Check `ApplyControls(bSkipController)` call sites and whether raw-mode actuator values are also applied when a controller is cached.
5. **Units** — in the red run, log the first 5 ctrl vectors; slide-joint targets must be O(0.1–1) metres, not O(10–100) (cm leakage).
6. **Start pose** — if home keyframe was silently absent, qpos=0 self-collision + kp=1e6 is a classic instant-NaN; confirmed by (2).

- [ ] **Step 4: If Rung C is green but the editor still NaNs** (the EOD scenario) — the remaining delta is the live physics loop. Instrument `AMjArticulation::ApplyControls` and the physics engine's step site with a 20-step ctrl/qpos/qacc trace gated behind a CVar or `#if !UE_BUILD_SHIPPING` guard, run the Phase-3 setup (Task 7 Step 1–2) with the editor, and diff the live trace against Rung C's. Suspects unique to this layer: `ComputeAndApply` invoked with a stale/wrong `d`; mocap UE-authoritative re-stamping racing the solve; step-request batching draining ctrl at the wrong cadence.

- [ ] **Step 5: Write findings**

`docs/superpowers/specs/2026-07-07-tidybot-nan-findings.md`: root cause(s), the discriminating experiment for each, and the minimal fix proposal. Commit:
```bash
git add docs/superpowers/specs/2026-07-07-tidybot-nan-findings.md
git commit -m "docs(mink-ik): TidyBot NaN root-cause findings"
```

---

### Task 6: The fix + green regression gate

**Files:**
- Modify: whatever Task 5 names (importer / MjArticulation / controller — NOT URLabMink unless Task 3's gate proved a math bug)
- Modify: `Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp` (add the regression assertion for the specific root cause, if not already one of the ImportCompiles asserts)

**Interfaces:**
- Consumes: findings doc from Task 5.
- Produces: `URLab.MinkIK.TidyBot.ClosedLoopStable` green; all pre-existing suites still green.

- [ ] **Step 1: Implement the minimal fix for the confirmed root cause** (content determined by Task 5 — keep it surgical; one commit per independent cause).
- [ ] **Step 2: Run the full ladder**

```bash
./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "URLab.Mink"
./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "URLab.MinkIK.TidyBot"
```
Expected: `Result={Success}` on both filters — the whole ladder green.
- [ ] **Step 3: Regression sweep** — run the pre-existing suites touched by the fix (at minimum `--filter "URLab.Import"` if the importer changed, `--filter "URLab.Nav"` if MjArticulation changed). Expected: no new failures vs their pre-fix status.
- [ ] **Step 4: Commit** (explicit paths; message names the root cause, e.g. `fix(import): carry <option> integrator/cone/impratio through mjs_attach — TidyBot NaN root cause`).

---

### Task 7: Phase 3 — editor demo parity (bridge-driven E2E)

**Files:**
- Create: `Scripts/demos/tidybot_mink_demo.py` (or the repo's established client-script location — locate first: `grep -rn "configure_controller\|set_mocap_pose" docs/ Scripts/ --include=*.py --include=*.md`)
- Modify (only if needed): `docs/guides/` — a short "TidyBot mink IK demo" guide

**Interfaces:**
- Consumes: everything green through Task 6; the bridge ops that already exist (`scene.import_xml`, `add_controller`, `configure_controller`, sim reset/keyframe op, state queries — verify exact names in `Source/URLabEditor/Private/MjEditorOpHandlers.cpp` before writing the script).
- Produces: the spec's acceptance evidence (logs + screenshots).

- [ ] **Step 1: Locate the Python bridge client** (the EOD report used `client.scene.import_xml`, `client.runtime.configure_controller` — find the package/repo path, e.g. via `grep -rn "import_xml" docs/`). If the client lives outside this repo, note its path in the demo script header.

- [ ] **Step 2: Write the demo script** implementing the spec's setup: import `tidybot_scene_ue.xml` → `add_controller` with the exact example stack (same spec values as Task 4 Step 3, JSON form; DriveJoints = the 10) → apply the `home` keyframe → move the mocap body's UE actor to the current `pinch_site` pose (game-thread op — check for an existing `set_actor_transform`-style op; if none exists, position the mocap by streaming the first target instead and note it) → stream the shared target script at ~50 Hz with big step batches (the EOD found per-request micro-batches drop steps; use `step(n_steps=N)` batches) → poll qpos/EE each phase → assert the acceptance criteria (near-reach tracking, base drive on far circle, zero resets) → screenshot captures at each checkpoint.

- [ ] **Step 3: Run it with the editor open** (I drive: launch editor, start Simulate, run the script). Record pass/fail per acceptance item. Then the two manual-parity toggles, still scripted where possible:
  - `fix_base`: `configure_controller {"task_enabled": [true, true, true]}` (Damping on) → far target → assert base qpos stays within 2 cm while the arm stretches; then toggle back off.
  - Gizmo drag: drag `pinch_site_target` in the viewport (mocap is UE-authoritative — this is the supported gizmo path; note manual targets take precedence, so clear/avoid streamed targets first, i.e. use a `TargetMocapBody`-configured controller variant for this check or re-add the controller with the mocap body set).

- [ ] **Step 4: Commit the demo script + guide; update the findings doc with the acceptance evidence.**

```bash
git add Scripts/demos/tidybot_mink_demo.py docs/guides/tidybot-mink-ik-demo.md
git commit -m "demo(mink-ik): TidyBot bridge-driven demo — full mobile_tidybot.py parity in-editor"
```

---

## Execution notes

- Tasks 1→2→3 are strictly sequential (3 needs 2's fixture; the Task 3 decision gate governs everything after).
- Task 4 can start its Step 1 (merged scene) in parallel with Task 3.
- Tasks 5/6 are investigation-shaped: their steps are experiments, not predetermined edits. Keep each confirmed cause as its own commit with its own regression assertion.
- Task 7 needs the editor and me driving it interactively over the bridge; everything before it is headless.
- One targeted correction from the gizmo-path note in Task 7: the two target routes (mocap body vs streamed manual) are mutually exclusive per the controller's precedence rule — the demo should exercise both, reconfiguring between them via `add_controller` (idempotent update path exists).
