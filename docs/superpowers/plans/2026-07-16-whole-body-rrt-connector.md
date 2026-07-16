# Whole-Body RRT-Connect Free-Space Connector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A client-side 10-DoF (3 holonomic base + 7 arm) RRT-Connect planner over the bridge's compiled-model replica, executed in joint space through a new `posture_target` wire param on the mink controller — validated by finishing the suction pick (`GRIPPED+LIFTED`).

**Architecture:** Planner proposes (pure-Python collision-checked path over `client.model`), QP disposes (the existing whole-body mink controller tracks the plan as timed posture targets), guards insure (collision-avoidance limits stay active). The engine change is one wire param exposing the already-battle-tested `FMinkPostureTask::SetTarget`. Spec: `docs/superpowers/specs/2026-07-16-whole-body-rrt-connector-design.md`.

**Tech Stack:** UE 5.7 C++ (MjMinkIKController), MuJoCo 3.10 Python bindings, python-mink 1.2.0, numpy, py_trees, the URLab_Bridge client.

## Global Constraints

- Engine root: `/home/stuart/UE_ROOT/UnrealEngine`. Host project: `/home/stuart/Documents/Unreal Projects/Test/Test.uproject`.
- Compile (safe with editor open): `"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex` — success line `Result: Succeeded`.
- Automation tests REQUIRE the editor CLOSED (coordinate with the human): `./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "URLab.MinkIK"` — pass token `Result={Success}`.
- Python interpreter for ALL Scripts/demos work: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3`.
- The bridge venv's `mujoco` MUST stay at `3.10.0` (matches the server's mjb format). Do NOT apply `Scripts/mink_golden/requirements.txt` (it pins mujoco==3.9.0).
- Golden model `Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml` must stay byte-identical.
- Commit style: NO `Co-Authored-By` / `Claude-Session` trailers. NEVER `git add -A` — stage explicit paths only. Never stage `third_party/build_all.sh`, `Scripts/mink_golden/fixtures/*`, `docs/tidybot-nan-root-cause.txt`, or any generated `*_ue_ue.xml`.
- Shared tree: work in the main checkout (no worktree). Other agents may have uncommitted WIP.
- Unity build is ON: statics/namespaces in test .cpp files need file-unique names.
- clang-format ONLY your own touched C++ files: `/home/stuart/miniconda3/bin/clang-format -i <files>` (never `Scripts/format.sh`).
- Task spec indices in the demo payload: `EE_TASK, POSTURE_TASK, DAMPING_TASK, TWIST_TASK = 0, 1, 2, 3` (defined in `Scripts/demos/tidybot_twist_follow_demo.py:61`).
- Joint names (order matters): `joint_x, joint_y, joint_th` (base), `joint_1..joint_7` (arm). `BASE_JOINTS`/`ARM_JOINTS` constants live in `Scripts/demos/tidybot_mink_demo.py:77-78`.

---

### Task 1: `posture_target` wire in MjMinkIKController + automation test

**Files:**
- Modify: `Source/URLab/Public/MuJoCo/Components/Controllers/MjMinkIKController.h` (near `FManualTarget` struct ~line 390 and `ManualTargets` member ~line 438)
- Modify: `Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp` (five insertion points, given below)
- Test: `Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp` (append one test)

**Interfaces:**
- Produces (wire): `configure_controller` params `"posture_target": {"<joint name>": <qpos number>, ...}` (empty map `{}` clears the latch) and optional `"posture_task": <int spec index>` (default: first non-TwistFollow Posture spec). `get_controller_params` gains a `"posture_target"` object field (the latched map; empty object when none). Tasks 3-5 stream through this.
- Semantics later tasks rely on: the latched target is re-applied EVERY solve in reference space (unmentioned joints carry the current reference q); joints with zero cost in the routed task's subset are inert; TwistFollow overwrites its own spec's target while enabled.

**Background for the implementer (read these before coding):**
- `MjMinkIKController.cpp:697-702` — how `ManualTargets` is snapshotted under `TargetMutex` so the solve never holds the lock.
- `MjMinkIKController.cpp:852-897` — the TwistFollow block: the exact "build full `TargetQ` from `Config.GetQ()`, overwrite some entries, `B.AsPosture->SetTarget(TargetQ)`" pattern you are replicating.
- `MjMinkIKController.cpp:1272-1317` — the `target_pos` parse block: the pattern for latching a manual target from JSON under `TargetMutex`.
- `MjMinkIKController.cpp:179` — `SubsetCost`: why zero-cost joints are inert.

- [ ] **Step 1: Header — add the latch struct + members**

In `MjMinkIKController.h`, immediately after the `FManualTarget` struct definition (~line 397), add:

```cpp
	/** Latched manual joint-space posture target (the "posture_target" wire
	 *  param). JointQ: MuJoCo joint name -> qpos value. Re-applied to the
	 *  routed posture spec EVERY solve in reference space — unmentioned
	 *  joints carry the current reference q (TwistFollow's pattern).
	 *  bSet=false means no latch. */
	struct FManualPostureTarget
	{
		TMap<FString, double> JointQ;
		int32 SpecIndex = INDEX_NONE; // INDEX_NONE = first non-TwistFollow Posture spec
		bool bSet = false;
	};
```

Next to the `ManualTargets` member (~line 438), add:

```cpp
	FManualPostureTarget ManualPosture; // guarded by TargetMutex
	/** Re-armed to 8 on each posture_target apply; decremented per unknown-name
	 *  warning on the physics thread. Benign race, log-budget only. */
	int32 PostureNameWarnBudget = 0;
```

If `TargetMutex` is not already declared `mutable FCriticalSection`, make it `mutable` (Step 5's readback locks it from a const method).

- [ ] **Step 2: cpp — clear on Bind, snapshot in solve**

Find the `ManualTargets.Reset();` call (`MjMinkIKController.cpp:289`) and add directly after it:

```cpp
	ManualPosture = FManualPostureTarget();
```

Change the snapshot block at lines 697-702 to also copy the posture latch:

```cpp
	// Snapshot manual targets so the solve loop never holds the lock.
	TMap<int32, FManualTarget> Manual;
	FManualPostureTarget ManualP;
	{
		FScopeLock Lock(&TargetMutex);
		Manual = ManualTargets;
		ManualP = ManualPosture;
	}
```

- [ ] **Step 3: cpp — apply the latch each solve**

Insert AFTER the `bFirstIntegration` re-base block ends and `LastSimTime = Now;` (line 850), BEFORE the TwistFollow comment block (line 852):

```cpp
	// Manual joint-space posture target (the posture_target wire): re-applied
	// every solve like TwistFollow, in REFERENCE space — unmentioned joints
	// carry the current reference q. Routed to the explicit "posture_task"
	// spec index, else the first non-TwistFollow Posture spec. Deliberately
	// placed BEFORE the TwistFollow loop: if a caller routes it at a
	// TwistFollow spec while that task is enabled, TwistFollow's per-step
	// write wins — the documented precedence.
	if (ManualP.bSet)
	{
		for (FMinkIKState::FBuiltTask& B : Mink->BuiltTasks)
		{
			if (!B.AsPosture)
			{
				continue;
			}
			if (ManualP.SpecIndex == INDEX_NONE ? B.bTwistFollow : ManualP.SpecIndex != B.SpecIndex)
			{
				continue;
			}
			const bool bTaskOn = !CfgSpecEnabled.IsValidIndex(B.SpecIndex) || CfgSpecEnabled[B.SpecIndex];
			if (bTaskOn)
			{
				FMinkVec TargetQ = Config.GetQ();
				for (const TPair<FString, double>& JQ : ManualP.JointQ)
				{
					const int32 Jid = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_ANSI(*JQ.Key));
					if (Jid < 0 || (m->jnt_type[Jid] != mjJNT_SLIDE && m->jnt_type[Jid] != mjJNT_HINGE))
					{
						if (PostureNameWarnBudget-- > 0)
						{
							UE_LOG(LogURLabRuntime, Warning,
								TEXT("[MinkIK] posture_target: joint '%s' %s — entry ignored."),
								*JQ.Key,
								Jid < 0 ? TEXT("not found") : TEXT("is not a scalar (slide/hinge) joint"));
						}
						continue;
					}
					TargetQ[m->jnt_qposadr[Jid]] = JQ.Value;
				}
				B.AsPosture->SetTarget(TargetQ);
			}
			break; // routed to exactly one spec (latched even if that task is disabled)
		}
	}
```

Note `Config.GetQ()` returns the same `FMinkVec` the TwistFollow block uses at line 871 — if the actual accessor name differs, copy whatever line 871 does.

- [ ] **Step 4: cpp — parse in ApplyConfigInternal**

Append after the `target_pos` block closes (line ~1317, just before the function's closing brace):

```cpp
	// Streamed joint-space posture target (MuJoCo joint names -> qpos values).
	//   "posture_target": {"<joint>": q, ...}   empty map {} clears the latch
	//   "posture_task":   <spec index>          optional routing; default = the
	//                                           first non-TwistFollow Posture spec
	// Entries whose joints carry ZERO cost in the routed task's subset are
	// inert in the QP — the payload must weight every joint it wants driven.
	// While a TwistFollow task is enabled it rewrites its own spec's target
	// every step and wins over a posture_target routed at it.
	const TSharedPtr<FJsonObject>* PostureObj = nullptr;
	if (InParams->TryGetObjectField(TEXT("posture_target"), PostureObj) && PostureObj && PostureObj->IsValid())
	{
		FManualPostureTarget P;
		for (const auto& Pair : (*PostureObj)->Values)
		{
			double Q = 0.0;
			if (Pair.Value.IsValid() && Pair.Value->TryGetNumber(Q))
			{
				P.JointQ.Add(Pair.Key, Q);
			}
			else
			{
				UE_LOG(LogURLabRuntime, Warning,
					TEXT("[MinkIK] posture_target['%s']: expected a number — entry ignored."), *Pair.Key);
			}
		}
		double IdxV = 0.0;
		if (InParams->TryGetNumberField(TEXT("posture_task"), IdxV))
		{
			P.SpecIndex = (int32)IdxV;
		}
		P.bSet = P.JointQ.Num() > 0; // "posture_target": {} clears the latch
		FScopeLock Lock(&TargetMutex);
		ManualPosture = MoveTemp(P);
		PostureNameWarnBudget = 8; // re-arm the unknown-name warning per apply
	}
```

- [ ] **Step 5: cpp — schema + readback**

In `GetConfigSchema` (after the `task_costs` line, ~1107):

```cpp
	OutSchema->SetStringField(TEXT("posture_target"),
		TEXT("map<joint name, qpos> (empty map clears; zero-cost subset entries inert)"));
	OutSchema->SetStringField(TEXT("posture_task"),
		TEXT("int (routing spec index; default first non-twist posture)"));
```

In `GetCurrentConfigInternal` (after the `task_costs` array is set, ~line 1142):

```cpp
	// Latched manual posture target (empty object when none is set).
	{
		TSharedPtr<FJsonObject> P = MakeShared<FJsonObject>();
		FScopeLock Lock(&TargetMutex);
		if (ManualPosture.bSet)
		{
			for (const TPair<FString, double>& JQ : ManualPosture.JointQ)
			{
				P->SetNumberField(JQ.Key, JQ.Value);
			}
		}
		OutParams->SetObjectField(TEXT("posture_target"), P);
	}
```

- [ ] **Step 6: Write the automation test**

Append to `Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp`. Reuse the file's existing scaffold verbatim: sections 1, 2, and 4 of `FMjMinkIKTidybotFixBaseHolds` (`MjMinkIKControllerTests.cpp:1278-1404` — import/compile, component resolution into `TenJoints`/`QposAdrs`, home-keyframe reset + `AdoptRuntimeController`) and its `RunSteps` lambda (1435-1450). The differences are the task stack and the drive phases:

```cpp
// ============================================================================
// URLab.MinkIK.TidyBot.PostureTargetTracks
//   The posture_target wire (whole-body RRT connector execution channel):
//   a posture task whose cost subset spans ALL TEN joints (base + arm) must
//   track a streamed joint-space target when the frame task is disabled and
//   the posture cost is bumped live. Also: readback round-trip, {} clears,
//   unknown joint warns (budgeted), zero-cost subset entries are inert.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotPostureTargetTracks,
	"URLab.MinkIK.TidyBot.PostureTargetTracks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotPostureTargetTracks::RunTest(const FString&)
{
	using namespace MjMinkIKControllerTestsLocal;
	// [scaffold: import + compile + resolve PinchSite/TenJoints/QposAdrs — copy
	//  from FMjMinkIKTidybotFixBaseHolds sections 1-2 unchanged]

	// Stack: frame (will be disabled), posture over ALL TEN joints, base damping.
	UMjMinkIKController* Ctrl = NewObject<UMjMinkIKController>(Robot, TEXT("MinkIKPostureTarget"));
	FMinkTaskSpec Frame;
	Frame.Kind = EMinkTaskKind::Frame;
	Frame.Frame = PinchSite;
	Frame.TargetMocapBody = nullptr;
	Frame.PositionCost = 1.0f;
	Frame.OrientationCost = 1.0f;
	Frame.LmDamping = 1.0f;
	FMinkTaskSpec Posture;
	Posture.Kind = EMinkTaskKind::Posture;
	Posture.Cost = 1e-3f;
	Posture.Joints = TenJoints; // ALL TEN — the connector payload's divergence
	FMinkTaskSpec Damping;
	Damping.Kind = EMinkTaskKind::Damping;
	Damping.Cost = 5.0f; // lazy base
	Damping.Joints = BaseJoints;
	Ctrl->Tasks = {Frame, Posture, Damping};
	FMinkLimitSpec ConfLimit;
	Ctrl->Limits = {ConfLimit};
	Ctrl->DriveJoints = TenJoints;
	Ctrl->MaxIters = MaxIters;
	Ctrl->PosThreshold = PosThreshold;
	Ctrl->OriThreshold = OriThreshold;
	Ctrl->RegisterComponent();

	// [scaffold: home keyframe reset + AdoptRuntimeController + IsBound check +
	//  RunSteps lambda — copy from FixBaseHolds section 4 unchanged]

	// --- Phase A: stream a displaced whole-body target, must track -------------
	// Displacements chosen small enough to be reachable from home in 2000 steps
	// under the configuration limit, large enough to be unambiguous.
	const double Delta[10] = {0.30, -0.20, 0.40, 0.20, -0.20, 0.20, -0.20, 0.20, -0.20, 0.20};
	double TargetQ[10];
	for (int32 i = 0; i < 10; ++i)
	{
		TargetQ[i] = D->qpos[QposAdrs[i]] + Delta[i];
	}
	{
		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> Enabled;
		Enabled.Add(MakeShared<FJsonValueBoolean>(false)); // frame OFF
		Enabled.Add(MakeShared<FJsonValueBoolean>(true));  // posture ON
		Enabled.Add(MakeShared<FJsonValueBoolean>(true));  // damping ON
		Cfg->SetArrayField(TEXT("task_enabled"), Enabled);
		TSharedPtr<FJsonObject> Costs = MakeShared<FJsonObject>();
		TSharedPtr<FJsonObject> P1 = MakeShared<FJsonObject>();
		P1->SetNumberField(TEXT("cost"), 5.0);
		Costs->SetObjectField(TEXT("1"), P1); // bump posture: execution mode
		Cfg->SetObjectField(TEXT("task_costs"), Costs);
		TSharedPtr<FJsonObject> PT = MakeShared<FJsonObject>();
		for (int32 i = 0; i < 10; ++i)
		{
			PT->SetNumberField(JointNames[i], TargetQ[i]);
		}
		Cfg->SetObjectField(TEXT("posture_target"), PT);
		Ctrl->ApplyConfig(Cfg);
	}
	if (!RunSteps(2000))
	{
		S.Cleanup();
		return false;
	}
	for (int32 i = 0; i < 10; ++i)
	{
		const double Err = FMath::Abs(D->qpos[QposAdrs[i]] - TargetQ[i]);
		TestTrue(FString::Printf(TEXT("joint %s tracked (|err| %.4f < 0.05)"), JointNames[i], Err),
			Err < 0.05);
	}

	// --- Phase B: readback round-trip ------------------------------------------
	{
		TSharedPtr<FJsonObject> Params;
		Ctrl->GetCurrentConfig(Params); // if the getter is named differently, use
		                                // the method GetConfigSchema's callers use
		const TSharedPtr<FJsonObject>* PT = nullptr;
		if (TestTrue(TEXT("posture_target present in readback"),
				Params.IsValid() && Params->TryGetObjectField(TEXT("posture_target"), PT) && PT))
		{
			TestEqual(TEXT("readback entry count"), (*PT)->Values.Num(), 10);
			double V = 0.0;
			(*PT)->TryGetNumberField(TEXT("joint_x"), V);
			TestTrue(TEXT("joint_x round-trips"), FMath::Abs(V - TargetQ[0]) < 1e-9);
		}
	}

	// --- Phase C: unknown joint warns (budgeted), run survives -----------------
	AddExpectedError(TEXT("posture_target: joint 'joint_nope'"), EAutomationExpectedErrorFlags::Contains, 0);
	{
		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		TSharedPtr<FJsonObject> PT = MakeShared<FJsonObject>();
		PT->SetNumberField(TEXT("joint_nope"), 1.0);
		PT->SetNumberField(JointNames[0], TargetQ[0]);
		Cfg->SetObjectField(TEXT("posture_target"), PT);
		Ctrl->ApplyConfig(Cfg);
	}
	if (!RunSteps(50))
	{
		S.Cleanup();
		return false;
	}

	// --- Phase D: {} clears the latch ------------------------------------------
	{
		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		Cfg->SetObjectField(TEXT("posture_target"), MakeShared<FJsonObject>());
		Ctrl->ApplyConfig(Cfg);
		TSharedPtr<FJsonObject> Params;
		Ctrl->GetCurrentConfig(Params);
		const TSharedPtr<FJsonObject>* PT = nullptr;
		if (Params.IsValid() && Params->TryGetObjectField(TEXT("posture_target"), PT) && PT)
		{
			TestEqual(TEXT("cleared latch reads back empty"), (*PT)->Values.Num(), 0);
		}
	}
	if (!RunSteps(50)) // stepping after clear must not crash/yank
	{
		S.Cleanup();
		return false;
	}

	// --- Phase E: TwistFollow precedence — its per-step write wins ------------
	// Rebuild the controller with a TwistFollow spec appended (no UMjTwistController
	// sibling: expected warning; zero twist => its target holds the seed pose).
	// Route posture_target AT the twist spec explicitly; the base must NOT move.
	AddExpectedError(TEXT("TwistFollow has no UMjTwistController sibling"),
		EAutomationExpectedErrorFlags::Contains, 0);
	mj_resetDataKeyframe(M, D, KeyId);
	mj_forward(M, D);
	UMjArticulationController::NotifySimReset();
	FMinkTaskSpec Twist;
	Twist.Kind = EMinkTaskKind::TwistFollow;
	Twist.Cost = 1.0f;
	Twist.Joints = BaseJoints; // exactly x, y, th in order
	Ctrl->Tasks = {Frame, Posture, Damping, Twist};
	Robot->AdoptRuntimeController(Ctrl); // re-bind rebuilds the task stack
	{
		TSharedPtr<FJsonObject> Cfg = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> Enabled;
		Enabled.Add(MakeShared<FJsonValueBoolean>(false)); // frame OFF
		Enabled.Add(MakeShared<FJsonValueBoolean>(false)); // plain posture OFF
		Enabled.Add(MakeShared<FJsonValueBoolean>(false)); // damping OFF
		Enabled.Add(MakeShared<FJsonValueBoolean>(true));  // twist ON
		Cfg->SetArrayField(TEXT("task_enabled"), Enabled);
		TSharedPtr<FJsonObject> PT = MakeShared<FJsonObject>();
		PT->SetNumberField(TEXT("joint_x"), D->qpos[QposAdrs[0]] + 0.5);
		Cfg->SetObjectField(TEXT("posture_target"), PT);
		Cfg->SetNumberField(TEXT("posture_task"), 3); // route AT the twist spec
		Ctrl->ApplyConfig(Cfg);
	}
	const double BaseX0 = D->qpos[QposAdrs[0]];
	const bool bOk = RunSteps(500);
	TestTrue(FString::Printf(TEXT("twist_follow wins over posture_target routed at it "
								  "(|base x moved| %.4f < 0.05)"),
				 FMath::Abs(D->qpos[QposAdrs[0]] - BaseX0)),
		FMath::Abs(D->qpos[QposAdrs[0]] - BaseX0) < 0.05);
	S.Cleanup();
	return bOk;
}
```

Note on Phase E: if `AdoptRuntimeController` cannot re-bind the same controller instance after a task-list change, create a second `UMjMinkIKController` (unique name `TEXT("MinkIKPostureTwist")`) with the 4-task stack and adopt that instead — `AdoptRuntimeController` repoints with release/acquire fences, so adopting a fresh instance is the established path.

Notes for the implementer:
- The scaffold placeholders in square brackets are the ONLY parts you copy from `FMjMinkIKTidybotFixBaseHolds`; everything else above is verbatim new code.
- If `GetCurrentConfig` is not the public getter's name, find how the bridge dispatcher reads controller params (grep `GetCurrentConfigInternal` callers) and use that entry point.
- Unity build: the test name and any new file-scope statics must be unique.

- [ ] **Step 7: Compile**

Run the compile command from Global Constraints. Expected: `Result: Succeeded`.

- [ ] **Step 8: Run the suite (editor must be CLOSED — coordinate with the human)**

Run the test command from Global Constraints with `--filter "URLab.MinkIK"`. Expected: all tests pass including `URLab.MinkIK.TidyBot.PostureTargetTracks`, log contains `Result={Success}`.

- [ ] **Step 9: Format + commit**

```bash
/home/stuart/miniconda3/bin/clang-format -i \
  "Source/URLab/Public/MuJoCo/Components/Controllers/MjMinkIKController.h" \
  "Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp" \
  "Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp"
# re-compile after formatting if it changed anything, then:
git add "Source/URLab/Public/MuJoCo/Components/Controllers/MjMinkIKController.h" \
        "Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp" \
        "Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp"
git commit -m "feat(mink-ik): posture_target wire — stream joint-space targets over RPC"
```

---

### Task 2: planner module + offline tests (includes mink install)

**Files:**
- Create: `Scripts/demos/urlab_planner.py`
- Test: `Scripts/demos/test_urlab_planner.py`

**Interfaces:**
- Consumes: nothing from Task 1 (pure client-side; only the probe joins them).
- Produces: `PLANNED_JOINTS: list[str]` (10 names, base-first), `DEFAULT_V_LIMITS: dict[str, float]`, `Plan` dataclass (`waypoints: list[tuple[float, dict[str, float]]]` — seconds-from-start + named qpos map, JSON-ready; `stats: dict`), `PlanContext` (`from_client(client, joints=PLANNED_JOINTS)`, `q_start() -> np.ndarray`, `collision_free(q) -> bool`, `site_pose(q, site) -> (pos, quat_wxyz)`), `PlanError(RuntimeError)` with `.stage in {"goal_ik", "rrt"}`, `sample_goal_configs(ctx, pos, quat_wxyz, site="cup_site", n=8, seed=0) -> list[np.ndarray]`, `rrt_connect(ctx, q_start, goal_qs, max_iters=2000, step=0.15, base_disc=1.2, seed=0) -> list[np.ndarray] | None`, `shortcut(ctx, path, attempts=100, seed=0) -> list`, `time_parameterize(path, joints=PLANNED_JOINTS, v_limits=None) -> list[tuple[float, dict]]`, `plan_reach(client, pos, quat_wxyz, site="cup_site", ...) -> Plan`.

- [ ] **Step 1: Install python-mink into the bridge venv (mujoco must not move)**

```bash
V=/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin
$V/pip install "mink==1.2.0" daqp quadprog "qpsolvers>=4.12.0"
$V/python3 -c "import mujoco; assert mujoco.__version__ == '3.10.0', f'mujoco moved to {mujoco.__version__}'"
$V/python3 -c "import mink; import qpsolvers; print('mink OK')"
```

If the assert fails (pip upgraded mujoco as a mink dependency): `$V/pip install "mujoco==3.10.0"` and re-run both checks. Expected final output: `mink OK`.

- [ ] **Step 2: Write the failing tests**

Create `Scripts/demos/test_urlab_planner.py`. Style matches `test_urlab_skills.py` (plain asserts, no pytest):

```python
#!/usr/bin/env python3
"""Self-tests for the whole-body RRT connector (pure MuJoCo — no editor, no
client). Run: /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_planner.py"""
import numpy as np
import mujoco
import sys
sys.path.insert(0, ".")
from urlab_planner import (PlanContext, PlanError, rrt_connect, shortcut,
                           time_parameterize, sample_goal_configs, _edge_free)

# Synthetic scene: planar base (2 slides) + 1 hinge arm + a wall the base must
# route around. Wall at x=1.0, y half-size 0.5 => passable at |y| > ~0.75.
SCENE = """
<mujoco>
  <option timestep="0.005"/>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 1"/>
    <body name="wall" pos="1.0 0 0.5"><geom name="wall_g" type="box" size="0.1 0.5 0.5"/></body>
    <body name="base" pos="0 0 0.35">
      <joint name="joint_x" type="slide" axis="1 0 0" range="-5 5"/>
      <joint name="joint_y" type="slide" axis="0 1 0" range="-5 5"/>
      <geom name="base_g" type="box" size="0.2 0.2 0.1"/>
      <body name="link" pos="0 0 0.2">
        <joint name="joint_1" type="hinge" axis="0 0 1" range="-3 3"/>
        <geom name="arm_g" type="capsule" fromto="0 0 0 0.4 0 0" size="0.05"/>
        <site name="tip" pos="0.4 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
JOINTS = ["joint_x", "joint_y", "joint_1"]
m = mujoco.MjModel.from_xml_string(SCENE)
ctx = PlanContext(m, np.zeros(m.nq), joints=JOINTS)

# --- collision_free: free space ok, inside the wall not -----------------------
assert ctx.collision_free(np.array([0.0, 0.0, 0.0]))
assert not ctx.collision_free(np.array([1.0, 0.0, 0.0]))       # base in the wall
assert not ctx.collision_free(np.array([0.0, 0.0, 10.0]))      # joint limit

# --- margin filter: positive-distance margin contacts are NOT collisions ------
MARGIN_SCENE = """
<mujoco><worldbody>
  <body name="a" pos="0 0 1"><joint name="joint_x" type="slide" axis="1 0 0" range="-5 5"/>
    <geom name="cup_g" type="box" size="0.1 0.1 0.1" margin="0.05" gap="0.05"/></body>
  <body name="b" pos="0.22 0 1"><geom name="b_g" type="box" size="0.1 0.1 0.1"/></body>
</worldbody></mujoco>
"""
mm = mujoco.MjModel.from_xml_string(MARGIN_SCENE)
mctx = PlanContext(mm, np.zeros(mm.nq), joints=["joint_x"])
assert mctx.collision_free(np.array([0.0]))        # 2cm apart, inside 5cm margin: OK
assert not mctx.collision_free(np.array([0.05]))   # overlapping: collision

# --- baseline filter: pre-existing resting penetration is exempt --------------
REST_SCENE = """
<mujoco><worldbody>
  <geom name="floor" type="plane" size="10 10 1"/>
  <body name="base" pos="0 0 0.095">
    <joint name="joint_x" type="slide" axis="1 0 0" range="-5 5"/>
    <geom name="base_g" type="box" size="0.2 0.2 0.1"/>
  </body>
</worldbody></mujoco>
"""
rm = mujoco.MjModel.from_xml_string(REST_SCENE)
rctx = PlanContext(rm, np.zeros(rm.nq), joints=["joint_x"])
assert rctx.collision_free(np.array([0.0]))        # resting penetration at start
assert rctx.collision_free(np.array([2.0]))        # same pair elsewhere: still exempt

# --- rrt_connect: routes around the wall ---------------------------------------
q_start = np.array([-0.5, 0.0, 0.0])
q_goal = np.array([2.2, 0.0, 0.0])
assert not _edge_free(ctx, q_start, q_goal)        # straight line blocked
path = rrt_connect(ctx, q_start, [q_goal], seed=1)
assert path is not None, "planner failed to route around the wall"
assert np.allclose(path[0], q_start) and np.allclose(path[-1], q_goal)
for qa, qb in zip(path, path[1:]):
    assert _edge_free(ctx, qa, qb), "path has a colliding edge"

# --- shortcut: no longer, still valid ------------------------------------------
short = shortcut(ctx, path, seed=1)
assert len(short) <= len(path)
assert np.allclose(short[0], q_start) and np.allclose(short[-1], q_goal)
for qa, qb in zip(short, short[1:]):
    assert _edge_free(ctx, qa, qb)

# --- time_parameterize: monotone, respects per-joint velocity ------------------
wps = time_parameterize(short, joints=JOINTS,
                        v_limits={"joint_x": 0.6, "joint_y": 0.6, "joint_1": 0.5})
assert wps[0][0] == 0.0
for (ta, qa), (tb, qb) in zip(wps, wps[1:]):
    assert tb > ta
    for j, v in (("joint_x", 0.6), ("joint_y", 0.6), ("joint_1", 0.5)):
        assert abs(qb[j] - qa[j]) <= v * (tb - ta) + 1e-9

# --- goal IK (mink): tip reaches a nearby world pose ----------------------------
goals = sample_goal_configs(ctx, pos=[0.3, 0.4, 0.55], quat_wxyz=[1, 0, 0, 0],
                            site="tip", n=4, seed=0, ori_tol=1e9)  # position-only
assert goals, "goal IK found nothing for an easy pose"
p, _ = ctx.site_pose(goals[0], "tip")
assert np.linalg.norm(p - np.array([0.3, 0.4, 0.55])) < 0.02, p

print("urlab_planner self-tests OK")
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
cd "Scripts/demos" && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_planner.py
```

Expected: `ModuleNotFoundError: No module named 'urlab_planner'`.

- [ ] **Step 4: Write the module**

Create `Scripts/demos/urlab_planner.py`:

```python
#!/usr/bin/env python3
"""Whole-body free-space connector: RRT-Connect over the bridge client's
compiled-model replica (the mjb handshake), goal configs from python-mink IK.

Planner proposes / QP disposes / guards insure: this module only PROPOSES a
geometrically feasible joint-space path. Execution streams it through the mink
controller's posture_target wire (tidybot_planned_reach_probe.py); the QP's
velocity limits + collision guards stay active as the insurance layer.

Spec: docs/superpowers/specs/2026-07-16-whole-body-rrt-connector-design.md
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

# The 10 planned DoF, base first (order matters for goal seeding).
PLANNED_JOINTS = ["joint_x", "joint_y", "joint_th",
                  "joint_1", "joint_2", "joint_3", "joint_4",
                  "joint_5", "joint_6", "joint_7"]
# Execution pace defaults — match the demo QP velocity-limit constants
# (BASE_MAX_VEL/ARM_MAX_VEL in tidybot_mobile_manip_demo.py).
DEFAULT_V_LIMITS = {"joint_x": 0.6, "joint_y": 0.6, "joint_th": 1.0,
                    **{f"joint_{i}": 0.5 for i in range(1, 8)}}


class PlanError(RuntimeError):
    """Planning failed. .stage is 'goal_ik' or 'rrt' — they are fixed
    differently (pose/scene problem vs sampling-budget problem)."""

    def __init__(self, stage: str, msg: str):
        super().__init__(msg)
        self.stage = stage


@dataclass
class Plan:
    """Timed joint-space waypoints: waypoints[k] = (t_sec, {joint: qpos}),
    JSON-ready for the posture_target wire."""
    waypoints: list
    stats: dict = field(default_factory=dict)


class PlanContext:
    """Collision-checkable snapshot of the live scene over the client's model
    replica. Start state comes from a SYNCED physics read — never
    find_actors (actor roots are frozen at their spawn pose)."""

    def __init__(self, model, qpos0, joints=PLANNED_JOINTS):
        self.m = model
        self.d = mujoco.MjData(model)
        self.qpos0 = np.asarray(qpos0, dtype=float).copy()
        self.joints = list(joints)
        self.jids = []
        qadr = []
        for name in self.joints:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"planned joint {name!r} not in model")
            self.jids.append(jid)
            qadr.append(int(model.jnt_qposadr[jid]))
        self.qadr = np.asarray(qadr, dtype=int)
        # Sampling bounds: joint range where limited, else +-pi.
        lo, hi = [], []
        for jid in self.jids:
            if model.jnt_limited[jid]:
                lo.append(float(model.jnt_range[jid][0]))
                hi.append(float(model.jnt_range[jid][1]))
            else:
                lo.append(-np.pi)
                hi.append(np.pi)
        self.lo = np.asarray(lo)
        self.hi = np.asarray(hi)
        # Geom pairs already PENETRATING at the start config are pre-existing
        # (resting) contacts, not planner failures — exempt them everywhere.
        self.baseline = self._penetrating_pairs(self.q_start())

    @classmethod
    def from_client(cls, client, joints=PLANNED_JOINTS):
        """Snapshot live physics into a context. v1 constraint: call PRE-suction
        only — a direct-mode step zeroes actuator NetworkValues server-side."""
        client.step(n_steps=1)  # sync the mirror
        return cls(client.model, np.asarray(client.data.qpos), joints)

    def q_start(self):
        return self.qpos0[self.qadr].copy()

    def _set(self, q):
        self.d.qpos[:] = self.qpos0
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_collision(self.m, self.d)

    def _penetrating_pairs(self, q):
        self._set(q)
        pairs = set()
        for i in range(self.d.ncon):
            con = self.d.contact[i]
            # PENETRATION test, not contact test: geoms with margin (the
            # suction cup carries margin=0.03) report positive-distance
            # contacts inside the margin — suction sensing, not collision.
            if con.dist < -1e-6:
                g1, g2 = int(con.geom1), int(con.geom2)
                pairs.add((min(g1, g2), max(g1, g2)))
        return pairs

    def collision_free(self, q):
        q = np.asarray(q, dtype=float)
        if np.any(q < self.lo - 1e-9) or np.any(q > self.hi + 1e-9):
            return False
        return self._penetrating_pairs(q) <= self.baseline

    def site_pose(self, q, site):
        """World (pos, quat_wxyz) of a site at configuration q."""
        self._set(q)
        sid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, site)
        if sid < 0:
            raise ValueError(f"site {site!r} not in model")
        pos = self.d.site_xpos[sid].copy()
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, self.d.site_xmat[sid].reshape(9))
        return pos, quat


def _edge_free(ctx, qa, qb, resolution=0.05):
    """Every interpolated config on [qa, qb] collision-free. resolution is the
    max per-dimension step (m or rad) between checked configs."""
    qa = np.asarray(qa, dtype=float)
    qb = np.asarray(qb, dtype=float)
    n = int(np.ceil(np.max(np.abs(qb - qa)) / resolution)) + 1
    for a in np.linspace(0.0, 1.0, n + 1):
        if not ctx.collision_free((1.0 - a) * qa + a * qb):
            return False
    return True


def sample_goal_configs(ctx, pos, quat_wxyz, site="cup_site", n=8, seed=0,
                        iters=200, dt=0.05, pos_tol=5e-3, ori_tol=5e-2):
    """python-mink IK from n seeds -> deduped collision-free goal configs.
    Seed 0 is the current q; later seeds randomize the arm and place the base
    in a disc around the target xy (the whole point: the base participates)."""
    import mink  # deferred import: only goal sampling needs it

    rng = np.random.default_rng(seed)
    pos = np.asarray(pos, dtype=float)
    task = mink.FrameTask(frame_name=site, frame_type="site",
                          position_cost=1.0, orientation_cost=1.0, lm_damping=1.0)
    rot = mink.SO3(np.asarray(quat_wxyz, dtype=float))
    task.set_target(mink.SE3.from_rotation_and_translation(rot, pos))
    posture_cost = np.zeros(ctx.m.nv)
    for jid in ctx.jids:
        posture_cost[ctx.m.jnt_dofadr[jid]] = 1e-3
    limits = [mink.ConfigurationLimit(ctx.m)]

    goals = []
    for k in range(n):
        q = ctx.q_start()
        if k > 0:
            q = rng.uniform(ctx.lo, ctx.hi)
            r = rng.uniform(0.3, 0.9)          # base in an annulus around the
            th = rng.uniform(-np.pi, np.pi)    # target xy, roughly arm reach
            q[0] = np.clip(pos[0] + r * np.cos(th), ctx.lo[0], ctx.hi[0])
            q[1] = np.clip(pos[1] + r * np.sin(th), ctx.lo[1], ctx.hi[1])
        cfg = mink.Configuration(ctx.m)
        qfull = ctx.qpos0.copy()
        qfull[ctx.qadr] = q
        cfg.update(qfull)
        posture = mink.PostureTask(ctx.m, cost=posture_cost)
        posture.set_target_from_configuration(cfg)
        converged = False
        for _ in range(iters):
            v = mink.solve_ik(cfg, [task, posture], dt, "daqp",
                              damping=1e-3, limits=limits)
            cfg.integrate_inplace(v, dt)
            err = task.compute_error(cfg)
            if (np.linalg.norm(err[:3]) <= pos_tol
                    and np.linalg.norm(err[3:]) <= ori_tol):
                converged = True
                break
        if not converged:
            continue
        qg = np.asarray(cfg.q)[ctx.qadr].copy()
        if not ctx.collision_free(qg):
            continue
        if any(np.max(np.abs(qg - g)) < 0.05 for g in goals):
            continue  # dedupe near-identical solutions
        goals.append(qg)
    return goals


def _walk(tree, i):
    """Node chain root -> tree[i]."""
    out = []
    while i >= 0:
        out.append(tree[i][0])
        i = tree[i][1]
    out.reverse()
    return out


def rrt_connect(ctx, q_start, goal_qs, max_iters=2000, step=0.15,
                base_disc=1.2, seed=0, resolution=0.05):
    """Bidirectional RRT-Connect in the planned-joint space. The goal tree is
    seeded with ALL goal configs. Base dims (indices 0,1) sample from discs
    around the start/goal base xy; everything else uniform in limits. Returns
    q_start -> goal waypoints, or None."""
    if not goal_qs:
        return None
    q_start = np.asarray(q_start, dtype=float)
    goal_qs = [np.asarray(g, dtype=float) for g in goal_qs]
    if not ctx.collision_free(q_start):
        return None
    for g in goal_qs:  # trivial connect: straight line already free
        if _edge_free(ctx, q_start, g, resolution):
            return [q_start.copy(), g.copy()]

    rng = np.random.default_rng(seed)
    centers = [q_start[:2]] + [g[:2] for g in goal_qs]

    def sample():
        q = rng.uniform(ctx.lo, ctx.hi)
        c = centers[rng.integers(len(centers))]
        r = base_disc * np.sqrt(rng.uniform())
        th = rng.uniform(-np.pi, np.pi)
        q[0] = np.clip(c[0] + r * np.cos(th), ctx.lo[0], ctx.hi[0])
        q[1] = np.clip(c[1] + r * np.sin(th), ctx.lo[1], ctx.hi[1])
        return q

    ta = [(q_start.copy(), -1)]
    tb = [(g.copy(), -1) for g in goal_qs]
    a_is_start = True

    def nearest(tree, q):
        return int(np.argmin([np.linalg.norm(node[0] - q) for node in tree]))

    def extend(tree, q_target):
        """Greedy-step the nearest node toward q_target; returns the index of
        the last node added (None if the very first step collides)."""
        i = nearest(tree, q_target)
        added = None
        while True:
            q_near = tree[i][0]
            d = q_target - q_near
            dist = float(np.linalg.norm(d))
            q_new = q_target if dist <= step else q_near + d / dist * step
            if not _edge_free(ctx, q_near, q_new, resolution):
                return added
            tree.append((q_new.copy(), i))
            i = len(tree) - 1
            added = i
            if dist <= step:
                return added  # reached q_target exactly

    for _ in range(max_iters):
        q_rand = sample()
        ia = extend(ta, q_rand)
        if ia is not None:
            ib = extend(tb, ta[ia][0])
            if ib is not None and np.linalg.norm(tb[ib][0] - ta[ia][0]) < 1e-9:
                pa = _walk(ta, ia)  # tree-a root -> connect point
                pb = _walk(tb, ib)  # tree-b root -> connect point
                if a_is_start:
                    return pa + pb[::-1][1:]
                return pb + pa[::-1][1:]
        ta, tb = tb, ta  # classic RRT-Connect role swap
        a_is_start = not a_is_start
    return None


def shortcut(ctx, path, attempts=100, seed=0, resolution=0.05):
    """Random-pair shortcutting with full edge re-validation."""
    path = [np.asarray(q, dtype=float) for q in path]
    rng = np.random.default_rng(seed)
    for _ in range(attempts):
        if len(path) <= 2:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j - i < 2:
            continue
        if _edge_free(ctx, path[i], path[j], resolution):
            path = path[: i + 1] + path[j:]
    return path


def time_parameterize(path, joints=PLANNED_JOINTS, v_limits=None, min_dt=0.1):
    """Per-segment dwell = max joint displacement / that joint's velocity limit
    (floored at min_dt). Returns [(t_sec, {joint: qpos})] with plain floats
    (JSON-ready for the posture_target wire)."""
    vl = v_limits or DEFAULT_V_LIMITS
    v = np.asarray([vl[j] for j in joints], dtype=float)

    def named(q):
        return {j: float(x) for j, x in zip(joints, np.asarray(q, dtype=float))}

    t = 0.0
    waypoints = [(0.0, named(path[0]))]
    for qa, qb in zip(path, path[1:]):
        qa = np.asarray(qa, dtype=float)
        qb = np.asarray(qb, dtype=float)
        t += max(min_dt, float(np.max(np.abs(qb - qa) / v)))
        waypoints.append((t, named(qb)))
    return waypoints


def plan_reach(client, pos, quat_wxyz, site="cup_site", joints=PLANNED_JOINTS,
               n_goals=8, max_iters=2000, seed=0, v_limits=None):
    """Full pipeline: synced snapshot -> goal IK -> RRT-Connect -> shortcut ->
    timed waypoints. Raises PlanError('goal_ik'|'rrt') on failure."""
    import time as _time
    t0 = _time.time()
    ctx = PlanContext.from_client(client, joints)
    goals = sample_goal_configs(ctx, pos, quat_wxyz, site=site, n=n_goals, seed=seed)
    if not goals:
        raise PlanError("goal_ik",
                        f"no collision-free IK solution for {site} at {np.round(pos, 3)}")
    path = rrt_connect(ctx, ctx.q_start(), goals, max_iters=max_iters, seed=seed)
    if path is None:
        raise PlanError("rrt",
                        f"RRT-Connect exhausted {max_iters} iterations ({len(goals)} goals)")
    path = shortcut(ctx, path, seed=seed)
    wps = time_parameterize(path, joints, v_limits)
    return Plan(waypoints=wps, stats={
        "n_goals": len(goals), "n_waypoints": len(wps),
        "duration_s": wps[-1][0], "plan_wall_s": _time.time() - t0,
    })
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd "Scripts/demos" && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_planner.py
```

Expected: `urlab_planner self-tests OK`. If the goal-IK test fails on the solver, check `mink.solve_ik(..., "daqp", ...)` matches `Scripts/mink_golden/gen_tidybot_trace.py:89`'s call signature for the installed mink 1.2.0.

- [ ] **Step 6: Commit**

```bash
git add Scripts/demos/urlab_planner.py Scripts/demos/test_urlab_planner.py
git commit -m "feat(planner): whole-body RRT-Connect connector over the client model replica"
```

---

### Task 3: probe script + payload posture-subset change

**Files:**
- Create: `Scripts/demos/tidybot_planned_reach_probe.py`
- Modify: `Scripts/demos/tidybot_suction_pick_demo.py` (`suction_pick_controller_payload`, ~line 112-120)
- Modify: `Scripts/demos/test_urlab_skills.py` (one payload assert)

**Interfaces:**
- Consumes: Task 1's wire (`posture_target` via `client._rpc_configure_controller`), Task 2's `plan_reach`/`PlanError`/`Plan`. Existing helpers: `synced_site_pose`, `cup_down_quat`, `_object_z`, `_hold_suction` from `urlab_skills.py`; `stream_target` from `tidybot_mink_demo.py`; `PICK_STAGING`, `suction_pick_controller_payload` from `tidybot_suction_pick_demo.py`; `POSTURE_TASK`, `DAMPING_TASK` from `tidybot_twist_follow_demo.py`.
- Produces: the committed live-gate probe Task 4 runs.

- [ ] **Step 1: Widen the pick payload's posture subset to all 10 joints**

In `suction_pick_controller_payload` (`tidybot_suction_pick_demo.py`, after the `payload["tasks"][0]["frame"] = "cup_site"` line), add:

```python
    # Posture over ALL TEN joints (not just the arm): the posture_target wire
    # streams whole-body plans through this task, and SubsetCost zeroes any
    # joint not listed — base entries would be inert. Deliberate divergence
    # from the golden-parity example payload (which must stay arm-only).
    payload["tasks"][1]["joints"] = BASE_JOINTS + ARM_JOINTS
```

Add the import `BASE_JOINTS, ARM_JOINTS` from `tidybot_mink_demo` alongside the file's existing imports if not already present.

- [ ] **Step 2: Assert the payload change in test_urlab_skills.py**

Append to `Scripts/demos/test_urlab_skills.py` (before the final print), matching its plain-assert style:

```python
# suction payload: posture task must span all 10 joints (posture_target wire
# streams whole-body plans through it; SubsetCost makes unlisted joints inert).
from tidybot_suction_pick_demo import suction_pick_controller_payload
_payload = suction_pick_controller_payload([])
assert _payload["tasks"][1]["kind"] == "posture", _payload["tasks"][1]
assert _payload["tasks"][1]["joints"] == [
    "joint_x", "joint_y", "joint_th",
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7",
], _payload["tasks"][1]["joints"]
```

Note: check `suction_pick_controller_payload`'s actual signature (it takes the obstacle-body list) and pass a valid minimal argument.

- [ ] **Step 3: Run the skills tests**

```bash
cd "Scripts/demos" && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_skills.py
```

Expected: `urlab_skills self-tests OK` (the new asserts pass).

- [ ] **Step 4: Write the probe**

Create `Scripts/demos/tidybot_planned_reach_probe.py`:

```python
#!/usr/bin/env python3
"""Live probe for the whole-body RRT connector (spec:
docs/superpowers/specs/2026-07-16-whole-body-rrt-connector-design.md).

  --author : build the suction-pick scene (editor idle, NOT simulating), save.
  --drive  : (press Simulate first) nav to staging, PLAN a whole-body path to
             the pre-grasp pose, execute it via the posture_target wire with a
             tracking watchdog, then run the validated direct-descent suction
             endgame. Success line: GRIPPED+LIFTED.

Connect AFTER Simulate starts: the client's model replica (mjb handshake) is
captured at connect — a stale connect plans against a stale world.
"""
import argparse
import sys
import time

import numpy as np
import mujoco

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

from urlab_skills import synced_site_pose, cup_down_quat, _object_z
from urlab_planner import PLANNED_JOINTS, PlanError, plan_reach
from tidybot_mink_demo import stream_target
from tidybot_twist_follow_demo import POSTURE_TASK, DAMPING_TASK
from tidybot_suction_pick_demo import (ACTOR_ID, ASSETS, MODEL_XML,
                                       PICK_LOCATION, PICK_STAGING,
                                       suction_pick_controller_payload)

PRE_GRASP_M = 0.03      # plan target: cup this far above the box top
TRACK_TOL = 0.15        # rad/m — watchdog per-joint tracking tolerance
TRACK_GRACE_S = 4.0     # divergence longer than this aborts execution
SETTLE_TOL = 0.05       # final-waypoint convergence
SETTLE_TIMEOUT_S = 10.0


def log(m):
    print(f"[planned-reach] {m}", flush=True)


def fail(m):
    print(f"[planned-reach] FAIL: {m}", flush=True)
    sys.exit(1)


def author():
    """Same scene as the suction-pick demo authoring (see the demo module's
    constants); saved so later cycles can load instead of re-importing."""
    from tidybot_mobile_manip_demo import FLOOR, NAV_BOUNDS, OBSTACLES, AGENT_RADIUS
    from tidybot_guarded_reach_demo import TABLE

    c = URLabClient("tcp://localhost")
    c.connect()
    try:
        c.sim.stop()
    except URLabRPCError:
        pass
    c.scene.create_level(name="PlannedReachDemo", force_overwrite=True)
    c.scene.spawn_light(actor_id="pr_sun", kind="directional", intensity=6.0)
    c.scene.spawn_box(**FLOOR)
    obstacle_bodies = []
    for ob in OBSTACLES + [TABLE]:
        sb = c.scene.spawn_box(**ob)
        c.outliner.add_quick_convert(target=ob["actor_id"], static=True)
        obstacle_bodies.append(f"{sb['actor_name']}_MjBody")
    r = c.scene.spawn_nav_bounds(**NAV_BOUNDS, agent_radius=AGENT_RADIUS, timeout_s=30.0)
    log(f"navmesh baked={r.get('nav_data_present')}")
    bp_box = c.scene.import_xml(path=str(ASSETS / "suction_box.xml"))
    c.scene.spawn_actor(blueprint=bp_box, actor_id="pick_box", location=PICK_LOCATION)
    bp = c.scene.import_xml(path=str(MODEL_XML))
    c.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
    c.scene.ensure_manager()
    st = c.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
    log(f"nav stack created={st.get('created')}")
    ik = c.ik.add_controller(target=ACTOR_ID,
                             **suction_pick_controller_payload(obstacle_bodies))
    warnings = ik.get("warnings") or []
    if warnings:
        log(f"!! WARNINGS: {warnings}")
    try:
        c.scene.save_level()
    except Exception as e:
        log(f"save_level note (non-blocking): {e}")
    log("AUTHORED. Press SIMULATE, then run --drive")


def drive():
    c = URLabClient("tcp://localhost", step_mode="direct", step_port=5559)
    c.connect()
    c.sim.start(timeout_s=180.0)
    name = next((r.name for r in c.outliner.find_actors(
        class_filter="AMjArticulation", in_pie=True) if r.actor_id == ACTOR_ID), None)
    if name is None:
        fail(f"{ACTOR_ID!r} not found")
    if c.runtime.set_active_controller(articulation=name,
                                       controller="mink_ik").get("active") != "MjMinkIKController":
        fail("mink not active")
    try:
        c.runtime.set_mode("live")
    except URLabRPCError as e:
        log(f"set_mode note: {e}")
    c.runtime.set_paused(paused=False)
    time.sleep(1.5)

    def configure(p):
        c._rpc_configure_controller(articulation=name, params=p)

    def hold_suction():
        try:
            c.runtime.set_suction(articulation=name, value=1.0)
        except Exception:
            pass

    def synced_q():
        """Planned-joint vector from a synced physics read. PRE-suction only
        (a direct-mode step zeroes actuator NetworkValues server-side)."""
        c.step(n_steps=1)
        m, d = c.model, c.data
        q = np.empty(len(PLANNED_JOINTS))
        for i, jn in enumerate(PLANNED_JOINTS):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            q[i] = d.qpos[m.jnt_qposadr[jid]]
        return q

    # --- Phase A: nav to staging (frame off, twist follows) --------------------
    configure({"task_enabled": [False, True, True, True],
               "task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})
    g = c.runtime.set_nav_goal(articulation=name, x=PICK_STAGING[0], y=PICK_STAGING[1])
    if not g.get("accepted"):
        fail(f"nav goal rejected: {g}")
    log(f"driving to staging {PICK_STAGING}")
    deadline = time.time() + 90.0
    s = {"state": "?"}
    while time.time() < deadline:
        s = c.runtime.get_nav_status(articulation=name)
        if s["state"] in ("arrived", "failed"):
            break
        time.sleep(1.0)
    if s["state"] != "arrived":
        fail(f"nav {s['state']}")
    log("arrived at staging")
    configure({"task_costs": {str(DAMPING_TASK): {"cost": 5.0}}})

    # --- Phase B: PLAN whole-body to pre-grasp ---------------------------------
    surf, qaff = synced_site_pose(c, "affordance_suction_top")
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, qaff)
    normal = np.asarray(R).reshape(3, 3)[:, 2]
    normal = normal / (np.linalg.norm(normal) or 1.0)
    pre_grasp = np.asarray(surf) + PRE_GRASP_M * normal
    q_cup = cup_down_quat(normal)
    log(f"box top {np.round(surf, 3)}; planning to pre-grasp {np.round(pre_grasp, 3)}")
    try:
        plan = plan_reach(c, pre_grasp, q_cup, site="cup_site")
    except PlanError as e:
        fail(f"planning failed at stage '{e.stage}': {e}")
    log(f"PLAN: {plan.stats}")

    # --- Phase C: execute via the posture_target wire ---------------------------
    # Handoff contract: frame OFF, twist_follow OFF; posture cost bumped so the
    # streamed target dominates; damping + collision guards stay on (insurance).
    configure({"task_enabled": [False, True, True, False],
               "task_costs": {str(POSTURE_TASK): {"cost": 5.0}}})
    t0 = time.time()
    diverged_since = None
    wp_idx = 0
    while wp_idx < len(plan.waypoints):
        t_wp, wp = plan.waypoints[wp_idx]
        now = time.time() - t0
        if now < t_wp:
            time.sleep(min(0.2, t_wp - now))
        configure({"posture_target": wp})
        # Watchdog on the CURRENT waypoint once its scheduled time has passed.
        if time.time() - t0 >= t_wp:
            q = synced_q()
            err = max(abs(q[i] - wp[jn]) for i, jn in enumerate(PLANNED_JOINTS))
            if err > TRACK_TOL:
                diverged_since = diverged_since or time.time()
                if time.time() - diverged_since > TRACK_GRACE_S:
                    configure({"posture_target": {},
                               "task_costs": {str(POSTURE_TASK): {"cost": 1e-3}}})
                    fail(f"watchdog: waypoint {wp_idx} tracking err {err:.3f} "
                         f"> {TRACK_TOL} for {TRACK_GRACE_S}s — aborted, latch cleared")
            else:
                diverged_since = None
                wp_idx += 1
                log(f"  waypoint {wp_idx}/{len(plan.waypoints)} (err {err:.3f})")
    # Settle on the final waypoint.
    t_wp, wp = plan.waypoints[-1]
    settle_deadline = time.time() + SETTLE_TIMEOUT_S
    while time.time() < settle_deadline:
        q = synced_q()
        err = max(abs(q[i] - wp[jn]) for i, jn in enumerate(PLANNED_JOINTS))
        if err < SETTLE_TOL:
            break
        configure({"posture_target": wp})
        time.sleep(0.3)
    else:
        fail(f"final waypoint never settled (err {err:.3f})")
    log(f"plan executed — at pre-grasp (err {err:.3f})")

    # --- Phase D: validated direct-descent suction endgame ----------------------
    # Back to frame-task control: clear the posture latch, restore its
    # regularizer cost, re-enable the frame task (twist_follow stays off).
    configure({"posture_target": {},
               "task_costs": {str(POSTURE_TASK): {"cost": 1e-3}},
               "task_enabled": [True, True, True, False]})
    p0, _ = synced_site_pose(c, "cup_site")
    stream_target(c, name, p0, q_cup)  # seed at current pose (no yank)
    target = np.asarray(surf)          # seat ON the box top
    stream_target(c, name, target, q_cup)
    engaged = False
    t0 = time.time()
    while time.time() - t0 < 8.0:
        time.sleep(0.3)
        cup, _ = synced_site_pose(c, "cup_site")
        stream_target(c, name, target, q_cup)
        d = float(np.linalg.norm(cup - np.asarray(surf)))
        if not engaged and d <= 0.05:
            c.runtime.set_suction(articulation=name, value=1.0)
            engaged = True
            log(f"suction ON at d={d:.3f}")
        if engaged:
            hold_suction()
        log(f"  cup={np.round(cup, 3)} d_to_top={d:.3f}")
        if engaged and d <= 0.02:
            break
    if not engaged:
        c.runtime.set_suction(articulation=name, value=1.0)
        log("force-engaged suction")
    hold_suction()
    time.sleep(1.0)
    hold_suction()

    z_before = _object_z(c, "pick_box")
    hold_suction()
    cup, _ = synced_site_pose(c, "cup_site")
    hold_suction()
    lift_to = cup + np.array([0.0, 0.0, 0.12])
    log(f"lifting from z={cup[2]:.3f} to {lift_to[2]:.3f}")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        a = min(1.0, (time.time() - t0) / 3.0)
        stream_target(c, name, (1 - a) * cup + a * lift_to, q_cup)
        time.sleep(0.05)
    z_after = _object_z(c, "pick_box")
    hold_suction()
    rose = z_after - z_before
    log(f"box z {z_before:.3f} -> {z_after:.3f}  rose {rose:+.3f}  "
        f"{'GRIPPED+LIFTED' if rose > 0.05 else 'no lift'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--author", action="store_true")
    mode.add_argument("--drive", action="store_true")
    args = ap.parse_args()
    author() if args.author else drive()
```

Implementer notes:
- Verify every imported name exists (e.g., `PICK_STAGING` in `tidybot_suction_pick_demo.py`, `_object_z`/`synced_site_pose`/`cup_down_quat` in `urlab_skills.py`, `stream_target` in `tidybot_mink_demo.py`) and match their real signatures — adjust call sites, not the helpers.
- `sys.path`: the probe lives IN `Scripts/demos`, so sibling imports work when run from that directory; do not add path hacks.
- This file cannot be fully tested offline (needs the live editor). Verify it imports cleanly: `cd Scripts/demos && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 -c "import tidybot_planned_reach_probe"` — expected: no output, exit 0. (The mutually-exclusive-group argparse only runs under `__main__`.)

- [ ] **Step 5: Commit**

```bash
git add Scripts/demos/tidybot_planned_reach_probe.py \
        Scripts/demos/tidybot_suction_pick_demo.py \
        Scripts/demos/test_urlab_skills.py
git commit -m "feat(planner): planned-reach live probe + 10-joint posture subset in the pick payload"
```

---

### Task 4: LIVE GATE — human-in-loop (do NOT dispatch to a subagent)

This task is executed collaboratively in the main session: the human presses Simulate (never Play) and sets up lighting; the agent runs the probe and reads results. Automation cannot do this.

**Procedure:**

- [ ] **Step 1:** Human confirms the editor is open and idle. Run:
  `cd Scripts/demos && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 tidybot_planned_reach_probe.py --author`
  Expected: `AUTHORED. Press SIMULATE, then run --drive`, no guard warnings.
- [ ] **Step 2:** Human presses SIMULATE and confirms.
- [ ] **Step 3:** Run:
  `cd Scripts/demos && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 tidybot_planned_reach_probe.py --drive`
- [ ] **Step 4: Success criteria (all three):**
  1. `PLAN:` stats line printed (plan found; note `plan_wall_s` and `n_waypoints`).
  2. `plan executed — at pre-grasp` (watchdog never tripped).
  3. Final line contains `GRIPPED+LIFTED` (box physics z rose > 5 cm).
- [ ] **Step 5: Expected live-tuning knobs if it doesn't gate first try** (tune, re-run, keep notes; commit knob changes to the probe):
  - Watchdog trips: raise `TRACK_TOL` (QP tracking lag, not failure) or slow the plan (`DEFAULT_V_LIMITS` scale down).
  - Plan fails at `goal_ik`: check the box spawned, connect happened AFTER Simulate (stale-mjb constraint), and widen the goal-seed annulus.
  - Plan fails at `rrt`: raise `max_iters`, or raise `base_disc`.
  - Seat/lift fails but pre-grasp was reached: this is the shelved adhesion question — widen cup margin/gain (`tidybot_suction_ue.xml`: `margin/gap 0.03 -> 0.05`, adhesion `gain 100 -> 300`) as the spec's named fallback.
- [ ] **Step 6:** On success, record the run (stats lines) in the progress ledger and commit any tuning deltas.

---

### Task 5: `PlannedReach` skill + tree swap + full suites

Only after Task 4 gates. **Files:**
- Modify: `Scripts/demos/urlab_skills.py` (add `PlannedReach` behaviour)
- Modify: `Scripts/demos/tidybot_suction_pick_demo.py` (swap `ReachRamp` -> `PlannedReach` in the tree)
- Modify: `Scripts/demos/test_urlab_skills.py` (pure-function asserts)

**Interfaces:**
- Consumes: Task 2's `plan_reach`/`PlanError`; Task 1's wire via `configure_controller`; the blackboard keys `ResolveAffordance` already publishes in `urlab_skills.py` (read that class first — reuse its exact key names for the affordance point, normal, and cup quat).
- Produces: `PlannedReach(name, bb, duration=None)` py_trees behaviour, drop-in at `ReachRamp`'s position in the pick tree (same SUCCESS/FAILURE/RUNNING contract).

- [ ] **Step 1: Extract the waypoint scheduler as a pure function + failing tests**

In `urlab_skills.py`, add a module-level pure helper (testable offline):

```python
def due_waypoint(waypoints, elapsed_s):
    """Index of the waypoint that should currently be streamed: the LAST
    waypoint whose scheduled time <= elapsed_s (0 before the first)."""
    idx = 0
    for i, (t, _q) in enumerate(waypoints):
        if t <= elapsed_s:
            idx = i
    return idx
```

Append to `test_urlab_skills.py`:

```python
# due_waypoint: schedule lookup for PlannedReach streaming.
from urlab_skills import due_waypoint
_wps = [(0.0, {"j": 0.0}), (1.0, {"j": 1.0}), (2.5, {"j": 2.0})]
assert due_waypoint(_wps, -0.1) == 0
assert due_waypoint(_wps, 0.0) == 0
assert due_waypoint(_wps, 1.7) == 1
assert due_waypoint(_wps, 99.0) == 2
```

Run: `cd Scripts/demos && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_skills.py` — expected FAIL (`ImportError: cannot import name 'due_waypoint'`) before the helper lands, PASS after.

- [ ] **Step 2: Write `PlannedReach`**

Add to `urlab_skills.py`, following the file's behaviour conventions (`initialise`/`update`, `self.bb` blackboard, `log` style — read `ReachRamp` first and mirror its structure):

```python
class PlannedReach(py_trees.behaviour.Behaviour):
    """Whole-body planned reach to the affordance pre-grasp pose: plans an
    RRT-Connect path in initialise (blocking, seconds — acceptable at BT
    rate for v1), then streams posture_target waypoints on schedule.
    SUCCESS when the final waypoint tracks within tolerance; FAILURE on
    PlanError or watchdog divergence. Replaces ReachRamp's straight-line
    carpet — the planned BASE path executes exactly instead of being
    rediscovered by the greedy QP.

    Handoff contract (spec section: execution): frame + twist_follow tasks
    disabled during execution; posture cost bumped; both restored and the
    posture latch cleared in terminate() regardless of outcome."""

    PRE_GRASP_M = 0.03
    TRACK_TOL = 0.15
    TRACK_GRACE_S = 4.0
    SETTLE_TOL = 0.05

    def __init__(self, name, bb):
        super().__init__(name)
        self.bb = bb
        self._plan = None
        self._t0 = None
        self._diverged_since = None
        self._failed = None

    def initialise(self):
        from urlab_planner import PLANNED_JOINTS, PlanError, plan_reach
        bb = self.bb
        client = bb.client
        self._joints = PLANNED_JOINTS
        self._failed = None
        self._diverged_since = None
        # ResolveAffordance publishes bb.affordance (an Affordance dataclass:
        # point, normal, quat_cup_down, waypoints) and bb.q_cup.
        aff = bb.affordance
        pre_grasp = np.asarray(aff.point, dtype=float) \
            + self.PRE_GRASP_M * np.asarray(aff.normal, dtype=float)
        try:
            self._plan = plan_reach(client, pre_grasp, np.asarray(bb.q_cup, dtype=float))
        except PlanError as e:
            self._plan = None
            bb.fail_reason = f"PlannedReach[{self.name}]: plan {e.stage}: {e}"
            self._failed = bb.fail_reason
            return
        client._rpc_configure_controller(articulation=bb.name, params={
            "task_enabled": [False, True, True, False],
            "task_costs": {"1": {"cost": 5.0}},
        })
        self._t0 = time.time()

    def update(self):
        if self._failed:
            self.feedback_message = self._failed
            return py_trees.common.Status.FAILURE
        bb = self.bb
        client = bb.client
        elapsed = time.time() - self._t0
        idx = due_waypoint(self._plan.waypoints, elapsed)
        t_wp, wp = self._plan.waypoints[idx]
        client._rpc_configure_controller(articulation=bb.name,
                                         params={"posture_target": wp})
        # Watchdog + terminal check on synced state.
        q = _synced_planned_q(client, self._joints)
        err = max(abs(q[i] - wp[j]) for i, j in enumerate(self._joints))
        is_last = idx == len(self._plan.waypoints) - 1
        if is_last and elapsed >= t_wp and err < self.SETTLE_TOL:
            return py_trees.common.Status.SUCCESS
        if err > self.TRACK_TOL and elapsed >= t_wp:
            self._diverged_since = self._diverged_since or time.time()
            if time.time() - self._diverged_since > self.TRACK_GRACE_S:
                self.feedback_message = f"watchdog: err {err:.3f} at waypoint {idx}"
                return py_trees.common.Status.FAILURE
        else:
            self._diverged_since = None
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        bb = self.bb
        try:
            bb.client._rpc_configure_controller(articulation=bb.name, params={
                "posture_target": {},
                "task_costs": {"1": {"cost": 1e-3}},
                "task_enabled": [True, True, True, False],
            })
        except Exception:
            pass
```

Also add the module-level synced-read helper next to the other private helpers:

```python
def _synced_planned_q(client, joints):
    """Planned-joint vector from a synced physics read. PRE-suction only (a
    direct-mode step zeroes actuator NetworkValues server-side)."""
    client.step(n_steps=1)
    m, d = client.model, client.data
    out = np.empty(len(joints))
    for i, jn in enumerate(joints):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        out[i] = d.qpos[m.jnt_qposadr[jid]]
    return out
```

Implementer notes:
- Blackboard keys (verified in `urlab_skills.py`): `bb.client`, `bb.name`, `bb.q_cup`, `bb.fail_reason`, and `bb.affordance` — an `Affordance` dataclass (`urlab_skills.py:165`) with fields `point`, `normal`, `quat_cup_down`, `waypoints`. `ReachRamp` consumed the `waypoints` list; `PlannedReach` uses `point` + `normal` directly and ignores `waypoints`.
- The task-index literal `"1"` is `POSTURE_TASK`; import and use the constant if `urlab_skills.py` already imports from `tidybot_twist_follow_demo`, else define it locally with a comment.
- `mujoco` and `time` are already imported by `urlab_skills.py`; verify rather than re-import.

- [ ] **Step 3: Swap the tree node**

In `tidybot_suction_pick_demo.py`'s tree construction, replace the `ReachRamp` node with `PlannedReach` (same position: after `CorridorClear`, before `DescendEngage`). Keep `ReachRamp` itself in `urlab_skills.py` (other demos use it). Update the demo's module docstring tree diagram if it lists node names.

- [ ] **Step 4: Run all offline suites**

```bash
cd "Scripts/demos" && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_skills.py \
  && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_planner.py
```

Expected: both `OK` lines. Then the engine suite (editor CLOSED): the test command from Global Constraints with `--filter "URLab.MinkIK"` — expected `Result={Success}`.

- [ ] **Step 5: Golden-model guard**

```bash
git status --porcelain Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml
```

Expected: no output (byte-identical).

- [ ] **Step 6: Commit**

```bash
git add Scripts/demos/urlab_skills.py \
        Scripts/demos/tidybot_suction_pick_demo.py \
        Scripts/demos/test_urlab_skills.py
git commit -m "feat(skills): PlannedReach — RRT-connector reach swapped into the suction-pick tree"
```

---

## Post-plan follow-ups (recorded, not tasks)

- Full pick-tree live re-validation with `PlannedReach` (a second Task-4-style session, cheaper since the connector is proven).
- Fix the direct-mode `client.step()` NetworkValue-zeroing engine bug before any held-object (mid-grip sync) planning.
- Strip the TEMP DIAG commit (9b75c3f) before merging the branch (pre-existing branch item, noted in `docs/handoff/2026-07-16-suction-pick-bug.md`).
- Promote `urlab_planner.py` into the URLab_Bridge client library once the API stabilizes.
