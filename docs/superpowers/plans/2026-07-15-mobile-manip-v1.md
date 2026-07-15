# Mobile Manipulation v1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One tidybot that navigates around obstacles to a staging pose, then reaches an EE target near a table — via a new `set_active_controller` runtime op and a sequential bridge-orchestrated demo.

**Architecture:** The engine gains one primitive: a runtime op that repoints an articulation's bound controller using the existing `AMjArticulation::AdoptRuntimeController` (safe Bind + release-fence publish). Sequencing lives in `Scripts/demos/tidybot_mobile_manip_demo.py`: live-mode nav drive (validated 2026-07-15) → controller switch → direct-mode IK reach (the proven `tidybot_mink_demo.py` pattern, importing its helpers).

**Tech Stack:** UE 5.7 C++ (URLab runtime module + dispatcher), UE automation tests, Python 3 + `urlab_client` (bridge venv).

**Spec:** `docs/superpowers/specs/2026-07-15-mobile-manip-v1-design.md` — read it first.

## Global Constraints

- Branch: `feat/nav-stack` (main checkout, no worktree; pushes go to the `fork` remote). Commit ONLY explicit file paths, never `git add -A` (uncommitted third-party WIP in tree: `third_party/build_all.sh`, `Scripts/mink_golden/*` — NEVER stage).
- Commit messages: plain subject + body. NO `Co-Authored-By` / `Claude-Session` trailers (attribution disabled).
- Compile: `"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex` — success `Result: Succeeded`. Requires the editor CLOSED (check `pgrep UnrealEditor`).
- Test: `./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "URLab.Nav"` — pass token `Result={Success}`. Editor must be closed.
- Format ONLY lines you add: `/home/stuart/miniconda3/bin/clang-format -i --lines=<start>:<end> <file>` for the shared files `RpcDispatcher.cpp`/`MjEditorOpHandlers.cpp` (whole-file format reflows unrelated code — has bitten twice); whole-file format is OK only for files this plan creates.
- Log category: `LogURLab` (include `"Utils/URLabLogging.h"`), never `LogTemp` (suppressed in this editor).
- Unity build ON: file-scope helpers in test files need module-unique names.
- The LIVE demo run is NOT part of any task — it happens collaboratively afterwards (the user presses Simulate; see `[[collaborative-dev-nav]]` memory). Tasks verify by compile + automation tests + `py_compile` only.

---

### Task 1: `set_active_controller` runtime op (+ `GetActiveController` accessor)

**Files:**
- Modify: `Source/URLab/Public/MuJoCo/Core/MjArticulation.h` (public accessor next to `AdoptRuntimeController`, ~line 524)
- Modify: `Source/URLab/Public/Bridge/RpcDispatcher.h` (declare `HandleSetActiveController` next to `HandleSetNavGoal`)
- Modify: `Source/URLab/Private/Bridge/RpcDispatcher.cpp` (handler + registration)
- Modify: `Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp` (new test)

**Interfaces:**
- Consumes: `AMjArticulation::AdoptRuntimeController(UMjArticulationController*)` (existing, game-thread; Binds + publishes); `AAMjManager::GetArticulation(FString)` (UE-actor-name keyed); the `HandleSetNavGoal` off-thread marshal idiom (RpcDispatcher.cpp — read it and copy its safety shape exactly: by-value `TWeakObjectPtr`, thread-safe `TSharedPtr` result struct, by-value `FEvent*`, `Wait(2000)`).
- Produces: `UMjArticulationController* AMjArticulation::GetActiveController() const` (game-thread read of `CachedController`); wire op `set_active_controller` (`runtime` ns) with reply `set_active_controller_ok`: `active:string` (resolved class name), `was_active:bool`. Errors: `not_ready`, `unknown_articulation`, `unknown_controller`, `ambiguous`. Task 2's script calls `client.runtime.set_active_controller(articulation=..., controller="base_drive"|"mink_ik")`.

- [ ] **Step 1: Write the failing test**

Append to `Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp` (add includes `"MuJoCo/Components/Controllers/MjBaseDriveController.h"`, `"MuJoCo/Components/Controllers/MjPassthroughController.h"`, `"Bridge/RpcDispatcher.h"` if not present; `FMjUESession`/dispatcher/session idiom as in `MjNavOpsTests.cpp`):

```cpp
// URLab.Nav.Ops.SetActiveController — repoints the bound controller between
// two attached UMjArticulationController components; idempotent; errors on
// unknown/ambiguous tokens.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavDemoSetActiveController,
	"URLab.Nav.Ops.SetActiveController",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavDemoSetActiveController::RunTest(const FString&)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	// Attach two controllers. PostSetup already bound one (whichever
	// FindComponentByClass found); the op must be able to select either.
	UMjBaseDriveController* Drive =
		NewObject<UMjBaseDriveController>(S.Robot, TEXT("DriveCtrl"));
	S.Robot->AddInstanceComponent(Drive);
	Drive->RegisterComponent();
	UMjPassthroughController* Pass =
		NewObject<UMjPassthroughController>(S.Robot, TEXT("PassCtrl"));
	S.Robot->AddInstanceComponent(Pass);
	Pass->RegisterComponent();

	FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));
	auto Req = [&](const TCHAR* Token) {
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), TEXT("set_active_controller"));
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		R->SetStringField(TEXT("articulation"), S.Robot->GetName());
		R->SetStringField(TEXT("controller"), Token);
		return R;
	};

	// Select the passthrough.
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req(TEXT("passthrough")));
	FString Active;
	TestTrue(TEXT("active field"), Reply->TryGetStringField(TEXT("active"), Active));
	TestEqual(TEXT("passthrough active"), Active, TEXT("MjPassthroughController"));
	TestTrue(TEXT("bound repointed"),
		S.Robot->GetActiveController() == (UMjArticulationController*)Pass);

	// Idempotent repeat.
	Reply = Disp->Dispatch(Req(TEXT("passthrough")));
	bool bWasActive = false;
	TestTrue(TEXT("was_active field"), Reply->TryGetBoolField(TEXT("was_active"), bWasActive));
	TestTrue(TEXT("was already active"), bWasActive);

	// Switch to the base drive.
	Reply = Disp->Dispatch(Req(TEXT("base_drive")));
	Reply->TryGetStringField(TEXT("active"), Active);
	TestEqual(TEXT("base drive active"), Active, TEXT("MjBaseDriveController"));
	TestTrue(TEXT("bound repointed again"),
		S.Robot->GetActiveController() == (UMjArticulationController*)Drive);

	// Unknown token.
	Reply = Disp->Dispatch(Req(TEXT("warp_drive")));
	FString Code;
	TestTrue(TEXT("error code"), Reply->TryGetStringField(TEXT("code"), Code));
	TestEqual(TEXT("unknown_controller"), Code, TEXT("unknown_controller"));

	// Ambiguous token ("controller" is a substring of both class names).
	Reply = Disp->Dispatch(Req(TEXT("controller")));
	Reply->TryGetStringField(TEXT("code"), Code);
	TestEqual(TEXT("ambiguous"), Code, TEXT("ambiguous"));

	S.Cleanup();
	return true;
}
```

NOTE: verify `FMjUESession`'s robot does not already carry a controller that would make the "ambiguous" expectation wrong (read `MjTestHelpers.h`; the rig historically has none). If PostSetup logs errors when binding `UMjBaseDriveController` on the one-joint rig (base joints unresolved), that is expected and harmless — the op's contract is repointing, not successful base resolution.

- [ ] **Step 2: Compile to verify failure**

Run the compile command. Expected: FAIL — `GetActiveController` is not a member of `AMjArticulation`.

- [ ] **Step 3: Implement**

`MjArticulation.h`, in the public section right after the `AdoptRuntimeController` declaration:

```cpp
	/** The currently-bound controller (CachedController), or null when the
	 *  default per-actuator ctrl path is active. Game-thread read — pairs
	 *  with AdoptRuntimeController; not for the physics thread. */
	class UMjArticulationController* GetActiveController() const
	{
		return CachedController;
	}
```

`RpcDispatcher.h`, next to `HandleSetNavGoal`'s declaration:

```cpp
	TSharedPtr<FJsonObject> HandleSetActiveController(const TSharedPtr<FJsonObject>& Req);
```

`RpcDispatcher.cpp` — registration, in the runtime block next to `set_nav_goal` (line-scoped clang-format only):

```cpp
	Reg(TEXT("set_active_controller"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetActiveController(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("active:string"), TEXT("was_active:bool")},
		/*Required=*/{TEXT("articulation"), TEXT("controller")});
```

Handler, placed after `HandleGetNavStatus` (copy the marshal safety shape from `HandleSetNavGoal` — read it first):

```cpp
// -----------------------------------------------------------------------------
// set_active_controller — repoint the articulation's bound controller.
//
// Matches `controller` against the attached UMjArticulationController
// subclasses' class names, normalised to lowercase with underscores removed
// ("base_drive" -> MjBaseDriveController, "mink_ik" -> MjMinkIKController),
// then AdoptRuntimeController() publishes the pick to the physics thread.
// Sequencing (e.g. "only switch after nav arrives") is the caller's job.
// -----------------------------------------------------------------------------
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetActiveController(
	const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	FString ArtName, Token;
	Req->TryGetStringField(TEXT("articulation"), ArtName);
	Req->TryGetStringField(TEXT("controller"), Token);

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(TEXT("unknown_articulation"), ArtName);

	// Normalise: lowercase, underscores stripped, so "base_drive" matches
	// "MjBaseDriveController".
	FString Needle = Token.ToLower().Replace(TEXT("_"), TEXT(""));
	if (Needle.IsEmpty())
		return MakeError(TEXT("missing_field"),
			TEXT("set_active_controller requires non-empty 'controller'"));

	// Component discovery + AdoptRuntimeController are game-thread-only.
	struct FSwitchResult
	{
		FThreadSafeBool bWasActive{false};
		FThreadSafeCounter Matches{0};
		FString ActiveClass;      // written on the game thread before Trigger
		FString AttachedClasses;  // for the unknown_controller message
		FCriticalSection StrLock; // guards the two FStrings
	};
	TSharedPtr<FSwitchResult, ESPMode::ThreadSafe> Result =
		MakeShared<FSwitchResult, ESPMode::ThreadSafe>();

	auto DoSwitch = [Needle, Result](AMjArticulation* ArtPtr) {
		TArray<UMjArticulationController*> Ctrls;
		ArtPtr->GetComponents<UMjArticulationController>(Ctrls);
		UMjArticulationController* Match = nullptr;
		int32 N = 0;
		FString All;
		for (UMjArticulationController* C : Ctrls)
		{
			if (!C)
				continue;
			const FString Cls = C->GetClass()->GetName();
			if (!All.IsEmpty())
				All += TEXT(", ");
			All += Cls;
			if (Cls.ToLower().Replace(TEXT("_"), TEXT("")).Contains(Needle))
			{
				Match = C;
				++N;
			}
		}
		Result->Matches.Set(N);
		{
			FScopeLock Lock(&Result->StrLock);
			Result->AttachedClasses = All;
		}
		if (N != 1 || !Match)
			return;
		if (ArtPtr->GetActiveController() == Match)
		{
			Result->bWasActive = true;
		}
		else
		{
			ArtPtr->AdoptRuntimeController(Match);
		}
		FScopeLock Lock(&Result->StrLock);
		Result->ActiveClass = Match->GetClass()->GetName();
	};

	if (IsInGameThread())
	{
		DoSwitch(Art);
	}
	else
	{
		// Same stack-safety rules as HandleSetNavGoal: weak actor ptr,
		// shared result, event by value — nothing borrowed from this frame.
		TWeakObjectPtr<AMjArticulation> WeakArt(Art);
		FEvent* Done = FPlatformProcess::GetSynchEventFromPool(false);
		AsyncTask(ENamedThreads::GameThread, [WeakArt, DoSwitch, Done]() {
			if (AMjArticulation* ArtPtr = WeakArt.Get())
				DoSwitch(ArtPtr);
			Done->Trigger();
		});
		Done->Wait(2000);
		FPlatformProcess::ReturnSynchEventToPool(Done);
	}

	const int32 N = Result->Matches.GetValue();
	FString ActiveClass, Attached;
	{
		FScopeLock Lock(&Result->StrLock);
		ActiveClass = Result->ActiveClass;
		Attached = Result->AttachedClasses;
	}
	if (N == 0)
		return MakeError(TEXT("unknown_controller"),
			FString::Printf(TEXT("no attached controller matches '%s' (attached: %s)"),
				*Token, Attached.IsEmpty() ? TEXT("none") : *Attached));
	if (N > 1)
		return MakeError(TEXT("ambiguous"),
			FString::Printf(TEXT("'%s' matches %d controllers (attached: %s)"),
				*Token, N, *Attached));
	if (ActiveClass.IsEmpty())
		return MakeError(TEXT("not_ready"),
			TEXT("controller switch did not complete (game thread stalled?)"));

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_active_controller_ok"));
	Reply->SetStringField(TEXT("active"), ActiveClass);
	Reply->SetBoolField(TEXT("was_active"), Result->bWasActive);
	return Reply;
}
```

NOTE: the lambda-captured `DoSwitch` (capturing `Needle` and `Result` by value) is itself captured by value into the AsyncTask — verify it compiles cleanly; if the nested capture fights the compiler, inline the body into the AsyncTask lambda instead. Also confirm `N > 1` counting: `Match` keeps the last hit but `N` carries the count — the `N != 1` guard makes that safe.

- [ ] **Step 4: Compile; run the test (editor closed)**

Compile: `Result: Succeeded`. Then run with `--filter "URLab.Nav.Ops.SetActiveController"`; expect `Result={Success}`. Also run the full `--filter "URLab.Nav"` once; expect 23/23.

- [ ] **Step 5: Format (line-scoped on RpcDispatcher.cpp; whole-file OK on the test) + commit**

```bash
git add Source/URLab/Public/MuJoCo/Core/MjArticulation.h Source/URLab/Public/Bridge/RpcDispatcher.h Source/URLab/Private/Bridge/RpcDispatcher.cpp Source/URLabEditor/Private/Tests/MjNavDemoOpsTests.cpp
git commit -m "feat(bridge): set_active_controller runtime op — repoint the bound articulation controller"
```

---

### Task 2: `tidybot_mobile_manip_demo.py`

**Files:**
- Create: `Scripts/demos/tidybot_mobile_manip_demo.py`

**Interfaces:**
- Consumes: Task 1's `client.runtime.set_active_controller(articulation=<UE name>, controller=...)` → reply `active`/`was_active`. Imports from the sibling `Scripts/demos/tidybot_mink_demo.py`: `build_controller_payload(draw_target=..., lazy_base_cost=...)` (the add_controller JSON), `stream_target(client, art_prefix, pos, quat, ...)`, `Tracker` (EE pose via client-side FK on synced qpos), `resolve_id_by_suffix`. Nav-side ops exactly as `tidybot_nav_demo.py` uses them (`create_level`, `spawn_box`, `add_quick_convert`, `spawn_nav_bounds(agent_radius=55)`, `add_nav_stack`, `set_nav_goal`/`get_nav_status`, `set_paused`, `find_actors(in_pie=True)` name resolution).
- Produces: exit 0 = all phases pass. Venv python: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python`. Lint-only for the executor (`py_compile`); the live run is collaborative.

- [ ] **Step 1: Write the script**

Create `Scripts/demos/tidybot_mobile_manip_demo.py`:

```python
#!/usr/bin/env python3
"""TidyBot mobile manipulation v1 — sequential drive-then-reach over the bridge.

Phase A (LIVE): the nav stack drives the base around an obstacle wall to a
staging pose near a table (arm passed through, holding home).
SWITCH: set_active_controller repoints the bound controller to the mink IK.
Phase B (DIRECT): stream an EE target above the table in step batches (the
tidybot_mink_demo pattern); accept when the pinch_site tracks it to tolerance.

Spec: docs/superpowers/specs/2026-07-15-mobile-manip-v1-design.md.
Run with the bridge venv's python, editor open (the user drives Simulate):
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
        Scripts/demos/tidybot_mobile_manip_demo.py
Exits nonzero on any phase failure.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

# Reuse the mink demo's proven helpers (same directory).
from tidybot_mink_demo import (
    Tracker,
    build_controller_payload,
    stream_target,
)

MODEL_XML = (
    Path(__file__).resolve().parents[1]
    / "mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"
)
ACTOR_ID = "mm_tidybot"  # unique — never tidybot_0 / nav_tidybot

# Scene (MuJoCo metres). Wall at x=2 forces the detour (nav-demo layout);
# the table sits past the staging pose. Table top at z = 0.70.
FLOOR = dict(actor_id="mm_floor", location=(0.0, 0.0, -0.1), size=(20.0, 20.0, 0.2))
OBSTACLES = [
    dict(actor_id="mm_wall", location=(2.0, 0.0, 0.4), size=(0.4, 3.0, 0.8)),
    dict(actor_id="mm_block_a", location=(3.0, -1.6, 0.4), size=(0.6, 0.6, 0.8)),
    dict(actor_id="mm_block_b", location=(1.0, 1.8, 0.4), size=(0.6, 0.6, 0.8)),
]
TABLE = dict(actor_id="mm_table", location=(5.2, 0.0, 0.35), size=(0.8, 1.2, 0.7))
NAV_BOUNDS = dict(center=(0.0, 0.0, 0.5), extent=(12.0, 12.0, 2.0))

STAGING = (4.2, 0.0)            # nav goal: in front of the table
DETOUR_MIN_Y = 1.0              # same detour assertion as the nav demo
NAV_TIMEOUT_S = 90.0
NAV_POLL_S = 0.5

# Reach target: above the table's near edge. Reachable from staging with the
# arm (~0.9 m envelope); the lazy base may legally shuffle to assist.
REACH_TARGET = np.array([4.9, 0.0, 0.85])
REACH_BATCHES = 120             # x STRIDE steps of ramp + settle
STRIDE = 10                     # steps per target update (0.02 s @ dt 0.002)
RAMP_BATCHES = 80               # target interpolates over these, then holds
ACCEPT_EE_ERR = 0.02            # m, sustained over the final batches
ACCEPT_SUSTAIN = 10             # final batches that must all be within tol


def log(msg: str) -> None:
    print(f"[mm-demo] {msg}", flush=True)


def fail(msg: str) -> None:
    print(f"[mm-demo] FAIL: {msg}", flush=True)
    sys.exit(1)


def robot_xy(client, actor_id: str):
    rows = client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True)
    for r in rows:
        if r.actor_id == actor_id:
            return r.location[0], r.location[1]
    raise RuntimeError(f"{actor_id!r} not found in the PIE world")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="tcp://localhost")
    ap.add_argument("--port", type=int, default=5560,
                    help="direct step-server port (mink-demo default)")
    ap.add_argument("--keep-open", action="store_true")
    args = ap.parse_args()

    # step_mode="direct" wires the step transport phase B needs; phase A
    # overrides to live via set_mode after PIE starts (the server-side mode
    # is what gates stepping — the client construct just picks transports).
    client = URLabClient(args.host, step_mode="direct", step_port=args.port)
    client.connect()
    log("connected")

    try:
        # ---- Scene (isolated level; see navigation-demo.md warning) --------
        log("creating MobileManipDemo level + scene")
        client.scene.create_level(name="MobileManipDemo")
        client.scene.spawn_box(**FLOOR)  # UE-only floor (MJCF plane = physics)
        for ob in OBSTACLES + [TABLE]:
            client.scene.spawn_box(**ob)
            client.outliner.add_quick_convert(target=ob["actor_id"], static=True)
        r = client.scene.spawn_nav_bounds(**NAV_BOUNDS, agent_radius=55.0, timeout_s=30.0)
        if not r.get("nav_data_present"):
            fail(f"navmesh did not bake: {r}")

        # ---- Robot: both controller stacks, pre-PIE ------------------------
        bp = client.scene.import_xml(path=str(MODEL_XML))
        client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
        client.scene.ensure_manager()
        stack = client.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
        log(f"nav stack: created={stack.get('created')}")
        ik = client.scene.add_controller(
            target=ACTOR_ID, **build_controller_payload(draw_target=True,
                                                        lazy_base_cost=5.0))
        log(f"ik controller: was_existing={ik.get('was_existing')}")

        # ---- Phase A: drive (live) ------------------------------------------
        log("entering PIE (sim.start)")
        client.sim.start(timeout_s=180.0)
        try:
            client.runtime.set_mode("live")
        except URLabRPCError as exc:
            log(f"set_mode(live) note: {exc}")
        client.runtime.set_paused(paused=False)  # bridge live is never auto-unpaused

        nav_name = None
        for row in client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True):
            if row.actor_id == ACTOR_ID:
                nav_name = row.name
                break
        if nav_name is None:
            fail("robot not found in PIE world")
        log(f"articulation name={nav_name!r}")

        # Two controllers are attached; the initial bind is order-dependent.
        r = client.runtime.set_active_controller(articulation=nav_name,
                                                 controller="base_drive")
        log(f"active controller -> {r.get('active')} (was_active={r.get('was_active')})")
        if r.get("active") != "MjBaseDriveController":
            fail(f"expected base drive active, got {r}")

        log(f"Phase A: set_nav_goal {STAGING}")
        r = client.runtime.set_nav_goal(articulation=nav_name,
                                        x=STAGING[0], y=STAGING[1])
        if not r.get("accepted"):
            fail(f"staging goal rejected: {r}")
        max_abs_y, state = 0.0, "navigating"
        deadline = time.time() + NAV_TIMEOUT_S
        while time.time() < deadline:
            s = client.runtime.get_nav_status(articulation=nav_name)
            state = s["state"]
            x, y = robot_xy(client, ACTOR_ID)
            max_abs_y = max(max_abs_y, abs(y))
            log(f"  A: {state:<10} dist={s['distance_to_goal']:.2f} pos=({x:.2f},{y:.2f})")
            if state in ("arrived", "failed"):
                break
            time.sleep(NAV_POLL_S)
        if state != "arrived":
            fail(f"Phase A did not arrive (state={state})")
        if max_abs_y < DETOUR_MIN_Y:
            fail(f"Phase A no detour (max |y|={max_abs_y:.2f})")
        log(f"Phase A ARRIVED (detour max |y|={max_abs_y:.2f} m)")

        # ---- Switch ----------------------------------------------------------
        r = client.runtime.set_active_controller(articulation=nav_name,
                                                 controller="mink_ik")
        if r.get("active") != "MjMinkIKController":
            fail(f"switch to IK failed: {r}")
        log("switched to MjMinkIKController")

        # ---- Phase B: reach (direct) ----------------------------------------
        client.runtime.set_mode("direct")
        tracker = Tracker(client, ACTOR_ID)  # verify Tracker's ctor args below
        p0, q0 = tracker.ee_pose()
        log(f"Phase B: seed target = current EE pose {np.round(p0, 3)}")
        stream_target(client, nav_name, p0, q0)
        client.step(n_steps=STRIDE)

        ok_streak = 0
        for k in range(REACH_BATCHES):
            a = min(1.0, k / float(RAMP_BATCHES))
            tgt = (1.0 - a) * p0 + a * REACH_TARGET
            stream_target(client, nav_name, tgt, q0)
            client.step(n_steps=STRIDE)
            ee, _ = tracker.ee_pose()
            err = float(np.linalg.norm(ee - REACH_TARGET))
            if k % 10 == 0 or a >= 1.0:
                log(f"  B: batch {k:3d} a={a:.2f} ee_err={err:.4f}")
            if not np.all(np.isfinite(client.data.qpos)):
                fail(f"non-finite qpos at batch {k}")
            ok_streak = ok_streak + 1 if (a >= 1.0 and err < ACCEPT_EE_ERR) else 0
            if ok_streak >= ACCEPT_SUSTAIN:
                break
        if ok_streak < ACCEPT_SUSTAIN:
            fail(f"Phase B did not settle within {ACCEPT_EE_ERR} m "
                 f"(streak={ok_streak}/{ACCEPT_SUSTAIN})")
        log(f"Phase B REACHED target (err<{ACCEPT_EE_ERR} m sustained x{ACCEPT_SUSTAIN})")

        log("ALL PHASES PASSED")
    finally:
        if not args.keep_open:
            try:
                client.sim.stop()
            except URLabRPCError:
                pass


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify the imported helpers' real signatures**

Read `Scripts/demos/tidybot_mink_demo.py` and confirm (fix the new script where reality differs — do NOT change the mink demo):
- `Tracker.__init__`'s actual arguments (it may take the client plus an articulation/prefix or resolve `pinch_site` itself) and that `ee_pose()` returns `(pos, quat)` in MuJoCo metres.
- `stream_target(client, art_prefix, pos, quat, ...)` — the `art_prefix` it expects is the runtime articulation name (same string as `nav_name`)?
- `build_controller_payload(...)` keyword names (`draw_target`, `lazy_base_cost`) and that `client.scene.add_controller(target=..., **payload)` is the correct call shape (the mink demo may call it differently — copy its exact call).
- Whether importing `tidybot_mink_demo` at module scope has side effects (it should be import-safe since it guards with `if __name__ == "__main__"` — verify).
- `client.step(n_steps=...)` availability while the session was started without `step_mode` asserted post-`set_mode("direct")` — the mink demo's connect/start sequence is the reference; mirror any mode re-assertion it does after `sim.start`.

- [ ] **Step 3: Lint**

Run: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python -m py_compile Scripts/demos/tidybot_mobile_manip_demo.py` — expect exit 0. (Import-time verification of `tidybot_mink_demo` helpers happens at the live run; py_compile does not execute imports.)

- [ ] **Step 4: Commit**

```bash
git add Scripts/demos/tidybot_mobile_manip_demo.py
git commit -m "demo(manip): sequential drive-then-reach mobile manipulation E2E"
```

---

## Verification (whole feature, controller + user)

1. Compile green; `URLab.Nav` filter green (23 tests) editor-closed.
2. Collaborative live run per [[collaborative-dev-nav]]: user opens the editor and presses Simulate; controller runs the demo phases with the user's go at each editor-state change; expect `ALL PHASES PASSED` and visually: detour drive, stop at staging, arm reach to the table.
3. Push to `fork`; update `.superpowers/sdd/progress.md` + the nav-stack memory.
4. Follow-up after a green live run: a short "Mobile manipulation" section in the docs (not part of this plan).
