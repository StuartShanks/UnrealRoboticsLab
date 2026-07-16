# Suction Pick v1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** TidyBot drives to a table, top-down picks an affordance-annotated box via suction, carries it home, and releases it — one guarded whole-body mink controller the entire run, orchestrated by a py_trees behavior tree.

**Architecture:** Two small C++ changes (mink non-drive ctrl pass-through; `set_suction` runtime op) unlock suction through the existing actuator staging. A new model variant swaps the 2f85 fingers for a suction cup + adhesion actuator; a pickable asset carries an affordance site whose FRAME encodes contact point + approach normal. Python skills (py_trees behaviours over bridge verbs) sequence drive → reach → engage → verify → carry → release, with kinematic condition gates (reach annulus, client-side `mj_ray` corridor probe).

**Tech Stack:** UE 5.7 / C++ (URLab plugin), MuJoCo (adhesion actuator, mj_ray), Python bridge client (`urlab_client`, local mujoco mirror), py_trees.

**Spec:** `docs/superpowers/specs/2026-07-16-suction-pick-v1-design.md` — binding requirements; read it first.

## Global Constraints

- Branch `feat/nav-stack`; push to remote `fork` (`git@github.com:StuartShanks/UnrealRoboticsLab.git`). NO `Co-Authored-By`/`Claude-Session` trailers on commits. NEVER `git add -A` — explicit paths only. Never stage `third_party/build_all.sh` or `Scripts/mink_golden/fixtures/*` / `tidybot_scene_ue_ue.xml` (another agent's WIP).
- **Golden-parity model untouched:** `Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml` and everything under `Scripts/mink_golden/fixtures/` must be byte-identical after this plan.
- Compile (editor-safe): `"/home/stuart/UE_ROOT/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-Project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -WaitMutex` → `Result: Succeeded`.
- Automation tests (REQUIRE the UE editor CLOSED): `./Scripts/build_and_test_linux.sh --engine /home/stuart/UE_ROOT/UnrealEngine --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter "<suite>"` → `Tests : N / N passed`.
- Format ONLY files you touched: `/home/stuart/miniconda3/bin/clang-format -i <files>` (never `Scripts/format.sh`).
- Bridge venv python: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python`.
- Unity build is ON: file-scope statics in test .cpp files need unique names; new test files may shift unity chunks — if an UNTOUCHED test file suddenly fails to compile with "undeclared identifier", add the missing `#include` to THAT file (it was inheriting includes from a neighbor).
- Doctrine (from the validated axis-1 stack): every streamed EE goal is RAMPED (bounded error — never stream a far target directly); controller payloads use `max_iters: 1` + QP velocity limits; a world-frame EE "hold" during base motion is forbidden (disable the EE task; posture holds); seed the manual target at the current pose BEFORE re-enabling the EE task.
- The pick object is EXCLUDED from all collision-guard groups; scenery (walls, blocks, table) is guarded.

---

### Task 1: Mink non-drive ctrl pass-through

The mink writes ONLY its drive ctrl ids (`MjMinkIKController.cpp` ~line 943: `d->ctrl[DriveCtrlIds[i]] = Q[DriveQposAddrs[i]]`), unlike `UMjBaseDriveController` which passes every non-base binding through (`MjBaseDriveController.cpp:109-120`). Consequence: a non-drive actuator's `SetNetworkControl` value (suction, fingers) NEVER reaches `d->ctrl` while the mink is bound. Fix = the same pass-through loop, honoring the contract "a bound controller replaces the articulation's default ctrl path entirely".

**Files:**
- Modify: `Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp` (ComputeAndApply, just before the `d->ctrl[DriveCtrlIds[i]]` write block ~line 938)
- Test: `Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp` (append)

**Interfaces:**
- Consumes: `Bindings` / `FActuatorBinding {ActuatorMjID, QposAddr, QvelAddr, Component}` (inherited, `MjArticulationController.h:39`); `UMjActuator::ResolveDesiredControl(uint8 Source)`, `UMjActuator::SetNetworkControl(float)`.
- Produces: while a mink is bound, any non-drive actuator's staged value reaches `d->ctrl` every physics step. Task 3's op and Task 5's `Suction` skill rely on this.

- [ ] **Step 1: Write the failing test** — append to `MjMinkIKControllerTests.cpp`. Uses the GOLDEN tidybot model: `fingers_actuator` is the perfect non-drive actuator (10 drive joints exclude it). Same fixture/joint-resolution boilerplate as `FMjMinkIKTidybotTwistFollow` in the same file — copy its sections 1–2 (import+compile, resolve ten joints) verbatim, then:

```cpp
// ============================================================================
// URLab.MinkIK.TidyBot.NonDrivePassThrough
//   A bound controller replaces the articulation's default ctrl path entirely
//   (the BaseDrive contract) — so the mink must pass non-drive actuator values
//   (fingers, suction) through to d->ctrl. Encodes the latent gap where
//   SetNetworkControl on the gripper was a silent no-op under a bound mink.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMinkIKTidybotNonDrivePassThrough,
	"URLab.MinkIK.TidyBot.NonDrivePassThrough",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMinkIKTidybotNonDrivePassThrough::RunTest(const FString&)
{
	using namespace MjMinkIKControllerTestsLocal;
	// [fixture import+compile + resolve TenJoints — copy from TwistFollow test]
	// ... S, M, D, Robot, TenJoints, ArmJoints as in FMjMinkIKTidybotTwistFollow ...

	// Resolve fingers_actuator (non-drive) + its component.
	const int32 FingersActId = FindIdBySuffix(M, mjOBJ_ACTUATOR, M->nu, TEXT("fingers_actuator"));
	if (!TestTrue(TEXT("fingers_actuator found"), FingersActId >= 0)) { S.Cleanup(); return false; }

	// Minimal mink stack: posture only; DriveJoints = the ten joints (fingers excluded).
	UMjMinkIKController* Ctrl = NewObject<UMjMinkIKController>(Robot, TEXT("MinkIKPassThrough"));
	FMinkTaskSpec Posture;
	Posture.Kind = EMinkTaskKind::Posture;
	Posture.Cost = 1e-3f;
	Posture.Joints = ArmJoints;
	Ctrl->Tasks = {Posture};
	FMinkLimitSpec ConfLimit;
	Ctrl->Limits = {ConfLimit};
	Ctrl->DriveJoints = TenJoints;
	Ctrl->MaxIters = 1;
	Ctrl->RegisterComponent();

	const int32 KeyId = FindIdBySuffix(M, mjOBJ_KEY, M->nkey, TEXT("home"));
	mj_resetDataKeyframe(M, D, KeyId);
	mj_forward(M, D);
	Robot->AdoptRuntimeController(Ctrl);
	if (!TestTrue(TEXT("controller bound"), Ctrl->IsBound())) { S.Cleanup(); return false; }

	// Stage a network value on the fingers actuator, run one controller pass.
	UMjActuator* Fingers = nullptr;
	for (UMjActuator* A : Robot->GetActuators())
		if (A && A->GetMjID() == FingersActId) { Fingers = A; break; }
	if (!TestNotNull(TEXT("fingers actuator component"), Fingers)) { S.Cleanup(); return false; }
	Fingers->SetNetworkControl(123.0f);
	Ctrl->ComputeAndApply(M, D, 0);
	TestEqual(TEXT("non-drive NetworkValue reaches d->ctrl under a bound mink"),
		(double)D->ctrl[FingersActId], 123.0, 1e-6);
	S.Cleanup();
	return true;
}
```

  (If `UMjActuator` exposes no `GetMjID()`, match by `GetMjName()` suffix `fingers_actuator` instead — check `MjActuator.h`; one of the two exists, the sibling tests use component-by-mjid matching via `FindComponentByMjId` for typed components.)

- [ ] **Step 2: Compile + red-run.** Compile (command in Global Constraints; expect `Succeeded`). With the editor closed: `./Scripts/build_and_test_linux.sh ... --filter "URLab.MinkIK.TidyBot.NonDrivePassThrough"` → expect **1 failed** ("reaches d->ctrl" assert: ctrl is 0.0, not 123.0).

- [ ] **Step 3: Implement the pass-through** in `MjMinkIKController.cpp::ComputeAndApply`, immediately BEFORE the `// data.ctrl[actuator_ids] = configuration.q[dof_ids]` block:

```cpp
	// Pass-through for every non-drive binding: a bound controller replaces
	// the articulation's default ctrl path entirely (BaseDrive contract), so
	// non-drive actuators (gripper, suction) must keep receiving their
	// UI/network values through us — otherwise SetNetworkControl is a silent
	// no-op while the mink is bound.
	for (const FActuatorBinding& B : Bindings)
	{
		if (B.ActuatorMjID < 0 || !B.Component || DriveCtrlIds.Contains(B.ActuatorMjID))
		{
			continue;
		}
		d->ctrl[B.ActuatorMjID] = B.Component->ResolveDesiredControl(Source);
	}
```

  (`DriveCtrlIds` is a ≤10-element TArray; `Contains` linear scan at 500 Hz is fine. `Source` is ComputeAndApply's `uint8 Source` parameter.)

- [ ] **Step 4: Compile + green-run.** Same filter → PASS. Then full regression: `--filter "URLab.MinkIK"` → all pass (expect 11/11).

- [ ] **Step 5: Format + commit.**

```bash
/home/stuart/miniconda3/bin/clang-format -i \
  Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp \
  Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp
# recompile once after formatting, then:
git add Source/URLab/Private/MuJoCo/Components/Controllers/MjMinkIKController.cpp \
        Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp
git commit -m "fix(mink): pass non-drive actuator values through to d->ctrl

A bound controller owns the whole ctrl vector (BaseDrive contract), but the
mink wrote only its drive ids — SetNetworkControl on any other actuator
(fingers, suction) was a silent no-op while the mink was bound. Adds the
same pass-through loop BaseDrive has. Test encodes the gap (red-run
verified)."
git push fork feat/nav-stack
```

---

### Task 2: Suction model variant + pickable asset (+ import tests)

**Files:**
- Create: `Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml` (copy of `tidybot_scene_ue.xml`, then edit)
- Create: `Scripts/demos/assets/suction_box.xml`
- Test: `Source/URLabEditor/Private/Tests/MjSuctionModelTests.cpp` (new file)

**Interfaces:**
- Produces: model with site `cup_site` (the new EE frame), adhesion actuator named `suction` (type `EMjActuatorType::Adhesion`), the same 10 drive actuators (`joint_x/y/th`, `joint_1..7`); asset with body `pick_box` + site `affordance_suction_top`. Tasks 3–6 depend on these exact names.

- [ ] **Step 1: Create the variant.** `cp Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml`, then edit `tidybot_suction_ue.xml` ONLY:
  1. Inside `<body name="base" ...>` (the 2f85 gripper base, childclass="2f85"): DELETE the four finger subtrees entirely — `<body name="right_driver">…</body>`, `<body name="right_spring_link">…</body>`, `<body name="left_driver">…</body>`, `<body name="left_spring_link">…</body>` (each closes its own nested couplers/followers/pads).
  2. In the same `<body name="base">`, ADD the cup (a dedicated child body so the adhesion force acts only on the cup geom):

```xml
        <body name="cup" pos="0 0 0.10">
          <geom name="cup_geom" type="cylinder" size="0.03 0.01" rgba="0.15 0.15 0.15 1"/>
          <site name="cup_site" pos="0 0 0.01" quat="0 1 0 0"/>
        </body>
```

  (The 2f85 base's finger axis is its local +z — the drivers mounted at z≈0.055; the old `pinch_site` sits on `bracelet_link` at local z −0.1815 ≈ base-frame z +0.12. The cup face at base-frame z 0.11 is a few mm proximal of the old pinch point. `quat="0 1 0 0"` matches `pinch_site`'s frame convention so existing target-quat conventions carry over. Leave `pinch_site` in place — harmless, and lets the verification step compare the two.)
  3. DELETE the finger plumbing sections: in `<contact>`, the six `<exclude .../>` lines referencing driver/spring_link/coupler/follower bodies; the entire `<tendon>…</tendon>` block; the entire `<equality>…</equality>` block; and in `<actuator>`, the `fingers_actuator` `<general …>` line. ADD in `<actuator>`:

```xml
    <adhesion name="suction" body="cup" ctrlrange="0 1" gain="100"/>
```

  4. Leave EVERYTHING else identical (10 position actuators, keyframe, defaults — unused 2f85 default classes are harmless).
  5. Sanity-compile locally before the UE test: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python -c "import mujoco; m = mujoco.MjModel.from_xml_path('Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml'); print('nu', m.nu, 'nsite', m.nsite)"` → prints without error, `nu` = 11 (10 drive + suction).

- [ ] **Step 2: Create the asset** `Scripts/demos/assets/suction_box.xml` (exact content from the spec):

```xml
<mujoco model="suction_box">
  <worldbody>
    <body name="pick_box">
      <freejoint/>
      <geom name="pick_box_geom" type="box" size="0.05 0.05 0.05"
            density="300" rgba="0.9 0.6 0.1 1"/>
      <site name="affordance_suction_top" pos="0 0 0.05" zaxis="0 0 1"
            type="cylinder" size="0.01 0.001" rgba="0 1 0 0.5"/>
    </body>
  </worldbody>
</mujoco>
```

  Sanity: same python one-liner with this path → `nsite` = 1.

- [ ] **Step 3: Write the failing UE import test** — new file `Source/URLabEditor/Private/Tests/MjSuctionModelTests.cpp` (unique static/namespace names — unity build):

```cpp
// (standard URLab copyright header — copy from MjBaseIntegrateTests.cpp)
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Core/MjArticulation.h"
#include <mujoco/mujoco.h>

namespace MjSuctionModelTestsLocal
{
inline int32 FindIdBySuffixLocal(const mjModel* M, mjtObj Type, int32 Count, const TCHAR* Name)
{
	const FString Want(Name);
	for (int32 i = 0; i < Count; ++i)
	{
		const char* Nm = mj_id2name(const_cast<mjModel*>(M), Type, i);
		if (Nm && FString(UTF8_TO_TCHAR(Nm)).EndsWith(Want))
			return i;
	}
	return -1;
}
} // namespace MjSuctionModelTestsLocal

// URLab.Suction.ModelVariant — the cup variant imports+compiles: adhesion
// actuator present and typed, cup_site resolves, the 10 drive actuators are
// intact, and the finger linkage is gone.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSuctionModelVariant,
	"URLab.Suction.ModelVariant",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjSuctionModelVariant::RunTest(const FString&)
{
	using namespace MjSuctionModelTestsLocal;
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml"));
	if (!FPaths::FileExists(XmlPath)) { AddError(TEXT("suction variant missing")); return false; }
	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath) || !S.Compile()) { AddError(S.LastError); S.Cleanup(); return false; }
	mjModel* M = S.Model();

	TestTrue(TEXT("cup_site resolves"),
		FindIdBySuffixLocal(M, mjOBJ_SITE, M->nsite, TEXT("cup_site")) >= 0);
	TestTrue(TEXT("suction actuator resolves"),
		FindIdBySuffixLocal(M, mjOBJ_ACTUATOR, M->nu, TEXT("suction")) >= 0);
	TestTrue(TEXT("fingers gone"),
		FindIdBySuffixLocal(M, mjOBJ_ACTUATOR, M->nu, TEXT("fingers_actuator")) < 0);
	TestEqual(TEXT("11 actuators (10 drive + suction)"), (int32)M->nu, 11);
	// Typed adhesion component generated by the importer:
	bool bAdhesionTyped = false;
	for (UMjActuator* A : S.Robot->GetActuators())
		if (A && A->Type == EMjActuatorType::Adhesion) { bAdhesionTyped = true; break; }
	TestTrue(TEXT("UMjAdhesionActuator typed component present"), bAdhesionTyped);
	S.Cleanup();
	return true;
}

// URLab.Suction.PickAsset — the affordance-annotated free body imports.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSuctionPickAsset,
	"URLab.Suction.PickAsset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjSuctionPickAsset::RunTest(const FString&)
{
	using namespace MjSuctionModelTestsLocal;
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Scripts/demos/assets/suction_box.xml"));
	if (!FPaths::FileExists(XmlPath)) { AddError(TEXT("suction_box asset missing")); return false; }
	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath) || !S.Compile()) { AddError(S.LastError); S.Cleanup(); return false; }
	mjModel* M = S.Model();
	TestTrue(TEXT("affordance site resolves"),
		FindIdBySuffixLocal(M, mjOBJ_SITE, M->nsite, TEXT("affordance_suction_top")) >= 0);
	TestTrue(TEXT("free joint present"), M->njnt >= 1 && M->jnt_type[0] == mjJNT_FREE);
	S.Cleanup();
	return true;
}
```

- [ ] **Step 4: Red-run** (before creating the files if you wrote the test first, or expected-green if files exist — run `--filter "URLab.Suction"`; both must PASS once files are in place; if the variant fails to compile the test prints the compiler error via `S.LastError`).
- [ ] **Step 5: Verify goldens untouched:** `git status --short Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml Scripts/mink_golden/fixtures/` → EMPTY output (the untracked `tidybot_ctrl_parity.csv`/`tidybot_scene_ue_ue.xml` WIP files must NOT be staged). Run `--filter "URLab.MinkIK"` once more → all pass (golden fixtures untouched).
- [ ] **Step 6: Format the test file + commit** (add exactly: the two new XML files + `MjSuctionModelTests.cpp`):

```bash
git add Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml \
        Scripts/demos/assets/suction_box.xml \
        Source/URLabEditor/Private/Tests/MjSuctionModelTests.cpp
git commit -m "feat(suction): cup model variant + affordance-annotated pick asset

tidybot_suction_ue.xml: 2f85 finger subtree -> cup body/geom/site + adhesion
actuator (golden-parity model untouched). suction_box.xml: free-body pickable
with an affordance site whose frame encodes contact point + surface normal."
git push fork feat/nav-stack
```

---

### Task 3: `set_suction` runtime op

**Files:**
- Modify: `Source/URLab/Private/Bridge/RpcDispatcher.cpp` (Reg block ~line 226 area + new handler near `HandleSetActiveController`)
- Modify: `Source/URLab/Public/Bridge/RpcDispatcher.h` (declare `HandleSetSuction`)
- Test: `Source/URLabEditor/Private/Tests/MjNavOpsTests.cpp` (append)

**Interfaces:**
- Consumes: Task 1's pass-through (delivery), Task 2's suction variant (an adhesion actuator to find). `AMjArticulation::GetActuators()`, `UMjActuator::SetNetworkControl(float)`, `EMjActuatorType::Adhesion`, the `HandleSetNavGoal` marshal idiom (`RpcDispatcher.cpp:2638` — by-value TWeakObjectPtr + TSharedPtr result + FEvent Wait(2000)).
- Produces: op `set_suction` `{articulation, value:[0..1]}` → reply `{op:"set_suction_ok", actuator:string, value:float}`; errors `not_ready`/`unknown_articulation`/`no_adhesion_actuator`/`ambiguous_adhesion_actuator`. Python: `client.runtime.set_suction(articulation=..., value=1.0)` (auto-synthesized from meta).

- [ ] **Step 1: Write the failing ops test** — append to `MjNavOpsTests.cpp` (it has the `NavReq` helper + `FMjUESession` pattern; extend `NavReq` usage with a value field):

```cpp
// URLab.Nav.Ops.SetSuction — set_suction stages the value on the articulation's
// adhesion actuator; robots without one error cleanly.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNavOpsSetSuction,
	"URLab.Nav.Ops.SetSuction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjNavOpsSetSuction::RunTest(const FString&)
{
	// Error path: the plain test-rig robot has no adhesion actuator.
	{
		FMjUESession S;
		if (!S.Init([](FMjUESession&) {})) { AddError(S.LastError); S.Cleanup(); return false; }
		FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
		Disp->SetActiveSessionIdForTest(TEXT("test-session"));
		TSharedPtr<FJsonObject> Req = NavReq(TEXT("set_suction"), S.Robot->GetName());
		Req->SetNumberField(TEXT("value"), 1.0);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		FString Err;
		TestTrue(TEXT("error field"), Reply->TryGetStringField(TEXT("error"), Err));
		TestEqual(TEXT("no_adhesion_actuator"), Err, TEXT("no_adhesion_actuator"));
		S.Cleanup();
	}
	// Happy path: the suction variant has exactly one adhesion actuator.
	{
		const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
			TEXT("UnrealRoboticsLab/Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml"));
		FMjXmlImportSession S;
		if (!S.InitFromFile(XmlPath) || !S.Compile()) { AddError(S.LastError); S.Cleanup(); return false; }
		FURLabRpcDispatcher* Disp = S.Manager->BridgeServer->GetDispatcher();
		Disp->SetActiveSessionIdForTest(TEXT("test-session"));
		TSharedPtr<FJsonObject> Req = NavReq(TEXT("set_suction"), S.Robot->GetName());
		Req->SetNumberField(TEXT("value"), 0.7);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		double V = 0.0;
		TestTrue(TEXT("value echoed"), Reply->TryGetNumberField(TEXT("value"), V));
		TestEqual(TEXT("value"), V, 0.7, 1e-6);
		FString ActName;
		TestTrue(TEXT("actuator named"), Reply->TryGetStringField(TEXT("actuator"), ActName));
		TestTrue(TEXT("named 'suction'"), ActName.Contains(TEXT("suction")));
		S.Cleanup();
	}
	return true;
}
```

  (If `FMjXmlImportSession` lacks `Manager`/`BridgeServer` — check how `MjNavOpsTests` builds its dispatcher vs how `MjMinkIKControllerTests` builds `FMjXmlImportSession`; if the import-session has no dispatcher, use `FMjUESession` + `import_xml`-style spawn OR add the suction robot via `S.Manager` from an `FMjUESession` variant — mirror whatever `MjNavDemoOpsTests.cpp` does for its imported-robot ops tests, which exercise exactly this combination.)

- [ ] **Step 2: Compile + red-run** `--filter "URLab.Nav.Ops.SetSuction"` → FAIL (`unknown op` / missing fields).

- [ ] **Step 3: Implement.** In `RpcDispatcher.h`, next to `HandleSetActiveController`: `TSharedPtr<FJsonObject> HandleSetSuction(const TSharedPtr<FJsonObject>& Req);`. In `RpcDispatcher.cpp` Reg block (after `set_active_controller`):

```cpp
	Reg(TEXT("set_suction"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetSuction(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("actuator:string"), TEXT("value:float")},
		/*Required=*/{TEXT("articulation"), TEXT("value")});
```

  Handler (mirror `HandleSetNavGoal`'s marshal comments/structure exactly — by-value captures, heap result, `FEvent` from pool, `Wait(2000)`):

```cpp
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetSuction(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	FString ArtName;
	Req->TryGetStringField(TEXT("articulation"), ArtName);
	double Value = 0.0;
	Req->TryGetNumberField(TEXT("value"), Value);
	Value = FMath::Clamp(Value, 0.0, 1.0);
	FString ActFilter; // optional disambiguator
	Req->TryGetStringField(TEXT("actuator"), ActFilter);

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(TEXT("unknown_articulation"), ArtName);

	// Component discovery is game-thread-owned; marshal like HandleSetNavGoal.
	struct FSuctionResult
	{
		FThreadSafeBool bFound{false};
		FThreadSafeBool bAmbiguous{false};
		FString Name; // written before Trigger, read after Wait — ordered by the event
	};
	TSharedPtr<FSuctionResult, ESPMode::ThreadSafe> Result =
		MakeShared<FSuctionResult, ESPMode::ThreadSafe>();
	TWeakObjectPtr<AMjArticulation> WeakArt(Art);
	const float ValueF = (float)Value;
	auto DoSet = [WeakArt, ValueF, ActFilter, Result]() {
		if (AMjArticulation* ArtPtr = WeakArt.Get())
		{
			UMjActuator* Match = nullptr;
			for (UMjActuator* A : ArtPtr->GetActuators())
			{
				if (!A || A->Type != EMjActuatorType::Adhesion)
					continue;
				if (!ActFilter.IsEmpty() && !A->GetMjName().Contains(ActFilter))
					continue;
				if (Match) { Result->bAmbiguous = true; return; }
				Match = A;
			}
			if (Match)
			{
				Match->SetNetworkControl(ValueF);
				Result->Name = Match->GetMjName();
				Result->bFound = true;
			}
		}
	};
	if (IsInGameThread())
	{
		DoSet();
	}
	else
	{
		FEvent* Done = FPlatformProcess::GetSynchEventFromPool(false);
		AsyncTask(ENamedThreads::GameThread, [DoSet, Done]() { DoSet(); Done->Trigger(); });
		Done->Wait(2000);
		FPlatformProcess::ReturnSynchEventToPool(Done);
	}

	if (Result->bAmbiguous)
		return MakeError(TEXT("ambiguous_adhesion_actuator"),
			TEXT("multiple adhesion actuators — pass 'actuator' to disambiguate"));
	if (!Result->bFound)
		return MakeError(TEXT("no_adhesion_actuator"),
			FString::Printf(TEXT("'%s' has no adhesion actuator"), *ArtName));

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_suction_ok"));
	Reply->SetStringField(TEXT("actuator"), Result->Name);
	Reply->SetNumberField(TEXT("value"), Value);
	return Reply;
}
```

- [ ] **Step 4: Compile + green-run** the filter, then `--filter "URLab.Nav"` regression (expect all prior + 1 new pass).
- [ ] **Step 5: Format touched files + commit** (`RpcDispatcher.cpp`, `RpcDispatcher.h`, `MjNavOpsTests.cpp`); message: `feat(bridge): set_suction runtime op — stage adhesion ctrl via NetworkControl`; push.

---

### Task 4: Python skills — `urlab_skills.py` (+ pure-function self-tests)

**Files:**
- Create: `Scripts/demos/urlab_skills.py`
- Create: `Scripts/demos/test_urlab_skills.py` (pure functions only; runs standalone)
- Modify (env, not committed): `pip install py_trees` into the bridge venv

**Interfaces:**
- Consumes: `URLabClient` (`client.model` / `client.data` mujoco mirror; `client.runtime.set_nav_goal/get_nav_status/set_suction`, `client._rpc_configure_controller`, `client.outliner.find_actors`, `client.step`, `client.runtime.set_mode/set_paused`); `tidybot_mink_demo.stream_target`, `resolve_id_by_suffix`.
- Produces (exact signatures Tasks 5 uses):
  - `resolve_affordance(client, body_suffix: str, prefix="affordance_suction") -> Affordance` where `Affordance = dataclass(point: np.ndarray, normal: np.ndarray, quat_cup_down: np.ndarray, waypoints: list[np.ndarray])` (waypoints = `[point+0.15·n, point+0.02·n, point]`)
  - `reach_annulus_ok(shoulder_xy_z: np.ndarray, target: np.ndarray, r_min=0.30, r_max=0.85) -> bool`
  - `corridor_clear(client, p_from, p_to, exclude_body_suffixes: list[str], tolerance=0.01) -> tuple[bool, float]` (client-side `mujoco.mj_ray` on the mirror; multi-exclusion by marching past excluded hits, ≤8 marches)
  - py_trees behaviours: `Drive(name, bb, goal_xy)`, `ResolveAffordance(...)`, `Reachable(...)`, `CorridorClear(...)`, `ReachRamp(name, bb, waypoints_key, quat_key, duration)`, `DescendEngage(...)`, `VerifyAttach(...)`, `StowCarry(...)`, `Release(...)` — each `update()` returns `py_trees.common.Status.{SUCCESS,FAILURE,RUNNING}` and sets `bb.fail_reason` on FAILURE.

- [ ] **Step 1: Install py_trees:** `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/pip install py_trees` → note the installed version in the commit message. (Env-only; nothing to commit for this step.)
- [ ] **Step 2: Write the pure functions + failing self-test.** `test_urlab_skills.py` (plain asserts, no pytest dep):

```python
#!/usr/bin/env python3
"""Self-tests for urlab_skills pure functions (no editor, no client).
Run: /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python test_urlab_skills.py"""
import numpy as np
import sys
sys.path.insert(0, ".")
from urlab_skills import approach_waypoints, cup_down_quat, reach_annulus_ok

# approach_waypoints: template from an affordance frame.
pt = np.array([1.0, 2.0, 0.5]); n = np.array([0.0, 0.0, 1.0])
w = approach_waypoints(pt, n)
assert np.allclose(w[0], [1.0, 2.0, 0.65]), w[0]   # pre-approach +15cm
assert np.allclose(w[1], [1.0, 2.0, 0.52]), w[1]   # engage +2cm
assert np.allclose(w[2], pt), w[2]                  # contact
# non-unit normals are normalized
w2 = approach_waypoints(pt, np.array([0.0, 0.0, 2.0]))
assert np.allclose(w2[0], [1.0, 2.0, 0.65]), w2[0]

# cup_down_quat: cup axis anti-parallel to the site normal. For n=+z the cup
# points down — matches the pinch/cup frame convention quat (w,x,y,z)=(0,1,0,0).
q = cup_down_quat(np.array([0.0, 0.0, 1.0]))
assert np.allclose(np.abs(q), [0.0, 1.0, 0.0, 0.0], atol=1e-6), q

# reach annulus
shoulder = np.array([4.2, 0.0, 0.49])
assert reach_annulus_ok(shoulder, np.array([4.9, 0.0, 0.5]))          # 0.70 -> ok
assert not reach_annulus_ok(shoulder, np.array([5.3, 0.0, 0.5]))      # 1.10 -> too far
assert not reach_annulus_ok(shoulder, np.array([4.3, 0.0, 0.45]))     # 0.11 -> too close
print("urlab_skills self-tests OK")
```

- [ ] **Step 3: Run it → fails** (`ModuleNotFoundError: urlab_skills`).
- [ ] **Step 4: Implement `urlab_skills.py`.** Pure functions first:

```python
#!/usr/bin/env python3
"""Reusable pick-pipeline skills: py_trees behaviours over URLab bridge verbs,
plus the pure kinematic helpers (annulus, affordance template, corridor ray).

Doctrine (validated 2026-07-16, see docs/roadmap/navigation-and-mobile-manip.md):
every streamed EE goal is RAMPED (bounded error); a world-frame EE hold during
base motion is forbidden (disable the EE task, posture holds); seed the manual
target at the current pose BEFORE re-enabling the EE task."""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import mujoco
import numpy as np
import py_trees

PRE_APPROACH_M = 0.15
ENGAGE_M = 0.02


def approach_waypoints(point: np.ndarray, normal: np.ndarray) -> list:
    """Template trajectory from an affordance frame: pre-approach -> engage ->
    contact, along the outward surface normal."""
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    p = np.asarray(point, dtype=float)
    return [p + PRE_APPROACH_M * n, p + ENGAGE_M * n, p.copy()]


def cup_down_quat(normal: np.ndarray) -> np.ndarray:
    """Target EE quat (wxyz) with the cup axis anti-parallel to the affordance
    normal. The cup/pinch frame convention has the tool axis along local -z of
    the (0,1,0,0) frame, so for n=+z (top pick) this returns (0,1,0,0)."""
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    z_axis = -n                       # tool axis points INTO the surface
    # Build a frame: pick any x orthogonal to z.
    helper = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(helper, z_axis); x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rot = np.stack([x_axis, y_axis, z_axis], axis=1).reshape(9)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, rot)
    return q


def reach_annulus_ok(shoulder: np.ndarray, target: np.ndarray,
                     r_min: float = 0.30, r_max: float = 0.85) -> bool:
    """Bent-arm reachability annulus (empirically mapped 2026-07-16)."""
    d = float(np.linalg.norm(np.asarray(target) - np.asarray(shoulder)))
    return r_min <= d <= r_max


def corridor_clear(client, p_from, p_to, exclude_body_suffixes,
                   tolerance: float = 0.01, max_marches: int = 8):
    """Client-side mj_ray on the mirrored model/data (run right after a synced
    step so the mirror is fresh). mj_ray excludes ONE body; multiple exclusions
    march past excluded hits. Returns (clear, first_blocking_distance)."""
    m, d = client.model, client.data
    excluded = set()
    for suf in exclude_body_suffixes:
        for b in range(m.nbody):
            nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
            if nm and nm.endswith(suf):
                # body + full subtree
                for b2 in range(m.nbody):
                    p = b2
                    while p != 0:
                        if p == b:
                            excluded.add(b2)
                            break
                        p = m.body_parentid[p]
                excluded.add(b)
    p0 = np.asarray(p_from, dtype=float)
    vec = np.asarray(p_to, dtype=float) - p0
    length = float(np.linalg.norm(vec))
    direction = vec / length
    travelled = 0.0
    for _ in range(max_marches):
        geomid = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(m, d, p0, direction, None, 1, -1, geomid)
        if dist < 0 or travelled + dist >= length - tolerance:
            return True, length
        hit_body = m.geom_bodyid[int(geomid[0])]
        if hit_body in excluded:
            step = dist + 1e-4
            p0 = p0 + step * direction
            travelled += step
            continue
        return False, travelled + dist
    return False, travelled
```

  Then the behaviours (each a `py_trees.behaviour.Behaviour` subclass; blackboard is a plain shared object):

```python
@dataclass
class PickBlackboard:
    client: object = None
    name: str = ""                 # articulation name in PIE
    object_actor_id: str = ""
    ee_task: int = 0
    damping_task: int = 2
    affordance: object = None      # Affordance
    fail_reason: str = ""
    home_xy: tuple = (0.0, 0.0)
    q_cup: np.ndarray = None


@dataclass
class Affordance:
    point: np.ndarray
    normal: np.ndarray
    quat_cup_down: np.ndarray
    waypoints: list = field(default_factory=list)
```

  Behaviour implementations wrap the exact verb sequences validated in the demos (Drive = `tidybot_twist_follow_demo` Phase A poll loop incl. the reset-fingerprint abort; ReachRamp = seed-then-enable + 6 s waypoint ramp via `stream_target`; DescendEngage = final-leg ramp + `client.runtime.set_suction(articulation=bb.name, value=1.0)` when the cup site is within 2 cm of the affordance point (cup pose read via the Tracker pattern against `cup_site`); VerifyAttach = ramped +10 cm lift, then compare the object's z via `find_actors(in_pie=True)` before/after (SUCCESS if risen ≥ 0.05), one retry from 1 cm lower; StowCarry = ramp EE to `current + [−0.15, 0, +0.25]` then `task_enabled [False, True, True, True]`; Release = seed-then-enable at current, ramp to a drop pose 0.35 m forward / 0.45 m high of the base, `set_suction value=0.0`, verify the object's z separates from the cup by ≥ 0.05 within 3 s, re-stow). Every FAILURE sets `bb.fail_reason`. Skills that need synced reads use the shared `synced_ee`-style dip helper, defined once in this module as `synced_site_pose(client, site_suffix)`.

- [ ] **Step 5: Self-test green:** `cd Scripts/demos && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python test_urlab_skills.py` → `urlab_skills self-tests OK`. Also `python -m py_compile urlab_skills.py test_urlab_skills.py`.
- [ ] **Step 6: Commit** (`Scripts/demos/urlab_skills.py`, `Scripts/demos/test_urlab_skills.py`); message: `feat(skills): py_trees pick skills + kinematic checks (annulus, affordance template, client-side corridor ray)`; push.

---

### Task 5: The demo — `tidybot_suction_pick_demo.py`

**Files:**
- Create: `Scripts/demos/tidybot_suction_pick_demo.py`

**Interfaces:**
- Consumes: everything above. Scene constants from `tidybot_mobile_manip_demo` (`FLOOR`, `OBSTACLES`, `NAV_BOUNDS`, `AGENT_RADIUS`, `STAGING`); `tidybot_guarded_reach_demo.TABLE` + guard-group builders (`ROBOT_LINKS`, `LINK_STANDOFF`); `tidybot_twist_follow_demo.twist_follow_controller_payload`; `urlab_skills` behaviours.
- Produces: the committed round-trip demo, exit 0 on success / nonzero on failure; `--keep-open` flag.

- [ ] **Step 1: Write the demo.** Structure (committed-demo format, mirrors `tidybot_guarded_reach_demo.py`):
  1. **Scene**: `create_level("SuctionPickDemo", force_overwrite=True)`; light; floor; `OBSTACLES + [TABLE]` quick-converted, capturing `actor_name` → `obstacle_bodies` (`"<name>_MjBody"`).
  2. **Pick asset**: `bp_box = client.scene.import_xml(path=str(ASSETS/"suction_box.xml"))`; `spawn_actor(blueprint=bp_box, actor_id="pick_box", location=(5.0, 0.0, 0.66))` (on the table top: table top z=0.6 + box half 0.05 + 1 cm settle). The object is EXCLUDED from all guard groups.
  3. **Robot**: import `tidybot_suction_ue.xml`; `add_nav_stack`; controller payload = `twist_follow_controller_payload()` with the EE Frame task's `frame` switched to `"cup_site"`, plus guard entries `{ROBOT_LINKS × obstacle_bodies, min 0.10, detect 0.40}` and `{["cup"] × obstacle_bodies, min 0.02, detect 0.20}` (cup close-quarters — the finger envelope is gone). Fail fast on `warnings`.
  4. **Tree** (py_trees Sequence, memory=True; VerifyAttach wrapped in `py_trees.decorators.Retry(num_failures=1)`):

```
ResolveAffordance("pick_box") → Reachable → Drive(STAGING) → CorridorClear
→ ReachRamp(pre-approach, cup-down) → DescendEngage → VerifyAttach
→ StowCarry → Drive(home=(0,0)) → Release
```

  5. **Run loop**: `tree.tick_tock(period_ms=250)` or a manual tick loop with a 240 s wall-clock ceiling; on root FAILURE print `bb.fail_reason` and exit 1; on SUCCESS print `ALL PHASES PASSED — suction pick round trip`.
  6. Sim mode plumbing identical to the guarded demo (live for phases, synced dips for reads, damping relax/restore around transits, `sim.stop` unless `--keep-open`).
- [ ] **Step 2: `python -m py_compile Scripts/demos/tidybot_suction_pick_demo.py`** → clean; run `test_urlab_skills.py` once more (imports still consistent).
- [ ] **Step 3: Commit** (`Scripts/demos/tidybot_suction_pick_demo.py`); message: `demo(suction-pick): round-trip pick via BT-over-skills — nav, grab, nav while holding, release`; push.

---

### Task 6: Live validation + records (collaborative — controller + user)

This task is interactive by design (the user drives Simulate; see `[[collaborative-dev-nav]]` conventions). NOT for a headless subagent — the session controller runs it with the user.

- [ ] **Step 1:** Split author/drive probes into `$CLAUDE_JOB_DIR/tmp` (author = scene+robot+asset+payload from the demo's own functions; drive = the tree) following the mm_* probe pattern.
- [ ] **Step 2:** Live runs with the user until "ALL PHASES PASSED" + the user confirms the visuals (pick, carry, release). Expected debugging surface: adhesion `gain` (100 N start), engage distance (2 cm), carry stability at 0.6 m/s.
- [ ] **Step 3:** Fold any live findings back into the committed files; re-run `URLab.MinkIK` + `URLab.Nav` + `URLab.Suction` suites green; final commit + push.
- [ ] **Step 4:** Update `.superpowers/sdd/progress.md` + `docs/roadmap/navigation-and-mobile-manip.md` (milestone → DONE + findings); update the `nav-roadmap` memory.

---

## Self-Review (performed at write time)

- **Spec coverage:** pass-through→T1; set_suction→T3; check_clearance (client-side, spec amended)→T4 `corridor_clear`; model variant + asset + affordance convention→T2; template trajectories→T4 `approach_waypoints`; skills/tree/error-handling→T4/T5; testing section→each task + T6; guards/object-exclusion→T5; deferred list→untouched. No gaps.
- **Placeholders:** T4 Step 4's behaviour bodies are summarized against named, validated reference implementations (exact demos/functions cited) rather than reprinted — the referenced code is in-repo, not hypothetical. All new algorithms (waypoints, quat, ray-march, op handler, tests) are given in full.
- **Type consistency:** `Affordance`/`PickBlackboard` fields match T5's tree usage; `set_suction` reply fields match T3's test; `cup_site`/`suction`/`pick_box`/`affordance_suction_top` names consistent across T2/T3/T4/T5.
