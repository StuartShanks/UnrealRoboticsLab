# URLabMink (1:1 mink C++ Port) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A new UE runtime module `URLabMink` that ports mink v1.2.0 (differential inverse kinematics on MuJoCo) to C++ 1:1, with numerical-parity golden tests generated from Python mink.

**Architecture:** `URLabMink` depends only on the MuJoCo C API (already installed under `third_party/install/MuJoCo`), UE's bundled Eigen, and a vendored Apache-2.0 Goldfarb–Idnani QP solver (qpmad). It does NOT depend on `URLab`; `URLab` consumes it. Parity is proven layer-by-layer against JSON fixtures produced by a pinned Python mink (`Scripts/mink_golden/`). The port calls the same `mju_*`/`mj_*` MuJoCo C functions Python mink calls through its bindings, so float paths match almost bit-for-bit.

**Tech Stack:** UE 5.7/5.8 C++ module, MuJoCo C API, Eigen (UE module), qpmad 1.4.0 (header-only, vendored), UE Automation Tests, Python 3.13 venv (`mink==1.2.0`, `mujoco==3.9.0`, `quadprog`) for fixture generation only.

## Global Constraints

Every task implicitly includes this section.

- **Branch:** all work happens on `feat/urlabmink` (already created; spec committed as `30403fc`).
- **Working tree hygiene:** the tree contains UNRELATED uncommitted VR-teleop WIP (`Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp`, `third_party/build_all.sh`, `Source/URLab/{Private,Public}/MuJoCo/Components/Controllers/MjEndEffectorController.*`). NEVER `git add -A`, `git add .`, or `git commit -a`. Always `git add` explicit paths.
- **Repo root:** `/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab` (note the space in the path — always quote). All relative paths below are relative to this root.
- **Engine / project:**
  - `UE_ENGINE=/home/stuart/UnrealEngine` (verify once in Task 1 via the project's `EngineAssociation` GUID `727189A0-ED7E-4B6F-9D5E-DB6529AF769B` in `~/.config/Epic/UnrealEngine/Install.ini`).
  - Project: `/home/stuart/Documents/Unreal Projects/Test/Test.uproject`, editor target `TestEditor`.
  - **Build:** `"$UE_ENGINE/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development "-project=/home/stuart/Documents/Unreal Projects/Test/Test.uproject"` — expected: exits 0, no errors.
  - **Run tests:** `"$UE_ENGINE/Engine/Binaries/Linux/UnrealEditor-Cmd" "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" -ExecCmds="Automation RunTests URLab.Mink; Quit" -unattended -nopause -nosplash -nullrhi -stdout | grep -E "Test Completed|Result={"` — expected per test: `Result={Passed}`.
  - Full pre-PR run: `./Scripts/build_and_test_linux.sh --engine "$UE_ENGINE" --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter URLab.Mink` — expected exit 0.
- **Pinned versions:** mink `v1.2.0` (commit `198674e30efcd8ffe24b58155c60abc014198b9f`), python `mujoco==3.9.0`, `quadprog` (freeze exact in `requirements.lock`), qpmad tag `1.4.0` (commit `b2fd8d57d973cab5d74decc1bbfc3c622d111240`, Apache-2.0). A clone of mink v1.2.0 already exists at `/tmp/claude-1000/-home-stuart/f1f825fb-0529-4fc7-9e1b-6f6ba700bbed/scratchpad/mink` for reference (re-clone if missing: `git clone --depth 50 https://github.com/kevinzakka/mink && git checkout v1.2.0`).
- **Parity tolerances** (max absolute element difference, `MaxAbsDiff(a,b) <= Tol * FMath::Max(1.0, MaxAbs(b))`):
  - `TOL_LIE = 1e-10` (lie ops), `TOL_KIN = 1e-9` (Configuration), `TOL_OBJ = 1e-8` (task/limit matrices), `TOL_QP = 1e-7` (QP solutions), `TOL_IK = 1e-6` (end-to-end `solve_ik` velocities and integrated q).
- **Standard file header** — EVERY new `.h`, `.cpp`, `.cs`, `.py`, `.sh` file starts with this exact header (adjust comment token: `//` for C++/C#, `#` for Python/shell). Code blocks below abbreviate it as `// [URLab standard file header]` / `# [URLab standard file header]`; that abbreviation always means this full text:

```
// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.
```

- **Formatting:** run `./Scripts/format.sh` before every commit (repo uses Epic clang-format, tabs).
- **Test naming:** all automation tests are named `URLab.Mink.<Area>.<Case>`, flags `EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter`, wrapped in `#if WITH_DEV_AUTOMATION_TESTS`.
- **C++ conventions:** doubles everywhere (mjtNum). Eigen included via `THIRD_PARTY_INCLUDES_START` / `THIRD_PARTY_INCLUDES_END` around `#include "Eigen/Dense"`. MuJoCo via `#include <mujoco/mujoco.h>`. Type aliases (defined in `MinkTypes.h`): `using FMinkVec = Eigen::VectorXd; using FMinkMat = Eigen::MatrixXd; using FMinkVec3 = Eigen::Vector3d; using FMinkVec6 = Eigen::Matrix<double,6,1>; using FMinkMat3 = Eigen::Matrix3d; using FMinkMat6 = Eigen::Matrix<double,6,6>; using FMinkRowMat = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;`
- **Deliberate deviations from Python mink** (document nowhere else; these are the only allowed ones):
  1. Exceptions → `bIsValid` flags on construction + `bool`/status-enum returns on hot paths, with `UE_LOG(LogURLabMink, ...)` carrying the exact Python exception message text.
  2. No abstract `MatrixLieGroup` base (Python dataclass-ism); SO3/SE3 are standalone structs with the same method set.
  3. `sample_uniform` takes an explicit `FRandomStream&` (no global RNG); excluded from parity fixtures.
  4. `VelocityLimit` takes `TArray<TPair<FString, FMinkVec>>` (not a map) to preserve Python-dict insertion order, which fixes G row order.
  5. `CollisionAvoidanceLimit`: both sides sort deduped geom-id pairs ascending (Python's `list(set(...))` order is not reproducible); the fixture generator sorts `limit.geom_id_pairs` after construction so rows correspond.
  6. `contrib/keyboard_teleop` is NOT ported (MuJoCo-viewer-specific).
- **Fixture invariant:** every fixture case stores all inputs explicitly (q vectors, tangents, targets); randomness exists only inside the Python generator (seeded). JSON floats round-trip exactly (Python repr ↔ double).

---

### Task 1: Module scaffold — `URLabMink` builds, loads, and has a running smoke test

**Files:**
- Create: `Source/URLabMink/URLabMink.Build.cs`
- Create: `Source/URLabMink/Public/MinkTypes.h`
- Create: `Source/URLabMink/Private/URLabMinkModule.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkModuleTests.cpp`
- Modify: `UnrealRoboticsLab.uplugin` (add module entry)

**Interfaces:**
- Consumes: `third_party/install/MuJoCo` (already built), UE `Eigen` module.
- Produces: module `URLabMink` (Runtime), log category `LogURLabMink`, header `MinkTypes.h` with the aliases from Global Constraints plus:
  - `enum class EMinkFrameType : uint8 { Body, Geom, Site };`
  - `URLABMINK_API mjtObj MinkFrameTypeToObj(EMinkFrameType Type);`
  - `URLABMINK_API bool MinkFrameTypeFromString(const FString& S, EMinkFrameType& Out);` (accepts "body"/"geom"/"site")
  - `URLABMINK_API int32 MinkDofWidth(int32 JointType);` / `MinkQposWidth(int32 JointType);` / `MinkConstraintWidth(int32 EqType);`
  - `constexpr double MinkEpsilon = 1e-10;` (mink float64 epsilon)
  - `URLABMINK_API FMinkMat3 MinkSkew(const FMinkVec3& X);`

- [ ] **Step 1: Resolve and export the engine path**

```bash
grep -B3 "727189A0-ED7E-4B6F-9D5E-DB6529AF769B" ~/.config/Epic/UnrealEngine/Install.ini
export UE_ENGINE=/home/stuart/UnrealEngine   # adjust to whatever the grep shows
```
Expected: an entry mapping the GUID to an engine root (expected `/home/stuart/UnrealEngine`).

- [ ] **Step 2: Write `Source/URLabMink/URLabMink.Build.cs`**

```csharp
// [URLab standard file header]

using UnrealBuildTool;
using System.IO;

public class URLabMink : ModuleRules
{
	public URLabMink(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// qpmad (vendored Goldfarb-Idnani QP solver) throws on ill-formed
		// problems; the FMinkQp wrapper catches at the boundary.
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"Eigen"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"CoreUObject",
			"Projects",
			"Json"
		});

		// MuJoCo headers + import lib from the shared third_party install.
		// URLab.Build.cs owns the drift checks and runtime staging of the
		// shared libraries; this module only consumes the installed artifacts.
		string MujocoPath = Path.Combine(PluginDirectory, "third_party", "install", "MuJoCo");
		PublicIncludePaths.Add(Path.Combine(MujocoPath, "include"));
		string LibPath = Path.Combine(MujocoPath, "lib");
		if (Directory.Exists(LibPath))
		{
			if (Target.Platform == UnrealTargetPlatform.Win64)
			{
				foreach (string LibFile in Directory.GetFiles(LibPath, "*.lib", SearchOption.AllDirectories))
				{
					PublicAdditionalLibraries.Add(LibFile);
				}
			}
			else if (Target.Platform == UnrealTargetPlatform.Linux)
			{
				foreach (string LibFile in Directory.GetFiles(LibPath, "*.so", SearchOption.AllDirectories))
				{
					PublicAdditionalLibraries.Add(LibFile);
				}
			}
		}
	}
}
```

- [ ] **Step 3: Write `Source/URLabMink/Public/MinkTypes.h`**

```cpp
// [URLab standard file header]

#pragma once

#include "CoreMinimal.h"
#include <mujoco/mujoco.h>

THIRD_PARTY_INCLUDES_START
#include "Eigen/Dense"
THIRD_PARTY_INCLUDES_END

URLABMINK_API DECLARE_LOG_CATEGORY_EXTERN(LogURLabMink, Log, All);

using FMinkVec = Eigen::VectorXd;
using FMinkMat = Eigen::MatrixXd;
using FMinkVec3 = Eigen::Vector3d;
using FMinkVec6 = Eigen::Matrix<double, 6, 1>;
using FMinkMat3 = Eigen::Matrix3d;
using FMinkMat6 = Eigen::Matrix<double, 6, 6>;
using FMinkRowMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/** mink lie/utils.py get_epsilon for float64. */
constexpr double MinkEpsilon = 1e-10;

/** mink constants.py SUPPORTED_FRAMES. */
enum class EMinkFrameType : uint8
{
	Body,
	Geom,
	Site
};

URLABMINK_API mjtObj MinkFrameTypeToObj(EMinkFrameType Type);
URLABMINK_API bool MinkFrameTypeFromString(const FString& S, EMinkFrameType& Out);

/** mink constants.py dof_width / qpos_width / constraint_width. */
URLABMINK_API int32 MinkDofWidth(int32 JointType);
URLABMINK_API int32 MinkQposWidth(int32 JointType);
URLABMINK_API int32 MinkConstraintWidth(int32 EqType);

/** mink lie/utils.py skew. */
URLABMINK_API FMinkMat3 MinkSkew(const FMinkVec3& X);
```

- [ ] **Step 4: Write `Source/URLabMink/Private/URLabMinkModule.cpp`**

```cpp
// [URLab standard file header]

#include "MinkTypes.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY(LogURLabMink);

mjtObj MinkFrameTypeToObj(EMinkFrameType Type)
{
	switch (Type)
	{
		case EMinkFrameType::Body: return mjOBJ_BODY;
		case EMinkFrameType::Geom: return mjOBJ_GEOM;
		default: return mjOBJ_SITE;
	}
}

bool MinkFrameTypeFromString(const FString& S, EMinkFrameType& Out)
{
	if (S == TEXT("body")) { Out = EMinkFrameType::Body; return true; }
	if (S == TEXT("geom")) { Out = EMinkFrameType::Geom; return true; }
	if (S == TEXT("site")) { Out = EMinkFrameType::Site; return true; }
	return false;
}

int32 MinkDofWidth(int32 JointType)
{
	switch (JointType)
	{
		case mjJNT_FREE: return 6;
		case mjJNT_BALL: return 3;
		default: return 1; // slide, hinge
	}
}

int32 MinkQposWidth(int32 JointType)
{
	switch (JointType)
	{
		case mjJNT_FREE: return 7;
		case mjJNT_BALL: return 4;
		default: return 1; // slide, hinge
	}
}

int32 MinkConstraintWidth(int32 EqType)
{
	switch (EqType)
	{
		case mjEQ_CONNECT: return 3;
		case mjEQ_WELD: return 6;
		default: return 1; // joint, tendon
	}
}

FMinkMat3 MinkSkew(const FMinkVec3& X)
{
	FMinkMat3 M;
	// clang-format off
	M <<   0.0, -X(2),  X(1),
	      X(2),   0.0, -X(0),
	     -X(1),  X(0),   0.0;
	// clang-format on
	return M;
}

IMPLEMENT_MODULE(FDefaultModuleImpl, URLabMink)
```

- [ ] **Step 5: Register the module in `UnrealRoboticsLab.uplugin`** — in the `"Modules"` array, insert BEFORE the `URLab` entry:

```json
		{
			"Name": "URLabMink",
			"Type": "Runtime",
			"LoadingPhase": "Default"
		},
```

- [ ] **Step 6: Write the failing smoke test `Source/URLabMink/Private/Tests/MinkModuleTests.cpp`**

```cpp
// [URLab standard file header]

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Modules/ModuleManager.h"
#include "MinkTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkModuleLoadsTest,
	"URLab.Mink.Module.Loads",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkModuleLoadsTest::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("URLabMink module is loaded"),
		FModuleManager::Get().IsModuleLoaded(TEXT("URLabMink")));

	// MuJoCo links and answers.
	TestTrue(TEXT("mj_versionString non-null"), mj_versionString() != nullptr);

	// Constants parity spot-checks (mink constants.py).
	TestEqual(TEXT("dof_width(free)"), MinkDofWidth(mjJNT_FREE), 6);
	TestEqual(TEXT("qpos_width(ball)"), MinkQposWidth(mjJNT_BALL), 4);
	TestEqual(TEXT("constraint_width(weld)"), MinkConstraintWidth(mjEQ_WELD), 6);

	// Skew is antisymmetric with the right entries.
	const FMinkMat3 S = MinkSkew(FMinkVec3(1.0, 2.0, 3.0));
	TestEqual(TEXT("skew(0,1)"), S(0, 1), -3.0);
	TestEqual(TEXT("skew(2,0)"), S(2, 0), -2.0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
```

- [ ] **Step 7: Build (expect the FIRST build to fail before files exist, then succeed once all files are in place)**

Run the Build command from Global Constraints. Expected: exit 0.

- [ ] **Step 8: Run the smoke test**

Run the test command with filter `URLab.Mink.Module`. Expected: `Result={Passed}` for `URLab.Mink.Module.Loads`.

- [ ] **Step 9: Format and commit**

```bash
./Scripts/format.sh
git add Source/URLabMink UnrealRoboticsLab.uplugin
git commit -m "feat(URLabMink): scaffold runtime module with MuJoCo+Eigen wiring"
```

---

### Task 2: Golden-fixture harness — Python generator, MJCF models, lie fixtures

**Files:**
- Create: `Scripts/mink_golden/requirements.txt`, `Scripts/mink_golden/README.md`, `Scripts/mink_golden/gen_golden.py`
- Create: `Scripts/mink_golden/models/arm3.xml`, `Scripts/mink_golden/models/floating.xml`
- Create (generated, committed): `Scripts/mink_golden/fixtures/lie.json`, `Scripts/mink_golden/requirements.lock`
- Modify: `.gitignore` (add `Scripts/mink_golden/.venv/`)

**Interfaces:**
- Produces: `python gen_golden.py --layers <comma-list>` writes `fixtures/<layer>.json`. Layers added in this task: `lie`. Later tasks append generator functions and layers: `configuration`, `utils`, `qp`, `task_posture`, `task_frame`, `task_relative_frame`, `task_com`, `task_damping`, `task_dof_freezing`, `task_kinetic_energy`, `task_equality`, `limit_configuration`, `limit_velocity`, `limit_collision`, `solve_ik`.
- Fixture JSON schema for `lie.json` (all numbers are exact-round-trip doubles):

```json
{
  "so3": {
    "exp":      [{"tangent": [3], "wxyz": [4]}],
    "log":      [{"wxyz": [4], "tangent": [3]}],
    "matrix":   [{"wxyz": [4], "matrix": [[3x3]], "wxyz_back": [4]}],
    "multiply": [{"a": [4], "b": [4], "out": [4]}],
    "apply":    [{"wxyz": [4], "v": [3], "out": [3]}],
    "inverse":  [{"wxyz": [4], "out": [4]}],
    "ljac":     [{"tangent": [3], "m": [[3x3]]}],
    "ljacinv":  [{"tangent": [3], "m": [[3x3]]}],
    "rpy":      [{"wxyz": [4], "rpy": [3]}],
    "rminus":   [{"a": [4], "b": [4], "out": [3]}],
    "interpolate": [{"a": [4], "b": [4], "alpha": 0.25, "out": [4]}]
  },
  "se3": {
    "exp":      [{"tangent": [6], "wxyz_xyz": [7]}],
    "log":      [{"wxyz_xyz": [7], "tangent": [6]}],
    "multiply": [{"a": [7], "b": [7], "out": [7]}],
    "inverse":  [{"wxyz_xyz": [7], "out": [7]}],
    "apply":    [{"wxyz_xyz": [7], "v": [3], "out": [3]}],
    "adjoint":  [{"wxyz_xyz": [7], "m": [[6x6]]}],
    "ljac":     [{"tangent": [6], "m": [[6x6]]}],
    "ljacinv":  [{"tangent": [6], "m": [[6x6]]}],
    "jlog":     [{"wxyz_xyz": [7], "m": [[6x6]]}],
    "rminus":   [{"a": [7], "b": [7], "out": [6]}]
  }
}
```

- [ ] **Step 1: Write `Scripts/mink_golden/requirements.txt`**

```
# [URLab standard file header]
mujoco==3.9.0
mink==1.2.0
quadprog
qpsolvers>=4.12.0
numpy
```

- [ ] **Step 2: Write `Scripts/mink_golden/models/arm3.xml`**

```xml
<mujoco model="arm3">
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="link1" pos="0 0 0.1">
      <joint name="j1" type="hinge" axis="0 0 1" range="-2.9 2.9" limited="true"/>
      <geom name="g1" type="capsule" fromto="0 0 0 0.3 0 0" size="0.04"/>
      <body name="link2" pos="0.3 0 0">
        <joint name="j2" type="hinge" axis="0 1 0" range="-1.9 1.9" limited="true"/>
        <geom name="g2" type="capsule" fromto="0 0 0 0.28 0 0" size="0.035"/>
        <body name="link3" pos="0.28 0 0">
          <joint name="j3" type="hinge" axis="0 1 0" range="-2.4 2.4" limited="true"/>
          <geom name="g3" type="capsule" fromto="0 0 0 0.2 0 0" size="0.03"/>
          <site name="ee" pos="0.2 0 0" size="0.01"/>
        </body>
      </body>
    </body>
  </worldbody>
  <keyframe>
    <key name="home" qpos="0 0.4 -0.8"/>
  </keyframe>
</mujoco>
```

- [ ] **Step 3: Write `Scripts/mink_golden/models/floating.xml`** (free joint, ball joint, hinge, mocap body, site — exercises every joint width)

```xml
<mujoco model="floating">
  <compiler angle="radian"/>
  <worldbody>
    <body name="mocap_target" mocap="true" pos="0.5 0 1">
      <geom name="gm" type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="base" pos="0 0 1">
      <freejoint name="root"/>
      <geom name="gb" type="box" size="0.1 0.1 0.05"/>
      <body name="head" pos="0 0 0.2">
        <joint name="neck" type="ball" range="0 1.0" limited="true"/>
        <geom name="gh" type="sphere" size="0.06"/>
        <body name="antenna" pos="0 0 0.1">
          <joint name="ant" type="hinge" axis="1 0 0" range="-1 1" limited="true"/>
          <geom name="ga" type="capsule" fromto="0 0 0 0 0 0.15" size="0.01"/>
          <site name="tip" pos="0 0 0.15" size="0.005"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
```

- [ ] **Step 4: Write `Scripts/mink_golden/gen_golden.py`** (framework + lie layer; later tasks append `gen_<layer>` functions and register them in `LAYERS`)

```python
# [URLab standard file header]
"""Golden-fixture generator: runs pinned Python mink and dumps JSON fixtures
that the URLabMink C++ automation tests replay. All inputs are stored in the
fixture; randomness only ever runs here, seeded."""

import argparse
import json
import pathlib

import mink
import mujoco
import numpy as np
from mink.lie import SE3, SO3

HERE = pathlib.Path(__file__).parent
FIXTURES = HERE / "fixtures"
MODELS = HERE / "models"


def load_model(name: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MODELS / f"{name}.xml"))


def j(x):
    """numpy -> plain python for exact-round-trip json."""
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


def sample_tangents(rng, dim):
    """Deterministic tangent set incl. edge cases (tiny angle, near pi)."""
    out = [np.zeros(dim)]
    for scale in (1e-12, 1e-7, 1e-3, 0.5, 1.5):
        out.append(scale * rng.standard_normal(dim))
    near_pi = rng.standard_normal(dim)
    near_pi *= (np.pi - 1e-6) / np.linalg.norm(near_pi[-3:] if dim == 6 else near_pi)
    out.append(near_pi)
    return out


def gen_lie():
    rng = np.random.default_rng(20260702)
    so3_tangents = sample_tangents(rng, 3)
    se3_tangents = sample_tangents(rng, 6)
    so3s = [SO3.exp(t) for t in so3_tangents] + [
        SO3(wxyz=q / np.linalg.norm(q))
        for q in [rng.standard_normal(4) for _ in range(4)]
    ]
    se3s = [SE3.exp(t) for t in se3_tangents] + [
        SE3.from_rotation_and_translation(r, rng.standard_normal(3)) for r in so3s[:4]
    ]
    v3s = [rng.standard_normal(3) for _ in range(3)]

    so3 = {
        "exp": [{"tangent": j(t), "wxyz": j(SO3.exp(t).wxyz)} for t in so3_tangents],
        "log": [{"wxyz": j(g.wxyz), "tangent": j(g.log())} for g in so3s],
        "matrix": [
            {"wxyz": j(g.wxyz), "matrix": j(g.as_matrix()),
             "wxyz_back": j(SO3.from_matrix(g.as_matrix()).wxyz)}
            for g in so3s
        ],
        "multiply": [
            {"a": j(a.wxyz), "b": j(b.wxyz), "out": j((a @ b).wxyz)}
            for a, b in zip(so3s, so3s[1:])
        ],
        "apply": [
            {"wxyz": j(g.wxyz), "v": j(v), "out": j(g.apply(v))}
            for g, v in zip(so3s, v3s * 3)
        ],
        "inverse": [{"wxyz": j(g.wxyz), "out": j(g.inverse().wxyz)} for g in so3s],
        "ljac": [{"tangent": j(t), "m": j(SO3.ljac(t))} for t in so3_tangents],
        "ljacinv": [{"tangent": j(t), "m": j(SO3.ljacinv(t))} for t in so3_tangents],
        "rpy": [{"wxyz": j(g.wxyz), "rpy": j(np.array(g.as_rpy_radians()))} for g in so3s],
        "rminus": [
            {"a": j(a.wxyz), "b": j(b.wxyz), "out": j(a.rminus(b))}
            for a, b in zip(so3s, so3s[1:])
        ],
        "interpolate": [
            {"a": j(a.wxyz), "b": j(b.wxyz), "alpha": 0.25,
             "out": j(a.interpolate(b, 0.25).wxyz)}
            for a, b in zip(so3s, so3s[1:])
        ],
    }
    se3 = {
        "exp": [{"tangent": j(t), "wxyz_xyz": j(SE3.exp(t).wxyz_xyz)} for t in se3_tangents],
        "log": [{"wxyz_xyz": j(g.wxyz_xyz), "tangent": j(g.log())} for g in se3s],
        "multiply": [
            {"a": j(a.wxyz_xyz), "b": j(b.wxyz_xyz), "out": j((a @ b).wxyz_xyz)}
            for a, b in zip(se3s, se3s[1:])
        ],
        "inverse": [{"wxyz_xyz": j(g.wxyz_xyz), "out": j(g.inverse().wxyz_xyz)} for g in se3s],
        "apply": [
            {"wxyz_xyz": j(g.wxyz_xyz), "v": j(v), "out": j(g.apply(v))}
            for g, v in zip(se3s, v3s * 4)
        ],
        "adjoint": [{"wxyz_xyz": j(g.wxyz_xyz), "m": j(g.adjoint())} for g in se3s],
        "ljac": [{"tangent": j(t), "m": j(SE3.ljac(t))} for t in se3_tangents],
        "ljacinv": [{"tangent": j(t), "m": j(SE3.ljacinv(t))} for t in se3_tangents],
        "jlog": [{"wxyz_xyz": j(g.wxyz_xyz), "m": j(g.jlog())} for g in se3s],
        "rminus": [
            {"a": j(a.wxyz_xyz), "b": j(b.wxyz_xyz), "out": j(a.rminus(b))}
            for a, b in zip(se3s, se3s[1:])
        ],
    }
    return {"so3": so3, "se3": se3}


LAYERS = {
    "lie": gen_lie,
    # Later tasks register: configuration, utils, qp, task_*, limit_*, solve_ik
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--layers", default=",".join(LAYERS))
    args = parser.parse_args()
    FIXTURES.mkdir(exist_ok=True)
    for layer in args.layers.split(","):
        data = LAYERS[layer]()
        path = FIXTURES / f"{layer}.json"
        path.write_text(json.dumps(data))
        print(f"wrote {path} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Write `Scripts/mink_golden/README.md`** — brief: purpose, venv setup commands (below), regeneration command, pin policy (mink v1.2.0; regenerating with a different version invalidates parity), and that `fixtures/` and `requirements.lock` are committed while `.venv/` is not.

- [ ] **Step 6: Create the venv, install pins, generate, freeze**

```bash
cd "Scripts/mink_golden"
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip freeze > requirements.lock
python gen_golden.py --layers lie
python -c "import mink, json, pathlib; d=json.loads(pathlib.Path('fixtures/lie.json').read_text()); print('so3 exp cases:', len(d['so3']['exp']))"
deactivate
```
Expected: `wrote .../lie.json`, `so3 exp cases: 7`.

- [ ] **Step 7: Add `.gitignore` entry** — append line `Scripts/mink_golden/.venv/` to the repo `.gitignore`.

- [ ] **Step 8: Commit**

```bash
git add Scripts/mink_golden .gitignore
git commit -m "test(URLabMink): golden-fixture generator + MJCF models + lie fixtures (mink v1.2.0 pinned)"
```

---

### Task 3: `FMinkSO3` — SO3 port with golden parity tests

**Files:**
- Create: `Source/URLabMink/Public/Lie/MinkSO3.h`, `Source/URLabMink/Private/Lie/MinkSO3.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkTestUtils.h`, `Source/URLabMink/Private/Tests/MinkTestUtils.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkLieTests.cpp`

**Interfaces:**
- Consumes: `MinkTypes.h`.
- Produces `FMinkSO3` (POD-ish struct, parameters `(w,x,y,z)`):

```cpp
struct URLABMINK_API FMinkSO3
{
	double Wxyz[4] = {1.0, 0.0, 0.0, 0.0};

	static FMinkSO3 Identity();
	static FMinkSO3 FromWxyz(const double* InWxyz);
	static FMinkSO3 FromMatrix(const FMinkMat3& M);
	static FMinkSO3 FromXRadians(double Theta);
	static FMinkSO3 FromYRadians(double Theta);
	static FMinkSO3 FromZRadians(double Theta);
	static FMinkSO3 FromRpyRadians(double Roll, double Pitch, double Yaw);
	static FMinkSO3 Exp(const FMinkVec3& Tangent);
	static FMinkSO3 SampleUniform(FRandomStream& Rng);

	FMinkMat3 AsMatrix() const;
	FMinkVec3 Log() const;
	FMinkMat3 Adjoint() const;      // == AsMatrix()
	FMinkSO3 Inverse() const;
	FMinkSO3 Normalize() const;
	FMinkSO3 Multiply(const FMinkSO3& Other) const;
	FMinkVec3 Apply(const FMinkVec3& Target) const;
	double ComputeRollRadians() const;
	double ComputePitchRadians() const;
	double ComputeYawRadians() const;
	FMinkVec3 AsRpyRadians() const; // (roll, pitch, yaw)

	static FMinkMat3 Ljac(const FMinkVec3& Other);
	static FMinkMat3 Ljacinv(const FMinkVec3& Other);
	static FMinkMat3 Rjac(const FMinkVec3& Other);    // Ljac(-Other)
	static FMinkMat3 Rjacinv(const FMinkVec3& Other); // Ljacinv(-Other)

	FMinkSO3 RPlus(const FMinkVec3& Other) const;  // this * Exp(Other)
	FMinkVec3 RMinus(const FMinkSO3& Other) const; // (Other^-1 * this).Log()
	FMinkSO3 Interpolate(const FMinkSO3& Other, double Alpha) const;
	FMinkSO3 Clamp(const FMinkVec3& RpyLower, const FMinkVec3& RpyUpper) const;
};
```
- Produces test helpers (`MinkTestUtils.h`), used by every later test file:

```cpp
FString MinkGoldenDir();                       // <plugin>/Scripts/mink_golden
bool MinkLoadFixture(const FString& LayerName, TSharedPtr<FJsonObject>& Out);
mjModel* MinkLoadModel(const FString& ModelName);      // mj_loadXML from models/
FMinkVec MinkJsonVec(const TArray<TSharedPtr<FJsonValue>>& A);
FMinkMat MinkJsonMat(const TArray<TSharedPtr<FJsonValue>>& A); // array-of-row-arrays
bool MinkExpectNear(FAutomationTestBase& Test, const TCHAR* What,
	const FMinkMat& Actual, const FMinkMat& Expected, double Tol); // rel-or-abs rule from Global Constraints
```

**Implementation notes (translate mink `lie/so3.py` line-for-line; call the SAME mju functions):**
- `FromMatrix`: `mju_mat2Quat(Wxyz, RowMajorCopyOf(M).data())` (copy `M` into a row-major `double[9]` first — Eigen default is col-major).
- `AsMatrix`: `mju_quat2Mat(m9, Wxyz)` then map row-major `m9` into `FMinkMat3`.
- `Exp`: copy tangent to `axis[3]`; `theta = mju_normalize3(axis)`; `mju_axisAngle2Quat(Wxyz, axis, theta)`.
- `Log`: copy quat; multiply by `double Sign = (q[0] > 0.0) - (q[0] < 0.0)` — NOTE Python `np.sign(0.0) == 0.0`, replicate exactly (w==0 zeroes the quat and returns zeros via the norm branch); `norm = mju_normalize3(v)`; `if (norm < MinkEpsilon) return zeros; return 2*atan2(norm, w)*v`.
- `Inverse`: `mju_negQuat`. `Normalize`: `mju_normalize4`. `Multiply`: `mju_mulQuat`. `Apply`: `mju_rotVecQuat`.
- `Ljac`/`Ljacinv`: transcribe the Taylor/exact `alpha`/`beta` branches verbatim, including the outer-product trick (`ljac = beta*(other otherᵀ - (other·other) I) + alpha*skew(other) + I`; for `Ljacinv` the `-0.5*skew` variant). Threshold: `theta < MinkEpsilon`.
- RPY formulas: direct arithmetic transcription.
- `Clamp`: `FromRpyRadians(clip(roll), clip(pitch), clip(yaw))`.
- `MinkGoldenDir()`: `IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"))->GetBaseDir() + "/Scripts/mink_golden"`.
- `MinkExpectNear`: flattened max-abs compare with the rule `diff <= Tol * max(1.0, maxAbs(Expected))`; on failure `Test.AddError` printing What, max diff, and first offending index.

- [ ] **Step 1: Write the failing test `MinkLieTests.cpp` (SO3 section)** — one automation test `URLab.Mink.Lie.SO3` that loads `lie.json` and loops every `so3` sub-array, asserting with `TOL_LIE`:

```cpp
// [URLab standard file header]

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Lie/MinkSO3.h"
#include "MinkTestUtils.h"

#if WITH_DEV_AUTOMATION_TESTS

static const double TOL_LIE = 1e-10;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkLieSO3Test,
	"URLab.Mink.Lie.SO3",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkLieSO3Test::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("lie"), Root)) { AddError(TEXT("lie.json missing — run gen_golden.py")); return false; }
	const TSharedPtr<FJsonObject> So3 = Root->GetObjectField(TEXT("so3"));

	for (const auto& V : So3->GetArrayField(TEXT("exp")))
	{
		const auto C = V->AsObject();
		const FMinkVec T = MinkJsonVec(C->GetArrayField(TEXT("tangent")));
		const FMinkSO3 G = FMinkSO3::Exp(T.head<3>());
		MinkExpectNear(*this, TEXT("so3.exp"),
			Eigen::Map<const Eigen::Vector4d>(G.Wxyz), MinkJsonVec(C->GetArrayField(TEXT("wxyz"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("log")))
	{
		const auto C = V->AsObject();
		const FMinkVec Q = MinkJsonVec(C->GetArrayField(TEXT("wxyz")));
		MinkExpectNear(*this, TEXT("so3.log"),
			FMinkSO3::FromWxyz(Q.data()).Log(), MinkJsonVec(C->GetArrayField(TEXT("tangent"))), TOL_LIE);
	}
	// ... identical loops for matrix (AsMatrix + FromMatrix round-trip), multiply,
	// apply, inverse, ljac, ljacinv, rpy, rminus, interpolate — each comparing the
	// corresponding FMinkSO3 method against the fixture field with TOL_LIE.
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
```
(Write ALL the listed loops — the `...` above marks repetition of the same three-line pattern per fixture key, not an omission of logic. Every fixture key in the Task 2 schema gets a loop.)

- [ ] **Step 2: Write `MinkTestUtils.h/.cpp`** implementing the helper signatures above (JSON via `FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(FileContents), RootValue)`; `MinkLoadModel` via `mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Err, sizeof(Err))` with `AddError` on null).

- [ ] **Step 3: Build and run — verify the test FAILS to compile / link (FMinkSO3 absent), i.e. red state.**

- [ ] **Step 4: Implement `MinkSO3.h/.cpp` per the interface + implementation notes.**

- [ ] **Step 5: Build; run test filter `URLab.Mink.Lie`. Expected: `URLab.Mink.Lie.SO3 ... Result={Passed}`.**

- [ ] **Step 6: Format and commit**

```bash
./Scripts/format.sh
git add Source/URLabMink
git commit -m "feat(URLabMink): FMinkSO3 with golden parity vs python mink"
```

---

### Task 4: `FMinkSE3` — SE3 port with golden parity tests

**Files:**
- Create: `Source/URLabMink/Public/Lie/MinkSE3.h`, `Source/URLabMink/Private/Lie/MinkSE3.cpp`
- Modify: `Source/URLabMink/Private/Tests/MinkLieTests.cpp` (add `URLab.Mink.Lie.SE3`)

**Interfaces:**
- Consumes: `FMinkSO3`, `MinkTypes.h`.
- Produces `FMinkSE3` (parameters `(qw,qx,qy,qz, x,y,z)`; tangent `(vx,vy,vz, wx,wy,wz)` — translation FIRST, matching mink):

```cpp
struct URLABMINK_API FMinkSE3
{
	double WxyzXyz[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	static FMinkSE3 Identity();
	static FMinkSE3 FromWxyzXyz(const double* In);
	static FMinkSE3 FromRotationAndTranslation(const FMinkSO3& Rotation, const FMinkVec3& Translation);
	static FMinkSE3 FromRotation(const FMinkSO3& Rotation);
	static FMinkSE3 FromTranslation(const FMinkVec3& Translation);
	static FMinkSE3 FromMatrix(const Eigen::Matrix4d& M);
	static FMinkSE3 FromMocapId(const mjData* Data, int32 MocapId);
	static FMinkSE3 Exp(const FMinkVec6& Tangent);
	static FMinkSE3 SampleUniform(FRandomStream& Rng);

	FMinkSO3 Rotation() const;
	FMinkVec3 Translation() const;
	Eigen::Matrix4d AsMatrix() const;
	FMinkVec6 Log() const;
	FMinkMat6 Adjoint() const;
	FMinkSE3 Inverse() const;
	FMinkSE3 Normalize() const;
	FMinkSE3 Multiply(const FMinkSE3& Other) const;
	FMinkVec3 Apply(const FMinkVec3& Target) const;

	static FMinkMat6 Ljac(const FMinkVec6& Other);
	static FMinkMat6 Ljacinv(const FMinkVec6& Other);
	static FMinkMat6 Rjac(const FMinkVec6& Other);
	static FMinkMat6 Rjacinv(const FMinkVec6& Other);
	FMinkMat6 Jlog() const; // Rjacinv(Log())

	FMinkSE3 RPlus(const FMinkVec6& Other) const;
	FMinkVec6 RMinus(const FMinkSE3& Other) const;
	FMinkSE3 Interpolate(const FMinkSE3& Other, double Alpha) const;
};
```

**Implementation notes (transcribe `lie/se3.py`):**
- `Exp`: `Rotation = SO3::Exp(tangent.tail<3>())`; `theta = mju_norm3(omega)`; if `theta*theta < MinkEpsilon` → `VMat = Rotation.AsMatrix()`, else the exact `(1-cos)/t2`, `(t-sin)/t3` V-matrix; translation `= VMat * tangent.head<3>()`.
- `Inverse`: `mju_negQuat` + `mju_rotVecQuat(out+4, -xyz, invQuat)`.
- `Multiply`: `mju_mulQuat` for quats; `mju_rotVecQuat(t, other.xyz, this.quat)` then `+= this.xyz`.
- `Log`: `omega = Rotation().Log()`; small/large branches for `VinvMat` exactly as Python (`0.5*theta*cos(half)/sin(half)` branch); `tangent << VinvMat * Translation(), omega`.
- `Adjoint`: blocks `[[R, skew(t)R],[0, R]]` (translation-first convention).
- `Ljac`/`Ljacinv`: block layout exactly as Python (`[:3,:3]=so3 ljac; [:3,3:]=Q or -Linv Q Linv; [3:,3:]=so3 term`) using the `_getQ` helper — transcribe `_getQ` (A,B,C,D coefficients incl. Taylor branches) into a file-static `static FMinkMat3 GetQ(const FMinkVec6& C)`.
- Guard both Ljac/Ljacinv with the `theta_squared < MinkEpsilon → Identity` early-out, matching Python.
- `FromMocapId`: quat from `Data->mocap_quat + 4*MocapId`, pos from `Data->mocap_pos + 3*MocapId`.

- [ ] **Step 1: Extend `MinkLieTests.cpp` with `URLab.Mink.Lie.SE3`** — same fixture-loop pattern as Task 3 Step 1, one loop per `se3` fixture key (`exp, log, multiply, inverse, apply, adjoint, ljac, ljacinv, jlog, rminus`), all at `TOL_LIE`.
- [ ] **Step 2: Build → confirm red (compile fails on missing FMinkSE3).**
- [ ] **Step 3: Implement `MinkSE3.h/.cpp`.**
- [ ] **Step 4: Build; run `URLab.Mink.Lie`. Expected: both SO3 and SE3 `Result={Passed}`.**
- [ ] **Step 5: Format + commit** — `git add Source/URLabMink && git commit -m "feat(URLabMink): FMinkSE3 with golden parity vs python mink"`.

---

### Task 5: `FMinkConfiguration` + `MinkUtils` — kinematics with golden parity

**Files:**
- Create: `Source/URLabMink/Public/MinkConfiguration.h`, `Source/URLabMink/Private/MinkConfiguration.cpp`
- Create: `Source/URLabMink/Public/MinkUtils.h`, `Source/URLabMink/Private/MinkUtils.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkConfigurationTests.cpp`
- Modify: `Scripts/mink_golden/gen_golden.py` (add `configuration` and `utils` layers)

**Interfaces:**
- Consumes: `FMinkSE3`, `FMinkSO3`, `MinkTypes.h`.
- Produces:

```cpp
class URLABMINK_API FMinkConfiguration
{
public:
	// Owns its own mjData (mj_makeData), like Python Configuration. Model not owned.
	explicit FMinkConfiguration(const mjModel* InModel, const double* Q = nullptr);
	~FMinkConfiguration();
	FMinkConfiguration(const FMinkConfiguration&) = delete;
	FMinkConfiguration& operator=(const FMinkConfiguration&) = delete;

	void Update(const double* Q = nullptr); // mj_kinematics + mj_comPos + (neq>0) mj_makeConstraint
	bool UpdateFromKeyframe(const FString& KeyName); // false + error log if unknown
	bool CheckLimits(double Tol = 1e-6, bool bSafetyBreak = true) const; // false iff violated AND bSafetyBreak

	bool GetFrameJacobian(const FString& FrameName, EMinkFrameType FrameType, FMinkMat& OutJac /*6 x nv*/) const;
	bool GetTransformFrameToWorld(const FString& FrameName, EMinkFrameType FrameType, FMinkSE3& Out) const;
	bool GetTransform(const FString& SourceName, EMinkFrameType SourceType,
		const FString& DestName, EMinkFrameType DestType, FMinkSE3& Out) const;

	FMinkVec Integrate(const FMinkVec& Velocity, double Dt) const;
	void IntegrateInplace(const FMinkVec& Velocity, double Dt);
	FMinkMat GetInertiaMatrix() const;

	FMinkVec GetQ() const;
	int32 Nv() const { return Model->nv; }
	int32 Nq() const { return Model->nq; }

	const mjModel* Model;
	mjData* Data;
	FMinkMat EyeNv; // cached identity, mirrors configuration._eye_nv

private:
	int32 ResolveFrameId(const FString& FrameName, EMinkFrameType FrameType) const; // -1 invalid; cached
	mutable TMap<TPair<FString, uint8>, int32> FrameIdCache;
	// Precomputed limited-joint arrays (jnt ids, qpos adr, ranges), mirrors __init__.
	TArray<int32> LimitedJntIds;
	TArray<int32> LimitedQposAdr;
	TArray<double> LimitedLower, LimitedUpper;
};

// MinkUtils.h — mink utils.py ports:
URLABMINK_API bool MinkMoveMocapToFrame(const mjModel* M, mjData* D,
	const FString& MocapName, const FString& FrameName, EMinkFrameType FrameType);
URLABMINK_API void MinkGetFreejointDims(const mjModel* M, TArray<int32>& OutQIds, TArray<int32>& OutVIds);
URLABMINK_API bool MinkCustomConfigurationVector(const mjModel* M, const FString& KeyName /*empty = qpos0*/,
	const TArray<TPair<FString, FMinkVec>>& JointValues, FMinkVec& OutQ);
URLABMINK_API TArray<int32> MinkGetBodyBodyIds(const mjModel* M, int32 BodyId);
URLABMINK_API TArray<int32> MinkGetSubtreeBodyIds(const mjModel* M, int32 BodyId);
URLABMINK_API TArray<int32> MinkGetBodyGeomIds(const mjModel* M, int32 BodyId);
URLABMINK_API TArray<int32> MinkGetBodyJointIds(const mjModel* M, int32 BodyId);
URLABMINK_API TArray<int32> MinkGetSubtreeGeomIds(const mjModel* M, int32 BodyId);
URLABMINK_API TArray<int32> MinkGetSubtreeJointIds(const mjModel* M, int32 BodyId);
```

**Implementation notes:**
- `GetFrameJacobian` — the parity-critical function; transcribe exactly:

```cpp
FMinkRowMat Jac(6, Model->nv);
switch (FrameType)
{
	case EMinkFrameType::Body: mj_jacBody(Model, Data, Jac.data(), Jac.data() + 3 * Model->nv, Id); break;
	case EMinkFrameType::Geom: mj_jacGeom(Model, Data, Jac.data(), Jac.data() + 3 * Model->nv, Id); break;
	case EMinkFrameType::Site: mj_jacSite(Model, Data, Jac.data(), Jac.data() + 3 * Model->nv, Id); break;
}
// Left-multiply by A[T_fw] = blockdiag(R_fw, R_fw) where R_fw = R_wf^T.
const double* Xmat = XmatPtr(FrameType, Id); // xmat / geom_xmat / site_xmat, row-major 9
Eigen::Map<const FMinkRowMat> RWf(Xmat, 3, 3);
OutJac.resize(6, Model->nv);
OutJac.topRows(3) = RWf.transpose() * Jac.topRows(3);
OutJac.bottomRows(3) = RWf.transpose() * Jac.bottomRows(3);
```
- Pose accessors: `xpos/geom_xpos/site_xpos` (+3*id) and `xmat/...` (+9*id); build `FMinkSE3::FromRotationAndTranslation(FMinkSO3::FromMatrix(...), pos)`. `GetTransform` = `Dest.Inverse().Multiply(Source)`.
- `Integrate`: copy qpos; `mj_integratePos(Model, q.data(), Velocity.data(), Dt)`.
- `GetInertiaMatrix`: `mj_makeM(Model, Data)`, then `mju_sym2dense(Out.data(), Data->M, Model->M_rownnz, Model->M_rowadr, Model->M_colind, Model->nv)` into a row-major buffer mapped back (M is symmetric so major-order is moot).
- `CheckLimits`: vectorized loop over the precomputed limited arrays; violation ⇒ if `bSafetyBreak` log Error with the Python message text (`"Joint %d (%s) violates configuration limits %g <= %g <= %g"`) and return false; else per-violation Verbose log, return true.
- `UpdateFromKeyframe`: `mj_name2id(mjOBJ_KEY)`; copy `Model->key_qpos + Id*Model->nq` and `Update`.
- Utils: direct transcriptions (stack-based subtree walk identical to Python so ordering matches fixtures).

- [ ] **Step 1: Extend `gen_golden.py`** — add and register:

```python
def _frame_cases(model, data, names_types):
    out = []
    for name, ftype in names_types:
        cfg_like = {}  # via mink API below
    return out  # (see full function)


def gen_configuration():
    rng = np.random.default_rng(20260703)
    result = {"models": {}}
    frames = {
        "arm3": [("ee", "site"), ("link2", "body"), ("g3", "geom")],
        "floating": [("tip", "site"), ("head", "body"), ("gb", "geom")],
    }
    for model_name, frame_list in frames.items():
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for i in range(4):
            q = model.qpos0.copy() + 0.3 * rng.standard_normal(model.nq)
            # Normalize free/ball quats so q is a valid configuration.
            for jnt in range(model.njnt):
                t, adr = model.jnt_type[jnt], model.jnt_qposadr[jnt]
                if t == mujoco.mjtJoint.mjJNT_FREE:
                    q[adr + 3 : adr + 7] /= np.linalg.norm(q[adr + 3 : adr + 7])
                elif t == mujoco.mjtJoint.mjJNT_BALL:
                    q[adr : adr + 4] /= np.linalg.norm(q[adr : adr + 4])
            cfg.update(q=q)
            v = rng.standard_normal(model.nv)
            case = {
                "q": j(q),
                "frames": [
                    {"name": n, "type": t,
                     "jac": j(cfg.get_frame_jacobian(n, t)),
                     "pose": j(cfg.get_transform_frame_to_world(n, t).wxyz_xyz)}
                    for n, t in frame_list
                ],
                "transform": {
                    "src": frame_list[0][0], "src_type": frame_list[0][1],
                    "dst": frame_list[1][0], "dst_type": frame_list[1][1],
                    "pose": j(cfg.get_transform(frame_list[0][0], frame_list[0][1],
                                                frame_list[1][0], frame_list[1][1]).wxyz_xyz)},
                "integrate": {"v": j(v), "dt": 0.02, "q_out": j(cfg.integrate(v, 0.02))},
                "inertia": j(cfg.get_inertia_matrix()),
            }
            cases.append(case)
        entry = {"cases": cases}
        if model.nkey:
            cfg.update_from_keyframe("home")
            entry["keyframe"] = {"name": "home", "q": j(cfg.q)}
        result["models"][model_name] = entry
    return result


def gen_utils():
    out = {"models": {}}
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        q_ids, v_ids = mink.get_freejoint_dims(model)
        entry = {
            "freejoint_q_ids": q_ids, "freejoint_v_ids": v_ids,
            "subtree": [
                {"body": model.body(bid).name,
                 "body_ids": mink.get_subtree_body_ids(model, bid),
                 "geom_ids": mink.get_subtree_geom_ids(model, bid),
                 "joint_ids": mink.get_subtree_joint_ids(model, bid)}
                for bid in range(model.nbody)
            ],
        }
        if model_name == "floating":
            data = mujoco.MjData(model)
            mujoco.mj_kinematics(model, data)
            mink.move_mocap_to_frame(model, data, "mocap_target", "tip", "site")
            entry["move_mocap"] = {"mocap": "mocap_target", "frame": "tip", "type": "site",
                                   "pos": j(data.mocap_pos[0]), "quat": j(data.mocap_quat[0])}
        out["models"][model_name] = entry
    return out
```
Register both in `LAYERS`, regenerate (`python gen_golden.py --layers configuration,utils` inside the venv), and sanity-print one case.

- [ ] **Step 2: Write failing tests `MinkConfigurationTests.cpp`** — `URLab.Mink.Configuration.Kinematics` (loads each model via `MinkLoadModel`, constructs `FMinkConfiguration`, replays every case: jacobians `TOL_KIN`, poses `TOL_KIN`, integrate `TOL_KIN`, inertia `TOL_KIN`, keyframe q exact-ish `TOL_KIN`) and `URLab.Mink.Utils.Helpers` (freejoint dims, subtree id arrays compared exactly as int arrays, move_mocap pos/quat `TOL_KIN`). Free the model with `mj_deleteModel` at test end.
- [ ] **Step 3: Build → red.** 
- [ ] **Step 4: Implement `MinkConfiguration.cpp` + `MinkUtils.cpp`.**
- [ ] **Step 5: Build; run `URLab.Mink`. Expected: all tests pass.**
- [ ] **Step 6: Format + commit** — `git add Source/URLabMink Scripts/mink_golden && git commit -m "feat(URLabMink): FMinkConfiguration + utils with golden parity"`.

---

### Task 6: QP backend — vendor qpmad 1.4.0 + `FMinkQp` wrapper

**Files:**
- Create: `Source/URLabMink/Private/ThirdParty/qpmad/` (vendored headers)
- Create: `Source/URLabMink/Public/MinkQp.h`, `Source/URLabMink/Private/MinkQp.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkQpTests.cpp`
- Modify: `ThirdPartyNotices.txt`, `Scripts/mink_golden/gen_golden.py` (add `qp` layer)

**Interfaces:**
- Produces:

```cpp
// Mirrors qpsolvers.Problem(H, c, G, h, A, b):
//   min ½ xᵀH x + cᵀx   s.t.  G x ≤ h,  A x = b
struct URLABMINK_API FMinkQpProblem
{
	FMinkMat H;
	FMinkVec C;
	TOptional<FMinkMat> G;
	TOptional<FMinkVec> HIneq;
	TOptional<FMinkMat> A;
	TOptional<FMinkVec> B;
};

// Returns false (+ Warning log) when the solver reports no solution.
URLABMINK_API bool MinkSolveQp(const FMinkQpProblem& Problem, FMinkVec& OutX);
```

- [ ] **Step 1: Vendor qpmad**

```bash
cd /tmp/claude-1000/-home-stuart/f1f825fb-0529-4fc7-9e1b-6f6ba700bbed/scratchpad
git clone --depth 1 --branch 1.4.0 https://github.com/asherikov/qpmad
grep -m1 "Apache License" qpmad/LICENSE   # expected: "Apache License" (verify!)
mkdir -p "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Source/URLabMink/Private/ThirdParty"
cp -r qpmad/include/qpmad "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Source/URLabMink/Private/ThirdParty/qpmad"
```
Also copy `qpmad/LICENSE` to `Source/URLabMink/Private/ThirdParty/qpmad/LICENSE`, and write `Source/URLabMink/Private/ThirdParty/qpmad/VENDORED.md` recording: upstream URL, tag `1.4.0`, commit `b2fd8d57d973cab5d74decc1bbfc3c622d111240`, date vendored, and "no local modifications" (or list them if any become necessary).

- [ ] **Step 2: Append to `ThirdPartyNotices.txt`** (match the file's existing entry format): qpmad — Copyright Alexander Sherikov, Apache License 2.0, https://github.com/asherikov/qpmad, vendored at tag 1.4.0 under `Source/URLabMink/Private/ThirdParty/qpmad/`.

- [ ] **Step 3: Extend `gen_golden.py` with the `qp` layer** — random strictly-convex QPs solved by the SAME backend family mink uses on the Python side (Goldfarb–Idnani via `quadprog`):

```python
def gen_qp():
    import qpsolvers
    rng = np.random.default_rng(20260704)
    cases = []
    for n, n_ineq, n_eq in [(3, 0, 0), (5, 4, 0), (6, 8, 2), (9, 12, 3), (9, 0, 2)]:
        R = rng.standard_normal((n, n))
        H = R.T @ R + 1e-6 * np.eye(n)
        c = rng.standard_normal(n)
        G = rng.standard_normal((n_ineq, n)) if n_ineq else None
        h = (np.abs(rng.standard_normal(n_ineq)) + 0.1) if n_ineq else None
        A = rng.standard_normal((n_eq, n)) if n_eq else None
        b = 0.1 * rng.standard_normal(n_eq) if n_eq else None
        x = qpsolvers.solve_qp(H, c, G, h, A, b, solver="quadprog")
        assert x is not None
        cases.append({"H": j(H), "c": j(c), "G": j(G) if G is not None else None,
                      "h": j(h) if h is not None else None,
                      "A": j(A) if A is not None else None,
                      "b": j(b) if b is not None else None, "x": j(x)})
    return {"cases": cases}
```
Regenerate: `python gen_golden.py --layers qp`.

- [ ] **Step 4: Write failing test `MinkQpTests.cpp`** — `URLab.Mink.Qp.Golden`: replay every case through `MinkSolveQp`, compare `x` at `TOL_QP`. Plus `URLab.Mink.Qp.Infeasible`: `H=I₂, c=0, G=[[1,0],[-1,0]], h=[-1,-1]` (x₀≤-1 ∧ x₀≥1 — infeasible) must return false without crashing.

- [ ] **Step 5: Implement `MinkQp.cpp`** — map to qpmad's general-constraint form (stack equalities as `lb == ub` rows on top of inequalities as `(-∞, h]` rows):

```cpp
// [URLab standard file header]

#include "MinkQp.h"

THIRD_PARTY_INCLUDES_START
#include "ThirdParty/qpmad/solver.h"
THIRD_PARTY_INCLUDES_END

bool MinkSolveQp(const FMinkQpProblem& Problem, FMinkVec& OutX)
{
	const int32 N = Problem.H.rows();
	const int32 NumEq = Problem.A ? Problem.A->rows() : 0;
	const int32 NumIneq = Problem.G ? Problem.G->rows() : 0;

	FMinkMat A(NumEq + NumIneq, N);
	FMinkVec Lb(NumEq + NumIneq), Ub(NumEq + NumIneq);
	if (NumEq)
	{
		A.topRows(NumEq) = *Problem.A;
		Lb.head(NumEq) = *Problem.B;
		Ub.head(NumEq) = *Problem.B;
	}
	if (NumIneq)
	{
		A.bottomRows(NumIneq) = *Problem.G;
		Lb.tail(NumIneq).setConstant(-std::numeric_limits<double>::infinity());
		Ub.tail(NumIneq) = *Problem.HIneq;
	}

	FMinkMat HCopy = Problem.H; // qpmad factorizes in place
	qpmad::Solver Solver;
	try
	{
		const qpmad::Solver::ReturnStatus Status = (NumEq + NumIneq) > 0
			? Solver.solve(OutX, HCopy, Problem.C, A, Lb, Ub)
			: Solver.solve(OutX, HCopy, Problem.C);
		if (Status != qpmad::Solver::OK)
		{
			UE_LOG(LogURLabMink, Warning, TEXT("MinkSolveQp: qpmad returned status %d"), (int32)Status);
			return false;
		}
	}
	catch (const std::exception& E)
	{
		UE_LOG(LogURLabMink, Warning, TEXT("MinkSolveQp: qpmad threw: %hs"), E.what());
		return false;
	}
	return true;
}
```
(Adjust the include path/API name to the vendored headers if they differ — check `ThirdParty/qpmad/solver.h` exists after Step 1; qpmad's primary header is `qpmad/solver.h`.)

- [ ] **Step 6: Build; run `URLab.Mink.Qp`. Expected: both tests pass.**
- [ ] **Step 7: Format + commit** — `git add Source/URLabMink ThirdPartyNotices.txt Scripts/mink_golden && git commit -m "feat(URLabMink): vendor qpmad 1.4.0 + FMinkQp wrapper with quadprog-parity tests"`.

---

### Task 7: Task base classes + `FMinkPostureTask`

**Files:**
- Create: `Source/URLabMink/Public/Tasks/MinkTask.h`, `Source/URLabMink/Private/Tasks/MinkTask.cpp`
- Create: `Source/URLabMink/Public/Tasks/MinkPostureTask.h`, `Source/URLabMink/Private/Tasks/MinkPostureTask.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkTaskTests.cpp`
- Modify: `Scripts/mink_golden/gen_golden.py` (add `task_posture` layer)

**Interfaces:**
- Produces (mirrors `tasks/task.py` incl. the residual fast path used by `solve_ik`):

```cpp
struct URLABMINK_API FMinkObjective
{
	FMinkMat H;
	FMinkVec C;
	double Value(const FMinkVec& X) const; // xᵀHx + c·x  (mink Objective.value)
};

struct URLABMINK_API FMinkResidual
{
	FMinkMat WeightedJacobian; // cost[:,None] * J
	FMinkVec WeightedError;    // cost * (-gain * error)
	double Mu = 0.0;           // lm_damping * ||werr||²
};

enum class EMinkTaskStatus : uint8 { Ok, NoResidualForm, Error };

class URLABMINK_API FMinkBaseTask
{
public:
	virtual ~FMinkBaseTask() = default;
	virtual bool ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const = 0;
	virtual EMinkTaskStatus ComputeQpResidual(const FMinkConfiguration& Configuration, FMinkResidual& Out) const
	{
		return EMinkTaskStatus::NoResidualForm; // dense fallback, like BaseTask.compute_qp_residual -> None
	}
};

class URLABMINK_API FMinkTask : public FMinkBaseTask
{
public:
	FMinkTask(const FMinkVec& InCost, double InGain, double InLmDamping); // validates gain∈[0,1], lm≥0 → bIsValid
	bool bIsValid = true;
	FMinkVec Cost;
	double Gain = 1.0;
	double LmDamping = 0.0;

	virtual bool ComputeError(const FMinkConfiguration& Configuration, FMinkVec& Out) const = 0;
	virtual bool ComputeJacobian(const FMinkConfiguration& Configuration, FMinkMat& Out) const = 0;

	bool ComputeQpObjective(const FMinkConfiguration& Configuration, FMinkObjective& Out) const override;
	EMinkTaskStatus ComputeQpResidual(const FMinkConfiguration& Configuration, FMinkResidual& Out) const override;

protected:
	void WeightedResidual(const FMinkVec& Error, const FMinkMat& Jacobian, FMinkResidual& Out) const;
	void AssembleQp(const FMinkVec& Error, const FMinkMat& Jacobian, const FMinkMat& EyeNv, FMinkObjective& Out) const;
};

class URLABMINK_API FMinkPostureTask : public FMinkTask
{
public:
	FMinkPostureTask(const mjModel* Model, const FMinkVec& Cost, double Gain = 1.0, double LmDamping = 0.0);
	bool SetCost(const FMinkVec& Cost);        // scalar (size 1) or size nv
	bool SetTarget(const FMinkVec& TargetQ);   // size nq
	void SetTargetFromConfiguration(const FMinkConfiguration& Configuration);
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override;    // mj_differentiatePos(target, q), freejoint dofs zeroed
	bool ComputeJacobian(const FMinkConfiguration&, FMinkMat&) const override; // I(nv) with freejoint cols zeroed
	TOptional<FMinkVec> TargetQ;
	int32 K = 0, NqCached = 0;
protected:
	const mjModel* ModelRef;
	TArray<int32> FreeVIds;
};
```

**Implementation notes:**
- `WeightedResidual`: exactly `werr = Cost.cwiseProduct(-Gain * Error); wjac = Cost.asDiagonal() * Jacobian; Mu = LmDamping * werr.squaredNorm();`
- `AssembleQp`: `H = wjacᵀ wjac; if (Mu > 0) H += Mu * EyeNv; C = -(wjacᵀ werr);` — note Python computes `c = -weighted_error @ weighted_jacobian` (same thing).
- `FMinkTask::ComputeQpResidual`: calls ComputeError/ComputeJacobian; either failing → `EMinkTaskStatus::Error`.
- Posture error: `mj_differentiatePos(Model, qvel.data(), 1.0, TargetQ->data(), q.data())` — note argument order `qpos1=target, qpos2=current` (Python computes `q ⊖ target`); then `qvel[FreeVIds] = 0`. No target → log `"No target set for FMinkPostureTask"`, return false.

- [ ] **Step 1: Extend `gen_golden.py` with `task_posture`:**

```python
def _valid_q(rng, model):
    q = model.qpos0.copy() + 0.3 * rng.standard_normal(model.nq)
    for jnt in range(model.njnt):
        t, adr = model.jnt_type[jnt], model.jnt_qposadr[jnt]
        if t == mujoco.mjtJoint.mjJNT_FREE:
            q[adr + 3 : adr + 7] /= np.linalg.norm(q[adr + 3 : adr + 7])
        elif t == mujoco.mjtJoint.mjJNT_BALL:
            q[adr : adr + 4] /= np.linalg.norm(q[adr : adr + 4])
    return q


def _task_case(cfg, task):
    err = task.compute_error(cfg)
    jac = task.compute_jacobian(cfg)
    H, c = task.compute_qp_objective(cfg)
    res = task.compute_qp_residual(cfg)
    out = {"error": j(err), "jacobian": j(jac), "H": j(H), "c": j(c)}
    if res is not None:
        wj, we, mu = res
        out["residual"] = {"wjac": j(wj), "werr": j(we), "mu": j(mu)}
    return out


def gen_task_posture():
    rng = np.random.default_rng(20260705)
    out = {"models": {}}
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for cost, gain, lm in [(1.0, 1.0, 0.0), (0.5, 0.7, 0.1), ("vector", 1.0, 0.05)]:
            cost_vec = (np.abs(rng.standard_normal(model.nv)) if cost == "vector"
                        else np.array([cost]))
            task = mink.PostureTask(model, cost=cost_vec, gain=gain, lm_damping=lm)
            target = _valid_q(rng, model)
            task.set_target(target)
            q = _valid_q(rng, model)
            cfg.update(q=q)
            cases.append({"q": j(q), "cost": j(cost_vec), "gain": gain, "lm": lm,
                          "target_q": j(target), **_task_case(cfg, task)})
        out["models"][model_name] = cases
    return out
```
Regenerate (`--layers task_posture`).

- [ ] **Step 2: Write failing test in `MinkTaskTests.cpp`** — `URLab.Mink.Tasks.Posture`: per model/case build `FMinkPostureTask`, `SetTarget`, `Update(q)`, compare `error`, `jacobian`, `H`, `c`, and residual triple at `TOL_OBJ`. Add two validation asserts: `FMinkPostureTask(model, cost, /*gain*/ 1.5, 0.0).bIsValid == false`, negative-cost `SetCost` returns false.
- [ ] **Step 3: Build → red. Implement. Build; run `URLab.Mink.Tasks`. Expected: pass.**
- [ ] **Step 4: Format + commit** — `"feat(URLabMink): task base + FMinkPostureTask with golden parity"`.

---

### Task 8: `FMinkFrameTask`

**Files:**
- Create: `Source/URLabMink/Public/Tasks/MinkFrameTask.h`, `Source/URLabMink/Private/Tasks/MinkFrameTask.cpp`
- Modify: `Source/URLabMink/Private/Tests/MinkTaskTests.cpp`, `Scripts/mink_golden/gen_golden.py` (add `task_frame`)

**Interfaces:**

```cpp
class URLABMINK_API FMinkFrameTask : public FMinkTask
{
public:
	FMinkFrameTask(const FString& FrameName, EMinkFrameType FrameType,
		const FMinkVec& PositionCost /*1 or 3*/, const FMinkVec& OrientationCost /*1 or 3*/,
		double Gain = 1.0, double LmDamping = 0.0);
	bool SetPositionCost(const FMinkVec& PositionCost);
	bool SetOrientationCost(const FMinkVec& OrientationCost);
	void SetTarget(const FMinkSE3& TransformTargetToWorld);
	bool SetTargetFromConfiguration(const FMinkConfiguration& Configuration);
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override;
	bool ComputeJacobian(const FMinkConfiguration&, FMinkMat&) const override;
	// Overridden to share the frame transform between error & jacobian (mirrors _error_and_jacobian):
	bool ComputeQpObjective(const FMinkConfiguration&, FMinkObjective&) const override;
	EMinkTaskStatus ComputeQpResidual(const FMinkConfiguration&, FMinkResidual&) const override;

	FString FrameName;
	EMinkFrameType FrameType;
	TOptional<FMinkSE3> TransformTargetToWorld;
	static constexpr int32 K = 6;
private:
	bool ErrorAndJacobian(const FMinkConfiguration&, FMinkVec&, FMinkMat&) const;
};
```

**Implementation notes** (transcribe `frame_task.py`, pure-Python branch):
- Error: `target.RMinus(frame)` where `RMinus(other) = (other.Inverse().Multiply(*this)).Log()` — i.e. `(frame⁻¹ ∘ target).log()`.
- Jacobian: `T_tb = target.Inverse().Multiply(frame); J = -T_tb.Jlog() * FrameJac;`
- Costs: `Cost.head(3) = positionCost (broadcast if scalar); Cost.tail(3) = orientationCost;` negative → error log + false, exactly mirroring the Python validation messages.
- No target set → `Error`/`false` with `"No target set for FMinkFrameTask"`.

- [ ] **Step 1: `gen_golden.py` add `gen_task_frame`** — same shape as `gen_task_posture`; frames: `("ee","site")` on arm3 and `("tip","site")`, `("head","body")` on floating; targets = `SE3.sample_uniform()` seeded via constructing from seeded rng quat+pos (use `SE3.from_rotation_and_translation(SO3(wxyz=qn), p)` with `qn` a normalized seeded random quat) plus one case `set_target_from_configuration`. Cost variants: scalar/scalar, vec3/vec3, one with `lm_damping=0.1`, one with `gain=0.5`. Store `"target": [7]` per case. Regenerate.
- [ ] **Step 2: Add failing `URLab.Mink.Tasks.Frame` loops (error/jacobian/H/c/residual @ `TOL_OBJ`).**
- [ ] **Step 3: Implement; build; test green.**
- [ ] **Step 4: Format + commit** — `"feat(URLabMink): FMinkFrameTask with golden parity"`.

---

### Task 9: `FMinkRelativeFrameTask` + `FMinkComTask`

**Files:**
- Create: `Source/URLabMink/Public/Tasks/MinkRelativeFrameTask.h` + `.cpp`, `Source/URLabMink/Public/Tasks/MinkComTask.h` + `.cpp` (Private for .cpp)
- Modify: `MinkTaskTests.cpp`, `gen_golden.py` (add `task_relative_frame`, `task_com`)

**Interfaces:**

```cpp
class URLABMINK_API FMinkRelativeFrameTask : public FMinkTask
{
public:
	FMinkRelativeFrameTask(const FString& FrameName, EMinkFrameType FrameType,
		const FString& RootName, EMinkFrameType RootType,
		const FMinkVec& PositionCost, const FMinkVec& OrientationCost,
		double Gain = 1.0, double LmDamping = 0.0);
	bool SetPositionCost(const FMinkVec&);
	bool SetOrientationCost(const FMinkVec&);
	void SetTarget(const FMinkSE3& TransformTargetToRoot);
	bool SetTargetFromConfiguration(const FMinkConfiguration&);
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override;
	bool ComputeJacobian(const FMinkConfiguration&, FMinkMat&) const override;
	bool ComputeQpObjective(const FMinkConfiguration&, FMinkObjective&) const override;
	EMinkTaskStatus ComputeQpResidual(const FMinkConfiguration&, FMinkResidual&) const override;
	FString FrameName, RootName;
	EMinkFrameType FrameType, RootType;
	TOptional<FMinkSE3> TransformTargetToRoot;
};

class URLABMINK_API FMinkComTask : public FMinkTask
{
public:
	FMinkComTask(const FMinkVec& Cost /*1 or 3*/, double Gain = 1.0, double LmDamping = 0.0);
	bool SetCost(const FMinkVec&);
	bool SetTarget(const FMinkVec3& TargetCom);
	void SetTargetFromConfiguration(const FMinkConfiguration&); // data->subtree_com[1]
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override;    // subtree_com[1] - target
	bool ComputeJacobian(const FMinkConfiguration&, FMinkMat&) const override; // mj_jacSubtreeCom(..., 1)
	TOptional<FMinkVec3> TargetCom;
	static constexpr int32 K = 3;
};
```

**Implementation notes:**
- RelativeFrameTask error (careful, argument order differs from FrameTask!): `frame.RMinus(target)` where `frame = GetTransform(FrameName→RootName)` — i.e. `(target⁻¹ ∘ frame).log()`.
- Jacobian: `T_ft = target.Inverse().Multiply(frame); J = T_ft.Jlog() * (JFrame - frame.Inverse().Adjoint() * JRoot);` (positive sign, unlike FrameTask).
- ComTask jacobian buffer is row-major 3×nv for `mj_jacSubtreeCom`; body index 1 hardcoded like Python.

- [ ] **Step 1: `gen_golden.py`** — `gen_task_relative_frame` (floating model: frame `("tip","site")` relative to root `("base","body")`; arm3: `("ee","site")` vs `("link1","body")`); `gen_task_com` (both models, scalar + vec3 costs). Regenerate.
- [ ] **Step 2: Failing tests `URLab.Mink.Tasks.RelativeFrame`, `URLab.Mink.Tasks.Com` (@ `TOL_OBJ`).**
- [ ] **Step 3: Implement; build; green.**
- [ ] **Step 4: Format + commit** — `"feat(URLabMink): relative-frame + com tasks with golden parity"`.

---

### Task 10: `FMinkDampingTask`, `FMinkDofFreezingTask`, `FMinkKineticEnergyRegularizationTask`

**Files:**
- Create: `Source/URLabMink/Public/Tasks/MinkDampingTask.h` + `.cpp`, `.../MinkDofFreezingTask.h` + `.cpp`, `.../MinkKineticEnergyRegularizationTask.h` + `.cpp`
- Modify: `MinkTaskTests.cpp`, `gen_golden.py` (add `task_damping`, `task_dof_freezing`, `task_kinetic_energy`)

**Interfaces:**

```cpp
class URLABMINK_API FMinkDampingTask : public FMinkPostureTask
{
public:
	FMinkDampingTask(const mjModel* Model, const FMinkVec& Cost); // gain=0, lm=0
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override; // always zeros(nv), no target needed
};

class URLABMINK_API FMinkDofFreezingTask : public FMinkTask
{
public:
	FMinkDofFreezingTask(const mjModel* Model, const TArray<int32>& DofIndices, double Gain = 1.0);
	// validates non-empty, in-range, no duplicates → bIsValid; sorts indices; caches error/jacobian
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override;    // zeros(k)
	bool ComputeJacobian(const FMinkConfiguration&, FMinkMat&) const override; // selector rows
	TArray<int32> DofIndices;
};

class URLABMINK_API FMinkKineticEnergyRegularizationTask : public FMinkBaseTask
{
public:
	explicit FMinkKineticEnergyRegularizationTask(double Cost); // cost>=0 → bIsValid
	void SetDt(double Dt); // stores InvDtSq
	bool ComputeQpObjective(const FMinkConfiguration&, FMinkObjective&) const override;
	// inherits NoResidualForm residual (dense path) — this is the task that exercises it
	bool bIsValid = true;
	double Cost = 0.0;
	TOptional<double> InvDtSq;
};
```

**Implementation notes:** KE objective: `H = Cost * InvDtSq * Configuration.GetInertiaMatrix(); C = zeros(nv);` unset dt → error log `"No integration timestep set for FMinkKineticEnergyRegularizationTask"` + return false.

- [ ] **Step 1: `gen_golden.py`** — three small generators following the `_task_case` pattern (`DampingTask(model, cost)` scalar and vector; `DofFreezingTask(model, dof_indices=[0,2])` on arm3 and `[0,1,7]` on floating; `KineticEnergyRegularizationTask(cost=1e-4)` with `set_dt(0.02)` storing only `H`/`c`). Regenerate.
- [ ] **Step 2: Failing tests `URLab.Mink.Tasks.Damping`, `.DofFreezing`, `.KineticEnergy` (@ `TOL_OBJ`); DofFreezing also asserts duplicate-index and out-of-range ctor → `!bIsValid`.**
- [ ] **Step 3: Implement; build; green. Format + commit** — `"feat(URLabMink): damping, dof-freezing, kinetic-energy tasks with golden parity"`.

---

### Task 11: `FMinkEqualityConstraintTask` (+ equality model)

**Files:**
- Create: `Source/URLabMink/Public/Tasks/MinkEqualityConstraintTask.h` + `.cpp`
- Create: `Scripts/mink_golden/models/equality.xml`
- Modify: `MinkTaskTests.cpp`, `gen_golden.py` (add `task_equality`)

**Interfaces:**

```cpp
class URLABMINK_API FMinkEqualityConstraintTask : public FMinkTask
{
public:
	// Empty EqualityNamesOrIds = regulate ALL equality constraints (mink default).
	FMinkEqualityConstraintTask(const mjModel* Model, const FMinkVec& Cost,
		const TArray<FString>& EqualityNames = {}, const TArray<int32>& EqualityIds = {},
		double Gain = 1.0, double LmDamping = 0.0);
	bool SetCost(const FMinkVec& Cost); // scalar or per-equality (neq_total)
	bool ComputeError(const FMinkConfiguration&, FMinkVec&) const override;    // efc_pos rows of active eq constraints
	bool ComputeJacobian(const FMinkConfiguration&, FMinkMat&) const override; // efc_J rows (dense-ified if sparse)
private:
	void UpdateActiveConstraints(const FMinkConfiguration&) const;
	const mjModel* ModelRef;
	TArray<int32> EqIds;         // resolved, validated (active at qpos0)
	FMinkVec CostPerEq;          // per-equality cost
	mutable TArray<int32> ActiveRows; // efc row indices for our eq ids
	// NOTE: FMinkTask::Cost is mutated per-configuration (mirrors python); mark mutable.
};
```

**Implementation notes:**
- Model MUST have `mj_makeConstraint` run — `FMinkConfiguration::Update` already does when `neq > 0`.
- `UpdateActiveConstraints`: scan `Data->efc_type[i] == mjCNSTR_EQUALITY && EqIds.Contains(Data->efc_id[i])`; collect row indices; set `Cost` (base member, `mutable`) to `CostPerEq[Data->efc_id[row]]` per active row.
- Dense Jacobian: `if (mj_isSparse(Model))` → `mju_sparse2dense(dst, Data->efc_J, Data->efc_J_rownnz, Data->efc_J_rowadr, Data->efc_J_colind, ...)` over `nefc × nv`, else map `Data->efc_J` as row-major `nefc × nv`; then take `ActiveRows`.
- Ctor validation mirrors Python: unknown name / out-of-range id / `!Model->eq_active0[id]` / duplicates / zero constraints → `bIsValid=false` + the exact message text.

- [ ] **Step 1: Write `Scripts/mink_golden/models/equality.xml`**

```xml
<mujoco model="equality">
  <compiler angle="radian"/>
  <worldbody>
    <body name="a" pos="0 0 0.5">
      <joint name="ja" type="hinge" axis="0 1 0" range="-3 3" limited="true"/>
      <geom name="ga" type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/>
      <body name="b" pos="0.3 0 0">
        <joint name="jb" type="hinge" axis="0 1 0" range="-3 3" limited="true"/>
        <geom name="gb" type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/>
      </body>
    </body>
    <body name="c" pos="0.6 0 0.5">
      <joint name="jc" type="hinge" axis="0 1 0" range="-3 3" limited="true"/>
      <geom name="gc" type="capsule" fromto="0 0 0 0.2 0 0" size="0.03"/>
    </body>
  </worldbody>
  <equality>
    <connect name="eq_connect" body1="b" body2="c" anchor="0.3 0 0"/>
    <joint name="eq_joint" joint1="ja" joint2="jc" polycoef="0 1 0 0 0"/>
  </equality>
</mujoco>
```

- [ ] **Step 2: `gen_golden.py` `gen_task_equality`** — cases: all-equalities scalar cost; by-name `["eq_connect"]`; per-eq vector cost `[1.0, 0.5]`; each at 3 seeded q's (`_valid_q`). Store `error/jacobian/H/c/residual` via `_task_case`. Regenerate.
- [ ] **Step 3: Failing test `URLab.Mink.Tasks.Equality` (@ `TOL_OBJ`), plus ctor-validation asserts (unknown name → `!bIsValid`).**
- [ ] **Step 4: Implement; build; green. Format + commit** — `"feat(URLabMink): equality-constraint task with golden parity"`.

---

### Task 12: `FMinkConfigurationLimit` + `FMinkVelocityLimit`

**Files:**
- Create: `Source/URLabMink/Public/Limits/MinkLimit.h`, `.../MinkConfigurationLimit.h` + `.cpp`, `.../MinkVelocityLimit.h` + `.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkLimitTests.cpp`
- Modify: `gen_golden.py` (add `limit_configuration`, `limit_velocity`)

**Interfaces:**

```cpp
// MinkLimit.h
struct URLABMINK_API FMinkInequality
{
	TOptional<FMinkMat> G;
	TOptional<FMinkVec> H;
	bool IsInactive() const { return !G.IsSet() && !H.IsSet(); }
};

class URLABMINK_API FMinkLimit
{
public:
	virtual ~FMinkLimit() = default;
	virtual bool ComputeQpInequalities(const FMinkConfiguration& Configuration, double Dt, FMinkInequality& Out) const = 0;
};

class URLABMINK_API FMinkConfigurationLimit : public FMinkLimit
{
public:
	FMinkConfigurationLimit(const mjModel* Model, double Gain = 0.95, double MinDistanceFromLimits = 0.0);
	bool bIsValid = true; // gain ∈ (0,1]
	bool ComputeQpInequalities(const FMinkConfiguration&, double Dt, FMinkInequality& Out) const override;
private:
	const mjModel* ModelRef;
	double Gain;
	TArray<int32> Indices;      // limited dof indices
	FMinkVec Lower, Upper;      // nq vectors (±mjMAXVAL padding)
	TOptional<FMinkMat> ProjectionMatrix; // eye(nv)[indices]
};

class URLABMINK_API FMinkVelocityLimit : public FMinkLimit
{
public:
	// Ordered pairs preserve python-dict insertion order (G row order parity).
	FMinkVelocityLimit(const mjModel* Model, const TArray<TPair<FString, FMinkVec>>& Velocities);
	bool bIsValid = true; // free joint named / wrong shape → invalid
	bool ComputeQpInequalities(const FMinkConfiguration&, double Dt, FMinkInequality& Out) const override;
	TArray<int32> Indices;
	FMinkVec Limit;
private:
	TOptional<FMinkMat> ProjectionMatrix;
};
```

**Implementation notes:** transcribe both `compute_qp_inequalities` bodies exactly (ConfigurationLimit: two `mj_differentiatePos` calls with the documented argument swap; `G = [P; -P]`, `h = [gain*dqmax[idx]; gain*dqmin[idx]]`. VelocityLimit: `G = [P; -P]`, `h = [dt*limit; dt*limit]`; no-limited-joints → inactive `FMinkInequality`).

- [ ] **Step 1: `gen_golden.py`** — `gen_limit_configuration` (arm3 + floating; gain 0.95 and 0.5; `min_distance_from_limits` 0.0 and 0.05; 3 q's each; store `G`,`h`) and `gen_limit_velocity` (arm3: `{"j1": 3.14, "j2": 2.0, "j3": 2.0}`; floating: `{"neck": [1,1,1], "ant": 2.0}`; dt 0.02/0.005). Regenerate.
- [ ] **Step 2: Failing tests `URLab.Mink.Limits.Configuration`, `URLab.Mink.Limits.Velocity` (@ `TOL_OBJ`).**
- [ ] **Step 3: Implement; build; green. Format + commit** — `"feat(URLabMink): configuration + velocity limits with golden parity"`.

---

### Task 13: `MinkSolveIK` — objective/inequality/equality assembly + end-to-end parity

**Files:**
- Create: `Source/URLabMink/Public/MinkSolveIK.h`, `Source/URLabMink/Private/MinkSolveIK.cpp`
- Create: `Source/URLabMink/Private/Tests/MinkSolveIKTests.cpp`
- Modify: `gen_golden.py` (add `solve_ik`)

**Interfaces:**

```cpp
enum class EMinkIKStatus : uint8
{
	Success,
	NoSolutionFound,               // QP failed
	NotWithinConfigurationLimits,  // safety_break tripped
	TaskError,                     // a task reported Error (unset target etc.)
	LimitError
};

struct URLABMINK_API FMinkIKResult
{
	EMinkIKStatus Status = EMinkIKStatus::TaskError;
	FMinkVec Velocity; // tangent-space v (dq/dt), valid iff Success
	bool IsSuccess() const { return Status == EMinkIKStatus::Success; }
};

// build_ik: assemble the QP without solving. Limits semantics mirror python:
//   Limits == nullptr  -> default {ConfigurationLimit(model)}
//   Limits == &empty   -> no limits
URLABMINK_API bool MinkBuildIK(const FMinkConfiguration& Configuration,
	const TArray<const FMinkBaseTask*>& Tasks, double Dt,
	FMinkQpProblem& OutProblem, double Damping = 1e-12,
	const TArray<const FMinkLimit*>* Limits = nullptr,
	const TArray<const FMinkTask*>* Constraints = nullptr);

URLABMINK_API FMinkIKResult MinkSolveIK(const FMinkConfiguration& Configuration,
	const TArray<const FMinkBaseTask*>& Tasks, double Dt,
	double Damping = 1e-12, bool bSafetyBreak = false,
	const TArray<const FMinkLimit*>* Limits = nullptr,
	const TArray<const FMinkTask*>* Constraints = nullptr);
```

**Implementation notes (transcribe `solve_ik.py` exactly, including the residual-stacking optimization):**
- Objective: iterate tasks; `ComputeQpResidual` → `Ok`: stack `WeightedJacobian` rows / `WeightedError` entries, `MuTotal += Mu`; `NoResidualForm`: accumulate dense `H_dense/c_dense` via `ComputeQpObjective`; `Error`: return `TaskError`.
  Then `H = WᵀW` (or zeros if none), `c = -(Wᵀ errs)`, `H.diagonal().array() += Damping + MuTotal`, then `H += H_dense; c += c_dense` if present.
- Inequalities: default-limit rule above; skip inactive; vstack G / hstack h; none → unset.
- Equalities: `A` rows = `Constraints[i]->ComputeJacobian`, `b` = `-gain * error`.
- `MinkSolveIK`: `CheckLimits(1e-6, bSafetyBreak)` first (false → `NotWithinConfigurationLimits`); build; `MinkSolveQp`; failure → `NoSolutionFound` (log `"QP solver qpmad failed to find a solution."`); `Velocity = dq / Dt`.

- [ ] **Step 1: `gen_golden.py` `gen_solve_ik`** — scenario list, each stored with a full task/limit/constraint description the C++ test can reconstruct:

```python
def gen_solve_ik():
    rng = np.random.default_rng(20260706)
    scenarios = []

    def run(model_name, q0, tasks_desc, limits_mode, constraints_desc, damping, dt, steps=10):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cfg.update(q=q0)
        tasks, t_json = [], []
        for d in tasks_desc:
            if d["type"] == "frame":
                t = mink.FrameTask(d["frame"], d["frame_type"], d["position_cost"],
                                   d["orientation_cost"], d.get("gain", 1.0), d.get("lm", 0.0))
                t.set_target(mink.SE3(wxyz_xyz=np.array(d["target"])))
            elif d["type"] == "posture":
                t = mink.PostureTask(model, d["cost"], d.get("gain", 1.0), d.get("lm", 0.0))
                t.set_target(np.array(d["target_q"]))
            elif d["type"] == "damping":
                t = mink.DampingTask(model, d["cost"])
            elif d["type"] == "com":
                t = mink.ComTask(d["cost"]); t.set_target(np.array(d["target_com"]))
            tasks.append(t); t_json.append(d)
        limits = None if limits_mode == "default" else ([] if limits_mode == "none" else limits_mode)
        constraints = None
        c_json = None
        if constraints_desc:
            constraints = [mink.DofFreezingTask(model, d["dofs"]) for d in constraints_desc]
            c_json = constraints_desc
        v = mink.solve_ik(cfg, tasks, dt, solver="quadprog", damping=damping,
                          limits=limits, constraints=constraints)
        q = q0.copy()
        traj_cfg = mink.Configuration(model); traj_cfg.update(q=q0)
        for _ in range(steps):
            vi = mink.solve_ik(traj_cfg, tasks, dt, solver="quadprog", damping=damping,
                               limits=limits, constraints=constraints)
            traj_cfg.integrate_inplace(vi, dt)
        scenarios.append({"model": model_name, "q0": j(q0), "tasks": t_json,
                          "limits": limits_mode if isinstance(limits_mode, str) else "default",
                          "constraints": c_json, "damping": damping, "dt": dt,
                          "v": j(v), "steps": steps, "q_final": j(traj_cfg.q)})

    arm = load_model("arm3")
    home = arm.key_qpos[0].copy()
    ee_target = j(mink.SE3.from_rotation_and_translation(
        mink.SO3.identity(), np.array([0.45, 0.1, 0.35])).wxyz_xyz)
    run("arm3", home,
        [{"type": "frame", "frame": "ee", "frame_type": "site", "position_cost": 1.0,
          "orientation_cost": 0.2, "lm": 0.01, "target": ee_target},
         {"type": "posture", "cost": 1e-2, "target_q": j(home)}],
        "default", None, 1e-12, 0.02)
    run("arm3", home,
        [{"type": "frame", "frame": "ee", "frame_type": "site", "position_cost": 1.0,
          "orientation_cost": 0.0, "lm": 0.1, "target": ee_target},
         {"type": "damping", "cost": 1e-1}],
        "none", [{"dofs": [0]}], 1e-3, 0.02)
    flt = load_model("floating")
    q0 = flt.qpos0.copy()
    run("floating", q0,
        [{"type": "frame", "frame": "tip", "frame_type": "site", "position_cost": 1.0,
          "orientation_cost": 0.5, "target": j(mink.SE3.from_rotation_and_translation(
              mink.SO3.identity(), np.array([0.3, 0.2, 1.5])).wxyz_xyz)},
         {"type": "posture", "cost": 1e-3, "target_q": j(q0)}],
        "default", None, 1e-12, 0.01)
    return {"scenarios": scenarios}
```
Regenerate.

- [ ] **Step 2: Failing test `URLab.Mink.SolveIK.Golden`** — reconstructs each scenario (helper that builds tasks from the JSON descriptors — frame/posture/damping/com and DofFreezing constraints), asserts:
  1. single-call `v` @ `TOL_IK`;
  2. `steps`-long integrate loop final `q` @ `TOL_IK` (the strongest end-to-end check);
  3. plus two behavior tests: `URLab.Mink.SolveIK.SafetyBreak` (arm3 q outside j2 range, `bSafetyBreak=true` → `NotWithinConfigurationLimits`) and unset frame-task target → `TaskError`.
- [ ] **Step 3: Implement `MinkSolveIK.cpp`; build; green.**
- [ ] **Step 4: Format + commit** — `"feat(URLabMink): MinkSolveIK end-to-end golden parity vs python mink"`.

---

### Task 14: `FMinkCollisionAvoidanceLimit` (heaviest — sequenced last per spec)

**Files:**
- Create: `Source/URLabMink/Public/Limits/MinkCollisionAvoidanceLimit.h`, `Source/URLabMink/Private/Limits/MinkCollisionAvoidanceLimit.cpp`
- Create: `Scripts/mink_golden/models/scene_collision.xml`
- Modify: `MinkLimitTests.cpp`, `MinkSolveIKTests.cpp`, `gen_golden.py` (add `limit_collision`; extend `solve_ik` with one CA scenario)

**Interfaces:**

```cpp
struct URLABMINK_API FMinkGeomGroup
{
	TArray<FString> Names; // resolved via mj_name2id
	TArray<int32> Ids;     // used directly
};
using FMinkCollisionPair = TPair<FMinkGeomGroup, FMinkGeomGroup>;

class URLABMINK_API FMinkCollisionAvoidanceLimit : public FMinkLimit
{
public:
	FMinkCollisionAvoidanceLimit(const mjModel* Model, const TArray<FMinkCollisionPair>& GeomPairs,
		double Gain = 0.85, double MinimumDistanceFromCollisions = 0.005,
		double CollisionDetectionDistance = 0.01, double BoundRelaxation = 0.0,
		bool bBroadphase = true);
	bool bIsValid = true;
	bool ComputeQpInequalities(const FMinkConfiguration&, double Dt, FMinkInequality& Out) const override;

	int32 MaxNumContacts() const { return GeomIdPairs.Num(); }
	int32 BroadphaseMinPairs = 16; // mirrors _BROADPHASE_MIN_PAIRS; 0 forces broadphase
	bool bBroadphase = true;

private:
	TArray<int32> BroadphaseSurvivors(const mjData* Data) const;
	const mjModel* ModelRef;
	double Gain, MinDist, DetectionDist, BoundRelaxation;
	TArray<TPair<int32, int32>> GeomIdPairs; // deduped, filtered, SORTED ascending (see deviations)
	// Precomputed broadphase partitions (sphere-sphere / plane-geom / always-keep):
	TArray<int32> SsIdx, PgIdx, KeepIdx;
	TArray<int32> SsG1, SsG2, PgPlane, PgOther;
	TArray<double> SsRsum, PgROther;
};
```

**Implementation notes (transcribe `collision_avoidance_limit.py` in full):**
- Pair construction: homogenize names→ids; cartesian product per collision pair; filter `_is_welded_together` (`body_weldid` equal), `_are_geom_bodies_parent_child` (the weld-parent-weld logic verbatim), contype/conaffinity check; store `(min,max)`; dedupe with `TSet`; **sort ascending** (deviation #5).
- Narrow phase per surviving pair: `dist = mj_geomDistance(Model, Data, g1, g2, DetectionDist, Fromto)`; skip if `fabs(dist - DetectionDist) < 1e-12`; contact-normal Jacobian row: normalize `fromto[3:]-fromto[:3]`, `mj_jac(Model, Data, Jac2.data(), nullptr, Fromto+3, geom_bodyid[g2])`, same for Jac1 at `Fromto`, `Row = Normal · (Jac2 - Jac1)`; bound `dist > MinDist ? Gain*(dist-MinDist)/Dt + Relax : Relax`; `sign = dist >= 0 ? -1 : +1`; `G.row(idx) = sign*Row`. Rows for skipped pairs stay zero with `h = +inf` (Python leaves them; keep identical shapes).
- Broadphase: precompute the three partitions in the ctor (`rbound>0` both → sphere-sphere with `rsum`; one plane + bounded other → plane-geom; else keep). At query: sphere-sphere survives if `dist² <= (rsum+margin)²`; plane-geom if signed distance along plane z-axis (xmat cols 2,5,8) `<= margin + rOther`. Used only when `bBroadphase && MaxNumContacts >= BroadphaseMinPairs`.

- [ ] **Step 1: Write `Scripts/mink_golden/models/scene_collision.xml`** — the arm3 kinematic chain (same three links/joints/site, named `g1,g2,g3`) plus `<geom name="obstacle1" type="sphere" pos="0.35 0.1 0.25" size="0.06"/>`, `<geom name="obstacle2" type="box" pos="-0.3 0 0.2" size="0.05 0.05 0.2"/>`, and the floor plane, all with default contype/conaffinity.
- [ ] **Step 2: `gen_golden.py` `gen_limit_collision`** — pairs `[(["g1","g2","g3"], ["obstacle1","obstacle2","floor"]), (["g1"], ["g3"])]`; after constructing the limit do `limit.geom_id_pairs.sort(); limit.max_num_contacts = len(limit.geom_id_pairs)` (order deviation #5); cases at 4 q's (home, stretched toward obstacle1, near floor, near-self-collision), for gain/min-dist/detection-dist variants and `bound_relaxation=0.01`; ALSO store the sorted `geom_id_pairs` so the C++ test can assert pair-set equality. Extend `gen_solve_ik` with one arm3 scenario whose limits list = `[ConfigurationLimit, CollisionAvoidanceLimit(...)]` (limits_mode passed as the actual list; store descriptor `"limits": {"collision": {...params...}, "configuration": {"gain": 0.95}}`). Regenerate `limit_collision,solve_ik`.
- [ ] **Step 3: Failing tests** — `URLab.Mink.Limits.CollisionAvoidance`: pair-set equality (exact int compare vs fixture), then `G`,`h` @ `TOL_OBJ` twice per case: once linear (`bBroadphase=false`) and once forced broadphase (`BroadphaseMinPairs=0`) — both must match the same fixture. Extend `URLab.Mink.SolveIK.Golden` to reconstruct the CA scenario.
- [ ] **Step 4: Implement; build; green. Format + commit** — `"feat(URLabMink): collision-avoidance limit with golden parity (broadphase + narrowphase)"`.

---

### Task 15: URLab consumption, docs, final verification

**Files:**
- Modify: `Source/URLab/URLab.Build.cs` (add `"URLabMink"` to `PrivateDependencyModuleNames`)
- Create: `docs/guides/mink.md`
- Modify: `mkdocs.yml` (nav), `docs/changelog.md` (entry)

**Interfaces:**
- Consumes: everything above. Produces: URLab modules can `#include "MinkSolveIK.h"` etc.; the future EE controller/VR teleop consumes `FMinkFrameTask + MinkSolveIK` from here.

- [ ] **Step 1: Add `"URLabMink"` to `URLab.Build.cs` `PrivateDependencyModuleNames`** (after `"RPCLib"`).
- [ ] **Step 2: Write `docs/guides/mink.md`** — what URLabMink is (1:1 mink port, credit + link to mink and its authors, pinned version v1.2.0), the class map table (mink name ↔ URLabMink name, copy from the spec), a ~30-line C++ usage example (`FMinkConfiguration` on an `mjModel*`, `FMinkFrameTask` + `FMinkPostureTask`, default limits, `MinkSolveIK` in a 100-iteration convergence loop), the parity-testing story (`Scripts/mink_golden/`, how to regenerate, pin policy), and the deliberate-deviations list from Global Constraints.
- [ ] **Step 3: Add nav entry in `mkdocs.yml`** under `Guides:` after the `Recording & Replay` line, matching indentation:

```yaml
    - Differential IK (URLabMink): guides/mink.md
```

- [ ] **Step 4: Add a `docs/changelog.md` entry** under an "Unreleased" heading (create the heading if absent): `Added: URLabMink — 1:1 C++ port of mink v1.2.0 (differential IK: all tasks, all limits, QP solve) with golden-vector numerical parity tests against Python mink.`
- [ ] **Step 5: Full verification run**

```bash
./Scripts/build_and_test_linux.sh --engine "$UE_ENGINE" \
  --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" --filter URLab.Mink
```
Expected: exit 0; summary reports every `URLab.Mink.*` test passed. Then run the FULL existing suite once (`--filter URLab`) to prove no regressions in URLab itself. Expected: exit 0.

- [ ] **Step 6: Format + final commit**

```bash
./Scripts/format.sh
git add Source/URLab/URLab.Build.cs docs/guides/mink.md mkdocs.yml docs/changelog.md
git commit -m "feat(URLabMink): wire into URLab + docs page for the mink port"
```

---

## Spec coverage map (self-check)

| Spec item | Task |
|---|---|
| Module scaffold, uplugin, Build.cs, layering (no URLab dep) | 1 |
| Golden generator, pinned mink, committed fixtures, models | 2 (+5,7–14 layers) |
| lie/ SO3, SE3 (exp/log/adjoint/jacobians/jlog) | 3, 4 |
| configuration.py (FK, Jacobians, transforms, integrate, inertia, keyframes, limits check) | 5 |
| utils.py (mocap, freejoint dims, subtree helpers, custom config vector) | 5 |
| QP solver (GI family, permissive license, ThirdPartyNotices) | 6 |
| tasks: base/Objective/residual, posture | 7 |
| frame | 8 |
| relative-frame, com | 9 |
| damping, dof-freezing, kinetic-energy | 10 |
| equality-constraint | 11 |
| limits: configuration, velocity | 12 |
| solve_ik/build_ik (objective stacking, default limits, equality constraints, statuses) | 13 |
| collision-avoidance (incl. broadphase) — sequenced last | 14 |
| URLab consumes URLabMink; docs; error model; full-suite regression run | 15 (error model woven through 1–14) |
| Deviations documented (exceptions→status, RNG, ordering) | Global Constraints |
