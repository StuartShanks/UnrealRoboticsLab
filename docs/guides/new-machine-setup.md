# New-Machine Setup (URLab + HomeInterior demos)

What it takes to reproduce the working environment (as of 2026-07-29,
multi-pick gate 1 passed, gate 2 pending). Paths below are the reference
machine's; adjust to taste but keep them consistent — several scripts carry
absolute paths.

## 1. Unreal Engine (source build)

- UE 5.7 source build. Reference: `/home/stuart/UE_ROOT/UnrealEngine`
  (must contain `Engine/Build/BatchFiles/Linux/Build.sh`).
- Compile command used everywhere (editor-safe, incremental):
  ```
  "$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh" TestEditor Linux Development \
      "-Project=<project>/Test.uproject" -WaitMutex
  ```

## 2. Host project + Content (NOT in any git repo!)

- A UE project (reference: `~/Documents/Unreal Projects/Test/Test.uproject`)
  with this plugin under `Plugins/UnrealRoboticsLab`.
- **Fab content must be re-downloaded**: the HomeInterior pack
  (`/Game/Home_Interior/...`). Content is not versioned.
- **Manual level prep (one-time, in the editor UI):**
  1. Open the Fab map, save a working copy as `/Game/Home_Interior`
     (the scripts load `/Game/Home_Interior` first, falling back to
     `/Game/Home_Interior/Home_Interior`).
  2. Select `SM_Book_125` (outliner label may read `SM_Book_13`) →
     Mobility → **Movable** → save. `add_quick_convert(static=False)` does
     NOT set this, and a non-Movable actor cannot be simulated.
- Optional cleanup that matched our runs: the second book (`SM_Book_127` /
  label `SM_Book_14`) and the table candles were deleted from the working
  level. The multi-pick demo only needs `SM_Book_125`, `SM_Table_01*`,
  `SM_Kitchen_island_table*`, `SM_Table_00_32`.

## 3. Plugin repo (this repo)

```
cd <project>/Plugins
git clone git@github.com:StuartShanks/UnrealRoboticsLab.git
cd UnrealRoboticsLab && git checkout feat/nav-stack
git submodule update --init --recursive     # MuJoCo (3.10 pin), CoACD, libzmq
./third_party/build_all.sh                  # builds third-party libs
```
Then the UE compile (§1). NOTE: the reference machine carries local
uncommitted edits to `third_party/build_all.sh` (pre-existing, engine-path
tweaks) — if the stock script fails, fix the engine path flags locally.

Automation tests (editor must be CLOSED):
```
./Scripts/build_and_test_linux.sh --engine $UE_ROOT --project "<project>/Test.uproject" --filter "URLab.Nav"
```

## 4. Bridge repo (Python client) — the fiddly one

```
cd ~/Unreal_Robotics
git clone git@github.com:URLab-Sim/URLab_Bridge.git
cd URLab_Bridge
git remote add fork git@github.com:StuartShanks/URLab_Bridge.git
git fetch fork
git checkout -b stuart/fab-pick-client-fixes fork/stuart/fab-pick-client-fixes
uv sync --extra dev
```
Then the three REQUIRED manual venv fix-ups (none are in the lockfile):
```
uv pip install --python .venv/bin/python mujoco==3.10.0   # MUST match the engine's MuJoCo.
uv pip install --python .venv/bin/python mink py_trees    # planner IK + skills BT
```
**Gotchas that cost us hours:**
- `uv run`/`uv sync` re-pins mujoco to the lockfile's 3.8.1 — after ANY
  sync, reinstall 3.10.0. Run scripts with `.venv/bin/python` directly,
  never `uv run` (or use `uv run --no-sync`).
- The branch `stuart/fab-pick-client-fixes` carries required fixes (entity
  qpos sync, `set_suction` ctrl mirror). The org's
  `feat/remote_stepping_clean` does NOT have them yet (pending Buzz's
  merge); running the demos off it will silently break suction and mirror
  reads.

## 5. Run flow (the multi-pick demo)

All demo scripts run with the bridge venv python from `Scripts/demos/`:
```
BV=~/Unreal_Robotics/URLab_Bridge/.venv/bin/python
# editor OPEN and IDLE (not simulating):
$BV Scripts/demos/tidybot_multi_pick_author.py            # add --force-reimport after robot-XML edits
# press SIMULATE in the editor, then:
$BV Scripts/demos/tidybot_multi_pick_demo.py --one-way    # live gate 1 (passed 2026-07-26)
$BV Scripts/demos/tidybot_multi_pick_demo.py              # gate 2: full round trip (PENDING)
```
Offline test suites (no editor): `test_urlab_skills_multi.py`,
`test_urlab_planner.py` in `Scripts/demos/` (assert-scripts, bridge venv).

## 6. Conventions that keep you out of trouble

- Author only with the editor IDLE; never over a live Simulate.
- Restart Simulate to reset the scene (book back on the table).
- `set_mode("live")` is required for free-running physics with a
  direct-mode client (`set_paused(false)` alone does NOT run physics).
- Convert (quick_convert) anything the ARM might contact — MuJoCo only
  sees converted bodies; the navmesh sees everything.
- Cup-down reach ceiling ≈ 0.94 m — place/pick surfaces must be below it.
