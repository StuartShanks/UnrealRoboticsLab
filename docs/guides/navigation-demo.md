# Mobile Base Navigation

Drive a mobile manipulation base (tidybot-style: planar holonomic base + arm) to
a commanded 2D goal, avoiding static obstacles, using Unreal's Recast navmesh for
planning and MuJoCo actuators for motion. Planning runs on the game thread; the
tight servo runs on the physics thread; the two meet at a thread-safe **twist bus**.

!!! note "How motion is realised"
    MuJoCo stays physics-authoritative — Unreal is the visualizer. The navmesh
    only produces the **path**; a controller turns that path into base-actuator
    commands. You cannot move the robot with `AddMovementInput` / a
    `CharacterMovementComponent`: the next MuJoCo snapshot would overwrite it.

## Architecture

```
set_nav_goal (bridge) / SetNavGoal() (Blueprint)
        │  game thread
        ▼
UMjNavComponent ── Recast path query ── holonomic pure pursuit vs base pose
        │  robot-frame twist (vx fwd, vy left, yaw_rate CCW; m/s, rad/s)
        ▼
UMjTwistController.SetTwist(vx, vy, ω)        ← thread-safe bus (also WASD / set_twist)
        │  physics thread, per sub-step
        ▼
UMjBaseDriveController.ComputeAndApply
   rotate by live base yaw → integrate position targets (anti-windup leash)
   → d->ctrl[joint_x / joint_y / joint_th];  all other actuators passed through
        ▼
mj_step → robot moves → snapshot → Unreal transforms update → (loop)
```

Three drop-on components, one reusable primitive each:

| Component | Thread | Role |
|---|---|---|
| `UMjTwistController` | — | Thread-safe twist store (WASD/gamepad, `set_twist`, or nav write to it). Pre-existing. |
| `UMjBaseDriveController` | physics | Consumes the twist, drives the 3 base actuators, passes every other actuator through. |
| `UMjNavComponent` | game | Plans on the navmesh, follows with pure pursuit, writes the twist. |

Because a `UMjBaseDriveController` and a `UMjMinkIKController` are both
`UMjArticulationController`s, and an articulation binds exactly **one**, they are
mutually exclusive on a single robot today — use the base-drive controller for
navigation. Combining base-nav with arm-IK on one robot is future work.

## Setup

!!! warning "Level isolation is required"
    Nav needs the level to contain **exactly one** robot. An imported MJCF's base
    joints are named `joint_x` / `joint_y` / `joint_th`; if a second robot (e.g.
    an IK-only tidybot) already shares the level, MuJoCo compiles **both** into
    one model with duplicate joint names, and `UMjBaseDriveController` binds
    whichever robot's joints resolve first — usually the wrong one — so the base
    never drives, with no error raised anywhere. Give the nav robot its own
    level (`create_level`) and a unique `actor_id` (**not** `tidybot_0`, which
    existing IK demos use) so there's no ambiguity even if levels ever merge.

The setup below is written as the bridge calls that build it — this is exactly
what the scripted demo (§Scripted demo, below) does. Building the same scene by
hand in the editor GUI still works (noted inline); the bridge path is what's
actually exercised and kept green.

### 1. Environment (so Recast has something to bake)

Imported MJCF **plane** floors create no Unreal geometry, and imported geoms use
overlap-only collision — neither bakes a navmesh. Author the environment on the
Unreal side instead, which also gives it to MuJoCo for free:

```python
client.scene.create_level(name="NavDemo")   # fresh, nav-robot-only level (see warning above)

client.scene.spawn_box(actor_id="nav_floor", location=(0, 0, -0.1), size=(20, 20, 0.2))
client.scene.spawn_box(actor_id="nav_wall", location=(2, 0, 0.4), size=(0.4, 3.0, 0.8))
client.outliner.add_quick_convert(target="nav_wall", static=True)   # fixed obstacle
```

`spawn_box` places a blocking `StaticMesh` actor (`location`/`size` in MuJoCo
metres, full extents); `add_quick_convert(static=True)` gives it a MuJoCo static
geom too, so it exists in both worlds — Unreal collision for the navmesh, physics
collision for MuJoCo. Repeat per obstacle. (Manual equivalent: a large flat
`StaticMesh` actor with default **blocking** collision for the floor, box actors
for obstacles, each with a **`UMjQuickConvertComponent`** set to **`Static =
true`**.)

### 2. Robot

```python
bp = client.scene.import_xml(path=str(MODEL_XML))          # tidybot MJCF
client.scene.spawn_actor(blueprint=bp, actor_id="nav_tidybot", location=(0, 0, 0))
client.scene.ensure_manager()
client.scene.add_nav_stack(target="nav_tidybot", debug_draw=True)
```

`add_nav_stack` attaches all three drop-on components in one call:

- **MuJoCo Twist Controller** (`UMjTwistController`)
- **MuJoCo Base Drive Controller** (`UMjBaseDriveController`) — defaults target
  base joints `joint_x` / `joint_y` / `joint_th`; `ActuatorMode =
  PositionIntegrate` for the tidybot's position actuators.
- **MuJoCo Nav Component** (`UMjNavComponent`) — `debug_draw=True` shows the
  path + lookahead point while navigating in PIE.

Its reply reports `created` vs `existing` component lists plus any `warnings`.
Adding the same three components by hand (**Add Component** ×3 on the robot
Blueprint) works identically — `add_nav_stack` just automates it and also
accepts tuning overrides (`max_speed`, `lookahead`, `acceptance_radius`,
`decel_radius`, `stuck_timeout`, `base_joints`, `actuator_mode`) as extra
arguments. **Do not** also add the Mink IK Controller (single-controller rule
above).

### 3. Navmesh

```python
r = client.scene.spawn_nav_bounds(center=(0, 0, 0.5), extent=(12, 12, 2), agent_radius=55.0)
assert r["nav_data_present"]
```

`spawn_nav_bounds` places a `NavMeshBoundsVolume` sized from `extent`
(half-extents, metres) and rebuilds Recast, blocking until the bake finishes or
`timeout_s` (op default 10s) elapses. `agent_radius` (cm, **default 55**) is the
obstacle clearance baked into the navmesh — roughly the tidybot footprint plus
margin. Raise it for a wider berth around obstacles (at the cost of blocking
narrower gaps); lower it to let the path hug obstacles more closely. Manual
equivalent: add a `NavMeshBoundsVolume` covering the floor, set Project Settings
→ **Navigation Mesh** → **Agent Radius** ≈ 35-55 (tidybot footprint, in cm), and
press **P** in the viewport — you should see a green navmesh with holes carved
around the obstacle actors.

## Driving

### Manual (proves the twist → actuator path)

Play-in-Editor → possess the robot → **WASD** (translate/strafe), **Q/E** (yaw).
The base moves under real MuJoCo physics; the arm is untouched. If this works,
the twist bus and the base-drive servo are correct before nav enters the loop.

### Autonomous (bridge)

From the Python client, with PIE running and the bridge up. Two gotchas here are
easy to miss because nothing errors when you skip them:

- **Un-pause explicitly.** `set_mode("live")` does **not** unpause the engine —
  the auto-unpause in `RpcDispatcher::SetActiveStepMode` only fires when
  switching *into* `Direct`/`Puppet` (a bridge client has no editor Play/Simulate
  button to unpause with, and those modes assume "client drives physics"; Live
  is meant to be unpaused via that button). Without a matching
  `set_paused(paused=False)`, MuJoCo stays paused and the nav/twist/drive
  controllers write into a frozen sim — the goal is accepted, `get_nav_status`
  never leaves `navigating`, and nothing visibly moves.
- **Resolve the runtime name.** `set_nav_goal`/`get_nav_status` look the
  articulation up by its UE actor *name*, not the `actor_id` tag `spawn_actor`
  stashes — they're usually the same string, but PIE can suffix the name. Match
  your `actor_id` against `find_actors(in_pie=True)` to get the value that
  actually resolves.

```python
from urlab_client import URLabClient
import time

client = URLabClient("tcp://localhost")
ACTOR_ID = "nav_tidybot"  # unique — do not reuse "tidybot_0" (an IK tidybot's id)

# Resolve the runtime articulation name (see gotcha above).
art = next(r.name for r in client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True)
           if r.actor_id == ACTOR_ID)

client.runtime.set_mode("live")
client.runtime.set_paused(paused=False)   # required — see gotcha above

# Goal in MuJoCo metres (world frame), behind an obstacle:
r = client.runtime.set_nav_goal(articulation=art, x=4.0, y=0.0)
assert r["accepted"], "goal rejected — off the navmesh?"

while True:
    s = client.runtime.get_nav_status(articulation=art)
    print(s["state"], round(s["distance_to_goal"], 2))
    if s["state"] in ("arrived", "failed"):
        break
    time.sleep(0.5)
assert s["state"] == "arrived"
```

You can also call `SetNavGoal(WorldGoal)` (Blueprint) directly, or bind
`OnNavGoalReached` / `OnNavGoalFailed`.

### Scripted demo (one command)

The whole flow above — fresh level, floor + obstacles, navmesh bake, robot
import + spawn under a unique `actor_id`, nav-stack attach, PIE entry, live-mode
un-pause — is scripted end to end and runs as a single reproducible command
(editor open, bridge up):

```bash
/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
    Scripts/demos/tidybot_nav_demo.py
```

It:

1. Creates a fresh `NavDemo` level (level isolation, above) and spawns
   `nav_tidybot` — never `tidybot_0`, so it can't collide with an IK tidybot
   already in the project.
2. Authors a floor + 3 obstacles with `spawn_box` / `add_quick_convert`, then
   bakes the navmesh with `spawn_nav_bounds(agent_radius=55.0)`.
3. Enters PIE, sets `live` mode, and **un-pauses** — the gotcha above.
4. Drives a goal behind an obstacle wall and asserts: it reaches `arrived`,
   final distance ≤ `ACCEPT_DIST` (0.20 m), and the sampled track actually
   detoured (`max |y| ≥ 1.0 m` — a straight line to the goal never leaves
   `y = 0`).
5. Sends a goal 14 m out — well beyond the 20 m floor / ±12 m navmesh bounds —
   and asserts `set_nav_goal` returns `accepted: false`.

`--keep-open` leaves PIE running for inspection.

!!! note "Off-mesh tolerance"
    `set_nav_goal` snaps to the navmesh within a **1 m** horizontal tolerance
    (`ProjectPointToNavigation`'s search extent, `UMjNavComponent::SetNavGoal`).
    A goal at the *center* of an obstacle is often still within 1 m of nearby
    navmesh and gets **accepted** (it snaps to the edge). To reliably test
    rejection, aim well past the navmesh/floor extent rather than merely inside
    a wall.

### Ops reference

| Op | Namespace | Args | Reply |
|---|---|---|---|
| `create_level` | `scene` | `name`, `force_overwrite?` | `level_path` |
| `spawn_box` | `scene` | `actor_id`, `location`, `size` (MuJoCo metres, full extents), `yaw_deg?` | `actor_name`, `was_existing` |
| `add_quick_convert` | `outliner` | `target`, `static?` | `actor_name`, `static` |
| `spawn_nav_bounds` | `scene` | `center`, `extent` (half-extents, m), `agent_radius?` (cm, default 55), `timeout_s?` (default 10) | `nav_data_present`, `build_seconds` |
| `add_nav_stack` | `scene` | `target`, tuning overrides (`max_speed`, `lookahead`, `acceptance_radius`, `decel_radius`, `stuck_timeout`, `base_joints`, `actuator_mode`), `debug_draw?` | `created`, `existing`, `warnings` |
| `set_mode` | `runtime` | `mode`: `"live"` / `"direct"` / `"puppet"` | `previous_mode`, `current_mode` |
| `set_paused` | `runtime` | `paused: bool` | `paused` |
| `set_nav_goal` | `runtime` | `articulation`, `x`, `y` (MuJoCo metres) | `accepted: bool` (false = off-mesh / no path) |
| `get_nav_status` | `runtime` | `articulation` | `state`: `idle`/`navigating`/`arrived`/`failed`; `distance_to_goal` (m) |

## Tuning (`UMjNavComponent`)

| Property | Default | Effect |
|---|---|---|
| `MaxSpeed` / `MaxYawRate` | 0.6 m/s / 1.2 rad/s | Speed caps. |
| `LookaheadDist` | 60 cm | Larger = smoother, cuts corners more. |
| `AcceptanceRadius` | 15 cm | Distance at which the goal counts as reached. |
| `DecelRadius` | 75 cm | Start slowing within this of the goal. |
| `StuckTimeout` / `MinProgress` | 5 s / 5 cm | Watchdog: if it can't close `MinProgress` in `StuckTimeout`, it aborts (`failed`) and stops pushing. |

These are per-component pursuit/servo tuning (set via `add_nav_stack` args or
directly on the component). Navmesh **clearance** is a separate, bake-time
concern — that's `agent_radius` on `spawn_nav_bounds` (§Navmesh, above), which
controls how far the *path itself* stays from obstacles, not how the robot
follows it.

`UMjBaseDriveController` anti-windup: `MaxLeashLinear` (5 cm) / `MaxLeashAngular`
(0.1 rad) bound how far the integrated target may lead the live joint — this caps
the force the base pushes with when blocked, so a wedged robot rests instead of
grinding.

## Limitations (this version)

- Static obstacles only — no dynamic replanning (replans only on a new goal).
- Nav and arm-IK cannot run on the same robot simultaneously.
- No stuck-recovery beyond the watchdog abort.
- One robot per level for nav today — a shared level with another robot causes
  a duplicate-joint-name collision that silently drives the wrong robot (§Setup
  warning). Multi-robot nav / robot-plus-IK-robot coexistence is future work.

---

## Validation checklist (run once, in an editor session)

This stack was implemented with per-task **compile verification**. Items 1-2
were completed in a session that owned the editor; items 4-5 were exercised
live end-to-end via the scripted demo (§Scripted demo) on 2026-07-15. Item 3
(manual WASD) remains a manual-only spot-check.

1. ✅ **Automation suite** — green 2026-07-15:
   ```bash
   ./Scripts/build_and_test_linux.sh \
     --engine /home/stuart/UE_ROOT/UnrealEngine \
     --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" \
     --filter "URLab.Nav"
   ```
   All nav tests green (`Result={Success}`): `URLab.Nav.Pursuit.*` (5),
   `URLab.Nav.BaseDrive.*` (6), `URLab.Nav.Component.*` (5), `URLab.Nav.Ops.*` (6).
2. ✅ **Sign check** — `joint_y` sign confirmed correct; `URLab.Nav.BaseDrive.DriveWorldY`
   / `…YawThenForward` passed as-is. (If this ever regresses: flip the test
   rig's `joint_y` slide axis from `(0,-1,0)` to `(0,1,0)` — the MuJoCo
   `d->xpos` assertion is the source of truth for the sign. The servo code
   itself is convention-free.)
3. **Manual WASD** drive (§Driving) — base moves, arm untouched. Not automated;
   spot-check when touching the twist/base-drive path.
4. ✅ **E2E** — `Scripts/demos/tidybot_nav_demo.py` (§Scripted demo), validated
   live 2026-07-15: the robot detours around the obstacle wall, decelerates, and
   settles within `ACCEPT_DIST`; `get_nav_status` reaches `arrived`. Getting a
   clean run surfaced two real gotchas (now fixed in the script and documented
   above): a shared-level joint-name collision that silently drove the wrong
   robot, and a bridge Live session that never unpauses without an explicit
   `set_paused`.
5. ✅ **Failure paths** — covered by the scripted demo's off-mesh phase: a goal
   well beyond the floor/navmesh extent is correctly rejected
   (`accepted: false`). Wedging the robot against a wall with a tight
   `StuckTimeout` to observe `failed` + at-rest behavior is not scripted; still
   a manual check.
