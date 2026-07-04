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

### 1. Environment (so Recast has something to bake)

Imported MJCF **plane** floors create no Unreal geometry, and imported geoms use
overlap-only collision — neither bakes a navmesh. Author the environment on the
Unreal side instead, which also gives it to MuJoCo for free:

1. Place a floor: a large flat `StaticMesh` actor (e.g. a thin scaled cube,
   ~20 m × 20 m) with default **blocking** collision.
2. Place a few box/wall obstacle actors between the spawn and the goal area.
3. On **each** of those actors add a **`UMjQuickConvertComponent`** and set
   **`Static = true`** (fixed obstacle, no free joint). They now exist in both
   worlds: Unreal collision for the navmesh, MuJoCo static geoms for physics.

### 2. Robot

1. Import the tidybot MJCF (drag `scene.xml` into the Content Browser). See
   [Differential IK](mink.md) / the mobile-tidybot notes for obtaining the model
   (`mujoco_menagerie/stanford_tidybot`).
2. Place the generated Blueprint on the floor, then **Add Component** ×3:
     - **MuJoCo Twist Controller** (`UMjTwistController`)
     - **MuJoCo Base Drive Controller** (`UMjBaseDriveController`) — defaults
       target base joints `joint_x` / `joint_y` / `joint_th`; leave
       `ActuatorMode = PositionIntegrate` for the tidybot's position actuators.
     - **MuJoCo Nav Component** (`UMjNavComponent`) — set `bDebugDraw = true` to
       see the path + lookahead point while navigating.
3. Place one **MjManager**.
4. **Do not** also add the Mink IK Controller (single-controller rule above).

### 3. Navmesh

1. Add a **`NavMeshBoundsVolume`** covering the floor.
2. Project Settings → **Navigation Mesh** → set **Agent Radius ≈ 35** (the
   tidybot footprint, in cm).
3. Press **P** in the viewport: you should see a green navmesh with holes carved
   around the obstacle actors.

## Driving

### Manual (proves the twist → actuator path)

Play-in-Editor → possess the robot → **WASD** (translate/strafe), **Q/E** (yaw).
The base moves under real MuJoCo physics; the arm is untouched. If this works,
the twist bus and the base-drive servo are correct before nav enters the loop.

### Autonomous (bridge)

From the Python client, with PIE running and the bridge up:

```python
from urlab_client import URLabClient
import time

client = URLabClient("tcp://localhost")
ART = "tidybot"  # the articulation actor's name in the level

# Goal in MuJoCo metres (world frame), behind an obstacle:
r = client.runtime.set_nav_goal(articulation=ART, x=4.0, y=1.5)
assert r["accepted"], "goal rejected — off the navmesh?"

while True:
    s = client.runtime.get_nav_status(articulation=ART)
    print(s["state"], round(s["distance_to_goal"], 2))
    if s["state"] in ("arrived", "failed"):
        break
    time.sleep(0.5)
assert s["state"] == "arrived"
```

You can also call `SetNavGoal(WorldGoal)` (Blueprint) directly, or bind
`OnNavGoalReached` / `OnNavGoalFailed`.

### Ops reference

| Op (`client.runtime.…`) | Args | Reply |
|---|---|---|
| `set_nav_goal` | `articulation`, `x`, `y` (MuJoCo metres) | `accepted: bool` (false = off-mesh / no path) |
| `get_nav_status` | `articulation` | `state`: `idle`/`navigating`/`arrived`/`failed`; `distance_to_goal` (m) |

## Tuning (`UMjNavComponent`)

| Property | Default | Effect |
|---|---|---|
| `MaxSpeed` / `MaxYawRate` | 0.6 m/s / 1.2 rad/s | Speed caps. |
| `LookaheadDist` | 60 cm | Larger = smoother, cuts corners more. |
| `AcceptanceRadius` | 15 cm | Distance at which the goal counts as reached. |
| `DecelRadius` | 75 cm | Start slowing within this of the goal. |
| `StuckTimeout` / `MinProgress` | 5 s / 5 cm | Watchdog: if it can't close `MinProgress` in `StuckTimeout`, it aborts (`failed`) and stops pushing. |

`UMjBaseDriveController` anti-windup: `MaxLeashLinear` (5 cm) / `MaxLeashAngular`
(0.1 rad) bound how far the integrated target may lead the live joint — this caps
the force the base pushes with when blocked, so a wedged robot rests instead of
grinding.

## Limitations (this version)

- Static obstacles only — no dynamic replanning (replans only on a new goal).
- Nav and arm-IK cannot run on the same robot simultaneously.
- No stuck-recovery beyond the watchdog abort.

---

## Validation checklist (run once, in an editor session)

This stack was implemented with per-task **compile verification**; the automation
suite and the live E2E were **not executed** in the authoring session (the editor
held the project lock). Run these once in a session that owns the editor:

1. **Automation suite** (close the editor first — the runner needs the project lock):
   ```bash
   ./Scripts/build_and_test_linux.sh \
     --engine /home/stuart/UE_ROOT/UnrealEngine \
     --project "/home/stuart/Documents/Unreal Projects/Test/Test.uproject" \
     --filter "URLab.Nav"
   ```
   Expect all nav tests green (`Result={Success}`): `URLab.Nav.Pursuit.*` (5),
   `URLab.Nav.BaseDrive.*` (6), `URLab.Nav.Component.*` (5), `URLab.Nav.Ops.*` (3).
2. **Sign check** — if `URLab.Nav.BaseDrive.DriveWorldY` or `…YawThenForward`
   fails on an inverted axis, flip the test rig's `joint_y` slide axis from
   `(0,-1,0)` to `(0,1,0)` (the MuJoCo `d->xpos` assertion is the source of truth
   for the sign) and re-run. The servo code itself is convention-free.
3. **Manual WASD** drive (§Driving) — base moves, arm untouched.
4. **E2E** (§Autonomous) — robot detours around an obstacle, decelerates, stops
   within 15 cm; `get_nav_status` reaches `arrived`.
5. **Failure paths** — `set_nav_goal` into an obstacle → `accepted: false`; wedge
   the robot against a wall with a tight `StuckTimeout` → status `failed`, robot
   at rest (no force grinding).
