# Suction Pick v1 — Design

**Date:** 2026-07-16
**Branch:** feat/nav-stack
**Depends on:** validated axis-1 stack (task_costs `3895e15`, carrot `f1d403b`,
TwistFollow `5af9d0f`, CollisionAvoidance `96b1819`, demos `62ac238`/`8396a31`/`48ddb99`)
**Roadmap:** `docs/roadmap/navigation-and-mobile-manip.md` — Axis 3, NEXT MILESTONE

## Goal

"Nav, grab, nav while holding, release" — the full round trip. TidyBot drives
around the obstacle wall to a table, top-down picks an annotated object via
suction, carries it back to the start area, and releases it. **One controller
(the guarded twist-follow mink stack) for the entire run**, orchestrated by a
py_trees behavior tree over the bridge.

**Premise (locked):** no motion planner in v1. The reach annulus check and a
corridor-clearance gate verify the world is pickable-by-construction; a blocked
corridor ABORTS loudly. RRT-Connect is the named Reach-skill upgrade when a
cluttered pick makes that abort unacceptable (it plans in joint space, so it
will stream posture-ramp waypoints — a second execution flavor inside the same
skill, not a new architecture).

## Decisions (settled during brainstorm)

| Decision | Choice |
|---|---|
| Success bar | Full round trip (pick + carry home + release) |
| Attachment | MuJoCo adhesion actuator (force-limited; honest physics) |
| Task layer | py_trees BT (skills wrap bridge verbs; graduate to StateTree/BT.CPP later) |
| Cup | New model variant `tidybot_suction_ue.xml` (golden-parity model untouched) |
| set_suction path | Fix the mink non-drive pass-through + a thin runtime op over `SetNetworkControl` |
| Engage/verify | Distance-gated engage + lift-test attach verify (ground-truth poses) |
| Approach | Top-down via affordance-site frame; cup-down orientation target |
| Affordances | MJCF sites, frame-encodes-affordance; template-generated trajectories |

## Architecture

```
py_trees BT (Python, bridge client)          Sequence + condition gates + retry decorator
  └─ skills (urlab_skills.py)                Drive / ReachRamp / Descend / Suction / ...
       └─ bridge verbs                       set_nav_goal, configure_controller,
                                             set_suction, check_clearance, find_actors
            └─ guarded twist-follow mink     ONE controller, whole run (mink_ik active)
                 └─ MuJoCo                   adhesion force, contacts, ground truth
```

Upgrades slot INSIDE skills without changing the tree (interchangeability
test): RRT-Connect inside Reach, MPPI inside Drive, contact-sensed engage
inside Descend.

## Components

### C++ — URLab (each small, test-covered)

1. **Mink non-drive pass-through** (`MjMinkIKController.cpp::ComputeAndApply`).
   The mink currently writes ONLY its drive ctrl ids (line ~943) — unlike
   base-drive it has no pass-through, so a non-drive actuator's `NetworkValue`
   (suction, fingers) NEVER reaches `d->ctrl` while the mink is bound. Fix:
   the same loop base-drive has — for every binding whose ActuatorMjID is not
   a drive ctrl id, `d->ctrl[id] = Component->ResolveDesiredControl(Source)`.
   This honors the existing contract ("a bound controller replaces the
   articulation's default ctrl path entirely") and fixes the latent finger gap.

2. **`set_suction` runtime op** (`RpcDispatcher.cpp`, namespace `runtime`).
   Request `{articulation, value}` (value in [0,1]; bool `on` accepted as
   sugar). Resolves the articulation's adhesion actuator by type
   (`EMjActuatorType::Adhesion`); errors `no_adhesion_actuator` /
   `ambiguous_adhesion_actuator` (an optional `actuator` name param
   disambiguates). Calls `SetNetworkControl(value)` — the existing staging;
   `ApplyControls` + the new pass-through deliver it in live AND direct modes.
   Reply: `{actuator, value}`. Marshal: HandleSetNavGoal idiom.

3. **`check_clearance` — CLIENT-SIDE, no C++ op** (plan-time refinement).
   The Python client already mirrors the compiled model + synced data
   (`client.model` / `client.data`, refreshed by direct-mode steps — the
   Tracker pattern), and the `mujoco` bindings expose `mj_ray` locally. So the
   corridor probe is a pure-Python helper in `urlab_skills.py`: cast from
   `from` toward `to` on the mirror, marching past hits on excluded bodies
   (mj_ray's `bodyexclude` takes ONE body id; multiple exclusions = re-cast
   just past each excluded hit, capped at 8 marches). `clear` = no
   non-excluded hit within `|to-from|` minus 1 cm. Run it immediately after a
   synced dip so the mirror is fresh. v1 is a single center ray.

### Model — `Scripts/mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml`

Copy of `tidybot_scene_ue.xml` with:
- the 2f85 finger subtree (drivers/couplers/spring links/followers/pads and
  `fingers_actuator` + its tendon) REMOVED;
- a cup body on the 2f85 base: one simple geom (cylinder ~3 cm radius, ~2 cm
  tall, pointing along the old pinch axis) + `<site name="cup_site">` at the
  cup face — the new EE frame (the mink Frame task targets `cup_site`);
- `<adhesion name="suction" body="cup" ctrlrange="0 1" gain="100"/>` (gain =
  max suction force in Newtons; 100 N holds a 1 kg object at >10 g — tune down
  during live validation if it reads as unbreakable);
- the 10 drive actuators (3 base + 7 arm) byte-identical to the scene model.

The golden-parity model and fixtures are untouched by construction. The
`pinch_site_target` mocap body stays (harmless; targets stream manually).

### Assets — `Scripts/demos/assets/suction_box.xml`

The pickable object, imported (not quick-converted):

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

**Affordance convention:** sites named `affordance_suction*`; the site FRAME
encodes the affordance — position = contact point, **z-axis = outward surface
normal** (approach direction is −z; cup axis target is anti-parallel to z).
World poses come free from the sim (`site_xpos`/`site_xmat`, already readable
client-side) — no manual projection, robust to the object being bumped or
lying on its side.

**Trajectory template (v1):** waypoints generated from the site frame —
`[site + z·0.15 (pre-approach), site + z·0.02 (engage), site (contact)]` —
consumed by the existing bounded-error ramp execution. **Authored-trajectory
extension (specced, NOT implemented in v1):** a per-asset JSON sidecar
(`<asset>.approach.json`: list of `{pos:[xyz], quat:[wxyz]}` in the SITE's
local frame) used instead of the template when present. Same consumer, richer
producer.

### Python — `Scripts/demos/urlab_skills.py`

py_trees behaviours (blackboard: client, articulation name, object actor id,
resolved poses). Each wraps existing bridge verbs; each FAILS loudly with a
reason on timeout/rejection.

- `Drive(goal_xy)` — set_nav_goal + poll get_nav_status; SUCCESS on arrived,
  FAILURE on failed/timeout. Aborts with a diagnostic if dist jumps back to
  its start value (the physics-auto-reset fingerprint).
- `ResolveAffordance(object)` — find the object's `affordance_suction*` sites,
  pick the most upward-facing z (v1 heuristic), emit waypoints + cup-down
  target quat to the blackboard. Site poses read via a synced direct dip
  (Tracker pattern).
- `Reachable(pose)` — condition: bent-arm annulus from the parked base
  (0.30 m ≤ shoulder distance ≤ 0.85 m, empirically mapped 2026-07-16).
- `CorridorClear(from, to)` — condition: `check_clearance` op, excluding the
  robot and the pick object.
- `ReachRamp(waypoints, quat, duration)` — enable the EE task seeded at the
  current pose (world-anchor rule: seed BEFORE enable), then ramp through the
  waypoints (bounded-error doctrine).
- `Descend+Engage` — ramp down the final waypoint leg; when cup_site is
  within 2 cm of the affordance point, `set_suction(1)`.
- `VerifyAttach` — lift 10 cm (ramped); SUCCESS if the object's live z (via
  find_actors — dynamic bodies pump-sync their actor transform) rose ≥ 5 cm;
  one retry from 1 cm lower, then FAILURE.
- `StowCarry` — ramp the EE to a tucked pose AT the current base position,
  then DISABLE the EE task: posture holds the joints for the drive home (a
  world-frame carry target would be the rubber-band bug).
- `Release` — ramp to a low drop pose, `set_suction(0)`, verify detach (object
  z stops tracking the cup), re-stow.

### Python — `Scripts/demos/tidybot_suction_pick_demo.py`

Authors the scene (obstacle course + table, quick-converted scenery, guarded;
`suction_box.xml` imported and spawned ON the table, **excluded from all guard
groups**), builds the guarded twist-follow payload against the suction model
variant (cup vs table gets its own guard entry at ~0.02 m — the finger
envelope is gone, close-quarters is now safe), then runs the tree:

```
Sequence
├─ ResolveAffordance(pick_box)
├─ Reachable(pick point from staging)?
├─ Drive(staging)
├─ CorridorClear(pre-approach → contact)?
├─ ReachRamp(pre-approach, cup-down)
├─ Descend+Engage
├─ VerifyAttach          (retry decorator: once)
├─ StowCarry
├─ Drive(home)
└─ Release
```

Guard config: scenery guarded (links 0.10 m; cup vs scenery 0.02 m); the pick
object in NO guard group. Lazy-base damping relaxed for transits, restored for
arm phases (task_costs, live). `max_iters=1` + velocity limits per doctrine.

## Error handling

- Tree aborts on any skill FAILURE → demo exits nonzero; `--keep-open` leaves
  the session for inspection.
- VerifyAttach: one retry, then fail (no infinite suck-lift loops).
- Blocked corridor: ABORT (v1 premise) — the failure message names RRT-Connect
  as the upgrade.
- Physics-reset fingerprint (dist jump): abort with diagnostic, never re-drive.

## Testing

- **C++ headless** (MinkIK/Nav suites): pass-through test — a non-drive
  actuator's `NetworkValue` reaches `d->ctrl` under a BOUND mink (encodes the
  latent bug, red-run against pre-fix code); `set_suction` ops test (value
  lands on the adhesion actuator; error paths); `check_clearance` ops test
  (known geometry: blocked vs clear ray, exclusion masking).
- **Model**: import/compile test for `tidybot_suction_ue.xml` (adhesion
  actuator present + typed, 10 drive actuators intact, cup_site resolves);
  `suction_box.xml` import test (free body + affordance site resolve). Golden
  fixtures untouched by construction.
- **Python**: py_compile; skills unit-testable against a stub client where
  cheap; live validation is the acceptance (below).
- **Live (collaborative, user Simulates)**: author/drive probes first, the
  committed demo after validation — the established loop. Acceptance: full
  round trip, object visibly picked/carried/released, no guard violations, no
  instability, "ALL PHASES PASSED".

## Deferred (named)

- Perception (poses are ground truth from the sim).
- Cluttered picks → RRT-Connect inside the Reach skill (posture-ramp waypoint
  flavor).
- Authored per-asset approach trajectories (format specced above; template
  covers v1).
- Reachability-aware affordance selection (v1: most-upward heuristic).
- Contact-sensed engage (op exposing the contact list) inside Descend.
- Place-on-target semantics; carry-force tuning beyond "gentle nav holds it".
- Dynamic worlds / multi-agent (axis-1 steps 4–5, axis 2 — triggers unmet).
