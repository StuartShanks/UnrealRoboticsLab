# Navigation & Mobile Manipulation Roadmap

Status: agreed 2026-07-16, after mobile-manip v1 (sequential swap, live-validated,
commit 435ec02) and design review. Two axes of work, in order.

## Guiding architecture

The **twist bus** (`UMjTwistController`) is the permanent seam for navigation
intent. It is `cmd_vel`-shaped: robot-frame `(Vx, Vy, YawRate)`, m/s, yaw CCW.

- **Producers** (interchangeable): pure pursuit (today), teleop/WASD,
  DetourCrowd proxy (later), MPPI local planner (later), a ROS bridge (later).
- **Consumers** (interchangeable): `UMjBaseDriveController` (today, armless
  bases / teleop), mink TwistFollow task (planned — whole-body).

Rules that fall out of this: never couple a producer to a consumer's private
surface (no nav→mink target coupling as architecture); physics authority stays
with MuJoCo (UE nav machinery only ever observes and emits intent); supervisory
semantics (arrival, stuck watchdog, `get_nav_status`) stay in `UMjNavComponent`
regardless of consumer.

## Axis 1 — nav intent through the whole-body QP (current)

Adopted from design review, amended order. **Steps 0–3 are DONE and
live-validated** (2026-07-16; commits 3895e15, f1d403b, 5af9d0f, 96b1819,
demos 62ac238/8396a31; full findings in `.superpowers/sdd/progress.md`):

0. ✅ **Per-task cost updates in `ApplyConfig`** (`task_costs` sparse map) —
   via the existing `MarkSpecsChanged` rebuild path, not a second mechanism.
1. ✅ **Carrot prototype** — validated whole-body nav-through-mink live.
   Enduring findings: `max_iters=1` is real-time tracking mode (the default 20
   is a per-step convergence loop that multiplies wall-clock speed ~20x — the
   real cause of v1's "instant clip"); QP velocity limits are the pace
   governors; a world-frame EE "hold" during transit is a rubber band to the
   spawn point (disable the task; posture holds the arm).
2. ✅ **`TwistFollow` task kind** — the mink as a twist-bus consumer;
   `MjBaseIntegrate` shared pure helpers (base-drive refactored onto them).
   Live transit profile identical to base_drive's — interchangeable consumers,
   zero streaming. Demo: `tidybot_twist_follow_demo.py` (swap demo kept as the
   base_drive-consumer reference).
3. ✅ **`EMinkLimitKind::CollisionAvoidance` surfaced** — GeomsA×GeomsB
   geom-or-body groups (body expansion is REQUIRED in practice: imported mesh
   geoms are unnamed). Live proof: EE commanded INSIDE a table (within
   kinematic reach) held at a stable 0.175 m standoff. The claim is a
   velocity-level no-contact guarantee on guarded links for ANY streamed
   target — NOT planning. Hard-won practice notes:
   - Quick-converted obstacles compile as `<UE actor name>_MjBody` (UE
     auto-names; `actor_id` is a tag) — capture `actor_name` from the
     `spawn_box` reply; a missing group resolves to 0 and the limit SKIPS
     (warning only in the bind log — verify pair count at bind).
   - Guard the FULL kinematic chain: one unguarded link contacting at 1e6-gain
     stiffness explodes physics (QACC → MuJoCo auto-reset mid-run). Grippers:
     envelope standoff covering the finger extent beats enumerating linkage
     bodies.
   - **Bounded-error streaming is doctrine**: unbounded far targets caused
     cost-ratio inversion (2 m EE error outguns a 5 cm leashed twist error at
     any cost), reach-envelope singularity parking (looks like a guard hold;
     isn't), and v1's clip. Always ramp/carrot streamed goals.
   - Reach-while-driving through clutter is NOT a default: the QP tracks and
     avoids, it does not plan — "reach only in free space" is policy above the
     controller. Stowed transit + reach-at-arrival is the pattern;
     reach-early is a free-corridor opt-in.
4. **Repath policy + dynamic navmesh.** Periodic repath + repath on path
   invalidation — but note the hidden scope: `spawn_nav_bounds` bakes statically;
   dynamic obstacles need `RuntimeGeneration=Dynamic` on the RecastNavMesh and
   obstacles registered as nav-relevant. Without that, a repath timer re-plans
   on a stale mesh (silent no-op). Budget this as "dynamic navmesh support".
5. **DetourCrowd producer (when multi-agent matters).** Observe-only proxy:
   stamp the crowd agent's position from the base `UMjBody`'s UE transform each
   tick, read its requested velocity onto the twist bus. Validate early that
   DetourCrowd tolerates per-tick stamping (velocity smoothing churn) and clamp
   its output to base dynamics.

Cross-cutting invariants:
- **Dual-representation invariant:** navmesh avoids UE geometry; the QP limit
  avoids MuJoCo geoms. Anything avoidable must exist in BOTH (quick_convert
  keeps them in sync by convention — demo floors are deliberately UE-only, MJCF
  bodies are nav-invisible). State it per scene; enforce at authoring.
- **One speed, two hats:** base `MinkVelocityLimit` and pursuit `MaxSpeed` must
  be tuned jointly — a QP cap below the commanded twist makes the base lag the
  carrot until the stuck watchdog fires on a healthy run.
- Keep `set_active_controller` + base_drive: still the right consumer for
  armless bases and teleop; the swap becomes a behavior switch, not a workaround.

## Axis 2 — how good the nav intent is (NEXT, after Axis 1)

The producer side. Today's producer is one-shot-planned pure pursuit: correct
for a single robot in a static baked scene, blind to everything else. Planned
upgrades, roughly in order:

1. **A real local planner — MPPI (or DWA) as a twist producer.** Sample N
   candidate twist sequences per tick, roll out ~1–2 s against a local costmap,
   score (obstacle clearance, global-path progress, smoothness), emit the best
   twist. Unfair advantages to exploit: ground-truth geometry from UE, and
   MuJoCo right there for collision/distance queries. This replaces pursuit
   *behind the bus* — nothing downstream changes. Likely the point where nav
   math earns its own `URLabNav` module (mirroring URLabMink: pure math in the
   module, `UMj*` components stay in URLab).
2. **Dynamic obstacles / multi-agent scenes.** Depends on Axis 1 step 4
   (dynamic navmesh) + step 5 (DetourCrowd, incl. NavMeshAgents as simulated
   pedestrians/forklifts — good use of UE's crowd sim as scene content, not
   robot control).
3. **The Nav2/ROS question.** The twist bus is already `cmd_vel`-shaped; a ROS2
   bridge (publish odom + map/scan, subscribe cmd_vel → twist bus) would let
   the actual industry nav stack run against URLab, Isaac-style. Decision
   hinges on audience: self-contained robot playground (stop at #1) vs sim
   backend for real robotics workflows (this is the destination). Deliberately
   deferred until Axis 1 lands and the audience question is answered.
4. **Sensor-driven tier (research):** simulated 2D lidar (MuJoCo rangefinder /
   UE raycasts) → costmaps/SLAM from sensor data instead of ground truth. Only
   meaningful combined with #3.

## Axis 3 — the task layer (agreed 2026-07-16, after axis-1 steps 0–3)

The layer the step-3 debugging proved missing: the QP tracks and avoids, it
does not plan — "when to reach", "drive first", "is the corridor free" are
decisions ABOVE the controller. Architecture: **behavior tree over skills over
controllers** (the standard mobile-manip shape). Skills are thin wrappers over
existing bridge verbs (Drive = set_nav_goal + monitor; Reach = task gating +
bounded-error ramp + plateau monitor); the tree sequences them with condition
nodes. Upgrades slot INSIDE skills without changing the tree — the same
interchangeability test as the twist bus:

- Kinematic checks (condition nodes): reachability = bent-arm annulus
  arithmetic (~0.3–0.85 m from the shoulder, mapped empirically); clearance =
  a small `mj_geomDistance` corridor-sweep bridge op (same collision world as
  the guard — dual-representation by construction).
- Reach-skill upgrade path: clearance gate now → RRT-Connect in joint space
  over MuJoCo collision checking (`URLabPlan` pure-math module, MoveIt-style
  planner-proposes / QP-disposes / guard-insures) when clutter demands it.
- Drive-skill upgrade path: MPPI twist producer (axis 2 #1).
- Whole-body sampling MPC (MJPC-style) is the biggest hammer: upgrades
  reference quality, does NOT replace the task layer. Not the first move.
- Implementation: Python (py_trees or plain FSM) while behaviors are
  experimental; graduate to UE StateTree (in-engine demos) or
  BehaviorTree.CPP/py_trees_ros (if the ROS bridge happens). Skills are
  bridge verbs, so the tree ports mechanically either way.

## NEXT MILESTONE — suction pick v1 (scoped, awaiting brainstorm/spec)

"Nav, grab, nav while holding" — the original ask — via suction, which
collapses grasping into the validated Reach primitive (drive the EE target to
the object; no grasp-pose synthesis, no finger choreography). Pieces:

1. BT skeleton + `skills.py` (Drive / StowArm / Reach / Descend / Suction /
   Lift) reproducing the guarded-reach demo as tree composition.
2. The two kinematic checks above (one op, one function).
3. `set_suction` runtime op + adhesion actuator (just another d->ctrl slot —
   the pass-through idiom already carries it through any active controller);
   contact/proximity check before engaging.
4. MJCF surgery: 2f85 finger subtree → suction cup (one simple geom). Kills
   the finger-envelope compromise (gripper becomes guardable at 2–3 cm — the
   close-quarters clearance a tabletop pick needs); mink golden fixtures need
   a once-over (EE frame moves).
5. Design consequences already settled: pickable objects are EXCLUDED from
   guard obstacle groups (the table stays guarded, the object doesn't — the
   one-pair-per-limit-entry surface supports this); carried object rides the
   frozen-arm hold (last-write-wins ctrl), weld-vs-adhesion decides drop
   physics.

Axis-1 steps 4–5 and axis 2 are NOT blockers for this milestone; their
triggers (dynamic worlds, multi-agent) remain unmet.
