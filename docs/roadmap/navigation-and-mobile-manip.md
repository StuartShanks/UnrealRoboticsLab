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

Adopted from design review, amended order:

0. **Surface per-task cost updates in `ApplyConfig`.** Known gap: live
   reconfigure of task costs is a NO-OP today (only scalars, `task_enabled`,
   frame targets). Blocks fast tuning loops in every later step.
1. **Carrot prototype (throwaway, zero-physics-C++).** Base-body Frame task in
   the `add_controller` stack; expose the pursuit lookahead **pose** (position
   AND yaw — heading comes from pursuit, a point is not enough) via
   `get_nav_status`; demo script streams it as `target_task=1` while streaming
   the EE as task 0 (`ApplyConfigInternal` already routes `target_task`).
   Purpose: validate whole-body nav-through-mink + tune costs. Not the
   architecture.
2. **`TwistFollow` task kind in `FMinkTaskSpec`.** Mink becomes a twist-bus
   consumer: resolve sibling `UMjTwistController` at Bind; rotate twist by base
   yaw from the IK reference (open-loop contract, not live qpos); integrate an
   internal base target with base-drive's leash/reseed semantics — **lift
   `MjBaseDriveController.cpp:150-160` into a shared pure function**, don't
   duplicate. Gate the lazy-base damping task via `task_enabled` while
   navigating (it fights the twist task for the base DOFs).
3. **Surface `EMinkLimitKind::CollisionAvoidance`.** `FMinkCollisionAvoidanceLimit`
   is already ported in URLabMink, spec-side only exposes Configuration/Velocity.
   Geom pairs by name (robot group vs environment group). This is the layer the
   navmesh cannot give (arm envelope, reach-over-table) — velocity-level, local,
   greedy; it prevents penetration, it does not plan around.
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

## Related, separate thread

Suction pickup (adhesion actuator + `set_suction` op + orientation-accurate
reach) replaces finger grasping for pick tasks — orthogonal to both axes,
composes with the swap architecture today and with TwistFollow later.
