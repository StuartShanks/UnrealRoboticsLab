# How the mink IK controller generalises

Short version: because `URLabMink` is a 1:1 port of mink and the "drive it" layer is
just a `UMjArticulationController` that writes `d->ctrl`, the *same* controller covers
every robot mink's own examples cover — arms, mobile manipulators, humanoids, hands,
quadrupeds. What changes per robot is **data, not code**: the task list and a few costs.

## The one pattern (recap)

A `UMjMinkIKController : UMjArticulationController` owns:

- one `FMinkConfiguration` (the IK *reference* state),
- a **list** of tasks (`FMinkFrameTask`, `FMinkPostureTask`, `FMinkDampingTask`, …),
- a list of limits (`FMinkConfigurationLimit`, `FMinkVelocityLimit`, `FMinkCollisionAvoidanceLimit`).

Each physics step (`ComputeAndApply`): set task targets → `MinkSolveIK` → `IntegrateInplace`
→ write `Config.GetQ()` into the bound actuators' `d->ctrl`. Never touches `qpos`; the
actuators are the drive, physics stays real. Exactly `mobile_tidybot.py`'s
`data.ctrl[actuator_ids] = configuration.q[dof_ids]`.

## What's robot-agnostic vs. what's configured

Everything structural comes straight from the MuJoCo model, so it's free for any imported MJCF:

| Comes from the model (free) | Configured per robot (the "setup") |
|---|---|
| DOF count, joint types (hinge/slide/free) | which frame(s) to drive, by name |
| qpos/qvel addressing, Jacobians | task costs / gains / lm-damping |
| actuator→DOF bindings (base class does this) | posture target, which limits |
| frame/site/body lookups by name | driven-joint subset (base? arm? hand?) |

The controller resolves frames/joints/actuators by name against the compiled model, so
"add it to an articulation" is: drop the component on, name the end-effector frame, pick a
task stack. No per-robot C++.

## Generalisation axes (mink's own example zoo is the proof set)

Since the port is 1:1, anything in `mink/examples` is expressible with the same controller
by changing the task list:

- **Single arm** (`arm_panda`, `arm_iiwa`, `arm_ur5e`): `FrameTask(ee) + PostureTask` + `ConfigurationLimit`. This is the base case.
- **Mobile manipulator** (`mobile_tidybot`, `mobile_stretch`, `mobile_kinova_leap`): the planar base is just three extra driven joints (`joint_x/joint_y/joint_th`) with their own actuators — nothing special. Add a `DampingTask` weighted on the base DOFs to make the base optionally "immobile" (the fix-base toggle in tidybot). Mobility = more entries in the driven set + one task.
- **Floating-base humanoid** (`humanoid_g1`, `humanoid_h1`, `biped_cassie`): the free-joint base is handled by mink's SE3 tangent; you regularise it with posture/damping over those DOFs and add `ComTask` for balance. Same solver, richer task list.
- **Dexterous hands / high-DOF** (`hand_shadow`, `arm_hand_iiwa_allegro`, `arm_hand_xarm_leap`): one `FrameTask` per fingertip (N tasks in the list) + posture. The solver blends them; no code change, just more tasks.
- **Dual-arm** (`arm_dual_panda`, `arm_dual_iiwa`): two `FrameTask`s, one per hand.
- **Quadruped** (`quadruped_go1`, `quadruped_spot`): frame tasks per foot + posture + limits.
- **Freezing / regularisation** (`arm_panda_dof_freezing`, `kinetic_energy_reg`): add
  `DofFreezingTask` or `KineticEnergyRegularizationTask` to the list.

So "generalising" isn't new integration work per robot — it's composing from the ported
task/limit catalogue: `FrameTask`, `RelativeFrameTask`, `PostureTask`, `ComTask`,
`DampingTask`, `EqualityConstraintTask`, `DofFreezingTask`,
`KineticEnergyRegularizationTask` × `ConfigurationLimit`, `VelocityLimit`,
`CollisionAvoidanceLimit`.

## Concrete: same controller, three configs

```
# Panda arm (arm_panda)
frames  = [ ("attachment", Site, pos=1.0, ori=1.0, lm=1.0) ]
tasks   = [ FrameTask, PostureTask(cost=1e-2) ]
limits  = [ ConfigurationLimit ]
driven  = joint1..7

# TidyBot mobile (mobile_tidybot)
frames  = [ ("pinch_site", Site, pos=1.0, ori=1.0, lm=1.0) ]
tasks   = [ FrameTask, PostureTask(cost[3:]=1e-3), DampingTask(cost[:3]=100 when fix_base) ]
limits  = [ ConfigurationLimit ]
driven  = joint_x, joint_y, joint_th, joint_1..7

# Shadow hand (hand_shadow)
frames  = [ one FrameTask per fingertip site ... ]
tasks   = [ *fingertip FrameTasks, PostureTask ]
limits  = [ ConfigurationLimit, (VelocityLimit) ]
driven  = all finger joints
```

Only the three lists differ. The `ComputeAndApply` loop is identical.

## The exposure generalises too

The task list is the API surface, and it maps cleanly onto the surfaces we already have:

- **Bridge:** the controller's existing `GetConfigSchema` / `ApplyConfig` hooks let a client
  declare the task stack + costs as JSON (`ik.configure(frame=…, tasks=[…], limits=[…])`),
  then just stream targets. Robot-agnostic wire format.
- **Blueprint / in-editor:** expose the task list as sub-objects / Details-panel entries on
  the component, so you compose the stack per robot without code and call
  `SetIKTarget(FVector, FQuat)` from a graph or VR.

Same controller, same solver core; the per-robot "setup" is authored data.

## Status

- **Done + tested:** the solver port (`URLab.Mink` suite 21/21) and the driver core proving
  it moves a real robot end-to-end (`URLab.Mink.LiveIK.Converges`).
- **Next (the real integration):** the `UMjArticulationController` version above — ctrl-driven,
  task list composed in `Bind`, config-exposed for bridge + Blueprint. That's the thing that
  generalises; the demo just proved the port drives a robot.
