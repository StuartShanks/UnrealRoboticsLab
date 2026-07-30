# MolmoSpaces–URLab Risk-First Puppet Mirror Design

**Status:** Approved in discussion on 2026-07-30; awaiting written-spec review.

**Supersedes:** The stepping boundary and task order in
`docs/superpowers/plans/2026-07-30-molmospaces-puppet-parity.md`.

## Objective

Establish that an official MolmoSpaces Franka episode can run unchanged while
URLab/Unreal mirrors its MuJoCo state in puppet mode.

The first milestone is infrastructure validation. It does not claim a research
contribution and does not yet include a VLA, AfterAct interventions, URLab
direct/live physics, or Unreal images as policy observations.

## Design principles

1. Test fatal assumptions before writing integration machinery.
2. Keep MolmoSpaces as the sole physics and observation authority.
3. Make URLab synchronization observational: enabling the mirror must not
   alter the Molmo trajectory.
4. Minimize modifications to MolmoSpaces core code.
5. Stop on incompatibility rather than hiding it with state remapping,
   disabled version checks, or partial-scene substitutions.

## Authoritative ownership

```text
MolmoSpaces
├── owns MjModel and MjData
├── calls env.step(n)
├── performs native object/cache/camera bookkeeping
└── produces policy observations

URLab Bridge
├── validates the external model against the URLab server model
├── serializes the already-advanced external state
└── sends qpos, qvel, ctrl and time to URLab

URLab / Unreal
├── imports the compiled Molmo scene
├── receives state in puppet mode
└── renders a synchronized mirror without advancing physics
```

There must never be two independently advancing MuJoCo simulations in puppet
mode.

## Runtime boundary

The previous plan placed the `mj_step` call inside `URLabClient.step(n)`. The
revised boundary retains Molmo's native execution path:

```python
client.attach_puppet_simulation(env.mj_model, env.current_data)

env.step(n)                   # Molmo owns physics and native bookkeeping
client.push_puppet_state()    # Transmit current state without integration
```

Reset follows the same rule:

```python
env.reset()
client.push_puppet_state()
```

`push_puppet_state()` is an explicit puppet-only API. It must:

- reject direct, live, and auto modes;
- require an attached external `MjModel` and `MjData`;
- never call `mj_step`;
- serialize the attached data's `qpos`, `qvel`, `ctrl`, and `time`;
- make the URLab server copy those values into its mirror and call
  `mj_forward`; and
- return the normal URLab observations and synchronization counter.

The existing internal zero-step puppet path can implement this operation, but
the public connector must not expose `client.step(0)` as the research-facing
API. That call has different semantics outside puppet mode.

For the first parity experiment, this synchronization can be explicit in the
smoke runner. An automatic wrapper or sampler hook is introduced only after
manual parity succeeds.

## Repository boundary

Clone the personal MolmoSpaces fork to:

```text
/home/stuart/Unreal_Robotics/molmospaces
```

Remotes and branch:

```text
origin   git@github.com:StuartShanks/molmospaces.git
upstream https://github.com/allenai/molmospaces.git
branch   feat/urlab-puppet
```

The Molmo-specific integration belongs in:

```text
molmo_spaces/integrations/urlab/
```

The generic external-puppet attachment belongs in:

```text
/home/stuart/Unreal_Robotics/URLab_Bridge/src/urlab_client/
```

Dependency direction:

```text
MolmoSpaces integration → URLab Bridge
URLab Bridge             → no MolmoSpaces dependency
```

The UnrealRoboticsLab C++ plugin is unchanged during the first milestone.

## Risk-first execution gates

### Gate 1: dependency compatibility

Demonstrate that the required MolmoSpaces CPU path and focused upstream tests
run with the exact MuJoCo version required by URLab Bridge.

Initial target:

```text
mujoco == 3.8.1
```

If the version upgrade breaks MolmoSpaces, stop and design process/version
isolation before changing either repository further.

### Gate 2: complete-scene import

Before implementing the connector:

1. load the first official Franka mini-benchmark episode;
2. compile its complete MolmoSpaces scene;
3. export the compiled MJCF;
4. import that MJCF into URLab;
5. start PIE; and
6. obtain the server's compiled model through the normal handshake.

This gate must use the complete episode scene. A robot-only or simplified
scene does not answer the feasibility question.

If URLab cannot import and compile the complete scene, stop. Preserve the
exported MJCF and URLab compiler log. Scene-import generalization becomes a
separate project.

### Gate 3: state-layout compatibility

The external Molmo model and URLab server model must agree on:

- `nq`, `nv`, `na`, `nu`, `njnt`, and `nbody`;
- joint types and qpos/dof addresses;
- body-parent structure;
- actuator transmissions;
- ordered joint, body, and actuator identities after tolerating URLab's
  namespace prefixes.

Do not add a silent permutation or partial-state mapping to pass this gate.

### Gate 4: observational parity

Create two executions from the same initial state and controls:

```text
control: native Molmo only
treatment: native Molmo + URLab state push after each step
```

Run 100 physics steps and compare:

- complete Molmo `qpos`;
- complete Molmo `qvel`;
- controls;
- simulation time; and
- reset state.

The connector passes only if enabling URLab mirroring leaves Molmo's
trajectory unchanged. URLab must also report the pushed time and remain
responsive and visibly synchronized.

## Minimal implementation after the gates

Only after complete-scene import succeeds:

1. add a generic external-puppet attachment to URLab Bridge;
2. preserve `client.model` and `client.data` as the server-model mirror;
3. store external `MjModel`/`MjData` references separately;
4. add the guarded `client.push_puppet_state()` operation;
5. add typed compatibility failures; and
6. add offline bridge tests plus the one-episode live parity runner.

The first passing milestone does not require:

- factoring `CPUMujocoEnv._after_physics_step`;
- replacing Molmo's native `env.step`;
- adding a custom task sampler;
- modifying Unreal C++;
- running a VLA; or
- using Unreal camera images.

## Automation after parity

After the manual parity gate passes, add the smallest useful automation:

- a scene-session helper for export, import, PIE lifecycle, and cleanup;
- a thin mirrored-environment wrapper or one environment-construction hook;
  and
- a JSON episode entry point.

The wrapper must delegate physics to the native Molmo environment and push the
resulting state afterwards. It must not move `mj_step` into URLab Bridge.

## Research progression

Once the mirror is observationally neutral:

1. run a frozen public OpenPI VLA using native Molmo observations;
2. verify that URLab mirroring still does not alter the trajectory;
3. add controlled outcome ambiguity, occlusion, and physical disturbances;
4. instrument post-action evidence and durable belief writes;
5. introduce delayed decisions that consume those beliefs; and
6. perform causal interventions on incorrect stored claims.

The connector enables these experiments. The publishable AfterAct
contribution remains the causal evaluation of post-action belief formation,
not the connector itself.

## Success criterion

Proceed beyond infrastructure only when one official Franka episode:

- imports completely into URLab;
- runs through Molmo's unmodified native step path;
- mirrors 100 steps and reset into Unreal;
- satisfies model-layout checks;
- produces no treatment-versus-control trajectory difference; and
- preserves Molmo's native policy observations.
