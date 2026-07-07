# TidyBot mink-IK Demo (bridge-driven)

Reproduce Kevin Zakka's [`mobile_tidybot.py`](https://github.com/kevinzakka/mink)
mink demo **live in the Unreal editor**: the Stanford TidyBot (planar holonomic
base + 7-DoF Kinova arm + Robotiq 2f85 gripper) tracks a Cartesian target with
differential IK, except the solve runs *inside* Unreal's physics thread (a
`UMjMinkIKController`) instead of a local Python loop. Python only streams targets
and reads state over the bridge.

!!! note "Headless vs live"
    The headless automation test `URLab.MinkIK.TidyBot.ClosedLoopStable`
    (`Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp`) proves the
    solver + import fidelity deterministically. **This demo adds the layers the
    harness bypasses:** PIE, the ZMQ bridge, the real physics thread, and target
    streaming over `configure_controller`.

## What you should see

The base holds roughly still while the arm reaches to a near target
(`REACH = (0, 0.35, 0.15)` m); then the target ramps out to `FAR = (0.9, 0, 0)` m
and traces a 0.4 m circle, and the **base drives** across the floor to keep the
end-effector on the circle. No sim resets, no NaNs. In the headless test tracking
error stays 0.007-0.24 m.

!!! danger "Known live-layer limitation (as of 2026-07-07)"
    Over the **bridge PIE path**, the `add_controller` route does **not** yet
    track live. `add_controller` attaches the controller as a runtime *instance
    component*; when the bridge enters PIE (`EPlaySessionWorldType::PlayInEditor`)
    the world is duplicated and the controller's `Frame` + `DriveJoints`
    `UPROPERTY` object references are **dropped** (they point into the editor
    actor's SCS-built site/joints and do not remap into the play world). The
    controller then binds with **0 tasks that need a frame / 0 driven actuators**
    (editor log: `[MinkIK] ... Frame task has no resolved frame component —
    skipped` / `Bound: 2 task(s) ... 0 driven actuator(s)`), so it just holds the
    `home` `ctrl` — the arm freezes at home and the base never drives. Everything
    *around* the IK is correct live (import fidelity, PIE, direct stepping, no
    NaN, `fix_base` path). The headless test binds in the **same** world, so it
    never hits this. Fixing it needs the controller baked into the Blueprint
    (`add_controller` `to_blueprint=true`, not yet implemented) or the controller
    resolving `Frame`/`DriveJoints` **by name at Bind** — both production C++
    changes. See `docs/superpowers/specs/2026-07-07-tidybot-nan-findings.md`
    (Live editor validation) for the full evidence.

## Prerequisites

1. The Unreal editor is open with the URLab plugin loaded. The bridge server
   **auto-starts** by default and listens on `tcp://localhost:5559`
   (`[URLabBridge] AutoStart=True`). No PIE needed to start — the script enters
   PIE itself.
2. The merged scene fixture exists:
   `Scripts/mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml` (a
   single-`<worldbody>` merge of Kevin's `scene.xml` — the importer fatal-crashes
   on multi-`<worldbody>` include-style scenes).
3. The `urlab_client` Python package + its deps (mujoco, pyzmq, msgpack, numpy,
   Pillow). The bridge repo ships a ready venv:
   `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv`.

## Run

With the editor open (bridge up), from the plugin root:

```bash
/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
    Scripts/demos/tidybot_mink_demo.py
```

Flags: `--host tcp://localhost` (default), `--port 5559` (default),
`--no-screenshots` (skip camera captures).

The script:

1. `apply_scene` — imports `tidybot_scene_ue.xml`, spawns one robot, saves a
   `TidybotMinkDemo` level, and `ensure_manager`.
2. `add_controller` — attaches + configures the `UMjMinkIKController` over RPC
   with the exact example stack (below).
3. `sim.start` — enters PIE, verifies sim options, `reset(keyframe_name="home")`.
4. Seeds the first target = the current `pinch_site` pose (substitutes for
   `mink.move_mocap_to_frame`), then streams the deterministic target script in
   large `step(n_steps=N)` batches, polling qpos / EE each phase.
5. Runs the `fix_base` toggle, prints a per-item acceptance table, exits nonzero
   on any failure.

Camera screenshots (robot base cam) land in `Scripts/demos/output/`
(git-ignored).

### The controller stack (matches the headless test exactly)

| Element | Value |
|---|---|
| Frame task | `pinch_site`, position_cost 1, orientation_cost 1, lm_damping 1 |
| Posture task | `joint_1..7`, cost 1e-3 |
| Damping task | `joint_x/y/th`, cost 100, **disabled** (fix_base enables it) |
| Limit | ConfigurationLimit (gain 0.95) |
| DriveJoints | the 10: `joint_x/y/th` + `joint_1..7` (no gripper) |
| Solver | max_iters 20, pos/ori threshold 1e-4 |

## Target streaming — why not the mocap body

The demo streams targets with `configure_controller` `params.target_pos`
(`[x,y,z]`, MuJoCo world metres) + `target_quat` (`[w,x,y,z]`, wxyz). The
controller routes these to a **manual-target store that takes precedence over the
mocap body**.

!!! warning "Do not use `set_mocap_pose` for this"
    `UMjBody::TickComponent` re-stamps `mocap_pos` from the UE actor transform
    every tick, so `set_mocap_pose` writes are overwritten. The mocap body is
    **UE-authoritative** — good for gizmo-dragging `pinch_site_target` in the
    viewport, surprising over RPC.

## fix_base toggle

Enabling the base Damping task immobilises the base so the arm alone must reach:

```python
client._rpc_configure_controller(
    articulation="tidybot",
    params={"task_enabled": [True, True, True],   # Frame, Posture, Damping(base)
            "target_pos": [x, y, z], "target_quat": [w, x, y, z]},
)
```

The demo commands a far/high target and asserts the base stays within **2 cm**
while the end-effector stretches, then toggles Damping back off
(`task_enabled=[True, True, False]`). This is the scripted equivalent of pressing
**Enter** in `mobile_tidybot.py`.

## Troubleshooting (live-layer landmines)

| Symptom | Cause / fix |
|---|---|
| Controller silently does nothing; qpos mirror looks duplicated | Articulation fell into `control_mode=raw` (mink solve bypassed). The demo sets `art.control_mode = UE_CONTROLLER` explicitly. The client's `ControllerKind` now includes `mink_ik`, so this no longer forces raw. |
| Steps seem dropped / robot barely moves | Live-mode step requests drain ~one per physics tick. Use **`direct`** step mode with large `step(n_steps=N)` batches (the demo does). |
| `set_mocap_pose` has no effect | Expected — mocap is UE-authoritative (see above). Use `configure_controller` target streaming. |
| Sim NaNs at DOF 3/5 within ~0.1 s | The old default-class actuator-gain import bug (`kp=-1`). Fixed at HEAD; the demo's `sim_options` check and stable tracking confirm the compiled model is correct. |
| PIE never reaches READY | Check the editor log for a compile error; the merged scene must be a single `<worldbody>`. |

## See also

- [Differential IK (URLabMink)](mink.md) — the controller internals.
- [Mobile Base Navigation](navigation-demo.md) — the base-drive counterpart.
- Findings + live acceptance numbers:
  `docs/superpowers/specs/2026-07-07-tidybot-nan-findings.md`.
