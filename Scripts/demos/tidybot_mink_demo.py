#!/usr/bin/env python3
"""TidyBot mink-IK bridge-driven demo — live-editor parity with mobile_tidybot.py.

Drives the Stanford TidyBot (planar holonomic base + 7-DoF Kinova arm + 2f85
gripper) through the URLab editor bridge, reproducing Kevin Zakka's
`mobile_tidybot.py` mink demo but with the IK solve running *inside* the Unreal
editor's physics thread (a `UMjMinkIKController`), not in a local Python loop.

This is the live-editor counterpart of the headless automation test
`URLab.MinkIK.TidyBot.ClosedLoopStable`
(Source/URLabEditor/Private/Tests/MjMinkIKControllerTests.cpp). It exercises the
layers the headless harness bypasses: PIE, the ZMQ bridge, the physics thread,
and real target streaming over `configure_controller`.

    Client package: `urlab_client` (external repo — editable install at
    /home/stuart/Unreal_Robotics/URLab_Bridge/src/urlab_client, venv at
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv).

Prerequisites
-------------
* The Unreal editor is open with the URLab plugin loaded and its bridge server
  listening (auto-starts by default on tcp://localhost:5559).
* Run with the bridge venv's Python:
      /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
          Scripts/demos/tidybot_mink_demo.py

What it does (phases)
---------------------
1. Import the merged scene `tidybot_scene_ue.xml`, spawn one robot, ensure an
   MjManager, attach + configure the mink controller (`add_controller`).
2. Enter PIE, verify sim options, reset to the `home` keyframe.
3. Seed the first target = the current pinch_site pose (substitutes for
   `mink.move_mocap_to_frame`), then stream the shared deterministic target
   script in large `step(n_steps=N)` batches, polling qpos/EE each phase.
4. Acceptance: near-reach tracking, far-circle base drive, no resets / NaN.
5. `fix_base` toggle: enable the base Damping task, command a far target, assert
   the base holds within 2 cm while the arm stretches; then toggle back.

Exits nonzero if any acceptance item fails.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover
    print("ERROR: mujoco not importable — run with the bridge venv's python.")
    raise

from urlab_client import URLabClient
from urlab_client.enums import ControlMode

# --------------------------------------------------------------------------- #
# Fixed demo constants — identical to the headless ClosedLoopStable test and to
# mobile_tidybot.py.
# --------------------------------------------------------------------------- #
MODEL_XML = (
    Path(__file__).resolve().parents[1]
    / "mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"
)
ACTOR_ID = "tidybot_0"
LEVEL_NAME = "TidybotMinkDemo"

FRAME_SITE = "pinch_site"
BASE_JOINTS = ["joint_x", "joint_y", "joint_th"]
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
DRIVE_JOINTS = BASE_JOINTS + ARM_JOINTS  # the example's 10 (no gripper)

NSTEPS = 2500
STRIDE = 10          # steps per target-update batch (0.02 s @ dt 0.002)
MAX_ITERS = 20
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4

REACH = np.array([0.0, 0.35, 0.15])
FAR = np.array([0.9, 0.0, 0.0])

# (step index -> tracking tolerance, m). Mirrors the headless checkpoints
# {299,899,999,1600,2200,2499}; batch-aligned to multiples of STRIDE here.
TRACK_TOL = {300: 0.02, 900: 0.02, 1000: 0.30, 1600: 0.10, 2200: 0.10, 2500: 0.10}

OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def target_pos(k: int, p0: np.ndarray) -> np.ndarray:
    """The shared deterministic target script (MuJoCo world metres)."""
    if k < 300:
        return p0
    if k < 400:
        return p0 + (k - 300) / 100.0 * REACH
    if k < 900:
        return p0 + REACH
    if k < 1000:
        s = (k - 900) / 100.0
        return p0 + (1.0 - s) * REACH + s * FAR
    theta = 2.0 * math.pi * (k - 1000) / 1200.0
    return p0 + FAR + np.array([0.4 * (math.cos(theta) - 1.0), 0.4 * math.sin(theta), 0.0])


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #
def log(msg: str) -> None:
    print(f"[demo] {msg}", flush=True)


def resolve_id_by_suffix(model, objtype, count: int, suffix: str) -> int:
    """Resolve a compiled MuJoCo object id by exact name, else by name suffix
    (UE import may prefix names)."""
    exact = mujoco.mj_name2id(model, objtype, suffix)
    if exact >= 0:
        return exact
    for i in range(count):
        nm = mujoco.mj_id2name(model, objtype, i)
        if nm and nm.endswith(suffix):
            return i
    return -1


class Tracker:
    """Reads EE (pinch_site) world pose and base displacement from the client's
    local model mirror, which the bridge refreshes from the server every step."""

    def __init__(self, client: URLabClient):
        self.client = client
        m = client.model
        self.site_id = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_SITE, m.nsite, FRAME_SITE)
        if self.site_id < 0:
            raise RuntimeError(f"site {FRAME_SITE!r} not found in compiled model")
        jx = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, "joint_x")
        jy = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, "joint_y")
        self.qx = m.jnt_qposadr[jx]
        self.qy = m.jnt_qposadr[jy]

    def _forward(self):
        mujoco.mj_forward(self.client.model, self.client.data)

    def ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        self._forward()
        d = self.client.data
        pos = np.array(d.site_xpos[self.site_id]).copy()
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.array(d.site_xmat[self.site_id]).reshape(9))
        return pos, quat

    def base_xy(self) -> np.ndarray:
        d = self.client.data
        return np.array([d.qpos[self.qx], d.qpos[self.qy]])


def build_controller_payload() -> dict:
    """The exact example stack, in add_controller JSON form (component refs by
    MjName). Matches the green headless test's FMinkTaskSpec setup."""
    return {
        "tasks": [
            {
                "kind": "frame",
                "frame": FRAME_SITE,
                "position_cost": 1.0,
                "orientation_cost": 1.0,
                "lm_damping": 1.0,
                "enabled": True,
            },
            {"kind": "posture", "cost": 1e-3, "joints": ARM_JOINTS, "enabled": True},
            # Base immobilisation task — off by default; the fix_base phase enables it.
            {"kind": "damping", "cost": 100.0, "joints": BASE_JOINTS, "enabled": False},
        ],
        "limits": [{"kind": "configuration"}],  # gain defaults to 0.95
        "drive_joints": DRIVE_JOINTS,
        "max_iters": MAX_ITERS,
        "pos_threshold": POS_THRESHOLD,
        "ori_threshold": ORI_THRESHOLD,
    }


def stream_target(client: URLabClient, art_prefix: str, pos: np.ndarray, quat: np.ndarray,
                  task_enabled: Optional[List[bool]] = None) -> None:
    """Latch a manual IK target (and optionally per-task enables) via
    configure_controller. Manual target wins over the mocap body."""
    params = {
        "target_pos": [float(pos[0]), float(pos[1]), float(pos[2])],
        "target_quat": [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])],
    }
    if task_enabled is not None:
        params["task_enabled"] = task_enabled
    client._rpc_configure_controller(articulation=art_prefix, params=params)


def try_screenshot(client: URLabClient, art, tag: str) -> Optional[str]:
    """Best-effort: capture the robot's base camera to output/<tag>.png."""
    try:
        client.step(n_steps=1, include_cameras=True)
        cam = art.cameras.get("base") or (next(iter(art.cameras.values())) if art.cameras else None)
        if cam is None or cam.latest_frame is None:
            return None
        from PIL import Image

        frame = np.asarray(cam.latest_frame)
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = frame[:, :, :3]
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        path = OUTPUT_DIR / f"{tag}.png"
        Image.fromarray(frame.astype(np.uint8)).save(path)
        return str(path)
    except Exception as exc:  # pragma: no cover - screenshots are best-effort
        log(f"screenshot {tag!r} skipped: {exc}")
        return None


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description="TidyBot mink-IK bridge demo")
    ap.add_argument("--host", default="tcp://localhost")
    ap.add_argument("--port", type=int, default=5559)
    ap.add_argument("--no-screenshots", action="store_true")
    args = ap.parse_args()

    if not MODEL_XML.exists():
        log(f"FATAL: model not found: {MODEL_XML}")
        return 2

    results: List[Tuple[str, bool, str]] = []

    def record(name: str, ok: bool, detail: str) -> None:
        results.append((name, ok, detail))
        log(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")

    log(f"connecting to {args.host}:{args.port} (direct step mode)")
    client = URLabClient(args.host, step_mode="direct", step_port=args.port)
    try:
        client.connect()
        log(f"connected: urlab={client.urlab_version} mujoco={client.mujoco_version} "
            f"manager_present={client.manager_present}")

        # Clean slate: editor-time scene authoring needs PIE stopped (scene ops
        # target the editor world; a lingering PIE would use a stale actor).
        if client.manager_present:
            log("a PIE session is live — stopping it for a clean authoring pass")
            client.sim.stop()
            client.connect()

        # --- 1. Author the scene (editor-time) -------------------------------
        # Spawn into the currently-open editor level (PIE plays the live world,
        # so no create/save is required — and saving a template map can fail).
        log(f"current level: {client.scene.current_level()}")
        log("importing scene + spawning robot")
        bp = client.scene.import_xml(str(MODEL_XML))
        handle = client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
        log(f"spawned {ACTOR_ID}: {handle.actor_name} (was_existing={handle.was_existing})")

        log("ensuring MjManager")
        client.scene.ensure_manager()

        # --- 2. Attach + configure the mink controller (editor op) -----------
        log("add_controller (mink IK, example stack)")
        add_reply = client._rpc(
            "add_controller", build_controller_payload(), expected_op="add_controller_ok"
        )
        warnings = add_reply.get("warnings", [])
        log(f"add_controller_ok: tasks={add_reply.get('tasks')} "
            f"limits={add_reply.get('limits')} drive_joints={add_reply.get('drive_joints')} "
            f"warnings={warnings}")
        cfg_ok = (
            add_reply.get("tasks") == 3
            and add_reply.get("limits") == 1
            and add_reply.get("drive_joints") == 10
            and not warnings
        )
        record("controller_configured", cfg_ok,
               f"tasks=3 limits=1 drive_joints=10, warnings={len(warnings)}")
        if not cfg_ok:
            raise RuntimeError(f"add_controller misconfigured: {add_reply}")

        # --- 3. Enter PIE ----------------------------------------------------
        log("entering PIE (sim.start) — this can take a while on first compile")
        start = client.sim.start(timeout_s=180.0)
        if not start.is_ready:
            raise RuntimeError(f"PIE did not reach READY: {start.state} {start.compile_error}")
        log(f"PIE ready; articulations={list(client.articulations)}")

        art = client.articulations.get("tidybot")
        if art is None:
            arts = list(client.articulations.values())
            if not arts:
                raise RuntimeError("no articulation after PIE start")
            art = arts[0]
        art_prefix = art.prefix
        log(f"articulation prefix={art_prefix!r} control_mode={art.control_mode}")

        # Entering PIE reverts the server to its live default; re-assert direct
        # stepping (n_steps runs synchronously) and unpause so batches advance.
        try:
            mode = client.runtime.set_mode("direct")
            log(f"step mode -> {mode}")
        except Exception as exc:
            log(f"set_mode(direct) warning: {exc}")
        client.runtime.set_paused(False)

        # --- 4. Verify sim options survived the import -----------------------
        m = client.model
        opt = m.opt
        opts_ok = (
            int(opt.integrator) == int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
            and int(opt.cone) == int(mujoco.mjtCone.mjCONE_ELLIPTIC)
            and abs(opt.impratio - 10.0) < 1e-6
            and abs(opt.timestep - 0.002) < 1e-9
        )
        record("sim_options", opts_ok,
               f"integrator={int(opt.integrator)} cone={int(opt.cone)} "
               f"impratio={opt.impratio} timestep={opt.timestep}")

        # --- 5. Reset to home, keep the controller in the loop ---------------
        # The UE import may prefix compiled names; resolve the exact keyframe
        # name from the client's model mirror (identical to the server's mjb).
        kid = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_KEY, m.nkey, "home")
        home_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, kid) if kid >= 0 else "home"
        log(f"reset to home keyframe (resolved name={home_name!r})")
        client.reset(keyframe_name=home_name)
        art.control_mode = ControlMode.UE_CONTROLLER  # keep the mink solve live (not raw)

        tracker = Tracker(client)
        p0, q0 = tracker.ee_pose()
        log(f"seed target = current pinch_site pose  P0={np.round(p0, 4)}  Q0={np.round(q0, 4)}")
        stream_target(client, art_prefix, p0, q0)  # substitutes move_mocap_to_frame

        # --- 6. Closed loop: stream target -> step batches -> poll -----------
        log(f"running {NSTEPS} steps in batches of {STRIDE}")
        prev_sim_time = -1.0
        diverged = False
        nan_detail = ""
        track_records = {}
        base_at_reach = None
        eval_pts = sorted(TRACK_TOL)
        screenshot_ks = {900, 2500}
        shots: List[str] = []

        k = 0
        while k < NSTEPS:
            tgt = target_pos(k, p0)
            stream_target(client, art_prefix, tgt, q0)
            client.step(n_steps=STRIDE)
            k += STRIDE

            # Monotonic sim time (no silent reset)
            st = client.sim_time
            if st <= prev_sim_time:
                diverged = True
                nan_detail = f"sim_time not advancing at K={k}: {st} <= {prev_sim_time}"
                break
            prev_sim_time = st

            # Finiteness
            if not np.all(np.isfinite(client.data.qpos)) or not np.all(np.isfinite(client.data.qvel)):
                diverged = True
                nan_detail = f"non-finite qpos/qvel at K={k}"
                break

            if k == 900:
                base_at_reach = float(np.linalg.norm(tracker.base_xy()))

            if k in eval_pts:
                ee, _ = tracker.ee_pose()
                err = float(np.linalg.norm(ee - target_pos(k, p0)))
                track_records[k] = err
                log(f"  K={k:4d}  track_err={err:.4f}  tol={TRACK_TOL[k]:.2f}  "
                    f"base|xy|={np.linalg.norm(tracker.base_xy()):.3f}")

            if not args.no_screenshots and k in screenshot_ks:
                s = try_screenshot(client, art, f"phase_k{k}")
                if s:
                    shots.append(s)
                    prev_sim_time = client.sim_time  # screenshot advanced 1 step

        # --- 7. Acceptance: no resets / NaN ----------------------------------
        record("no_resets_no_nan", not diverged,
               nan_detail or f"sim_time monotonic to {prev_sim_time:.3f}s, all finite")

        if not diverged:
            # Near-reach tracking (settled reach hold @ K=900) + base ~still there.
            reach_err = track_records.get(900, float("nan"))
            reach_ok = reach_err <= TRACK_TOL[900]
            base_still = (base_at_reach is not None and base_at_reach < 0.2)
            record("near_reach_tracking", reach_ok and base_still,
                   f"EE err @K900={reach_err:.4f}<= {TRACK_TOL[900]}, base|xy|@K900={base_at_reach:.3f}<0.2")

            # All tracking checkpoints within tolerance.
            track_ok = all(track_records.get(k, 9e9) <= TRACK_TOL[k] for k in eval_pts)
            record("tracking_all_checkpoints", track_ok,
                   "; ".join(f"K{k}={track_records.get(k, float('nan')):.4f}/{TRACK_TOL[k]}"
                             for k in eval_pts))

            # Far-circle base drive: base must have translated by the end.
            base_end = float(np.linalg.norm(tracker.base_xy()))
            record("far_circle_base_drive", base_end > 0.2,
                   f"base|xy|@end={base_end:.3f} > 0.2")

            # Live-defect diagnosis: if the EE never left home while targets
            # moved, the controller's Frame/DriveJoints component refs did not
            # survive PIE-world duplication (see editor log:
            # "[MinkIK] ... Frame task has no resolved frame component — skipped"
            # / "Bound: 2 task(s) ... 0 driven actuator(s)").
            ee_end, _ = tracker.ee_pose()
            ee_disp = float(np.linalg.norm(ee_end - p0))
            if not track_ok and ee_disp < 0.05 and base_end < 0.05:
                log("DIAGNOSIS: EE stayed at home despite moving targets and the "
                    "base never drove — the live UMjMinkIKController bound with the "
                    "Frame + DriveJoints component references NULL. PIE-world "
                    "duplication dropped the instance-component's UPROPERTY object "
                    "refs (add_controller attaches an instance component; its "
                    "pointers into the SCS-built site/joints do not remap into the "
                    "play world). Check the editor log for '0 driven actuator(s)'. "
                    "This is a live-only defect the same-world headless test cannot see.")

            # --- 8. fix_base toggle ------------------------------------------
            log("fix_base ON: enable base Damping task, command a far/high target")
            b0 = tracker.base_xy()
            ee_before, _ = tracker.ee_pose()
            fixed_target = p0 + FAR + np.array([0.4, 0.0, 0.2])  # up and out -> arm must stretch
            stream_target(client, art_prefix, fixed_target, q0,
                          task_enabled=[True, True, True])
            for _ in range(40):  # 400 steps
                client.step(n_steps=STRIDE)
            b1 = tracker.base_xy()
            ee_after, _ = tracker.ee_pose()
            base_hold = float(np.linalg.norm(b1 - b0))
            arm_stretch = float(np.linalg.norm(ee_after - ee_before))
            record("fix_base_hold", base_hold < 0.02,
                   f"base moved {base_hold*100:.2f} cm (<2 cm) while EE moved {arm_stretch*100:.1f} cm")

            # Toggle back off.
            log("fix_base OFF: restore base Damping disabled")
            stream_target(client, art_prefix, target_pos(2500, p0), q0,
                          task_enabled=[True, True, False])
            client.step(n_steps=STRIDE)

            if not args.no_screenshots:
                s = try_screenshot(client, art, "phase_fixbase")
                if s:
                    shots.append(s)

        if shots:
            log(f"screenshots: {shots}")

        # --- 9. Teardown -----------------------------------------------------
        log("stopping PIE")
        try:
            client.sim.stop()
        except Exception as exc:
            log(f"sim.stop warning: {exc}")

    finally:
        client.close()

    # --- summary ------------------------------------------------------------
    print("\n==================== ACCEPTANCE ====================")
    all_ok = True
    for name, ok, detail in results:
        all_ok = all_ok and ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    print("===================================================")
    print("RESULT:", "ALL PASS" if all_ok else "FAILURES PRESENT")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
