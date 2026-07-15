#!/usr/bin/env python3
"""TidyBot mobile manipulation v1 — sequential drive-then-reach over the bridge.

Phase A (LIVE): the nav stack drives the base around an obstacle wall to a
staging pose near a table (arm passed through, holding home).
SWITCH: set_active_controller repoints the bound controller to the mink IK.
Phase B (DIRECT): stream an EE target above the table in step batches (the
tidybot_mink_demo pattern); accept when the pinch_site tracks it to tolerance.

Spec: docs/superpowers/specs/2026-07-15-mobile-manip-v1-design.md.
Run with the bridge venv's python, editor open (the user drives Simulate):
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
        Scripts/demos/tidybot_mobile_manip_demo.py
Exits nonzero on any phase failure.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

# Reuse the mink demo's proven helpers (same directory).
from tidybot_mink_demo import (
    Tracker,
    build_controller_payload,
    stream_target,
)

MODEL_XML = (
    Path(__file__).resolve().parents[1]
    / "mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"
)
ACTOR_ID = "mm_tidybot"  # unique — never tidybot_0 / nav_tidybot

# Scene (MuJoCo metres). Wall at x=2 forces the detour (nav-demo layout);
# the table sits past the staging pose. Table top at z = 0.70.
FLOOR = dict(actor_id="mm_floor", location=(0.0, 0.0, -0.1), size=(20.0, 20.0, 0.2))
OBSTACLES = [
    dict(actor_id="mm_wall", location=(2.0, 0.0, 0.4), size=(0.4, 3.0, 0.8)),
    dict(actor_id="mm_block_a", location=(3.0, -1.6, 0.4), size=(0.6, 0.6, 0.8)),
    dict(actor_id="mm_block_b", location=(1.0, 1.8, 0.4), size=(0.6, 0.6, 0.8)),
]
TABLE = dict(actor_id="mm_table", location=(5.2, 0.0, 0.35), size=(0.8, 1.2, 0.7))
NAV_BOUNDS = dict(center=(0.0, 0.0, 0.5), extent=(12.0, 12.0, 2.0))

STAGING = (4.2, 0.0)            # nav goal: in front of the table
DETOUR_MIN_Y = 1.0              # same detour assertion as the nav demo
NAV_TIMEOUT_S = 90.0
NAV_POLL_S = 0.5

# Reach target: above the table's near edge. Reachable from staging with the
# arm (~0.9 m envelope); the lazy base may legally shuffle to assist.
REACH_TARGET = np.array([4.9, 0.0, 0.85])
REACH_BATCHES = 120             # x STRIDE steps of ramp + settle
STRIDE = 10                     # steps per target update (0.02 s @ dt 0.002)
RAMP_BATCHES = 80               # target interpolates over these, then holds
ACCEPT_EE_ERR = 0.02            # m, sustained over the final batches
ACCEPT_SUSTAIN = 10             # final batches that must all be within tol


def log(msg: str) -> None:
    print(f"[mm-demo] {msg}", flush=True)


def fail(msg: str) -> None:
    print(f"[mm-demo] FAIL: {msg}", flush=True)
    sys.exit(1)


def robot_xy(client, actor_id: str):
    rows = client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True)
    for r in rows:
        if r.actor_id == actor_id:
            return r.location[0], r.location[1]
    raise RuntimeError(f"{actor_id!r} not found in the PIE world")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="tcp://localhost")
    ap.add_argument("--port", type=int, default=5560,
                    help="direct step-server port (mink-demo default)")
    ap.add_argument("--keep-open", action="store_true")
    args = ap.parse_args()

    # step_mode="direct" wires the step transport phase B needs; phase A
    # overrides to live via set_mode after PIE starts (the server-side mode
    # is what gates stepping — the client construct just picks transports).
    client = URLabClient(args.host, step_mode="direct", step_port=args.port)
    client.connect()
    log("connected")

    try:
        # ---- Scene (isolated level; see navigation-demo.md warning) --------
        log("creating MobileManipDemo level + scene")
        client.scene.create_level(name="MobileManipDemo")
        client.scene.spawn_box(**FLOOR)  # UE-only floor (MJCF plane = physics)
        for ob in OBSTACLES + [TABLE]:
            client.scene.spawn_box(**ob)
            client.outliner.add_quick_convert(target=ob["actor_id"], static=True)
        r = client.scene.spawn_nav_bounds(**NAV_BOUNDS, agent_radius=55.0, timeout_s=30.0)
        if not r.get("nav_data_present"):
            fail(f"navmesh did not bake: {r}")

        # ---- Robot: both controller stacks, pre-PIE ------------------------
        bp = client.scene.import_xml(path=str(MODEL_XML))
        client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
        client.scene.ensure_manager()
        stack = client.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
        log(f"nav stack: created={stack.get('created')}")
        # add_controller lives in the `ik` namespace, NOT `scene` (verified
        # against RegEditor(TEXT("add_controller"), TEXT("ik"), ...) in
        # Source/URLabEditor/Private/MjEditorOpHandlers.cpp — the sibling
        # add_nav_stack registration is deliberately `scene`, and a comment
        # there calls out add_controller as the contrasting `ik` case).
        ik = client.ik.add_controller(
            target=ACTOR_ID, **build_controller_payload(draw_target=True,
                                                        lazy_base_cost=5.0))
        log(f"ik controller: was_existing={ik.get('was_existing')}")

        # ---- Phase A: drive (live) ------------------------------------------
        log("entering PIE (sim.start)")
        client.sim.start(timeout_s=180.0)
        try:
            client.runtime.set_mode("live")
        except URLabRPCError as exc:
            log(f"set_mode(live) note: {exc}")
        client.runtime.set_paused(paused=False)  # bridge live is never auto-unpaused
        time.sleep(2.0)  # let physics settle on spawn (mirrors tidybot_nav_demo.py)

        nav_name = None
        for row in client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True):
            if row.actor_id == ACTOR_ID:
                nav_name = row.name
                break
        if nav_name is None:
            fail("robot not found in PIE world")
        log(f"articulation name={nav_name!r}")

        # Two controllers are attached; the initial bind is order-dependent.
        r = client.runtime.set_active_controller(articulation=nav_name,
                                                 controller="base_drive")
        log(f"active controller -> {r.get('active')} (was_active={r.get('was_active')})")
        if r.get("active") != "MjBaseDriveController":
            fail(f"expected base drive active, got {r}")

        log(f"Phase A: set_nav_goal {STAGING}")
        r = client.runtime.set_nav_goal(articulation=nav_name,
                                        x=STAGING[0], y=STAGING[1])
        if not r.get("accepted"):
            fail(f"staging goal rejected: {r}")
        max_abs_y, state = 0.0, "navigating"
        deadline = time.time() + NAV_TIMEOUT_S
        while time.time() < deadline:
            s = client.runtime.get_nav_status(articulation=nav_name)
            state = s["state"]
            x, y = robot_xy(client, ACTOR_ID)
            max_abs_y = max(max_abs_y, abs(y))
            log(f"  A: {state:<10} dist={s['distance_to_goal']:.2f} pos=({x:.2f},{y:.2f})")
            if state in ("arrived", "failed"):
                break
            time.sleep(NAV_POLL_S)
        if state != "arrived":
            fail(f"Phase A did not arrive (state={state})")
        if max_abs_y < DETOUR_MIN_Y:
            fail(f"Phase A no detour (max |y|={max_abs_y:.2f})")
        log(f"Phase A ARRIVED (detour max |y|={max_abs_y:.2f} m)")

        # ---- Switch ----------------------------------------------------------
        r = client.runtime.set_active_controller(articulation=nav_name,
                                                 controller="mink_ik")
        if r.get("active") != "MjMinkIKController":
            fail(f"switch to IK failed: {r}")
        log("switched to MjMinkIKController")

        # ---- Phase B: reach (direct) ----------------------------------------
        # Entering PIE / driving live may have left the server off direct
        # stepping; re-assert + unpause (mirrors tidybot_mink_demo.py's
        # post-sim.start re-assertion sequence).
        try:
            mode = client.runtime.set_mode("direct")
            log(f"step mode -> {mode}")
        except Exception as exc:
            log(f"set_mode(direct) warning: {exc}")
        client.runtime.set_paused(False)

        # configure_controller (which stream_target calls under the hood)
        # targets the ACTIVE (bound) controller — HandleConfigureController
        # resolves Art->GetActiveController() first (RpcDispatcher.cpp),
        # falling back to first-match only when nothing is bound. Because we
        # switched the active controller to mink IK just above, the streamed
        # targets below land on the mink controller even though the base-drive
        # is also attached. (Before that fix, first-match could have hit the
        # base-drive and silently dropped the IK target — see commit 67e5dfa.)
        tracker = Tracker(client)  # takes only `client`; resolves pinch_site/
                                   # joint_x/joint_y itself from client.model
        p0, q0 = tracker.ee_pose()
        log(f"Phase B: seed target = current EE pose {np.round(p0, 3)}")
        stream_target(client, nav_name, p0, q0)
        client.step(n_steps=STRIDE)

        ok_streak = 0
        for k in range(REACH_BATCHES):
            a = min(1.0, k / float(RAMP_BATCHES))
            tgt = (1.0 - a) * p0 + a * REACH_TARGET
            stream_target(client, nav_name, tgt, q0)
            client.step(n_steps=STRIDE)
            ee, _ = tracker.ee_pose()
            err = float(np.linalg.norm(ee - REACH_TARGET))
            if k % 10 == 0 or a >= 1.0:
                log(f"  B: batch {k:3d} a={a:.2f} ee_err={err:.4f}")
            if not np.all(np.isfinite(client.data.qpos)):
                fail(f"non-finite qpos at batch {k}")
            ok_streak = ok_streak + 1 if (a >= 1.0 and err < ACCEPT_EE_ERR) else 0
            if ok_streak >= ACCEPT_SUSTAIN:
                break
        if ok_streak < ACCEPT_SUSTAIN:
            fail(f"Phase B did not settle within {ACCEPT_EE_ERR} m "
                 f"(streak={ok_streak}/{ACCEPT_SUSTAIN})")
        log(f"Phase B REACHED target (err<{ACCEPT_EE_ERR} m sustained x{ACCEPT_SUSTAIN})")

        log("ALL PHASES PASSED")
    finally:
        if not args.keep_open:
            try:
                client.sim.stop()
            except URLabRPCError:
                pass
        # close() reverts the server to live step mode before disconnecting —
        # important because this client promoted itself to 'direct' for Phase B.
        # Without it, --keep-open would leave the editor in direct/paused mode.
        try:
            client.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
