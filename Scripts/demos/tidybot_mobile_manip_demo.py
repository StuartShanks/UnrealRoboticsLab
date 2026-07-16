#!/usr/bin/env python3
"""TidyBot mobile manipulation v1 — sequential drive-then-reach over the bridge.

Phase A (LIVE): the nav stack drives the base around an obstacle wall to a
staging pose (arm passed through, holding home).
SWITCH: set_active_controller repoints the bound controller to the mink IK.
Phase B (LIVE): stream the EE goal once and let the mink track it as a
real-time, velocity-governed tracker (max_iters=1 + QP velocity limits — see
manip_controller_payload); accept when the pinch_site settles within
tolerance. v1 reaches a free-space pose; a table-surface / whole-body reach
is v1.1 (see REACH_TARGET note).

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
    ARM_JOINTS,
    BASE_JOINTS,
    Tracker,
    build_controller_payload,
    stream_target,
)

MODEL_XML = (
    Path(__file__).resolve().parents[1]
    / "mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"
)
ACTOR_ID = "mm_tidybot"  # unique — never tidybot_0 / nav_tidybot

# Scene (MuJoCo metres). Wall at x=2 forces the detour (nav-demo layout).
FLOOR = dict(actor_id="mm_floor", location=(0.0, 0.0, -0.1), size=(20.0, 20.0, 0.2))
OBSTACLES = [
    dict(actor_id="mm_wall", location=(2.0, 0.0, 0.4), size=(0.4, 3.0, 0.8)),
    dict(actor_id="mm_block_a", location=(3.0, -1.6, 0.4), size=(0.6, 0.6, 0.8)),
    dict(actor_id="mm_block_b", location=(1.0, 1.8, 0.4), size=(0.6, 0.6, 0.8)),
]
NAV_BOUNDS = dict(center=(0.0, 0.0, 0.5), extent=(12.0, 12.0, 2.0))

STAGING = (4.2, 0.0)            # nav goal the base drives to
DETOUR_MIN_Y = 1.0
NAV_TIMEOUT_S = 90.0
NAV_POLL_S = 0.5

# Reach target: a free-space pose ~0.6 m from the arm shoulder at the arrival
# pose — reachable arm-only (the base holds). v1's bar is "reach a pose".
# Reaching a target ON a table needs the base closer than the 55 cm nav
# clearance allows, so table-surface / whole-body reach is v1.1.
REACH_TARGET = np.array([4.5, 0.0, 1.1])
ACCEPT_EE_ERR = 0.02           # m, sustained
REACH_SETTLE_S = 15.0          # generous ceiling; the reach itself takes ~2-4 s
AGENT_RADIUS = 55.0            # navmesh obstacle clearance (cm)

# Velocity governance (replaces the old 6 s goal ramp). max_iters=1 makes the
# mink a real-time tracker — ONE Newton step per physics step — so the QP
# velocity limits below are the true wall-clock pace. (The default max_iters=20
# is the golden-parity convergence loop: it integrates up to 20 velocity-
# limited iterations PER STEP, i.e. up to 20x the cap — the actual mechanism
# behind v1's "instant clip", which the ramp only masked. Carrot-probe finding,
# 2026-07-16.)
BASE_MAX_VEL = 0.6             # m/s — matches UMjNavComponent.MaxSpeed
ARM_MAX_VEL = 0.5              # rad/s — deliberate but not sluggish (0.3 read
                               # as too slow in the step-2 live runs)


def manip_controller_payload() -> dict:
    """The mink stack for this demo: the example payload + real-time tracking
    (max_iters=1) + QP velocity limits on base and arm. Import this from
    probe/validation scripts so they exercise the exact committed stack."""
    payload = build_controller_payload(draw_target=True, lazy_base_cost=5.0)
    payload["max_iters"] = 1
    payload["limits"] += [
        {"kind": "velocity", "joints": BASE_JOINTS, "max_velocity": BASE_MAX_VEL},
        {"kind": "velocity", "joints": ARM_JOINTS, "max_velocity": ARM_MAX_VEL},
    ]
    return payload


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
    ap.add_argument("--port", type=int, default=5559,
                    help="direct step-server port (bridge StepPort default 5559)")
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
        client.scene.create_level(name="MobileManipDemo", force_overwrite=True)
        client.scene.spawn_light(actor_id="mm_sun", kind="directional", intensity=6.0)
        client.scene.spawn_box(**FLOOR)  # UE-only floor (MJCF plane = physics)
        for ob in OBSTACLES:
            client.scene.spawn_box(**ob)
            client.outliner.add_quick_convert(target=ob["actor_id"], static=True)
        r = client.scene.spawn_nav_bounds(**NAV_BOUNDS, agent_radius=AGENT_RADIUS, timeout_s=30.0)
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
        ik = client.ik.add_controller(target=ACTOR_ID, **manip_controller_payload())
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
        # find_actors returns the ACTOR ROOT (static at spawn), not the MuJoCo
        # base body, so max|y| can't measure the detour from here — arrival
        # around the wall confirms the path. (A base-body pose readback op would
        # let us assert the detour numerically; that's a follow-up.)
        if max_abs_y < DETOUR_MIN_Y:
            log(f"Phase A: detour unmeasurable via find_actors (actor-root); "
                f"arrival confirms the path. max|y|={max_abs_y:.2f}")
        log("Phase A ARRIVED at staging")

        # ---- Switch ----------------------------------------------------------
        r = client.runtime.set_active_controller(articulation=nav_name,
                                                 controller="mink_ik")
        if r.get("active") != "MjMinkIKController":
            fail(f"switch to IK failed: {r}")
        log("switched to MjMinkIKController")

        # ---- Phase B: reach (LIVE — velocity-governed) -----------------------
        # The controller runs as a real-time tracker (max_iters=1, see
        # manip_controller_payload), so we stream the FINAL goal once and the QP
        # velocity limits pace the reach — no wall-clock goal ramp needed (the
        # old 6 s ramp was masking the max_iters=20 per-step convergence loop).
        # Phase B runs live so the viewport renders continuously; direct mode is
        # used only for synced qpos reads (seed + acceptance).
        #
        # configure_controller (which stream_target uses) targets the ACTIVE
        # controller (GetActiveController first; RpcDispatcher.cpp, commit 67e5dfa),
        # so the streamed goals land on the mink IK even with the base-drive
        # attached — the first time two controllers coexist on one robot.
        tracker = Tracker(client)  # takes only `client`; resolves pinch_site itself

        def synced_ee():
            """Dip to direct, step once to sync client.data, read EE, resume live."""
            client.runtime.set_mode("direct")
            client.step(n_steps=1)
            p, q = tracker.ee_pose()
            client.runtime.set_mode("live")
            client.runtime.set_paused(paused=False)
            return p, q

        p0, q0 = synced_ee()  # seed read; home EE orientation becomes the goal quat
        log(f"Phase B: EE={np.round(p0, 3)} — streaming goal {REACH_TARGET}; "
            f"pace governed by ARM_MAX_VEL={ARM_MAX_VEL} rad/s")
        stream_target(client, nav_name, REACH_TARGET, q0)

        # Settle + verify (synced reads until sustained acceptance or timeout).
        ok_streak, err = 0, 9.9
        deadline_b = time.time() + REACH_SETTLE_S
        while time.time() < deadline_b:
            time.sleep(0.4)
            ee, _ = synced_ee()
            err = float(np.linalg.norm(ee - REACH_TARGET))
            log(f"  B settle: ee={np.round(ee, 3)} err={err:.4f}")
            stream_target(client, nav_name, REACH_TARGET, q0)  # re-assert after dip
            ok_streak = ok_streak + 1 if err < ACCEPT_EE_ERR else 0
            if ok_streak >= 3:
                break
        if ok_streak < 3:
            fail(f"Phase B did not settle within {ACCEPT_EE_ERR} m (last err={err:.4f})")
        log(f"Phase B REACHED target (err<{ACCEPT_EE_ERR} m, sustained)")

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
