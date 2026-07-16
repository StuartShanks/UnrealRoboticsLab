#!/usr/bin/env python3
"""TidyBot guarded table reach — collision avoidance inside the whole-body QP.

The third sibling in the mobile-manipulation demo set:
  tidybot_mobile_manip_demo.py   controller-swap architecture (base_drive -> mink)
  tidybot_twist_follow_demo.py   whole-body, swap-free (mink consumes the twist bus)
  THIS                           + CollisionAvoidance: a velocity-level no-contact
                                 guarantee on the guarded links, demonstrated by
                                 commanding the EE to a point INSIDE a table.

Phase A (LIVE): stowed twist-bus transit around the obstacle wall to staging —
the guards are armed but silent ("reach only in free space" is POLICY above the
controller: the QP tracks and avoids, it does not plan).
Phase B (LIVE): a 6 s goal ramp descends the arm (bounded instantaneous error —
streaming the far diagonal target directly parks the arm at its reach-envelope
singularity, stretched out before down), ending at a target deliberately inside
the table. WITHIN kinematic reach — so the stop is the CONSTRAINT, not the
envelope. Acceptance is a stable PLATEAU at a standoff inside the hold band:
the target can never be reached, by design.

The claim: any streamed target — sloppy, wrong, or adversarial — cannot produce
contact on the guarded links (velocity-level, enforced in every 500 Hz solve,
active during whole-body motion). NOT claimed: routing around obstacles.

Practice notes baked in (from the step-3 live debugging, 2026-07-16):
  - quick-converted obstacles compile as "<UE actor name>_MjBody"; the UE actor
    name is auto-generated (actor_id is only a tag) — capture "actor_name" from
    each spawn_box reply. A group that resolves to 0 geoms SKIPS the limit
    (warning only in the bind log) — this demo asserts the reply chain instead.
  - guard the FULL kinematic chain: one unguarded link contacting at 1e6-gain
    actuator stiffness explodes the physics (QACC -> MuJoCo auto-reset).
  - the gripper is guarded as an ENVELOPE (standoff covering the finger extent)
    rather than enumerating the 4-bar linkage bodies.

Run with the bridge venv's python, editor open:
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
        Scripts/demos/tidybot_guarded_reach_demo.py
Exits nonzero on any phase failure.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

from tidybot_mobile_manip_demo import (
    ACTOR_ID,
    AGENT_RADIUS,
    FLOOR,
    MODEL_XML,
    NAV_BOUNDS,
    OBSTACLES,
    STAGING,
)
from tidybot_mink_demo import Tracker, stream_target
from tidybot_twist_follow_demo import (
    DAMPING_TASK,
    twist_follow_controller_payload,
)

# The "table": a low work surface just past staging. x spans 4.85-5.35, top at
# z=0.6 — the navmesh carve (55 cm agent radius) ends walkable space ~4.30, so
# staging stays reachable and the base parks ~0.65 m from the face.
TABLE = dict(actor_id="gr_table", location=(5.1, 0.0, 0.3), size=(0.5, 0.8, 0.6))

# Commanded EE point INSIDE the table (past the front face, below the top) but
# WITHIN kinematic reach (~0.71 m from the shoulder at staging) — only the
# collision constraint can be what stops the approach.
TABLE_TARGET = np.array([4.9, 0.0, 0.5])

REACH_RAMP_SEC = 6.0
PLATEAU_EPS = 0.005       # m per sample once holding
PLATEAU_COUNT = 5         # consecutive stable samples to accept
HOLD_BAND = (0.08, 0.45)  # plateau err: stopped SHORT (guard), not "reached"
                          # (impossible) nor "never approached"; ~0.15-0.30
                          # expected with the 0.18 gripper envelope

# Guard groups. Body names expand to each body's (unnamed) mesh geoms.
ROBOT_LINKS = [
    "base_link",
    "shoulder_link",
    "half_arm_1_link",
    "half_arm_2_link",
    "forearm_link",
    "spherical_wrist_1_link",
    "spherical_wrist_2_link",
    "bracelet_link",
]
LINK_STANDOFF = 0.10
GRIPPER_BODY = "base"      # the 2f85 gripper base (exact-match wins)
GRIPPER_STANDOFF = 0.18    # envelope: covers the ~15 cm unguarded finger extent


def guarded_controller_payload(obstacle_bodies: list) -> dict:
    """The twist-follow stack + the two collision guards (full arm chain at
    LINK_STANDOFF; gripper as an envelope at GRIPPER_STANDOFF)."""
    payload = twist_follow_controller_payload()
    payload["limits"] += [
        {
            "kind": "collision_avoidance",
            "geoms_a": ROBOT_LINKS,
            "geoms_b": obstacle_bodies,
            "min_distance": LINK_STANDOFF,
            "detection_distance": 0.40,
        },
        {
            "kind": "collision_avoidance",
            "geoms_a": [GRIPPER_BODY],
            "geoms_b": obstacle_bodies,
            "min_distance": GRIPPER_STANDOFF,
            "detection_distance": 0.30,
        },
    ]
    return payload


def log(msg: str) -> None:
    print(f"[guard-demo] {msg}", flush=True)


def fail(msg: str) -> None:
    print(f"[guard-demo] FAIL: {msg}", flush=True)
    sys.exit(1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="tcp://localhost")
    ap.add_argument("--port", type=int, default=5559,
                    help="direct step-server port (bridge StepPort default 5559)")
    ap.add_argument("--keep-open", action="store_true")
    args = ap.parse_args()

    client = URLabClient(args.host, step_mode="direct", step_port=args.port)
    client.connect()
    log("connected")

    try:
        # ---- Scene: obstacle course + the table ------------------------------
        log("creating GuardedReachDemo level + scene")
        client.scene.create_level(name="GuardedReachDemo", force_overwrite=True)
        client.scene.spawn_light(actor_id="gr_sun", kind="directional", intensity=6.0)
        client.scene.spawn_box(**FLOOR)
        obstacle_bodies = []
        for ob in OBSTACLES + [TABLE]:
            sb = client.scene.spawn_box(**ob)
            client.outliner.add_quick_convert(target=ob["actor_id"], static=True)
            obstacle_bodies.append(f"{sb['actor_name']}_MjBody")
        log(f"obstacle MuJoCo bodies: {obstacle_bodies}")
        r = client.scene.spawn_nav_bounds(**NAV_BOUNDS, agent_radius=AGENT_RADIUS, timeout_s=30.0)
        if not r.get("nav_data_present"):
            fail(f"navmesh did not bake: {r}")

        # ---- Robot: nav stack + guarded whole-body mink ----------------------
        bp = client.scene.import_xml(path=str(MODEL_XML))
        client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
        client.scene.ensure_manager()
        stack = client.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
        log(f"nav stack: created={stack.get('created')}")
        ik = client.ik.add_controller(target=ACTOR_ID,
                                      **guarded_controller_payload(obstacle_bodies))
        if ik.get("warnings"):
            fail(f"add_controller warnings (unresolved guard names?): {ik['warnings']}")
        log("ik controller added — guards resolved without warnings")

        # ---- Sim up; mink only, whole run ------------------------------------
        log("entering PIE (sim.start)")
        client.sim.start(timeout_s=180.0)

        nav_name = None
        for row in client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True):
            if row.actor_id == ACTOR_ID:
                nav_name = row.name
                break
        if nav_name is None:
            fail("robot not found in PIE world")

        def configure(params):
            client._rpc_configure_controller(articulation=nav_name, params=params)

        r = client.runtime.set_active_controller(articulation=nav_name, controller="mink_ik")
        if r.get("active") != "MjMinkIKController":
            fail(f"expected mink active, got {r}")
        try:
            client.runtime.set_mode("live")
        except URLabRPCError as exc:
            log(f"set_mode(live) note: {exc}")
        client.runtime.set_paused(paused=False)
        time.sleep(2.0)

        configure({"task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})

        # ---- Phase A: stowed guarded transit ----------------------------------
        g = client.runtime.set_nav_goal(articulation=nav_name, x=STAGING[0], y=STAGING[1])
        if not g.get("accepted"):
            fail(f"staging goal rejected: {g}")
        log(f"Phase A: driving to {STAGING} — arm stowed, guards armed")
        state = "navigating"
        deadline = time.time() + 120.0
        while time.time() < deadline:
            s = client.runtime.get_nav_status(articulation=nav_name)
            state = s["state"]
            log(f"  A: {state:<10} dist={s['distance_to_goal']:.2f}")
            if state in ("arrived", "failed"):
                break
            time.sleep(1.0)
        if state != "arrived":
            fail(f"Phase A did not arrive (state={state})")
        log("Phase A ARRIVED (guards silent through the stowed transit)")

        configure({"task_costs": {str(DAMPING_TASK): {"cost": 5.0}}})

        # ---- Phase B: ramped descent to the in-table point --------------------
        tracker = Tracker(client)

        def synced_ee():
            client.runtime.set_mode("direct")
            client.step(n_steps=1)
            p, q = tracker.ee_pose()
            client.runtime.set_mode("live")
            client.runtime.set_paused(paused=False)
            return p, q

        pA, q0 = synced_ee()
        stream_target(client, nav_name, pA, q0)  # seed while disabled (no yank)
        configure({"task_enabled": [True, True, True, True]})
        log(f"EE task enabled (seeded at {np.round(pA, 3)})")

        # Visualize: red marker at the commanded point (hidden inside the table)
        # + one guide marker above the table top pointing at it. The live goal
        # itself is drawn by the controller (draw_target) throughout the ramp.
        tt = [float(v) for v in TABLE_TARGET]
        client.debug.draw_marker(location=tt, color=[1.0, 0.1, 0.1], ttl=-1)
        client.debug.draw_marker(location=[tt[0], tt[1], 1.1], color=[1.0, 1.0, 1.0], ttl=-1)
        log("markers: RED = commanded point (inside the table); WHITE = guide above it")

        log(f"Phase B: ramping the goal to {TABLE_TARGET} over {REACH_RAMP_SEC}s "
            f"— the guard must hold the approach")
        t0 = time.time()
        while True:
            a = min(1.0, (time.time() - t0) / REACH_RAMP_SEC)
            stream_target(client, nav_name, (1.0 - a) * pA + a * TABLE_TARGET, q0)
            time.sleep(0.04)
            if a >= 1.0:
                break

        plateau, prev_err, err = 0, None, 9.9
        deadline_b = time.time() + 30.0
        while time.time() < deadline_b:
            time.sleep(0.5)
            ee, _ = synced_ee()
            err = float(np.linalg.norm(ee - TABLE_TARGET))
            d = abs(err - prev_err) if prev_err is not None else 9.9
            log(f"  B hold: ee={np.round(ee, 3)} err={err:.4f} d={d:.4f}")
            stream_target(client, nav_name, TABLE_TARGET, q0)
            prev_err = err
            plateau = plateau + 1 if d < PLATEAU_EPS else 0
            if plateau >= PLATEAU_COUNT:
                break
        if plateau < PLATEAU_COUNT:
            fail(f"EE never settled to a stable standoff (last err={err:.4f})")
        if not (HOLD_BAND[0] < err < HOLD_BAND[1]):
            fail(f"plateau err {err:.4f} outside hold band {HOLD_BAND} — "
                 f"either penetrated (guard failed) or never approached")
        log(f"GUARD HOLD: EE stable at {err:.3f} m from the in-table point — "
            f"commanded into the obstacle, held off by the QP")
        log("ALL PHASES PASSED")
    finally:
        if not args.keep_open:
            try:
                client.sim.stop()
            except URLabRPCError:
                pass
        try:
            client.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
