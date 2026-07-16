#!/usr/bin/env python3
"""TidyBot whole-body mobile manipulation — the mink as a twist-bus consumer.

The swap-free sibling of tidybot_mobile_manip_demo.py: ONE controller (the
mink) is active for the entire run. The nav stack plans on the navmesh and
its pure pursuit writes the twist BUS exactly as it does for base_drive —
but here the mink's twist_follow task consumes the bus in-engine (500 Hz),
so this script streams NOTHING during the drive. Python's role is
set_nav_goal + watching, then a velocity-governed reach at staging.

Phase A (LIVE): nav goal set; pursuit -> twist bus -> mink twist_follow
drives the base around the wall. Zero target streaming, zero controller swap.
Phase B (LIVE): enable the EE task (seeded at the current pose — a stale or
world-frame "hold" target is a rubber band to the spawn point), stream the
final goal once, and the QP velocity limits pace the reach.

Run with the bridge venv's python, editor open:
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
        Scripts/demos/tidybot_twist_follow_demo.py
Exits nonzero on any phase failure.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

# Shared scene + tuning constants (same course, same governance).
from tidybot_mobile_manip_demo import (
    ACCEPT_EE_ERR,
    AGENT_RADIUS,
    ARM_MAX_VEL,
    BASE_MAX_VEL,
    FLOOR,
    MODEL_XML,
    NAV_BOUNDS,
    OBSTACLES,
    REACH_SETTLE_S,
    REACH_TARGET,
    STAGING,
)
from tidybot_mink_demo import (
    ARM_JOINTS,
    BASE_JOINTS,
    Tracker,
    build_controller_payload,
    stream_target,
)

ACTOR_ID = "tf_tidybot"
NAV_TIMEOUT_S = 120.0

# Task spec indices in twist_follow_controller_payload() — the config-surface
# calls below (task_enabled / task_costs) are positional.
EE_TASK, POSTURE_TASK, DAMPING_TASK, TWIST_TASK = 0, 1, 2, 3


def twist_follow_controller_payload() -> dict:
    """The whole-body stack: the example payload with the EE task DISABLED for
    transit (the posture task holds the arm joint-space; a world-frame EE hold
    would anchor the robot to its start), plus a twist_follow task over the
    base joints (ORDERED x, y, th), real-time tracking (max_iters=1), and QP
    velocity limits as the pace governors."""
    payload = build_controller_payload(draw_target=True, lazy_base_cost=5.0)
    payload["tasks"][0]["enabled"] = False
    payload["tasks"].append({
        "kind": "twist_follow",
        "joints": BASE_JOINTS,
        "cost": 1.0,
        "enabled": True,
    })
    payload["max_iters"] = 1
    payload["limits"] += [
        {"kind": "velocity", "joints": BASE_JOINTS, "max_velocity": BASE_MAX_VEL},
        {"kind": "velocity", "joints": ARM_JOINTS, "max_velocity": ARM_MAX_VEL},
    ]
    return payload


def log(msg: str) -> None:
    print(f"[tf-demo] {msg}", flush=True)


def fail(msg: str) -> None:
    print(f"[tf-demo] FAIL: {msg}", flush=True)
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
        # ---- Scene (isolated level) ----------------------------------------
        log("creating TwistFollowDemo level + scene")
        client.scene.create_level(name="TwistFollowDemo", force_overwrite=True)
        client.scene.spawn_light(actor_id="tf_sun", kind="directional", intensity=6.0)
        client.scene.spawn_box(**FLOOR)
        for ob in OBSTACLES:
            client.scene.spawn_box(**ob)
            client.outliner.add_quick_convert(target=ob["actor_id"], static=True)
        r = client.scene.spawn_nav_bounds(**NAV_BOUNDS, agent_radius=AGENT_RADIUS, timeout_s=30.0)
        if not r.get("nav_data_present"):
            fail(f"navmesh did not bake: {r}")

        # ---- Robot: nav stack (for the planner + bus) + whole-body mink -----
        bp = client.scene.import_xml(path=str(MODEL_XML))
        client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
        client.scene.ensure_manager()
        stack = client.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
        log(f"nav stack: created={stack.get('created')}")
        ik = client.ik.add_controller(target=ACTOR_ID, **twist_follow_controller_payload())
        log(f"ik controller: was_existing={ik.get('was_existing')}")

        # ---- Sim up; mink is the ONLY active controller, whole run ----------
        log("entering PIE (sim.start)")
        client.sim.start(timeout_s=180.0)

        nav_name = None
        for row in client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True):
            if row.actor_id == ACTOR_ID:
                nav_name = row.name
                break
        if nav_name is None:
            fail("robot not found in PIE world")
        log(f"articulation name={nav_name!r}")

        def configure(params):
            client._rpc_configure_controller(articulation=nav_name, params=params)

        r = client.runtime.set_active_controller(articulation=nav_name, controller="mink_ik")
        if r.get("active") != "MjMinkIKController":
            fail(f"expected mink active, got {r}")
        log("mink_ik active — the twist bus is the only nav interface from here")

        try:
            client.runtime.set_mode("live")
        except URLabRPCError as exc:
            log(f"set_mode(live) note: {exc}")
        client.runtime.set_paused(paused=False)  # bridge live is never auto-unpaused
        time.sleep(2.0)

        # Lazy-base damping competes with twist_follow for the base DOFs:
        # relax it for the transit (live task_costs), restore for the reach.
        configure({"task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})

        # ---- Phase A: bus-driven transit — nothing to stream -----------------
        g = client.runtime.set_nav_goal(articulation=nav_name, x=STAGING[0], y=STAGING[1])
        if not g.get("accepted"):
            fail(f"staging goal rejected: {g}")
        log(f"Phase A: goal {STAGING} accepted — watching (zero streaming)")
        state = "navigating"
        deadline = time.time() + NAV_TIMEOUT_S
        while time.time() < deadline:
            s = client.runtime.get_nav_status(articulation=nav_name)
            state = s["state"]
            log(f"  A: {state:<10} dist={s['distance_to_goal']:.2f}")
            if state in ("arrived", "failed"):
                break
            time.sleep(1.0)
        if state != "arrived":
            fail(f"Phase A did not arrive (state={state})")
        log("Phase A ARRIVED — bus-driven, in-engine, swap-free")

        configure({"task_costs": {str(DAMPING_TASK): {"cost": 5.0}}})

        # ---- Phase B: velocity-governed reach --------------------------------
        tracker = Tracker(client)

        def synced_ee():
            client.runtime.set_mode("direct")
            client.step(n_steps=1)
            p, q = tracker.ee_pose()
            client.runtime.set_mode("live")
            client.runtime.set_paused(paused=False)
            return p, q

        pA, q0 = synced_ee()
        # Seed the EE target at the CURRENT pose while the task is disabled,
        # THEN enable — enabling against the stale bind-pose target yanks.
        stream_target(client, nav_name, pA, q0)
        configure({"task_enabled": [True, True, True, True]})
        log(f"EE task enabled (seeded at {np.round(pA, 3)})")

        log(f"Phase B: streaming goal {REACH_TARGET}; pace governed by "
            f"ARM_MAX_VEL={ARM_MAX_VEL} rad/s")
        stream_target(client, nav_name, REACH_TARGET, q0)
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
        try:
            client.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
