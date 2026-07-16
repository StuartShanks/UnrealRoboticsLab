#!/usr/bin/env python3
"""Live probe for the whole-body RRT connector (spec:
docs/superpowers/specs/2026-07-16-whole-body-rrt-connector-design.md).

  --author : build the suction-pick scene (editor idle, NOT simulating), save.
  --drive  : (press Simulate first) nav to staging, PLAN a whole-body path to
             the pre-grasp pose, execute it via the posture_target wire with a
             tracking watchdog, then run the validated direct-descent suction
             endgame. Success line: GRIPPED+LIFTED.

Connect AFTER Simulate starts: the client's model replica (mjb handshake) is
captured at connect — a stale connect plans against a stale world.
"""
import argparse
import sys
import time

import numpy as np
import mujoco

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

from urlab_skills import synced_site_pose, cup_down_quat, _object_z
from urlab_planner import PLANNED_JOINTS, PlanError, plan_reach
from tidybot_mink_demo import stream_target, resolve_id_by_suffix
from tidybot_twist_follow_demo import POSTURE_TASK, DAMPING_TASK
from tidybot_suction_pick_demo import (ACTOR_ID, ASSETS, MODEL_XML,
                                       PICK_LOCATION, PICK_STAGING,
                                       suction_pick_controller_payload)

PRE_GRASP_M = 0.03      # plan target: cup this far above the box top
TRACK_TOL = 0.15        # rad/m — watchdog per-joint tracking tolerance
TRACK_GRACE_S = 4.0     # divergence longer than this aborts execution
SETTLE_TOL = 0.05       # final-waypoint convergence
SETTLE_TIMEOUT_S = 10.0


def log(m):
    print(f"[planned-reach] {m}", flush=True)


def fail(m):
    print(f"[planned-reach] FAIL: {m}", flush=True)
    sys.exit(1)


def author():
    """Same scene as the suction-pick demo authoring (see the demo module's
    constants); saved so later cycles can load instead of re-importing."""
    from tidybot_mobile_manip_demo import FLOOR, NAV_BOUNDS, OBSTACLES, AGENT_RADIUS
    from tidybot_guarded_reach_demo import TABLE

    c = URLabClient("tcp://localhost")
    c.connect()
    try:
        c.sim.stop()
    except URLabRPCError:
        pass
    c.scene.create_level(name="PlannedReachDemo", force_overwrite=True)
    c.scene.spawn_light(actor_id="pr_sun", kind="directional", intensity=6.0)
    c.scene.spawn_box(**FLOOR)
    obstacle_bodies = []
    for ob in OBSTACLES + [TABLE]:
        sb = c.scene.spawn_box(**ob)
        c.outliner.add_quick_convert(target=ob["actor_id"], static=True)
        obstacle_bodies.append(f"{sb['actor_name']}_MjBody")
    r = c.scene.spawn_nav_bounds(**NAV_BOUNDS, agent_radius=AGENT_RADIUS, timeout_s=30.0)
    log(f"navmesh baked={r.get('nav_data_present')}")
    bp_box = c.scene.import_xml(path=str(ASSETS / "suction_box.xml"))
    c.scene.spawn_actor(blueprint=bp_box, actor_id="pick_box", location=PICK_LOCATION)
    bp = c.scene.import_xml(path=str(MODEL_XML))
    c.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
    c.scene.ensure_manager()
    st = c.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
    log(f"nav stack created={st.get('created')}")
    ik = c.ik.add_controller(target=ACTOR_ID,
                             **suction_pick_controller_payload(obstacle_bodies))
    warnings = ik.get("warnings") or []
    if warnings:
        log(f"!! WARNINGS: {warnings}")
    try:
        c.scene.save_level()
    except Exception as e:
        log(f"save_level note (non-blocking): {e}")
    log("AUTHORED. Press SIMULATE, then run --drive")


def drive():
    c = URLabClient("tcp://localhost", step_mode="direct", step_port=5559)
    c.connect()
    c.sim.start(timeout_s=180.0)
    name = next((r.name for r in c.outliner.find_actors(
        class_filter="AMjArticulation", in_pie=True) if r.actor_id == ACTOR_ID), None)
    if name is None:
        fail(f"{ACTOR_ID!r} not found")
    if c.runtime.set_active_controller(articulation=name,
                                       controller="mink_ik").get("active") != "MjMinkIKController":
        fail("mink not active")
    try:
        c.runtime.set_mode("live")
    except URLabRPCError as e:
        log(f"set_mode note: {e}")
    c.runtime.set_paused(paused=False)
    time.sleep(1.5)

    def configure(p):
        c._rpc_configure_controller(articulation=name, params=p)

    def hold_suction():
        try:
            c.runtime.set_suction(articulation=name, value=1.0)
        except Exception:
            pass

    def synced_q():
        """Planned-joint vector from a synced physics read. PRE-suction only
        (a direct-mode step zeroes actuator NetworkValues server-side)."""
        c.step(n_steps=1)
        m, d = c.model, c.data
        q = np.empty(len(PLANNED_JOINTS))
        for i, jn in enumerate(PLANNED_JOINTS):
            # suffix resolve: the importer prefixes compiled joint names, so a
            # bare mj_name2id returns -1 and qposadr[-1] would read garbage.
            jid = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, jn)
            q[i] = d.qpos[m.jnt_qposadr[jid]]
        return q

    # --- Phase A: nav to staging (frame off, twist follows) --------------------
    configure({"task_enabled": [False, True, True, True],
               "task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})
    g = c.runtime.set_nav_goal(articulation=name, x=PICK_STAGING[0], y=PICK_STAGING[1])
    if not g.get("accepted"):
        fail(f"nav goal rejected: {g}")
    log(f"driving to staging {PICK_STAGING}")
    deadline = time.time() + 90.0
    s = {"state": "?"}
    while time.time() < deadline:
        s = c.runtime.get_nav_status(articulation=name)
        if s["state"] in ("arrived", "failed"):
            break
        time.sleep(1.0)
    if s["state"] != "arrived":
        fail(f"nav {s['state']}")
    log("arrived at staging")
    configure({"task_costs": {str(DAMPING_TASK): {"cost": 5.0}}})

    # --- Phase B: PLAN whole-body to pre-grasp ---------------------------------
    surf, qaff = synced_site_pose(c, "affordance_suction_top")
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, qaff)
    normal = np.asarray(R).reshape(3, 3)[:, 2]
    normal = normal / (np.linalg.norm(normal) or 1.0)
    pre_grasp = np.asarray(surf) + PRE_GRASP_M * normal
    q_cup = cup_down_quat(normal)
    log(f"box top {np.round(surf, 3)}; planning to pre-grasp {np.round(pre_grasp, 3)}")
    try:
        plan = plan_reach(c, pre_grasp, q_cup, site="cup_site")
    except PlanError as e:
        fail(f"planning failed at stage '{e.stage}': {e}")
    log(f"PLAN: {plan.stats}")

    # --- Phase C: execute via the posture_target wire ---------------------------
    # Handoff contract: frame OFF, twist_follow OFF; posture cost bumped so the
    # streamed target dominates; damping + collision guards stay on (insurance).
    # Also relax the base damping (5.0 -> 0.05, the transit value): the lazy-base
    # damping penalizes base VELOCITY and would fight the posture task driving
    # the base to each planned waypoint (the planner deliberately repositions
    # the base). Collision-avoidance guards are separate limits — unaffected.
    configure({"task_enabled": [False, True, True, False],
               "task_costs": {str(POSTURE_TASK): {"cost": 5.0},
                              str(DAMPING_TASK): {"cost": 0.05}}})
    t0 = time.time()
    diverged_since = None
    wp_idx = 0
    while wp_idx < len(plan.waypoints):
        t_wp, wp = plan.waypoints[wp_idx]
        now = time.time() - t0
        if now < t_wp:
            time.sleep(min(0.2, t_wp - now))
        configure({"posture_target": wp})
        # Watchdog on the CURRENT waypoint once its scheduled time has passed.
        if time.time() - t0 >= t_wp:
            q = synced_q()
            err = max(abs(q[i] - wp[jn]) for i, jn in enumerate(PLANNED_JOINTS))
            if err > TRACK_TOL:
                diverged_since = diverged_since or time.time()
                if time.time() - diverged_since > TRACK_GRACE_S:
                    configure({"posture_target": {},
                               "task_costs": {str(POSTURE_TASK): {"cost": 1e-3}}})
                    fail(f"watchdog: waypoint {wp_idx} tracking err {err:.3f} "
                         f"> {TRACK_TOL} for {TRACK_GRACE_S}s — aborted, latch cleared")
            else:
                diverged_since = None
                wp_idx += 1
                log(f"  waypoint {wp_idx}/{len(plan.waypoints)} (err {err:.3f})")
    # Settle on the final waypoint.
    t_wp, wp = plan.waypoints[-1]
    settle_deadline = time.time() + SETTLE_TIMEOUT_S
    while time.time() < settle_deadline:
        q = synced_q()
        err = max(abs(q[i] - wp[jn]) for i, jn in enumerate(PLANNED_JOINTS))
        if err < SETTLE_TOL:
            break
        configure({"posture_target": wp})
        time.sleep(0.3)
    else:
        configure({"posture_target": {},
                   "task_costs": {str(POSTURE_TASK): {"cost": 1e-3}},
                   "task_enabled": [True, True, True, False]})
        fail(f"final waypoint never settled (err {err:.3f})")
    log(f"plan executed — at pre-grasp (err {err:.3f})")

    # --- Phase D: validated direct-descent suction endgame ----------------------
    # Back to frame-task control: clear the posture latch, restore its
    # regularizer cost, re-enable the frame task (twist_follow stays off).
    # Restore the lazy-base damping (0.05 -> 5.0) alongside clearing the posture
    # latch: the endgame is arm-only frame-task control, base held lazy again.
    configure({"posture_target": {},
               "task_costs": {str(POSTURE_TASK): {"cost": 1e-3},
                              str(DAMPING_TASK): {"cost": 5.0}},
               "task_enabled": [True, True, True, False]})
    p0, _ = synced_site_pose(c, "cup_site")
    stream_target(c, name, p0, q_cup)  # seed at current pose (no yank)
    target = np.asarray(surf)          # seat ON the box top
    stream_target(c, name, target, q_cup)
    engaged = False
    t0 = time.time()
    while time.time() - t0 < 8.0:
        time.sleep(0.3)
        cup, _ = synced_site_pose(c, "cup_site")
        stream_target(c, name, target, q_cup)
        d = float(np.linalg.norm(cup - np.asarray(surf)))
        if not engaged and d <= 0.05:
            c.runtime.set_suction(articulation=name, value=1.0)
            engaged = True
            log(f"suction ON at d={d:.3f}")
        if engaged:
            hold_suction()
        log(f"  cup={np.round(cup, 3)} d_to_top={d:.3f}")
        if engaged and d <= 0.02:
            break
    if not engaged:
        c.runtime.set_suction(articulation=name, value=1.0)
        log("force-engaged suction")
    hold_suction()
    time.sleep(1.0)
    hold_suction()

    z_before = _object_z(c, "pick_box")
    hold_suction()
    cup, _ = synced_site_pose(c, "cup_site")
    hold_suction()
    lift_to = cup + np.array([0.0, 0.0, 0.12])
    log(f"lifting from z={cup[2]:.3f} to {lift_to[2]:.3f}")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        a = min(1.0, (time.time() - t0) / 3.0)
        stream_target(c, name, (1 - a) * cup + a * lift_to, q_cup)
        time.sleep(0.05)
    z_after = _object_z(c, "pick_box")
    hold_suction()
    rose = z_after - z_before
    log(f"box z {z_before:.3f} -> {z_after:.3f}  rose {rose:+.3f}  "
        f"{'GRIPPED+LIFTED' if rose > 0.05 else 'no lift'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--author", action="store_true")
    mode.add_argument("--drive", action="store_true")
    args = ap.parse_args()
    author() if args.author else drive()
