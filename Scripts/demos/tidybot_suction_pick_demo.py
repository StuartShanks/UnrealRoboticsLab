#!/usr/bin/env python3
"""TidyBot suction pick round trip — nav, grab, nav while holding, release.

The BT-over-skills sibling of the mobile-manipulation demo set: the guarded
twist-follow mink stack (tidybot_twist_follow_demo.py + the collision guards
from tidybot_guarded_reach_demo.py) drives a py_trees Sequence over the
reusable skills in urlab_skills.py, orchestrating the full "pick" story with
ONE controller for the entire run:

    ResolveAffordance -> Reachable -> Drive(staging) -> CorridorClear
    -> ReachRamp(pre-approach, cup-down) -> DescendEngage -> VerifyAttach
    -> StowCarry -> Drive(home) -> Release

Spec: docs/superpowers/specs/2026-07-16-suction-pick-v1-design.md.
No motion planner in v1 (locked premise): Reachable and CorridorClear are
loud-abort gates, not a planner — a blocked corridor or an out-of-annulus
affordance point fails the tree rather than replanning.

Run with the bridge venv's python, editor open (the user drives Simulate):
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
        Scripts/demos/tidybot_suction_pick_demo.py
Exits nonzero on tree FAILURE or a wall-clock timeout.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import py_trees

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

from tidybot_mobile_manip_demo import (
    AGENT_RADIUS,
    FLOOR,
    NAV_BOUNDS,
    OBSTACLES,
    STAGING,
)
from tidybot_guarded_reach_demo import LINK_STANDOFF, ROBOT_LINKS, TABLE
from tidybot_twist_follow_demo import DAMPING_TASK, EE_TASK, twist_follow_controller_payload
from urlab_skills import (
    CorridorClear,
    DescendEngage,
    Drive,
    PickBlackboard,
    Reachable,
    ReachRamp,
    Release,
    ResolveAffordance,
    StowCarry,
    VerifyAttach,
)

MODEL_XML = (
    Path(__file__).resolve().parents[1]
    / "mink_golden/models/stanford_tidybot/tidybot_suction_ue.xml"
)
ASSETS = Path(__file__).resolve().parent / "assets"
ACTOR_ID = "sp_tidybot"  # unique — never tidybot_0 / nav_tidybot / mm_tidybot / tf_tidybot

# The pick object: at the table's FRONT EDGE (table front face x=4.85; box
# half-width 0.05 -> center 4.90 sits fully on the table with its front flush to
# the edge) + box half-height (0.05) on the top (z=0.6) + a 1 cm settle. Front
# edge, not mid-table, so a nav-parked base reaches it y-aligned and vertical —
# a deep-table whole-body pick needs base *alignment* control (v1.1), not just
# the base *closeness* free base-assist gave (it drifted off-center in y).
PICK_LOCATION = (4.9, 0.0, 0.66)

# Pick-approach staging: directly in front of the box, ALIGNED in y, as close as
# the agent-radius-carved navmesh allows (table face 4.85 - 0.55 clearance =
# ~4.30 walkable edge; 4.35 keeps a small margin). From here the fixed-base arm
# reaches the front-edge box at ~0.59 m (inside its ~0.85 m envelope), straight
# ahead and straight down.
PICK_STAGING = (4.35, 0.0)

# Guard tuning. The full arm chain gets the same standoff as the guarded-
# reach sibling; the cup (no gripper envelope to hide behind) is guarded at
# close quarters since the whole point of the approach is to get it near the
# scenery. The pick object itself is EXCLUDED from every guard group below —
# guarding it would hold the cup off the very thing it's picking.
CUP_STANDOFF_M = 0.02
CUP_DETECTION_M = 0.20
LINK_DETECTION_M = 0.40

# The robot's own kinematic-chain root (excludes the whole robot subtree from
# the CorridorClear ray) + the pick object (its own affordance point sits at
# the ray's destination).
CORRIDOR_EXCLUDE = ["base_link", "pick_box"]

TREE_TIMEOUT_S = 240.0
TICK_PERIOD_S = 0.25


class SuctionBlackboard(PickBlackboard):
    """PickBlackboard + a computed single-point waypoint list for ReachRamp's
    generic `waypoints_key` contract. ReachRamp only carries the EE out to the
    pre-approach point (bb.affordance.waypoints[0]); DescendEngage owns the
    engage+contact legs directly off bb.affordance. No behaviour logic is
    reimplemented here — this is pure data plumbing between the two."""

    @property
    def pre_approach_waypoints(self):
        return [self.affordance.waypoints[0]] if self.affordance is not None else []


def suction_pick_controller_payload(obstacle_bodies: list) -> dict:
    """The twist-follow stack (tidybot_twist_follow_demo), EE frame switched
    to the cup variant's `cup_site`, plus the collision-avoidance guards:
    the full arm chain at LINK_STANDOFF, the cup at close quarters, and the
    2f85 mount plate (`base`) at close quarters too. `base` needs its own
    guard because ResolveGeomGroup resolves a body name to that body's OWN
    geoms only (no subtree recursion) — guarding `cup` does not protect its
    parent `base`, which is a real collision mesh one link above the cup."""
    payload = twist_follow_controller_payload()
    payload["tasks"][0]["frame"] = "cup_site"
    payload["limits"] += [
        {
            "kind": "collision_avoidance",
            "geoms_a": ROBOT_LINKS,
            "geoms_b": obstacle_bodies,
            "min_distance": LINK_STANDOFF,
            "detection_distance": LINK_DETECTION_M,
        },
        {
            "kind": "collision_avoidance",
            "geoms_a": ["cup"],
            "geoms_b": obstacle_bodies,
            "min_distance": CUP_STANDOFF_M,
            "detection_distance": CUP_DETECTION_M,
        },
        {
            "kind": "collision_avoidance",
            "geoms_a": ["base"],          # the 2f85 mount plate — a real collision mesh
            "geoms_b": obstacle_bodies,   # same obstacle set as the other guards (pick_box NOT included)
            "min_distance": 0.04,
            "detection_distance": 0.25,
        },
    ]
    return payload


def build_tree(bb: SuctionBlackboard) -> py_trees.trees.BehaviourTree:
    verify_attach = py_trees.decorators.Retry(
        name="VerifyAttach(retry x1)",
        child=VerifyAttach("VerifyAttach", bb),
        num_failures=1,
    )
    root = py_trees.composites.Sequence(
        name="SuctionPick",
        memory=True,
        children=[
            ResolveAffordance("ResolveAffordance", bb, body_suffix="pick_box"),
            Reachable("Reachable", bb, shoulder_xy=PICK_STAGING),
            Drive("DriveToStaging", bb, PICK_STAGING),
            CorridorClear(
                "CorridorClear", bb,
                p_from=lambda: bb.affordance.waypoints[0],
                p_to=lambda: bb.affordance.waypoints[2],
                exclude_body_suffixes=CORRIDOR_EXCLUDE,
            ),
            # No base-assist: the front-edge box is reachable from the aligned
            # nav-staging pose with a fixed base, so the base stays locked and the
            # descent is a clean, y-aligned, pure-vertical arm drop. (Free
            # base-assist gave closeness but drifted the base off-center in y,
            # descending beside the box; controlled base positioning for a
            # deep-table pick is v1.1 — SetBaseAssist stays in urlab_skills for it.)
            ReachRamp("ReachRamp", bb, waypoints_key="pre_approach_waypoints", quat_key="q_cup"),
            DescendEngage("DescendEngage", bb),
            verify_attach,
            StowCarry("StowCarry", bb),
            Drive("DriveHome", bb, bb.home_xy),
            Release("Release", bb),
        ],
    )
    return py_trees.trees.BehaviourTree(root)


def log(msg: str) -> None:
    print(f"[suction-pick-demo] {msg}", flush=True)


def fail(msg: str) -> None:
    print(f"[suction-pick-demo] FAIL: {msg}", flush=True)
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
        log("creating SuctionPickDemo level + scene")
        client.scene.create_level(name="SuctionPickDemo", force_overwrite=True)
        client.scene.spawn_light(actor_id="sp_sun", kind="directional", intensity=6.0)
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

        # ---- Pick asset: imported (not quick-converted), excluded from every
        # guard group below (obstacle_bodies never includes it). -------------
        bp_box = client.scene.import_xml(path=str(ASSETS / "suction_box.xml"))
        client.scene.spawn_actor(blueprint=bp_box, actor_id="pick_box", location=PICK_LOCATION)
        log(f"spawned pick_box at {PICK_LOCATION}")

        # ---- Robot: nav stack + guarded whole-body mink (cup EE frame) ------
        bp = client.scene.import_xml(path=str(MODEL_XML))
        client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
        client.scene.ensure_manager()
        stack = client.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
        log(f"nav stack: created={stack.get('created')}")
        ik = client.ik.add_controller(target=ACTOR_ID,
                                      **suction_pick_controller_payload(obstacle_bodies))
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

        r = client.runtime.set_active_controller(articulation=nav_name, controller="mink_ik")
        if r.get("active") != "MjMinkIKController":
            fail(f"expected mink active, got {r}")
        try:
            client.runtime.set_mode("live")
        except URLabRPCError as exc:
            log(f"set_mode(live) note: {exc}")
        client.runtime.set_paused(paused=False)
        time.sleep(2.0)

        # ---- Blackboard + tree ------------------------------------------------
        bb = SuctionBlackboard(
            client=client,
            name=nav_name,
            object_actor_id="pick_box",
            ee_task=EE_TASK,
            damping_task=DAMPING_TASK,
            home_xy=(0.0, 0.0),
        )
        tree = build_tree(bb)

        # ---- Run loop: tick until SUCCESS/FAILURE or the wall-clock ceiling --
        deadline = time.time() + TREE_TIMEOUT_S
        while time.time() < deadline:
            tree.tick()
            status = tree.root.status
            log(f"tick: root status={status}")
            if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                break
            time.sleep(TICK_PERIOD_S)

        if tree.root.status == py_trees.common.Status.SUCCESS:
            log("ALL PHASES PASSED — suction pick round trip")
        elif tree.root.status == py_trees.common.Status.FAILURE:
            fail(bb.fail_reason or "tree returned FAILURE (no fail_reason recorded)")
        else:
            fail(f"tree did not reach SUCCESS/FAILURE within {TREE_TIMEOUT_S}s "
                 f"(last status={tree.root.status})")
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
