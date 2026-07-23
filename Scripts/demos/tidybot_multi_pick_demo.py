#!/usr/bin/env python3
"""Multi-pick round trip (LIVE, Simulate running, authored by
tidybot_multi_pick_author.py): pick the Fab book off the small table, DRIVE
while holding it to the kitchen island, place it, re-pick it, drive back,
place it back. Fail-fast py_trees Sequence over urlab_skills; teardown in
finally. --one-way stops after the island place (live gate 1)."""
import argparse, sys, time
from dataclasses import dataclass
sys.path.insert(0, "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Scripts/demos")
import numpy as np
import py_trees
from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError
from urlab_client.results import PIEState
import urlab_skills as U

ACTOR_ID = "sp_tidybot"
BOOK = "SM_Book_125"
BOOK_HALF = 0.011
TICK_S = 0.25
RUN_TIMEOUT_S = 600.0


@dataclass
class Station:
    name: str
    aabb: tuple          # xmin, xmax, ymin, ymax
    top_z: float
    place_xy: tuple


TABLE = Station("table", (-13.20, -12.44, -15.63, -14.87), 0.55, (-12.71, -15.36))
ISLAND = Station("island", (-11.99, -10.17, -20.26, -19.38), 1.11, (-11.10, -19.60))
# Loaded-transit goals: a ring-legal point on each station's open side.
ISLAND_STAGING = (-11.10, -18.65)
TABLE_STAGING = (-12.18, -16.47)


def log(m): print(f"[multi-pick] {m}", flush=True)


def build_tree(bb, one_way: bool):
    # NOTE for the implementer: verify the existing skills' __init__
    # signatures (PlannedReach/DescendEngage/VerifyAttach) against
    # urlab_skills.py before wiring — the assembly test instantiates every
    # child, so a signature mismatch fails offline, not live.
    def pick_leg(station, tag):
        return [
            U.StageAt(f"stage_{tag}", bb, station.aabb, station.place_xy),
            U.ResolveActorTop(f"resolve_{tag}", bb, BOOK, BOOK_HALF),
            U.PlannedReach(f"reach_{tag}", bb),
            U.DescendEngage(f"descend_{tag}", bb),
            U.VerifyAttach(f"verify_{tag}", bb),
        ]
    children = (
        pick_leg(TABLE, "table")
        + [U.CarryTransit("carry_to_island", bb, ISLAND_STAGING, BOOK),
           U.PlaceOn("place_island", bb, ISLAND.place_xy, ISLAND.top_z,
                     BOOK_HALF, BOOK)]
    )
    if not one_way:
        children += (
            pick_leg(ISLAND, "island")[0:1]      # StageAt only; then re-pick
            + pick_leg(ISLAND, "island2")[1:]    # Resolve/Reach/Descend/Verify
            + [U.CarryTransit("carry_to_table", bb, TABLE_STAGING, BOOK),
               U.PlaceOn("place_table", bb, TABLE.place_xy, TABLE.top_z,
                         BOOK_HALF, BOOK)]
        )
    seq = py_trees.composites.Sequence("multi_pick_round_trip", memory=True)
    seq.add_children(children)
    return seq


def main(one_way: bool):
    c = URLabClient("tcp://localhost", step_mode="direct", step_port=5559)
    c.connect()
    if c.sim.status().state != PIEState.READY:
        log("FAIL: no READY Simulate session"); return 1
    c.sim.start(timeout_s=60.0)
    name = next((r.name for r in c.outliner.find_actors(
        class_filter="AMjArticulation", in_pie=True)
        if r.actor_id == ACTOR_ID), None)
    if name is None:
        log(f"FAIL: {ACTOR_ID!r} not found"); return 1
    if c.runtime.set_active_controller(
            articulation=name, controller="mink_ik").get("active") != "MjMinkIKController":
        log("FAIL: mink not active"); return 1
    try:
        c.runtime.set_mode("live")
    except URLabRPCError:
        pass
    c.runtime.set_paused(paused=False)
    time.sleep(1.0)

    bb = U.PickBlackboard()
    bb.client = c
    bb.name = name
    bb.object_actor_id = BOOK

    tree = build_tree(bb, one_way)
    log(f"tree: {[ch.name for ch in tree.children]}")
    tree.setup_with_descendants()
    deadline = time.time() + RUN_TIMEOUT_S
    rc = 1
    try:
        last = None
        while time.time() < deadline:
            tree.tick_once()
            cur = tree.tip().name if tree.tip() else "?"
            if cur != last:
                log(f"leg: {cur}")
                last = cur
            if tree.status == py_trees.common.Status.SUCCESS:
                log("ROUND TRIP COMPLETE" if not one_way else "ONE-WAY COMPLETE")
                rc = 0
                break
            if tree.status == py_trees.common.Status.FAILURE:
                log(f"FAILED at leg {cur}: {bb.fail_reason}")
                break
            time.sleep(TICK_S)
        else:
            log("FAIL: run timeout")
    finally:
        # teardown: suction off, posture cleared, EE task seed-then-enabled,
        # damping restored — always.
        try:
            c.runtime.set_suction(articulation=name, value=0.0)
        except Exception:
            pass
        try:
            p0, q0 = U.synced_site_pose(c, "cup_site")
            U.stream_target(c, name, p0, q0)
            c._rpc_configure_controller(articulation=name, params={
                "posture_target": {},
                "task_enabled": [True, True, True, False],
                "task_costs": {str(bb.damping_task): {"cost": 5.0}},
            })
        except Exception as e:
            log(f"teardown note: {e}")
    try:
        zf = U._actor_z_by_name(c, BOOK)
        log(f"final book z = {zf:.3f}")
    except Exception as e:
        log(f"final book-z read failed: {e}")
    return rc


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--one-way", action="store_true",
                    help="stop after placing on the island (live gate 1)")
    sys.exit(main(ap.parse_args().one_way))
