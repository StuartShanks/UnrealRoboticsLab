#!/usr/bin/env python3
"""Author HomeInterior for the multi-pick round trip (editor IDLE).
Table + island + side-table (SM_Table_00_32, the far place station) static hulls; SM_Book_125 dynamic (actor must be Movable —
verified via bounds); suction tidybot + nav stack (max_speed 0.4, the carry
doctrine's slow-everywhere v1) + controller. CUP GUARD IS EMPTY: the cup
must approach BOTH surfaces to pick/place; descends are vertical and
reaches planner-checked, while arm+base stay guarded."""
import sys
sys.path.insert(0, "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Scripts/demos")
from tidybot_mobile_manip_demo import AGENT_RADIUS
from tidybot_suction_pick_demo import (ACTOR_ID, MODEL_XML, CUP_STANDOFF_M,
                                       CUP_DETECTION_M, LINK_DETECTION_M)
from tidybot_guarded_reach_demo import LINK_STANDOFF, ROBOT_LINKS
from tidybot_mink_demo import ARM_JOINTS, BASE_JOINTS
from tidybot_twist_follow_demo import twist_follow_controller_payload
from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

LEVEL = ["/Game/Home_Interior", "/Game/Home_Interior/Home_Interior"]
BOOK = "SM_Book_125"
SPAWN = (-10.0, -18.0)


def log(m): print(f"[multi-author] {m}", flush=True)


def guarded_payload(link_bodies, cup_bodies, base_bodies):
    p = twist_follow_controller_payload()
    p["tasks"][0]["frame"] = "cup_site"
    p["tasks"][1]["joints"] = BASE_JOINTS + ARM_JOINTS
    p["limits"] += [
        {"kind": "collision_avoidance", "geoms_a": ROBOT_LINKS,
         "geoms_b": link_bodies, "min_distance": LINK_STANDOFF,
         "detection_distance": LINK_DETECTION_M},
        {"kind": "collision_avoidance", "geoms_a": ["base"],
         "geoms_b": base_bodies, "min_distance": 0.04,
         "detection_distance": 0.25},
    ]
    if cup_bodies:
        p["limits"].append(
            {"kind": "collision_avoidance", "geoms_a": ["cup"],
             "geoms_b": cup_bodies, "min_distance": CUP_STANDOFF_M,
             "detection_distance": CUP_DETECTION_M})
    return p


c = URLabClient("tcp://localhost")
c.connect()
if c.sim.status().state.value == "ready":
    print("[multi-author] REFUSING: Simulate RUNNING. Stop it first.", flush=True)
    sys.exit(1)

loaded = None
for cand in LEVEL:
    try:
        c.scene.load_level(cand); loaded = cand; break
    except URLabRPCError as e:
        log(f"load_level {cand!r}: {e}")
if loaded is None:
    print("[multi-author] FAIL: no level loaded.", flush=True); sys.exit(1)
log(f"LOADED {loaded!r}")

rows = c.outliner.find_actors(class_filter="StaticMeshActor")
def resolve(pref): return next((r.name for r in rows if r.name.startswith(pref)), None)

table = resolve("SM_Table_01"); island = resolve("SM_Kitchen_island_table")
side = resolve("SM_Table_00")
book = resolve(BOOK)
if not (table and island and side and book):
    print(f"[multi-author] FAIL: missing actors t={table} i={island} s={side} b={book}",
          flush=True); sys.exit(1)
c.outliner.add_quick_convert(target=table, by_name=True, static=True, complex_mesh=False)
c.outliner.add_quick_convert(target=island, by_name=True, static=True, complex_mesh=False)
c.outliner.add_quick_convert(target=side, by_name=True, static=True, complex_mesh=False)
b = c.outliner.get_actor_bounds(book, by_name=True)
c.outliner.add_quick_convert(target=book, by_name=True, static=False, complex_mesh=False)
log(f"static: {table}, {island}, {side}; DYNAMIC {book} (top z={b.max[2]:.3f} — must be Movable)")

# --force-reimport: destroy + re-import the robot blueprint (needed when the
# robot XML changed — the importer caches by path). Use with the editor IDLE.
bp = c.scene.import_xml(path=str(MODEL_XML),
                        force_reimport=("--force-reimport" in sys.argv))
c.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(SPAWN[0], SPAWN[1], 0.0))
c.scene.ensure_manager()
nav = c.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True, max_speed=0.4)
log(f"nav stack={nav.get('created')} (max_speed 0.4 — loaded-transit doctrine)")
r = c.scene.spawn_nav_bounds(center=(SPAWN[0], SPAWN[1], 0.5), extent=(15.0, 15.0, 2.0),
                             agent_radius=AGENT_RADIUS, timeout_s=60.0)
log(f"navmesh baked={r.get('nav_data_present')}")
ik = c.ik.add_controller(target=ACTOR_ID, **guarded_payload(
    link_bodies=[f"{table}_MjBody", f"{island}_MjBody", f"{side}_MjBody"],
    cup_bodies=[],                       # cup approaches BOTH surfaces
    base_bodies=[f"{table}_MjBody", f"{island}_MjBody", f"{side}_MjBody"]))
w = ik.get("warnings") or []
log(f"!! WARNINGS: {w}" if w else "no warnings — cup unguarded, arm/base guarded")
log("AUTHORED. Press SIMULATE, then run tidybot_multi_pick_demo.py.")
