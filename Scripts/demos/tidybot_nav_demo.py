#!/usr/bin/env python3
"""TidyBot navigation bridge demo — live E2E for the mobile-base nav stack.

Authors the whole demo scene over the bridge (floor, obstacle wall, navmesh
bounds), imports + spawns the TidyBot, attaches the nav stack
(twist + base-drive + nav components), enters PIE in live mode, then:

  1. HAPPY PATH  — set_nav_goal behind an obstacle wall; accept when
     get_nav_status reaches 'arrived', final distance <= ACCEPT_DIST, and the
     sampled track detoured (max |y| >= DETOUR_MIN_Y — the straight line to the
     goal has y == 0).
  2. OFF-MESH    — set_nav_goal inside the wall; accept when accepted == false.

Run with the bridge venv's python while the editor is open:
    /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python \
        Scripts/demos/tidybot_nav_demo.py

Exits nonzero if any acceptance item fails. See also tidybot_mink_demo.py
(same orchestration idioms) and docs/guides/navigation-demo.md.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

MODEL_XML = (
    Path(__file__).resolve().parents[1]
    / "mink_golden/models/stanford_tidybot/tidybot_scene_ue.xml"
)
ACTOR_ID = "tidybot_0"

# Scene layout (MuJoCo world metres). Wall at x=2 spans y in [-1.5, 1.5];
# robot starts at origin; goal is straight through the wall.
FLOOR = dict(actor_id="nav_floor", location=(0.0, 0.0, -0.1), size=(20.0, 20.0, 0.2))
OBSTACLES = [
    dict(actor_id="nav_wall", location=(2.0, 0.0, 0.4), size=(0.4, 3.0, 0.8)),
    dict(actor_id="nav_block_a", location=(3.0, -1.6, 0.4), size=(0.6, 0.6, 0.8)),
    dict(actor_id="nav_block_b", location=(1.0, 1.8, 0.4), size=(0.6, 0.6, 0.8)),
]
NAV_BOUNDS = dict(center=(0.0, 0.0, 0.5), extent=(12.0, 12.0, 2.0))

GOAL = (4.0, 0.0)          # behind the wall
OFFMESH_GOAL = (2.0, 0.0)  # inside the wall
ACCEPT_DIST = 0.20         # m: acceptance_radius (0.15) + slack
DETOUR_MIN_Y = 1.0         # m: sampled |y| must exceed this at least once
TIMEOUT_S = 90.0
POLL_S = 0.5


def log(msg: str) -> None:
    print(f"[nav-demo] {msg}", flush=True)


def fail(msg: str) -> None:
    print(f"[nav-demo] FAIL: {msg}", flush=True)
    sys.exit(1)


def robot_xy(client) -> tuple[float, float]:
    """Robot planar position in MuJoCo metres, read from the *live PIE* world.

    NOTE on a divergence from the naive approach: `outliner.get_actor_bounds`
    looks like the obvious op here, but `GetActorBoundsSync`
    (Source/URLabEditor/Private/MjLevelOps.cpp) always resolves
    `GEditor->GetEditorWorldContext().World()` -- the *editor* world, never
    `GEditor->PlayWorld`. During PIE the editor-world actor is a frozen copy
    at its pre-PIE spawn pose, so get_actor_bounds would silently report a
    position that never moves (the detour/arrival checks below would then
    fail even on a correct run). `find_actors(in_pie=True)` IS PIE-aware
    (`FindActorsSync` -> `PickWorld(bWantPie=True, ...)` picks
    `GEditor->PlayWorld` when PIE is running), so we use that instead,
    filtering client-side on `actor_id` (name_prefix would need to match
    UE's internal object name, not our friendly actor_id).

    Units: both ops convert UE cm -> MJ metres via the same
    `MjUtils::UEToMjPosition` (Y-flip baked in), so `location` here is
    already in MJ metres -- no /100 conversion needed.
    """
    rows = client.outliner.find_actors(class_filter="AMjArticulation", in_pie=True)
    for r in rows:
        if r.actor_id == ACTOR_ID:
            return r.location[0], r.location[1]
    raise RuntimeError(f"actor {ACTOR_ID!r} not found in the PIE world (rows={rows})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="tcp://localhost")
    ap.add_argument("--keep-open", action="store_true",
                    help="leave PIE running after the demo")
    args = ap.parse_args()

    client = URLabClient(args.host)
    client.connect()
    log("connected")

    try:
        # ---- Phase 1: scene ----------------------------------------------------
        log("authoring scene (floor, wall, nav bounds)")
        client.scene.spawn_box(**FLOOR)  # UE-only: MJCF plane is the physics floor
        for ob in OBSTACLES:
            client.scene.spawn_box(**ob)
            # add_quick_convert lives in the `outliner` namespace (not `scene`) --
            # verified against RegEditor(TEXT("add_quick_convert"), TEXT("outliner"), ...)
            # in Source/URLabEditor/Private/MjEditorOpHandlers.cpp.
            client.outliner.add_quick_convert(target=ob["actor_id"], static=True)
        r = client.scene.spawn_nav_bounds(**NAV_BOUNDS, timeout_s=30.0)
        if not r.get("nav_data_present"):
            fail(f"navmesh did not bake: {r}")
        log(f"navmesh ready in {r.get('build_seconds', 0):.1f}s")

        # ---- Phase 2: robot ----------------------------------------------------
        log(f"importing {MODEL_XML.name}")
        # import_xml returns a URLabBlueprint dataclass (class_path/short_name/
        # imported_now), not a dict -- pass it straight to spawn_actor (it accepts
        # a URLabBlueprint or a raw class-path string), matching tidybot_mink_demo.py.
        bp = client.scene.import_xml(path=str(MODEL_XML))
        client.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(0, 0, 0))
        client.scene.ensure_manager()
        stack = client.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True)
        if stack.get("warnings"):
            log(f"add_nav_stack warnings: {stack['warnings']}")
        log(f"nav stack: created={stack.get('created')} existing={stack.get('existing')}")

        # ---- Phase 3: PIE, live mode -------------------------------------------
        log("entering PIE (sim.start)")
        client.sim.start(timeout_s=180.0)
        try:
            client.runtime.set_mode("live")
        except URLabRPCError as exc:
            log(f"set_mode(live) warning: {exc}")
        time.sleep(2.0)  # let physics settle on spawn

        # ---- Phase 4: happy path ------------------------------------------------
        log(f"set_nav_goal {GOAL}")
        r = client.runtime.set_nav_goal(articulation=ACTOR_ID, x=GOAL[0], y=GOAL[1])
        if not r.get("accepted"):
            fail(f"happy-path goal rejected: {r}")

        max_abs_y = 0.0
        state = "navigating"
        deadline = time.time() + TIMEOUT_S
        while time.time() < deadline:
            s = client.runtime.get_nav_status(articulation=ACTOR_ID)
            state = s["state"]
            x, y = robot_xy(client)
            max_abs_y = max(max_abs_y, abs(y))
            log(f"  state={state:<10} dist={s['distance_to_goal']:.2f} pos=({x:.2f},{y:.2f})")
            if state in ("arrived", "failed"):
                break
            time.sleep(POLL_S)

        if state != "arrived":
            fail(f"did not arrive (state={state})")
        s = client.runtime.get_nav_status(articulation=ACTOR_ID)
        if s["distance_to_goal"] > ACCEPT_DIST:
            fail(f"arrived but distance {s['distance_to_goal']:.2f} > {ACCEPT_DIST}")
        if max_abs_y < DETOUR_MIN_Y:
            fail(f"no detour observed (max |y| = {max_abs_y:.2f} < {DETOUR_MIN_Y})"
                 " — did it drive through the wall?")
        log(f"ARRIVED, detour max |y| = {max_abs_y:.2f} m")

        # ---- Phase 5: off-mesh reject -------------------------------------------
        log(f"off-mesh goal {OFFMESH_GOAL} (inside the wall)")
        r = client.runtime.set_nav_goal(
            articulation=ACTOR_ID, x=OFFMESH_GOAL[0], y=OFFMESH_GOAL[1])
        if r.get("accepted"):
            fail("off-mesh goal was accepted — expected rejection")
        log("off-mesh goal correctly rejected")

        if not args.keep_open:
            try:
                client.sim.stop()
            except URLabRPCError:
                pass
        log("ALL ACCEPTANCE ITEMS PASSED")
    finally:
        client.close()


if __name__ == "__main__":
    main()
