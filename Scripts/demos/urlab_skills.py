#!/usr/bin/env python3
"""Reusable pick-pipeline skills: py_trees behaviours over URLab bridge verbs,
plus the pure kinematic helpers (annulus, affordance template, corridor ray).

Doctrine (validated 2026-07-16, see docs/roadmap/navigation-and-mobile-manip.md):
every streamed EE goal is RAMPED (bounded error); a world-frame EE hold during
base motion is forbidden (disable the EE task, posture holds); seed the manual
target at the current pose BEFORE re-enabling the EE task.

Behaviour verb-sequences are lifted from three validated demos (do not
reinvent the sequences — see each class docstring for its source):
  - tidybot_twist_follow_demo.py  Phase A nav poll loop, synced_ee() dip,
                                  seed-then-enable ramp, damping relax/restore.
  - tidybot_guarded_reach_demo.py ramped-waypoint descent + plateau/verify,
                                  find_actors(in_pie=True) object-pose reads.
  - tidybot_mink_demo.py          Tracker (site world-pose reads),
                                  stream_target, resolve_id_by_suffix,
                                  FRAME_SITE / BASE_JOINTS / ARM_JOINTS.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import mujoco
import numpy as np
import py_trees

from tidybot_mink_demo import resolve_id_by_suffix, stream_target
from tidybot_twist_follow_demo import POSTURE_TASK, DAMPING_TASK

PRE_APPROACH_M = 0.15
ENGAGE_M = 0.02

# Approximate shoulder height above the base plane (world z), used by the
# Reachable condition. Matches the empirically-mapped annulus fixture
# (shoulder = (base_x, base_y, 0.49) in the validated staging geometry).
SHOULDER_HEIGHT_M = 0.49

NAV_TIMEOUT_S = 120.0
# Physics-auto-reset guard (brief-mandated): the MuJoCo QACC auto-reset
# teleports the robot to spawn, so distance-to-goal jumps back toward its
# initial value in a single poll; threshold tuned live. Rounding the
# obstacle wall at ~0.6 m/s with ~1 s polls can legitimately move the
# straight-line distance-to-goal by up to ~0.7 m per poll, so the threshold
# must clear that with margin while staying well below a multi-metre
# teleport (tuned live in Task 6).
RESET_FINGERPRINT_JUMP_M = 1.5


# --------------------------------------------------------------------------- #
# Pure kinematic helpers (self-tested in test_urlab_skills.py).
# --------------------------------------------------------------------------- #
def approach_waypoints(point: np.ndarray, normal: np.ndarray) -> list:
    """Template trajectory from an affordance frame: pre-approach -> engage ->
    contact, along the outward surface normal."""
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    p = np.asarray(point, dtype=float)
    return [p + PRE_APPROACH_M * n, p + ENGAGE_M * n, p.copy()]


def quat_slerp(q0: np.ndarray, q1: np.ndarray, a: float) -> np.ndarray:
    """Spherical interpolation between two wxyz quats, shortest arc."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = (1.0 - a) * q0 + a * q1
        return q / np.linalg.norm(q)
    th = np.arccos(np.clip(d, -1.0, 1.0))
    return (np.sin((1.0 - a) * th) * q0 + np.sin(a * th) * q1) / np.sin(th)


def cup_down_quat(normal: np.ndarray) -> np.ndarray:
    """Target EE quat (wxyz) with the cup axis anti-parallel to the affordance
    normal. The cup/pinch frame convention has the tool axis along local -z of
    the (0,1,0,0) frame, so for n=+z (top pick) this returns (0,1,0,0)."""
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    z_axis = -n                       # tool axis points INTO the surface
    # Build a frame: Gram-Schmidt an arbitrary helper against z (NOT a
    # cross(helper, z) — that construction introduces a spurious 90-degree
    # twist about the tool axis; e.g. for n=+z it yields quat
    # (0, 0.707, 0.707, 0) instead of the (0, 1, 0, 0) the cup/pinch frame
    # convention requires).
    helper = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = helper - np.dot(helper, z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rot = np.stack([x_axis, y_axis, z_axis], axis=1).reshape(9)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, rot)
    return q


def reach_annulus_ok(shoulder: np.ndarray, target: np.ndarray,
                     r_min: float = 0.30, r_max: float = 0.85) -> bool:
    """Bent-arm reachability annulus (empirically mapped 2026-07-16)."""
    d = float(np.linalg.norm(np.asarray(target) - np.asarray(shoulder)))
    return r_min <= d <= r_max


def due_waypoint(waypoints, elapsed_s):
    """Index of the waypoint that should currently be streamed: the LAST
    waypoint whose scheduled time <= elapsed_s (0 before the first).

    Retained committed helper (with its self-tests) — a clock-scheduled
    lookup. PlannedReach now streams SELF-PACED (advances only when the
    current waypoint is tracked), so it no longer calls this, but the
    function stands on its own for any clock-paced streaming."""
    idx = 0
    for i, (t, _q) in enumerate(waypoints):
        if t <= elapsed_s:
            idx = i
    return idx


def corridor_clear(client, p_from, p_to, exclude_body_suffixes,
                   tolerance: float = 0.01, max_marches: int = 8):
    """Client-side mj_ray on the mirrored model/data (run right after a synced
    step so the mirror is fresh). mj_ray excludes ONE body; multiple exclusions
    march past excluded hits. Returns (clear, first_blocking_distance)."""
    m, d = client.model, client.data
    excluded = set()
    for suf in exclude_body_suffixes:
        for b in range(m.nbody):
            nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
            if nm and nm.endswith(suf):
                # body + full subtree
                for b2 in range(m.nbody):
                    p = b2
                    while p != 0:
                        if p == b:
                            excluded.add(b2)
                            break
                        p = m.body_parentid[p]
                excluded.add(b)
    p0 = np.asarray(p_from, dtype=float)
    vec = np.asarray(p_to, dtype=float) - p0
    length = float(np.linalg.norm(vec))
    if length < 1e-9:
        return True, 0.0
    direction = vec / length
    travelled = 0.0
    for _ in range(max_marches):
        geomid = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(m, d, p0, direction, None, 1, -1, geomid)
        if dist < 0 or travelled + dist >= length - tolerance:
            return True, length
        hit_body = m.geom_bodyid[int(geomid[0])]
        if hit_body in excluded:
            step = dist + 1e-4
            p0 = p0 + step * direction
            travelled += step
            continue
        return False, travelled + dist
    return False, travelled


# --------------------------------------------------------------------------- #
# Blackboard + affordance dataclasses.
# --------------------------------------------------------------------------- #
@dataclass
class PickBlackboard:
    client: object = None
    name: str = ""                 # articulation name in PIE
    object_actor_id: str = ""
    ee_task: int = 0
    damping_task: int = 2
    twist_task: int = 3            # twist-follow task spec index (base hold)
    affordance: object = None      # Affordance
    fail_reason: str = ""
    home_xy: tuple = (0.0, 0.0)
    q_cup: np.ndarray = None


@dataclass
class Affordance:
    point: np.ndarray
    normal: np.ndarray
    quat_cup_down: np.ndarray
    waypoints: list = field(default_factory=list)


# --------------------------------------------------------------------------- #
# Synced reads (Tracker pattern, generalized to any site by suffix).
# --------------------------------------------------------------------------- #
def synced_site_pose(client, site_suffix: str):
    """Direct-dip synced read of a site's world pose — the `synced_ee()`
    pattern from tidybot_twist_follow_demo.py / tidybot_guarded_reach_demo.py,
    generalized from the hardcoded pinch_site Tracker to any site by suffix.
    Steps once in direct mode to force a fresh physics/forward pass, reads
    site_xpos/site_xmat off the mirror, then restores live mode. Returns
    (pos: np.ndarray[3], quat_wxyz: np.ndarray[4])."""
    client.runtime.set_mode("direct")
    client.step(n_steps=1)
    m, d = client.model, client.data
    site_id = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_SITE, m.nsite, site_suffix)
    if site_id < 0:
        client.runtime.set_mode("live")
        client.runtime.set_paused(paused=False)
        raise RuntimeError(f"site suffix {site_suffix!r} not found in compiled model")
    mujoco.mj_forward(m, d)
    pos = np.array(d.site_xpos[site_id]).copy()
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.array(d.site_xmat[site_id]).reshape(9))
    client.runtime.set_mode("live")
    client.runtime.set_paused(paused=False)
    return pos, quat


def _hold_suction(client, name: str) -> None:
    """Re-assert suction=1.0. A direct-mode client.step (which every synced
    physics read does) ZEROS actuator NetworkValues server-side — an engine
    behavior (the step pushes the client's zero ctrl array), so any synced read
    silently un-sets an engaged suction. Grip/verify phases must re-assert after
    every stepping read. (Engine follow-up: direct-step should preserve
    NetworkValues it isn't explicitly overwriting.)"""
    try:
        client.runtime.set_suction(articulation=name, value=1.0)
    except Exception:
        pass  # never let a re-assert crash a tick


def _actor_xy_by_name(client, ue_name: str):
    """Live PIE actor xy (see _actor_z_by_name)."""
    for r in client.outliner.find_actors(class_filter="StaticMeshActor",
                                         in_pie=True):
        if r.name == ue_name:
            return np.array([float(r.location[0]), float(r.location[1])])
    raise RuntimeError(f"actor {ue_name!r} not found in PIE")


def _hull_world_aabb(client, ue_name: str):
    """Live world AABB of a quick-converted body's collision hull, from the
    client mirror (direct-dip synced read, same pattern as synced_site_pose).
    Unlike the actor PIVOT (a fixed point that equals the center-top only in
    the nominal flat pose), this tracks the object's ACTUAL geometry through
    flips and tilts. Returns (min3, max3) as np arrays; raises RuntimeError
    when no mirror body matches."""
    client.runtime.set_mode("direct")
    client.step(n_steps=1)
    m, d = client.model, client.data
    mujoco.mj_forward(m, d)
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    found = False
    for bid in range(m.nbody):
        bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
        if ue_name not in bname:
            continue
        adr, num = m.body_geomadr[bid], m.body_geomnum[bid]
        for gid in range(adr, adr + num):
            if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[gid]
                v0, nv = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                w = (d.geom_xmat[gid].reshape(3, 3)
                     @ m.mesh_vert[v0:v0 + nv].T).T + d.geom_xpos[gid]
                lo = np.minimum(lo, w.min(axis=0))
                hi = np.maximum(hi, w.max(axis=0))
            else:  # primitive geom: bounding-sphere box
                r = float(m.geom_rbound[gid])
                lo = np.minimum(lo, d.geom_xpos[gid] - r)
                hi = np.maximum(hi, d.geom_xpos[gid] + r)
        found = True
        break
    client.runtime.set_mode("live")
    client.runtime.set_paused(paused=False)
    if not found:
        raise RuntimeError(f"no mirror body matches {ue_name!r}")
    return lo, hi


def _actor_z_by_name(client, ue_name: str) -> float:
    """Live PIE actor z (engine truth — the actor follows the MuJoCo body).
    NOTE: for the Fab books the actor pivot is the mesh BOTTOM."""
    for r in client.outliner.find_actors(class_filter="StaticMeshActor",
                                         in_pie=True):
        if r.name == ue_name:
            return float(r.location[2])
    raise RuntimeError(f"actor {ue_name!r} not found in PIE")


def _object_z(client, object_actor_id: str) -> float:
    """Read the object's LIVE PHYSICS z (its MuJoCo body world pos via the synced
    mirror). find_actors(in_pie=True) must NOT be used here: for a spawned free
    body it returns the actor ROOT transform, which stays FROZEN at the spawn
    location and does not track physics — confirmed live (actor z stuck at 0.66
    while the physics body was on the floor at 0.05), so a lift is invisible to
    it. The body is matched by actor_id suffix (import may prefix the name).

    NAME fallback (additive, Fab actors): a quick-converted Fab actor (e.g. a
    book) has NO compiled MuJoCo body at all — it was never imported into the
    MJCF, so it can't suffer the frozen-root/physics-mismatch this function
    guards against. When no compiled body matches (bid < 0), fall back to the
    live actor position via find_actors, matched by actor_id OR by UE NAME —
    this is how bb.object_actor_id = "SM_Book_125" makes
    VerifyAttach / Release work unchanged for a Fab pick."""
    m = getattr(client, "model", None)
    bid = (resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody, object_actor_id)
           if m is not None else -1)
    if bid < 0:
        outliner = getattr(client, "outliner", None)
        if outliner is not None:
            for r in outliner.find_actors(class_filter="StaticMeshActor", in_pie=True):
                if getattr(r, "actor_id", None) == object_actor_id or r.name == object_actor_id:
                    return float(r.location[2])
        raise RuntimeError(f"object body {object_actor_id!r} not found in compiled model")
    client.runtime.set_mode("direct")
    client.step(n_steps=1)
    d = client.data
    z = float(d.xpos[bid][2])
    client.runtime.set_mode("live")
    client.runtime.set_paused(paused=False)
    return z


def _synced_planned_q(client, joints):
    """Planned-joint vector from a synced physics read. PRE-suction only (a
    direct-mode step zeroes actuator NetworkValues server-side). Joint names
    are SUFFIX-resolved (tidybot_planned_reach_probe.py's synced_q pattern):
    the importer prefixes compiled joint names (e.g.
    'tidybot_suction_ue_C_0_joint_x'), so a bare mj_name2id returns -1 and
    jnt_qposadr[-1] would read garbage."""
    client.step(n_steps=1)
    m, d = client.model, client.data
    out = np.empty(len(joints))
    for i, jn in enumerate(joints):
        jid = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, jn)
        out[i] = d.qpos[m.jnt_qposadr[jid]]
    return out


def _base_xy(client) -> np.ndarray:
    """Base WORLD xy: the mirror xpos of the body the base slides move. NOT
    the joint qpos — joint_x/joint_y are relative to the robot's SPAWN frame,
    so at a non-origin spawn (HomeInterior: (-10,-18)) qpos reads were ~14 m
    off in world terms: StageAt sorted its staging ring from the world ORIGIN
    (trying far/blocked candidates first) and would have verified arrivals
    against the wrong frame. Same world-vs-joint disease as the planner's
    goal_ik seeding (fixed in df03564); origin spawns masked it in-house."""
    client.step(n_steps=1)   # sync the mirror first (xpos is zeros pre-step)
    m = client.model
    jy = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, "joint_y")
    if jy < 0:
        raise RuntimeError("base joint not found")
    bid = int(m.jnt_bodyid[jy])   # moved by joint_x (ancestor) + joint_y
    d = client.data
    return np.array([float(d.xpos[bid][0]), float(d.xpos[bid][1])])


def resolve_affordance(client, body_suffix: str, prefix: str = "affordance_suction") -> Affordance:
    """Resolve the most-upward-facing `affordance_suction*` site on the named
    body (suffix-matched, e.g. 'pick_box') — the v1 heuristic (design doc
    2026-07-16). Site poses are read via a synced direct dip (the Tracker
    pattern). Affordance convention: site position = contact point, site
    z-axis = outward surface normal (approach direction is -z)."""
    client.runtime.set_mode("direct")
    client.step(n_steps=1)
    m, d = client.model, client.data
    mujoco.mj_forward(m, d)

    body_id = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody, body_suffix)
    if body_id < 0:
        client.runtime.set_mode("live")
        client.runtime.set_paused(paused=False)
        raise RuntimeError(f"body suffix {body_suffix!r} not found in compiled model")

    best_id, best_up = -1, -2.0
    for s in range(m.nsite):
        if m.site_bodyid[s] != body_id:
            continue
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SITE, s)
        if not nm or prefix not in nm:
            continue
        z_axis = np.array(d.site_xmat[s]).reshape(3, 3)[:, 2]
        up = float(z_axis[2])
        if up > best_up:
            best_up, best_id = up, s

    if best_id < 0:
        client.runtime.set_mode("live")
        client.runtime.set_paused(paused=False)
        raise RuntimeError(f"no {prefix!r} site found on body {body_suffix!r}")

    point = np.array(d.site_xpos[best_id]).copy()
    normal = np.array(d.site_xmat[best_id]).reshape(3, 3)[:, 2].copy()

    client.runtime.set_mode("live")
    client.runtime.set_paused(paused=False)

    quat = cup_down_quat(normal)
    waypoints = approach_waypoints(point, normal)
    return Affordance(point=point, normal=normal, quat_cup_down=quat, waypoints=waypoints)


# --------------------------------------------------------------------------- #
# py_trees behaviours.
# --------------------------------------------------------------------------- #
class Drive(py_trees.behaviour.Behaviour):
    """Bus-driven nav transit: set_nav_goal once (initialise), then poll
    get_nav_status every tick — nothing is streamed, the twist bus drives the
    base in-engine. Relaxes the lazy-base damping task (bb.damping_task) for
    the transit and restores it on arrival, live task_costs — lifted verbatim
    from tidybot_twist_follow_demo.py Phase A. Aborts if dist-to-goal jumps
    back up after progress was made (physics-auto-reset guard, brief-mandated:
    the MuJoCo QACC auto-reset teleports the robot to spawn, so distance-to-
    goal jumps back toward its initial value in a single poll; threshold
    tuned live) rather than silently re-driving across a reset."""

    def __init__(self, name, bb, goal_xy, timeout_s: float = NAV_TIMEOUT_S,
                 max_projection: float = None):
        super().__init__(name)
        self.bb = bb
        self.goal_xy = goal_xy
        self.timeout_s = float(timeout_s)
        self.max_projection = max_projection
        self._deadline = None
        self._min_dist = None

    def initialise(self):
        bb = self.bb
        client = bb.client
        client._rpc_configure_controller(
            articulation=bb.name,
            params={"task_costs": {str(bb.damping_task): {"cost": 0.05}}},
        )
        kw = {"articulation": bb.name, "x": self.goal_xy[0], "y": self.goal_xy[1]}
        if self.max_projection is not None:
            kw["max_projection"] = float(self.max_projection)
        g = client.runtime.set_nav_goal(**kw)
        self._min_dist = None
        if not g.get("accepted"):
            detail = ""
            if g.get("reason"):
                detail = f" ({g['reason']}, proj {g.get('projection_m', 0):.2f} m)"
            bb.fail_reason = f"Drive[{self.name}]: goal {self.goal_xy} rejected{detail}"
            self._deadline = -1.0  # sentinel: update() fails immediately
            return
        sp = g.get("start_projection_m")
        if sp is not None and sp > 0.05:
            self.logger.info(f"start {sp:.2f} m off-mesh — recovery leg")
        self._deadline = time.time() + self.timeout_s

    def update(self):
        bb = self.bb
        client = bb.client
        if self._deadline is None or self._deadline < 0:
            return py_trees.common.Status.FAILURE

        s = client.runtime.get_nav_status(articulation=bb.name)
        state = s["state"]
        dist = float(s["distance_to_goal"])

        if self._min_dist is None or dist < self._min_dist:
            self._min_dist = dist
        elif dist > self._min_dist + RESET_FINGERPRINT_JUMP_M:
            bb.fail_reason = (
                f"Drive[{self.name}]: dist-to-goal jumped {self._min_dist:.2f} -> "
                f"{dist:.2f} m -- physics-auto-reset fingerprint, aborting "
                "(never re-drive across a reset)"
            )
            return py_trees.common.Status.FAILURE

        if state == "arrived":
            client._rpc_configure_controller(
                articulation=bb.name,
                params={"task_costs": {str(bb.damping_task): {"cost": 5.0}}},
            )
            return py_trees.common.Status.SUCCESS
        if state == "failed":
            bb.fail_reason = f"Drive[{self.name}]: nav failed (state={state})"
            return py_trees.common.Status.FAILURE
        if time.time() >= self._deadline:
            bb.fail_reason = f"Drive[{self.name}]: timed out after {self.timeout_s}s"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"Drive[{self.name}]: failed"


def staging_ring(target_xy, station_aabb, here_xy, erode_m: float = 0.58,
                 radii=(0.95, 1.10), n_bearings: int = 16):
    """Staging candidates around a reach target beside a station: ring points
    at `radii` from `target_xy`, keeping only points OUTSIDE the station AABB
    inflated by the navmesh erosion (so the requested point is itself
    navigable, not silently projected), sorted by drive distance from
    `here_xy`. Pure — no client."""
    xmin, xmax, ymin, ymax = station_aabb
    here = np.asarray(here_xy, dtype=float)
    out = []
    for r in radii:
        for k in range(n_bearings):
            th = 2.0 * np.pi * k / n_bearings
            p = np.array([target_xy[0] + r * np.cos(th),
                          target_xy[1] + r * np.sin(th)])
            if (xmin - erode_m <= p[0] <= xmax + erode_m) and \
                    (ymin - erode_m <= p[1] <= ymax + erode_m):
                continue
            out.append(p)
    # Inner radius FIRST, drive distance second: reach quality dominates
    # drive cost. Sorted purely by drive distance the outer ring kept
    # winning, parking the base ~1.2 m out where the descend pins ~3 cm
    # short of the object (annulus edge + twist task holding the base) and
    # the grab seats weakly — carries dropped the object mid-transit (live).
    # At ~1.0 m every descend tracks to its target exactly.
    tgt = np.asarray(target_xy, dtype=float)
    out.sort(key=lambda p: (round(float(np.linalg.norm(p - tgt)), 3),
                            float(np.linalg.norm(p - here))))
    return out


class StageAt(py_trees.behaviour.Behaviour):
    """Staging-policy-as-a-skill: ring candidates around the reach target
    (staging_ring), each attempted with Drive in strict mode
    (max_projection) and VERIFIED on arrival against the REQUESTED point
    (defense-in-depth over the strict mode). SUCCESS on the first verified
    candidate; FAILURE when the ring / max_tries are exhausted. Subsumes the
    fab demo's inline staging loop."""

    def __init__(self, name, bb, station_aabb, target_xy, erode_m: float = 0.58,
                 verify_m: float = 0.30, max_projection: float = 0.30,
                 max_tries: int = 4, timeout_s: float = NAV_TIMEOUT_S):
        super().__init__(name)
        self.bb = bb
        self.station_aabb = station_aabb
        self.target_xy = target_xy
        self.erode_m = float(erode_m)
        self.verify_m = float(verify_m)
        self.max_projection = float(max_projection)
        self.max_tries = int(max_tries)
        self.timeout_s = float(timeout_s)
        self._candidates = None
        self._tries = 0
        self._drive = None

    def _next_drive(self):
        """Arm a Drive at the next candidate; None when exhausted."""
        while self._candidates and self._tries < self.max_tries:
            cand = self._candidates.pop(0)
            self._tries += 1
            d = Drive(f"{self.name}/cand{self._tries}", self.bb,
                      (float(cand[0]), float(cand[1])),
                      timeout_s=self.timeout_s,
                      max_projection=self.max_projection)
            d.initialise()
            return d, cand
        return None, None

    def initialise(self):
        here = _base_xy(self.bb.client)
        self._candidates = staging_ring(self.target_xy, self.station_aabb,
                                        here, self.erode_m)
        self._tries = 0
        self._last_reason = ""
        self.bb.fail_reason = ""
        self._drive, self._cand = self._next_drive()

    def update(self):
        bb = self.bb
        if self._drive is None:
            if not bb.fail_reason:
                bb.fail_reason = (f"StageAt[{self.name}]: ring exhausted "
                                  f"({self._tries} tries; last: {self._last_reason or 'n/a'})")
            return py_trees.common.Status.FAILURE
        status = self._drive.update()
        if status == py_trees.common.Status.RUNNING:
            return py_trees.common.Status.RUNNING
        if status == py_trees.common.Status.SUCCESS:
            actual = _base_xy(bb.client)
            miss = float(np.linalg.norm(actual - self._cand))
            if miss <= self.verify_m:
                print(f"[stage] {self.name}: staged at {np.round(actual, 2)} "
                      f"({miss:.2f} m from requested)", flush=True)
                return py_trees.common.Status.SUCCESS
            self._last_reason = (f"arrival {miss:.2f} m off requested "
                                 f"{np.round(self._cand, 2)}")
            print(f"[stage] {self.name}: {self._last_reason} — next candidate",
                  flush=True)
        else:
            # Candidate rejected/failed: keep its reason for the exhaustion
            # message and SAY it — swallowing per-candidate reasons cost a
            # live diagnosis (every reject looked identical from outside).
            self._last_reason = bb.fail_reason or "(no reason reported)"
            print(f"[stage] {self.name}: candidate {np.round(self._cand, 2)} "
                  f"failed: {self._last_reason} — next candidate", flush=True)
        # rejected / failed / unverified: advance the ring
        bb.fail_reason = ""
        self._drive, self._cand = self._next_drive()
        if self._drive is None:
            bb.fail_reason = (f"StageAt[{self.name}]: ring exhausted "
                              f"({self._tries} tries; last: {self._last_reason or 'n/a'})")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"StageAt[{self.name}]: failed"


class ResolveAffordance(py_trees.behaviour.Behaviour):
    """One-shot: resolve the pick object's affordance frame onto the
    blackboard (bb.affordance, bb.q_cup) via resolve_affordance's
    most-upward-facing site heuristic."""

    def __init__(self, name, bb, body_suffix: str, prefix: str = "affordance_suction"):
        super().__init__(name)
        self.bb = bb
        self.body_suffix = body_suffix
        self.prefix = prefix

    def update(self):
        bb = self.bb
        try:
            aff = resolve_affordance(bb.client, self.body_suffix, prefix=self.prefix)
        except RuntimeError as exc:
            bb.fail_reason = f"ResolveAffordance[{self.name}]: {exc}"
            return py_trees.common.Status.FAILURE
        bb.affordance = aff
        bb.q_cup = aff.quat_cup_down
        return py_trees.common.Status.SUCCESS


class ResolveActorTop(py_trees.behaviour.Behaviour):
    """Fab-object affordance: no MJCF affordance site exists on a
    quick-converted actor, so build the Affordance from the live actor pose.
    PIVOT SEMANTICS (measured live): in PIE the quick-converted actor's pivot
    is the hull TOP face — resting upright on the table it reads
    table_top + thickness; dropped upside-down on the floor it reads 0. (The
    EDITOR-world pivot is the mesh bottom — do not calibrate against that.)
    So the affordance point z is the pivot itself, no offset. Replaces
    ResolveAffordance in Fab trees; writes bb.affordance + bb.q_cup."""

    def __init__(self, name, bb, object_name: str, half_thickness: float):
        super().__init__(name)
        self.bb = bb
        self.object_name = object_name
        self.half = float(half_thickness)
        self._error = None

    def initialise(self):
        self._error = None
        try:
            up = np.array([0.0, 0.0, 1.0])
            q = cup_down_quat(up)
            # Preferred: the live hull's world AABB — center xy + true top z,
            # robust to the object landing flipped or tilted (the actor pivot
            # is only the center-top in the nominal flat pose; aiming at it
            # after a non-nominal place produced edge grabs, seen live).
            try:
                lo, hi = _hull_world_aabb(self.bb.client, self.object_name)
                point = np.array([(lo[0] + hi[0]) / 2.0,
                                  (lo[1] + hi[1]) / 2.0, hi[2]])
                print(f"[resolve] {self.name}: hull top {np.round(point, 3)} "
                      f"(pivot fallback not needed)", flush=True)
            except Exception:
                # Fallback: actor pivot (= top face in the nominal pose).
                found = None
                for r in self.bb.client.outliner.find_actors(
                        class_filter="StaticMeshActor", in_pie=True):
                    if r.name == self.object_name:
                        found = np.array(r.location, dtype=float)
                        break
                if found is None:
                    raise RuntimeError(
                        f"actor {self.object_name!r} not found in PIE")
                point = np.array([found[0], found[1], found[2]])
            self.bb.affordance = Affordance(point=point, normal=up,
                                            quat_cup_down=q)
            self.bb.q_cup = q
        except RuntimeError as e:
            self._error = str(e)

    def update(self):
        if self._error is not None:
            self.bb.fail_reason = f"ResolveActorTop[{self.name}]: {self._error}"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class CarryTransit(py_trees.behaviour.Behaviour):
    """Loaded transit: StowCarry's tuck (EE ramped near the base, EE task
    disabled, posture holds — transit doctrine), then a bus Drive with the
    suction re-asserted every tick and a drop guard (held object's live z
    below carry_z_min => attach lost mid-drive; fail fast, no recovery).
    Presents as ONE skill to the tree."""

    def __init__(self, name, bb, goal_xy, object_name: str,
                 carry_z_min: float = 0.45, tuck_duration: float = 3.0,
                 timeout_s: float = NAV_TIMEOUT_S):
        super().__init__(name)
        self.bb = bb
        self.goal_xy = goal_xy
        self.object_name = object_name
        self.carry_z_min = float(carry_z_min)
        self.tuck_duration = float(tuck_duration)
        self.timeout_s = float(timeout_s)
        self._stow = None
        self._drive = None

    def initialise(self):
        self._stow = StowCarry(f"{self.name}/tuck", self.bb,
                               duration=self.tuck_duration)
        self._stow.initialise()
        self._drive = None

    def update(self):
        bb = self.bb
        if self._drive is None:
            status = self._stow.update()
            if status == py_trees.common.Status.RUNNING:
                return py_trees.common.Status.RUNNING
            if status == py_trees.common.Status.FAILURE:
                return py_trees.common.Status.FAILURE
            self._drive = Drive(f"{self.name}/drive", bb, self.goal_xy,
                                timeout_s=self.timeout_s)
            self._drive.initialise()
        _hold_suction(bb.client, bb.name)
        try:
            z = _actor_z_by_name(bb.client, self.object_name)
        except RuntimeError as e:
            bb.fail_reason = f"CarryTransit[{self.name}]: {e}"
            return py_trees.common.Status.FAILURE
        if z < self.carry_z_min:
            bb.fail_reason = (f"CarryTransit[{self.name}]: object dropped "
                              f"mid-transit (z={z:.2f} < {self.carry_z_min})")
            return py_trees.common.Status.FAILURE
        return self._drive.update()

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"CarryTransit[{self.name}]: failed"


class Reachable(py_trees.behaviour.Behaviour):
    """Condition: is the resolved affordance point within the bent-arm reach
    annulus (0.30 m <= shoulder distance <= 0.85 m, empirically mapped
    2026-07-16) from a given (parked) base xy? One-shot, no RUNNING."""

    def __init__(self, name, bb, shoulder_xy, r_min: float = 0.30, r_max: float = 0.85):
        super().__init__(name)
        self.bb = bb
        self.shoulder_xy = shoulder_xy
        self.r_min = r_min
        self.r_max = r_max

    def update(self):
        bb = self.bb
        if bb.affordance is None:
            bb.fail_reason = f"Reachable[{self.name}]: no affordance resolved"
            return py_trees.common.Status.FAILURE
        shoulder = np.array([self.shoulder_xy[0], self.shoulder_xy[1], SHOULDER_HEIGHT_M])
        if reach_annulus_ok(shoulder, bb.affordance.point, self.r_min, self.r_max):
            return py_trees.common.Status.SUCCESS
        bb.fail_reason = (
            f"Reachable[{self.name}]: affordance point {np.round(bb.affordance.point, 3)} "
            f"outside the reach annulus from shoulder {np.round(shoulder, 3)}"
        )
        return py_trees.common.Status.FAILURE


class CorridorClear(py_trees.behaviour.Behaviour):
    """Condition: is the straight-line corridor from p_from to p_to free of
    non-excluded geometry? p_from / p_to may be a fixed 3-vector or a
    zero-arg callable resolved at tick time (so it can reference an
    affordance waypoint resolved earlier in the same tree tick, e.g.
    `lambda: bb.affordance.waypoints[0]`). Runs a synced dip first so the
    client-side mj_ray sees a fresh mirror (corridor_clear's contract).
    One-shot: v1 premise is no motion planner, so a blocked corridor is a
    loud abort, not a replan."""

    def __init__(self, name, bb, p_from, p_to, exclude_body_suffixes):
        super().__init__(name)
        self.bb = bb
        self.p_from = p_from
        self.p_to = p_to
        self.exclude_body_suffixes = list(exclude_body_suffixes)

    @staticmethod
    def _resolve(v):
        return np.asarray(v() if callable(v) else v, dtype=float)

    def update(self):
        bb = self.bb
        client = bb.client
        p_from = self._resolve(self.p_from)
        p_to = self._resolve(self.p_to)
        client.runtime.set_mode("direct")
        client.step(n_steps=1)
        clear, dist = corridor_clear(client, p_from, p_to, self.exclude_body_suffixes)
        client.runtime.set_mode("live")
        client.runtime.set_paused(paused=False)
        if clear:
            return py_trees.common.Status.SUCCESS
        bb.fail_reason = (
            f"CorridorClear[{self.name}]: blocked at {dist:.3f} m along "
            f"{np.round(p_from, 3)} -> {np.round(p_to, 3)} "
            "(v1 premise: no motion planner -- abort; RRT-Connect is the named upgrade)"
        )
        return py_trees.common.Status.FAILURE


class SetBaseAssist(py_trees.behaviour.Behaviour):
    """Relax (enabled=True) or restore (enabled=False) the base hold so the
    whole-body QP can recruit the base to help the arm reach — the guarded
    table-reach capability. ON drops the twist-follow and lazy-base damping
    task costs so the EE frame task pulls the base forward the last few cm (the
    base x table CollisionAvoidance limit stops it short of the surface); OFF
    restores them for a stable carry. Uses the live task_costs surface (a sparse
    map by spec index — orthogonal to task_enabled, so it does not disturb the
    EE-task gating the reach skills manage). SUCCESS immediately."""

    def __init__(self, name, bb, enabled: bool,
                 assist_twist_cost: float = 0.02, assist_damping_cost: float = 0.05,
                 hold_twist_cost: float = 1.0, hold_damping_cost: float = 5.0):
        super().__init__(name)
        self.bb = bb
        self.enabled = enabled
        self.assist_twist_cost = assist_twist_cost
        self.assist_damping_cost = assist_damping_cost
        self.hold_twist_cost = hold_twist_cost
        self.hold_damping_cost = hold_damping_cost

    def update(self):
        bb = self.bb
        tw = self.assist_twist_cost if self.enabled else self.hold_twist_cost
        dp = self.assist_damping_cost if self.enabled else self.hold_damping_cost
        try:
            bb.client._rpc_configure_controller(
                articulation=bb.name,
                params={"task_costs": {str(bb.twist_task): {"cost": tw},
                                       str(bb.damping_task): {"cost": dp}}},
            )
        except Exception as e:  # RPC failure -> fail the node, don't crash the tree
            bb.fail_reason = f"SetBaseAssist[{self.name}]: {e}"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class ReachRamp(py_trees.behaviour.Behaviour):
    """Seed-then-enable EE-task ramp: seeds the manual target at the CURRENT
    cup_site pose while the EE task is still disabled (enabling against a
    stale bind-pose target yanks — tidybot_twist_follow_demo.py Phase B),
    THEN enables all four tasks, then ramps the EE position linearly through
    bb.<waypoints_key> (a list of world points) over `duration` seconds total
    (split evenly across legs). Orientation slerps from the seed pose to
    bb.<quat_key> across the FIRST leg (then holds): snapping the target to
    cup-down in one tick while the cup is still stowed next to the mast is
    infeasible for the arm alone, so the whole-body QP recruits the base --
    live-measured as a ~0.6 m backward lunge at reach start. The wall-clock
    position ramp is tidybot_guarded_reach_demo.py's Phase B pattern."""

    def __init__(self, name, bb, waypoints_key, quat_key, duration: float = 6.0):
        super().__init__(name)
        self.bb = bb
        self.waypoints_key = waypoints_key
        self.quat_key = quat_key
        self.duration = float(duration)
        self._legs = []
        self._leg_idx = 0
        self._leg_start_pose = None
        self._leg_t0 = None
        self._per_leg = None
        self._quat = None
        self._q0 = None

    def initialise(self):
        bb = self.bb
        client = bb.client
        waypoints = getattr(bb, self.waypoints_key, None)
        if not waypoints:
            bb.fail_reason = f"ReachRamp[{self.name}]: no waypoints under {self.waypoints_key!r}"
            self._legs = []
            return
        self._quat = np.asarray(getattr(bb, self.quat_key), dtype=float)

        try:
            p0, q0 = synced_site_pose(client, "cup_site")
        except RuntimeError as e:
            bb.fail_reason = f"ReachRamp[{self.name}]: {e}"
            self._legs = []
            return
        stream_target(client, bb.name, p0, q0)  # seed while disabled -- no yank
        client._rpc_configure_controller(
            articulation=bb.name, params={"task_enabled": [True, True, True, True]},
        )

        self._q0 = np.asarray(q0, dtype=float)
        self._legs = [np.asarray(w, dtype=float) for w in waypoints]
        self._leg_idx = 0
        self._leg_start_pose = p0
        self._leg_t0 = time.time()
        self._per_leg = self.duration / max(1, len(self._legs))

    def update(self):
        bb = self.bb
        if not self._legs:
            return py_trees.common.Status.FAILURE
        client = bb.client
        target = self._legs[self._leg_idx]
        a = min(1.0, (time.time() - self._leg_t0) / max(self._per_leg, 1e-6))
        pos = (1.0 - a) * self._leg_start_pose + a * target
        quat = self._quat
        if self._leg_idx == 0 and self._q0 is not None:
            quat = quat_slerp(self._q0, self._quat, a)
        stream_target(client, bb.name, pos, quat)
        if a < 1.0:
            return py_trees.common.Status.RUNNING
        if self._leg_idx + 1 >= len(self._legs):
            return py_trees.common.Status.SUCCESS
        self._leg_idx += 1
        self._leg_start_pose = target
        self._leg_t0 = time.time()
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"ReachRamp[{self.name}]: failed"


class PlannedReach(py_trees.behaviour.Behaviour):
    """Whole-body planned reach to the affordance pre-grasp pose: plans an
    RRT-Connect path in initialise (blocking, seconds — acceptable at BT
    rate for v1), then streams posture_target waypoints SELF-PACED (advances
    to the next waypoint only once the current one is tracked, never on
    wall-clock). SUCCESS when the final waypoint settles within tolerance;
    FAILURE on plan error or a genuine stall (tracking error stops improving).
    Replaces ReachRamp's straight-line carpet — the planned BASE path
    executes exactly instead of being rediscovered by the greedy QP.

    Self-paced advancement is deliberately MORE conservative than the probe's
    proven Phase C (which also clock-gated each waypoint): here the QP is never
    yanked toward a far-ahead waypoint across an edge the planner never
    collision-checked — slow-but-correct tracking just takes longer instead of
    failing. The handoff CONFIG below is byte-identical to the probe's, but this
    streaming discipline was not itself run at the live gate (the probe was), so
    a full pick-tree live re-validation with PlannedReach is a recorded next step.

    Handoff contract (live-validated in tidybot_planned_reach_probe.py's
    Phase C/D): frame + twist_follow tasks disabled during execution;
    posture cost bumped AND the lazy-base damping cost relaxed (5.0 -> 0.05)
    so the streamed base waypoints aren't fought by the base-velocity
    penalty — both restored (damping back to 5.0, posture back to 1e-3) and
    the posture latch cleared in terminate() regardless of outcome."""

    PRE_GRASP_M = 0.03
    TRACK_TOL = 0.15
    SETTLE_TOL = 0.05
    # Progress-based watchdog: a 2-waypoint plan streams the goal directly and the
    # QP chases it at the velocity limit, so a big-but-valid joint move (e.g. a
    # 200 deg wrist roll = 7 s) keeps err high for many seconds WHILE making real
    # progress. Trip only when the best tracking error stops improving — never on
    # a fixed error timer (the old TRACK_GRACE_S=4 s false-tripped any reach that
    # took longer than 4 s, e.g. a far surface or a large base reorientation;
    # surfaced live on a house table pick).
    NO_PROGRESS_S = 6.0      # fail if best err hasn't improved for this long
    PROGRESS_EPS = 0.02      # min err drop that counts as progress
    PLAN_ATTEMPTS = 3        # RRT-Connect is stochastic: the place-hover query
                             # exhausted 2000 iters on ~2 of 3 live runs and
                             # planned fine on the third — re-rolling the seed
                             # is the cheap remedy before the streamed-approach
                             # fallback (spec's documented escalation)

    def __init__(self, name, bb, exclude_body=None, hold_suction=False):
        super().__init__(name)
        self.bb = bb
        self.exclude_body = exclude_body   # held-object suffix to park (see planner)
        self.hold_suction = bool(hold_suction)  # re-assert suction in the
        #   streamed fallback (synced reads zero it server-side) — pass True
        #   ONLY when an object is held (a place reach); on a pick approach
        #   early suction re-creates the peel-grab bug.
        self._plan = None
        self._joints = None
        self._wp_idx = 0
        self._best_err = float("inf")
        self._last_improve_t = None
        self._failed = None
        self._fallback_goal = None

    def initialise(self):
        from urlab_planner import PLANNED_JOINTS, PlanError, plan_reach
        bb = self.bb
        client = bb.client
        self._joints = PLANNED_JOINTS
        self._wp_idx = 0
        self._failed = None
        self._best_err = float("inf")
        self._fallback_goal = None
        # Guard a missing affordance (matches ReachRamp/Reachable/DescendEngage):
        # ResolveAffordance must have run first and published bb.affordance
        # (an Affordance dataclass: point, normal, quat_cup_down, waypoints)
        # and bb.q_cup.
        if bb.affordance is None:
            bb.fail_reason = f"PlannedReach[{self.name}]: no affordance resolved"
            self._failed = bb.fail_reason
            return
        aff = bb.affordance
        pre_grasp = np.asarray(aff.point, dtype=float) \
            + self.PRE_GRASP_M * np.asarray(aff.normal, dtype=float)
        self._plan = None
        for attempt in range(1, self.PLAN_ATTEMPTS + 1):
            try:
                # seed varies per attempt — the planner is DETERMINISTIC
                # (seed=0 throughout urlab_planner), so retrying with the
                # default seed replays the identical failing tree.
                self._plan = plan_reach(client, pre_grasp,
                                        np.asarray(bb.q_cup, dtype=float),
                                        exclude_body_suffix=self.exclude_body,
                                        seed=attempt - 1)
                if attempt > 1:
                    print(f"[reach] {self.name}: plan found on attempt {attempt}",
                          flush=True)
                break
            except PlanError as e:
                if attempt < self.PLAN_ATTEMPTS:
                    print(f"[reach] {self.name}: plan attempt {attempt} failed "
                          f"({e.stage}) — retrying", flush=True)
                    continue
                # Streamed-approach fallback (the spec's escalation): some
                # queries are structurally RRT-hostile (side-table region:
                # exhaustion across FRESH seeds with 2-5 goal configs). Carrot
                # the cup to the pre-grasp; the whole-body QP tracks (damping
                # relaxed so the base assists) and the guards still insure.
                # terminate() restores costs + seed-then-enables regardless.
                print(f"[reach] {self.name}: plan {e.stage} "
                      f"({self.PLAN_ATTEMPTS} attempts) — streamed-approach "
                      f"fallback", flush=True)
                try:
                    p0, q0 = synced_site_pose(client, "cup_site")
                except RuntimeError as e2:
                    bb.fail_reason = f"PlannedReach[{self.name}]: {e2}"
                    self._failed = bb.fail_reason
                    return
                stream_target(client, bb.name, p0, q0)  # seed-then-enable
                client._rpc_configure_controller(
                    articulation=bb.name,
                    params={"task_enabled": [True, True, True, False],
                            "task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})
                self._fallback_goal = np.asarray(pre_grasp, dtype=float)
                self._fb_quat = q0
                self._fb_best = float("inf")
                self._fb_improve_t = time.time()
                self._fb_t0 = time.time()
                return
            except Exception as e:
                # plan_reach can also raise ValueError (unresolved joint/site),
                # a mink import error, or an RPC error — deterministic, so no
                # retry: fail the tick cleanly rather than crash with a
                # traceback. No exec config has been sent yet, so there is
                # nothing to clean up.
                self._plan = None
                bb.fail_reason = f"PlannedReach[{self.name}]: plan failed: {e}"
                self._failed = bb.fail_reason
                return
        # Frame + twist_follow OFF, posture dominates; relax the lazy-base
        # damping (5.0 -> 0.05, the transit value) so it doesn't fight the
        # posture task driving the base through each planned waypoint (the
        # planner deliberately repositions the base) — mirrors the probe's
        # proven Phase C configure() exactly.
        client._rpc_configure_controller(articulation=bb.name, params={
            "task_enabled": [False, True, True, False],
            "task_costs": {str(POSTURE_TASK): {"cost": 5.0},
                           str(DAMPING_TASK): {"cost": 0.05}},
        })
        self._last_improve_t = time.time()

    def update(self):
        if self._failed:
            self.feedback_message = self._failed
            return py_trees.common.Status.FAILURE
        bb = self.bb
        client = bb.client
        if self._fallback_goal is not None:
            # Streamed-approach fallback (armed in initialise on plan failure).
            if self.hold_suction:
                _hold_suction(client, bb.name)
            cur, _ = synced_site_pose(client, "cup_site")
            err = self._fallback_goal - cur
            n = float(np.linalg.norm(err))
            if n < self._fb_best - 0.01:
                self._fb_best = n
                self._fb_improve_t = time.time()
            plateau = (time.time() - self._fb_improve_t) > 4.0
            if n < 0.03 or (plateau and n < 0.10):
                print(f"[reach] {self.name}: fallback reached pre-grasp "
                      f"(err {n:.3f} m)", flush=True)
                return py_trees.common.Status.SUCCESS
            if plateau or time.time() - self._fb_t0 > 25.0:
                bb.fail_reason = (f"PlannedReach[{self.name}]: streamed "
                                  f"fallback stalled {n:.2f} m from pre-grasp")
                return py_trees.common.Status.FAILURE
            step = err if n <= 0.04 else err * (0.04 / n)
            stream_target(client, bb.name, cur + step, self._fb_quat)
            return py_trees.common.Status.RUNNING
        idx = self._wp_idx
        _t_wp, wp = self._plan.waypoints[idx]
        # Stream the CURRENT waypoint, then measure how well it is tracked.
        client._rpc_configure_controller(articulation=bb.name,
                                         params={"posture_target": wp})
        q = _synced_planned_q(client, self._joints)
        err = max(abs(q[i] - wp[jn]) for i, jn in enumerate(self._joints))
        is_last = idx == len(self._plan.waypoints) - 1

        # Progress bookkeeping: any meaningful drop in err resets the no-progress
        # timer, so a slow-but-advancing move never trips the watchdog.
        if err < self._best_err - self.PROGRESS_EPS:
            self._best_err = err
            self._last_improve_t = time.time()

        if err <= self.TRACK_TOL:
            if not is_last:
                # Advance ONLY when the current waypoint is reached (self-paced,
                # never on the clock).
                self._wp_idx += 1
                self._best_err = float("inf")
                self._last_improve_t = time.time()
                return py_trees.common.Status.RUNNING
            if err < self.SETTLE_TOL:
                return py_trees.common.Status.SUCCESS
            # Last waypoint tracked but not settled: fall through to the
            # no-progress guard so it can't spin RUNNING forever.

        # Trip only on genuine STALL (best err not improving), not on error still
        # being high while the QP legitimately drives a long move.
        if time.time() - self._last_improve_t > self.NO_PROGRESS_S:
            self.feedback_message = f"no progress at waypoint {idx} (err {err:.3f})"
            bb.fail_reason = (
                f"PlannedReach[{self.name}]: no progress for {self.NO_PROGRESS_S}s "
                f"at waypoint {idx} (err {err:.3f})"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        bb = self.bb
        params = {
            "posture_target": {},
            "task_costs": {str(POSTURE_TASK): {"cost": 1e-3},
                           str(DAMPING_TASK): {"cost": 5.0}},
            "task_enabled": [True, True, True, False],
        }
        # SEED-THEN-ENABLE (ReachRamp's rule): the frame task ran DISABLED during
        # the reach, so its manual target is stale — the bind pose (spawn EE),
        # metres from where the cup now is. Re-enabling it without a fresh target
        # makes the whole-body QP lunge at the bind pose and swing the base wildly
        # for the ticks before DescendEngage streams its own target (seen live as
        # a violent base veer during the hand-off). Seed the target at the CURRENT
        # cup pose in the SAME call that re-enables it so the frame task holds
        # position instead of yanking.
        try:
            cup_pos, cup_quat = synced_site_pose(bb.client, "cup_site")
            params["target_pos"] = [float(v) for v in cup_pos]
            params["target_quat"] = [float(v) for v in cup_quat]
        except Exception:
            pass  # best-effort seed; still restore the config below
        try:
            bb.client._rpc_configure_controller(articulation=bb.name, params=params)
        except Exception:
            pass
        if new_status == py_trees.common.Status.FAILURE and not bb.fail_reason:
            bb.fail_reason = f"PlannedReach[{self.name}]: failed"


class DescendEngage(py_trees.behaviour.Behaviour):
    """Descend the cup to HOVER just ABOVE the object and engage suction. Ramps
    from the pre-approach pose down to a target HOVER_M above the object's top
    surface (snapshot ONCE at initialise so it doesn't chase a nudged box). Does
    NOT press into the object: the adhesion actuator grabs within its 3 cm margin,
    so pressing a light box only knocks it off the table -- hovering + suction
    lets adhesion pull the box UP to the cup instead. Suction engages LATE, in the
    settle phase (cup stationary at the hover) -- firing it mid-descent grabbed the
    object across a moving 3-5 cm gap, peeling it up nearest-edge-first into a
    tilted corner-dangle (seen live on the Fab book). Suction fires only once
    the cup has CLOSED the vertical gap to ENGAGE_GAP_M — the settle phase
    starts on the ramp TIMER with the arm still lagging ~3 cm up, and engaging
    on phase entry re-created the peel (seen live: gap 0.030, book still came
    up tilted). A settle-timeout fallback engage means a slow-converging arm
    still hands off to VerifyAttach (the real gate) instead of never engaging;
    a short post-engage dwell lets adhesion seat the object flat before the
    lift. FAILURE only on a read error."""

    ENGAGE_GAP_M = 0.015    # vertical cup-to-surface gap that triggers suction
    ENGAGE_DWELL_S = 0.75   # post-engage hold so adhesion seats the object flat
    HOVER_M = 0.012         # hover the cup this far ABOVE the surface. Do NOT press
                            # into the object: the cup grabs within the adhesion
                            # margin (3 cm), so pressing only knocks the light box
                            # off the table. Hover + suction -> adhesion pulls it up.
    SETTLE_TIMEOUT_S = 8.0  # terminal convergence is slow at annulus-edge
                            # staging (live: still closing at 3 s — the old
                            # window forced a far fallback engage that only
                            # VerifyAttach's retry rescued)

    def __init__(self, name, bb, ramp_duration: float = 4.0):
        super().__init__(name)
        self.bb = bb
        self.ramp_duration = float(ramp_duration)
        self._quat = None
        self._start_pose = None
        self._surf = None       # object surface point, SNAPSHOT once (see initialise)
        self._pressed = None    # fixed descent target (surf + HOVER_M*normal, ABOVE)
        self._t0 = None
        self._engaged = False
        self._engage_t = None
        self._settling = False
        self._settle_t0 = None
        self._damping_relaxed = False
        self._init_error = None

    def initialise(self):
        bb = self.bb
        self._init_error = None
        self._engaged = False
        self._engage_t = None
        self._settling = False
        self._settle_t0 = None
        self._damping_relaxed = False
        if bb.affordance is None:
            self._init_error = "no affordance resolved"
            return
        try:
            # Snapshot the object's affordance pose ONCE, now, while it is
            # sitting still. Do NOT chase it live: the affordance site rides the
            # free body, so a live target lunges after the box the instant it is
            # nudged (the jumping IK marker) and amplifies a knock-off. A fixed
            # snapshot gives a clean straight-down descent to where the box IS.
            try:
                surf, quat = synced_site_pose(bb.client, "affordance_suction_top")
                R = np.zeros(9)
                mujoco.mju_quat2Mat(R, quat)
                normal = np.asarray(R).reshape(3, 3)[:, 2]
                normal = normal / (np.linalg.norm(normal) or 1.0)
            except RuntimeError:
                # No MJCF affordance site (a quick-converted Fab object) — use the
                # surface snapshot ResolveActorTop already put on the blackboard.
                surf = np.asarray(bb.affordance.point, dtype=float)
                normal = np.asarray(bb.affordance.normal, dtype=float)
                normal = normal / (np.linalg.norm(normal) or 1.0)
            # Descend at the cup's CURRENT orientation, not affordance.quat_cup_down.
            # PlannedReach lands the cup already pointing down (roll-agnostic IK), so
            # re-imposing quat_cup_down's fixed roll makes the frame task spin the
            # axisymmetric cup's wrist — and the whole-body QP recruits the BASE to
            # help, swinging it into furniture (seen live). Holding the landed
            # orientation makes the descent a pure straight-down move; the roll is a
            # don't-care for the grasp.
            p0, self._quat = synced_site_pose(bb.client, "cup_site")
        except RuntimeError as e:
            self._init_error = str(e)
            return
        self._quat = np.asarray(self._quat, dtype=float)
        self._surf = np.asarray(surf, dtype=float)
        self._pressed = self._surf + self.HOVER_M * normal  # hover ABOVE, don't press
        self._start_pose = p0
        # Relax the lazy-base damping for the descend (restored in terminate):
        # staged ~1.2 m out, the hover target sits at the edge of the cup-down
        # annulus and with damping 5.0 the QP parks with a ~3 cm steady error
        # (seen live: the settle plateau at gap 0.029-0.030 every run) rather
        # than creep the base forward. PlannedReach's proven relax pattern.
        bb.client._rpc_configure_controller(
            articulation=bb.name,
            params={"task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})
        self._damping_relaxed = True
        self._t0 = time.time()

    def update(self):
        bb = self.bb
        client = bb.client
        if self._init_error is not None:
            bb.fail_reason = f"DescendEngage[{self.name}]: {self._init_error}"
            return py_trees.common.Status.FAILURE

        try:
            cup_pos, _ = synced_site_pose(client, "cup_site")
        except RuntimeError as e:
            bb.fail_reason = f"DescendEngage[{self.name}]: {e}"
            return py_trees.common.Status.FAILURE

        pressed = self._pressed  # fixed snapshot target
        dist_to_surf = float(np.linalg.norm(cup_pos - self._surf))

        if self._engaged:
            _hold_suction(client, bb.name)  # the synced read above zeroed it

        if not self._settling:
            # Ramp phase: bounded-error descent from the pre-approach pose.
            a = min(1.0, (time.time() - self._t0) / max(self.ramp_duration, 1e-6))
            stream_target(client, bb.name, (1.0 - a) * self._start_pose + a * pressed, self._quat)
            if a >= 1.0:
                self._settling = True
                self._settle_t0 = time.time()
            return py_trees.common.Status.RUNNING

        # Settle phase: hold the hover target; engage suction only once the
        # cup has actually closed the vertical gap (see class docstring), so
        # the grab happens across a static ~1 cm symmetric gap and the object
        # comes up flat instead of peeled.
        stream_target(client, bb.name, pressed, self._quat)
        gap = float(cup_pos[2] - self._surf[2])
        if not self._engaged:
            if gap <= self.ENGAGE_GAP_M:
                print(f"[descend] {self.name}: engage at gap={gap:.3f} "
                      f"(d={dist_to_surf:.3f})", flush=True)
            elif time.time() - self._settle_t0 >= self.SETTLE_TIMEOUT_S:
                # Arm never closed the gap — engage anyway and let
                # VerifyAttach decide on the actual lift.
                print(f"[descend] {self.name}: settle timeout — fallback "
                      f"engage at gap={gap:.3f}", flush=True)
            else:
                # A FIXED hover target leaves a ~2-3 cm steady tracking error
                # (PD droop at annulus-edge extension — live: every settle
                # plateaued at gap 0.030-0.035 and only VerifyAttach's retry
                # rescued the grab, weakly: a return carry dropped the book).
                # Walk the TARGET below the hover until the MEASURED gap
                # closes — the place descend's proven pattern; the 1 cm
                # margin bumper floors the real cup safely above the object.
                self._pressed[2] = max(self._surf[2] - 0.02,
                                       self._pressed[2] - 0.008)
                return py_trees.common.Status.RUNNING
            client.runtime.set_suction(articulation=bb.name, value=1.0)
            self._engaged = True
            self._engage_t = time.time()
            return py_trees.common.Status.RUNNING
        client.runtime.set_suction(articulation=bb.name, value=1.0)  # re-assert (read zeroed it)
        if time.time() - self._engage_t < self.ENGAGE_DWELL_S:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self._damping_relaxed:
            try:  # restore what initialise changed, regardless of outcome
                self.bb.client._rpc_configure_controller(
                    articulation=self.bb.name,
                    params={"task_costs": {str(DAMPING_TASK): {"cost": 5.0}}})
            except Exception:
                pass
            self._damping_relaxed = False
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"DescendEngage[{self.name}]: failed"


class VerifyAttach(py_trees.behaviour.Behaviour):
    """Ramped +10 cm lift; SUCCESS if the object's z (find_actors, in_pie)
    rose >= 5 cm. One retry: re-engage 1 cm below the original contact
    point, lift again, re-verify; FAILURE if the second attempt also fails
    (no infinite suck-lift loops, per the design doc's error handling).
    Ramp structure from tidybot_guarded_reach_demo.py; the object-pose read
    is tidybot_mobile_manip_demo.py's find_actors(in_pie=True) pattern."""

    LIFT_M = 0.10
    RISE_OK_M = 0.05
    RETRY_DROP_M = 0.01

    def __init__(self, name, bb, duration: float = 3.0):
        super().__init__(name)
        self.bb = bb
        self.duration = float(duration)
        self._t0 = None
        self._start_pose = None
        self._target = None
        self._quat = None
        self._z_before = None
        self._retried = False
        self._init_error = None

    def initialise(self):
        bb = self.bb
        client = bb.client
        self._init_error = None
        try:
            p0, cup_quat = synced_site_pose(client, "cup_site")
            self._z_before = _object_z(client, bb.object_actor_id)
        except RuntimeError as e:
            self._init_error = str(e)
            return
        _hold_suction(client, bb.name)  # the two synced reads above zeroed suction
        self._start_pose = p0
        # Hold the LANDED cup orientation (already cup-down after the roll-free
        # reach), NOT the canonical bb.q_cup. bb.q_cup pins a specific ROLL about
        # the tool axis, which is vertical when cup-down — the SAME axis as base
        # yaw (joint_th) — so the whole-body QP "corrects" that unwanted roll by
        # ROTATING THE BASE during the lift (seen live). The cup is axisymmetric,
        # so its roll is a don't-care; holding the current orientation keeps it
        # cup-down with zero roll correction -> the lift is a pure vertical raise.
        # (mju_mat2Quat gives a consistent-sign quat, so no flip risk.)
        self._quat = np.asarray(cup_quat, dtype=float)
        self._target = p0 + np.array([0.0, 0.0, self.LIFT_M])
        self._t0 = time.time()
        self._retried = False

    def update(self):
        bb = self.bb
        client = bb.client
        if self._init_error is not None:
            bb.fail_reason = f"VerifyAttach[{self.name}]: {self._init_error}"
            return py_trees.common.Status.FAILURE
        a = min(1.0, (time.time() - self._t0) / self.duration)
        pos = (1.0 - a) * self._start_pose + a * self._target
        stream_target(client, bb.name, pos, self._quat)
        if a < 1.0:
            return py_trees.common.Status.RUNNING

        try:
            z_after = _object_z(client, bb.object_actor_id)
        except RuntimeError as e:
            bb.fail_reason = f"VerifyAttach[{self.name}]: {e}"
            return py_trees.common.Status.FAILURE
        rose = z_after - self._z_before
        if rose >= self.RISE_OK_M:
            _hold_suction(client, bb.name)  # the _object_z read zeroed it — keep gripping for the carry
            return py_trees.common.Status.SUCCESS

        if self._retried:
            bb.fail_reason = (
                f"VerifyAttach[{self.name}]: object rose {rose:.3f} m < "
                f"{self.RISE_OK_M} m after retry"
            )
            return py_trees.common.Status.FAILURE

        # One retry: re-engage 1 cm below the original contact point.
        self._retried = True
        client.runtime.set_suction(articulation=bb.name, value=0.0)
        retry_point = bb.affordance.point - np.array([0.0, 0.0, self.RETRY_DROP_M])
        stream_target(client, bb.name, retry_point, self._quat)
        client.runtime.set_mode("direct")
        client.step(n_steps=1)  # let the ramp target land before re-engaging
        client.runtime.set_mode("live")
        client.runtime.set_paused(paused=False)
        client.runtime.set_suction(articulation=bb.name, value=1.0)
        try:
            self._z_before = _object_z(client, bb.object_actor_id)
        except RuntimeError as e:
            bb.fail_reason = f"VerifyAttach[{self.name}]: {e}"
            return py_trees.common.Status.FAILURE
        self._start_pose = retry_point
        self._target = retry_point + np.array([0.0, 0.0, self.LIFT_M])
        self._t0 = time.time()
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"VerifyAttach[{self.name}]: failed"


class StowCarry(py_trees.behaviour.Behaviour):
    """Ramp the EE to a tucked pose (current + [-0.15, 0, +0.25]), then
    DISABLE the EE task (task_enabled [False, True, True, True]) for the
    drive home — a world-frame EE hold during base motion is forbidden by
    doctrine; posture holds the joint-space pose instead. Ramp structure
    from tidybot_guarded_reach_demo.py; the task_enabled toggle is
    tidybot_twist_follow_demo.py's fix_base-style pattern."""

    OFFSET = np.array([-0.15, 0.0, 0.25])

    def __init__(self, name, bb, duration: float = 3.0):
        super().__init__(name)
        self.bb = bb
        self.duration = float(duration)
        self._t0 = None
        self._start_pose = None
        self._target = None
        self._quat = None
        self._stowed = False
        self._init_error = None

    def initialise(self):
        bb = self.bb
        client = bb.client
        self._init_error = None
        try:
            p0, cup_quat = synced_site_pose(client, "cup_site")
        except RuntimeError as e:
            self._init_error = str(e)
            return
        _hold_suction(client, bb.name)  # keep the grip through the carry (read zeroed it)
        self._start_pose = p0
        # Hold the LANDED cup orientation, NOT canonical bb.q_cup (see VerifyAttach):
        # pinning the roll makes the whole-body QP rotate the BASE to correct a
        # don't-care roll about the vertical tool axis. Holding the current
        # (already cup-down) orientation keeps the carried box flat with no base
        # yaw. mju_mat2Quat gives a consistent-sign quat, so the target won't flip.
        self._quat = np.asarray(cup_quat, dtype=float)
        self._target = p0 + self.OFFSET
        self._t0 = time.time()
        self._stowed = False

    def update(self):
        bb = self.bb
        client = bb.client
        if self._init_error is not None:
            bb.fail_reason = f"StowCarry[{self.name}]: {self._init_error}"
            return py_trees.common.Status.FAILURE
        a = min(1.0, (time.time() - self._t0) / self.duration)
        pos = (1.0 - a) * self._start_pose + a * self._target
        stream_target(client, bb.name, pos, self._quat)
        if a < 1.0:
            return py_trees.common.Status.RUNNING
        if not self._stowed:
            client._rpc_configure_controller(
                articulation=bb.name,
                params={"task_enabled": [False, True, True, True]},
            )
            self._stowed = True
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"StowCarry[{self.name}]: failed"


class Release(py_trees.behaviour.Behaviour):
    """Seed-then-enable at the current pose, ramp to a drop pose 0.35 m
    forward / 0.45 m high of the base (world frame, holding bb.q_cup — the
    cup-down orientation from ResolveAffordance), set_suction(0), verify the
    object separates from the cup by >= 5 cm within 3 s, then re-stow (a
    small retract + disable the EE task again). Seed-then-enable and ramp
    are tidybot_twist_follow_demo.py / tidybot_guarded_reach_demo.py's
    patterns; the separation check reuses the find_actors object-z read."""

    DROP_FORWARD_M = 0.35
    DROP_HEIGHT_M = 0.45
    SEPARATION_OK_M = 0.05
    SEPARATION_TIMEOUT_S = 3.0
    RESTOW_OFFSET = np.array([-0.15, 0.0, 0.15])

    def __init__(self, name, bb, duration: float = 4.0):
        super().__init__(name)
        self.bb = bb
        self.duration = float(duration)
        self._phase = "ramp"
        self._t0 = None
        self._start_pose = None
        self._target = None
        self._quat = None
        self._sep_t0 = None
        self._init_error = None

    def initialise(self):
        bb = self.bb
        client = bb.client
        self._init_error = None
        try:
            p0, q0 = synced_site_pose(client, "cup_site")
            base_xy = _base_xy(client)
        except RuntimeError as e:
            self._init_error = str(e)
            return
        stream_target(client, bb.name, p0, q0)  # seed while re-asserting -- no yank
        client._rpc_configure_controller(
            articulation=bb.name, params={"task_enabled": [True, True, True, True]},
        )
        quat = np.asarray(bb.q_cup, dtype=float) if bb.q_cup is not None else q0
        self._quat = quat
        self._start_pose = p0
        self._target = np.array([
            base_xy[0] + self.DROP_FORWARD_M, base_xy[1], self.DROP_HEIGHT_M,
        ])
        self._t0 = time.time()
        self._phase = "ramp"

    def update(self):
        bb = self.bb
        client = bb.client
        if self._init_error is not None:
            bb.fail_reason = f"Release[{self.name}]: {self._init_error}"
            return py_trees.common.Status.FAILURE

        if self._phase == "ramp":
            a = min(1.0, (time.time() - self._t0) / self.duration)
            pos = (1.0 - a) * self._start_pose + a * self._target
            stream_target(client, bb.name, pos, self._quat)
            if a < 1.0:
                return py_trees.common.Status.RUNNING
            client.runtime.set_suction(articulation=bb.name, value=0.0)
            self._sep_t0 = time.time()
            self._phase = "verify_separation"
            return py_trees.common.Status.RUNNING

        if self._phase == "verify_separation":
            try:
                cup_pos, _ = synced_site_pose(client, "cup_site")
                obj_z = _object_z(client, bb.object_actor_id)
            except RuntimeError as e:
                bb.fail_reason = f"Release[{self.name}]: {e}"
                return py_trees.common.Status.FAILURE
            if abs(obj_z - float(cup_pos[2])) >= self.SEPARATION_OK_M:
                self._phase = "restow"
                self._start_pose = cup_pos
                self._target = cup_pos + self.RESTOW_OFFSET
                self._t0 = time.time()
                return py_trees.common.Status.RUNNING
            if time.time() - self._sep_t0 >= self.SEPARATION_TIMEOUT_S:
                bb.fail_reason = (
                    f"Release[{self.name}]: object never separated from the cup "
                    f"within {self.SEPARATION_TIMEOUT_S}s"
                )
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        # phase == "restow"
        a = min(1.0, (time.time() - self._t0) / max(self.duration * 0.5, 1e-6))
        pos = (1.0 - a) * self._start_pose + a * self._target
        stream_target(client, bb.name, pos, self._quat)
        if a < 1.0:
            return py_trees.common.Status.RUNNING
        client._rpc_configure_controller(
            articulation=bb.name, params={"task_enabled": [False, True, True, True]},
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"Release[{self.name}]: failed"


class PlaceOn(py_trees.behaviour.Behaviour):
    """The pick's inverse: planned reach to a hover above the place point
    (via a synthetic Affordance + the module's PlannedReach), slow streamed
    descend until the OBJECT (not the cup) settles on the surface, release,
    verify, retract. The hover accounts for the object riding ~4-6 cm below
    the cup on the adhesion margin.

    PLANNER CAVEAT (recorded risk): the plan runs with the object HELD —
    the planner models it as a static body at its held pose, not attached.
    Believed benign (the phantom sits inside the start config's margin
    contacts; goals are elsewhere); live gate 1 is the arbiter, and the
    fallback is a straight streamed approach instead of a planned one."""

    PRE_PLACE_HOVER_M = 0.16   # cup hover above surface_z
    AFFORDANCE_DZ = 0.13       # hover minus PlannedReach's PRE_GRASP_M (0.03)
    CUP_FLOOR_M = 0.03         # never stream the cup below surface + this
    FLOOR_GRACE_S = 5.0        # arm catch-up time after the TARGET clamps at floor
    DESCEND_STEP_M = 0.015     # per-tick descend increment (bounded error)
    DWELL_S = 1.0
    RETRACT = np.array([-0.15, 0.0, 0.15])

    def __init__(self, name, bb, place_xy, surface_z, half_thickness,
                 object_name, settle_tol: float = 0.02,
                 descend_timeout_s: float = 12.0):
        super().__init__(name)
        self.bb = bb
        self.place_xy = place_xy
        self.surface_z = float(surface_z)
        self.half = float(half_thickness)
        # The live pivot is the object's TOP face (see ResolveActorTop), so a
        # resting object reads pivot = surface + full thickness.
        self._rest_z = self.surface_z + 2.0 * self.half
        self.object_name = object_name
        self.settle_tol = float(settle_tol)
        self.descend_timeout_s = float(descend_timeout_s)
        self._phase = "plan"
        self._reach = None
        self._quat = None
        self._cup_target = None
        self._t0 = None

    def initialise(self):
        bb = self.bb
        up = np.array([0.0, 0.0, 1.0])
        q = cup_down_quat(up)
        bb.affordance = Affordance(
            point=np.array([self.place_xy[0], self.place_xy[1],
                            self.surface_z + self.AFFORDANCE_DZ]),
            normal=up, quat_cup_down=q)
        bb.q_cup = q
        self._phase = "plan"
        self._reach = PlannedReach(f"{self.name}/reach", bb,
                                   exclude_body=self.object_name,
                                   hold_suction=True)  # object is HELD here
        self._reach.initialise()
        self._quat = None
        self._t0 = None
        self._floor_t0 = None
        self._z_prev = None
        self._z_prev_t = None
        self._damping_relaxed = False
        self._approach_goal = None
        self._best_err = float("inf")
        self._improve_t = None

    def update(self):
        bb = self.bb
        client = bb.client

        if self._phase == "plan":
            status = self._reach.update()
            if status == py_trees.common.Status.RUNNING:
                return py_trees.common.Status.RUNNING
            self._reach.terminate(status)
            if status == py_trees.common.Status.FAILURE:
                # Documented fallback (spec): the place-hover RRT query is
                # structurally hostile (2 goal configs; exhaustion across
                # FRESH seeds) — stream a bounded-error carrot to the hover
                # instead. The whole-body QP tracks it (damping relaxed so
                # the base closes the last gap) and the collision guards
                # still insure the chain.
                print(f"[place] {self.name}: plan failed — streamed-approach "
                      f"fallback", flush=True)
                bb.fail_reason = ""
                p0, q0 = synced_site_pose(client, "cup_site")
                stream_target(client, bb.name, p0, q0)  # seed-then-enable
                client._rpc_configure_controller(
                    articulation=bb.name,
                    params={"task_enabled": [True, True, True, False],
                            "task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})
                self._damping_relaxed = True
                self._quat = q0
                self._approach_goal = np.array(
                    [self.place_xy[0], self.place_xy[1],
                     self.surface_z + self.PRE_PLACE_HOVER_M])
                self._best_err = float("inf")
                self._improve_t = time.time()
                self._t0 = time.time()
                self._phase = "approach"
                return py_trees.common.Status.RUNNING
            # seed-then-enable at the landed pose, then descend
            p0, q0 = synced_site_pose(client, "cup_site")
            hover_err = float(np.linalg.norm(p0[:2] - np.asarray(self.place_xy)))
            print(f"[place] {self.name}: hover cup {np.round(p0, 3)} — "
                  f"{hover_err:.3f} m off target xy {self.place_xy}", flush=True)
            stream_target(client, bb.name, p0, q0)
            client._rpc_configure_controller(
                articulation=bb.name,
                params={"task_enabled": [True, True, True, False]})
            self._quat = q0
            self._cup_target = p0.copy()
            self._t0 = time.time()
            self._phase = "descend"
            return py_trees.common.Status.RUNNING

        if self._phase == "approach":
            # Streamed fallback: bounded-error carrot to the hover; the
            # descend phase takes over once close (or plateaued near enough
            # that a straight-down descend still lands ON the surface).
            _hold_suction(client, bb.name)
            cur, _ = synced_site_pose(client, "cup_site")
            err = self._approach_goal - cur
            n = float(np.linalg.norm(err))
            if n < self._best_err - 0.01:
                self._best_err = n
                self._improve_t = time.time()
            plateau = (time.time() - self._improve_t) > 4.0
            if n < 0.03 or (plateau and n < 0.10):
                hover_err = float(np.linalg.norm(
                    cur[:2] - np.asarray(self.place_xy)))
                print(f"[place] {self.name}: approach hover cup "
                      f"{np.round(cur, 3)} — {hover_err:.3f} m off target xy "
                      f"{self.place_xy}", flush=True)
                self._cup_target = cur.copy()
                self._t0 = time.time()
                self._floor_t0 = None
                self._z_prev = None
                self._phase = "descend"
                return py_trees.common.Status.RUNNING
            if plateau or time.time() - self._t0 > 25.0:
                bb.fail_reason = (f"PlaceOn[{self.name}]: streamed approach "
                                  f"stalled {n:.2f} m from the hover")
                return py_trees.common.Status.FAILURE
            step = err if n <= 0.04 else err * (0.04 / n)
            stream_target(client, bb.name, cur + step, self._quat)
            return py_trees.common.Status.RUNNING

        if self._phase == "descend":
            _hold_suction(client, bb.name)
            z_obj = _actor_z_by_name(client, self.object_name)
            if abs(z_obj - self._rest_z) <= self.settle_tol:
                try:  # telemetry only — never let a diagnostic read fail the phase
                    cup_now, _ = synced_site_pose(client, "cup_site")
                    obj_xy = _actor_xy_by_name(client, self.object_name)
                    print(f"[place] {self.name}: flat-settle — releasing at "
                          f"object z {z_obj:.3f} xy {np.round(obj_xy, 2)} "
                          f"cup {np.round(cup_now, 2)} (target xy {self.place_xy})",
                          flush=True)
                except Exception:
                    print(f"[place] {self.name}: flat-settle — releasing at "
                          f"object z {z_obj:.3f}", flush=True)
                self._phase = "release"
                self._t0 = None
                return py_trees.common.Status.RUNNING
            # CONTACT-BY-STALL: a pressed book tilts on first edge contact
            # (live: pivot read 0.756 with the cup at 0.715 — the flat-settle
            # check can never pass on a wedged object). If the object's z has
            # stopped changing while we're still commanding descent, it is ON
            # the surface: release and let gravity flatten it; the release
            # phase re-verifies after the settle.
            if self._z_prev is None or abs(z_obj - self._z_prev) > 0.004:
                self._z_prev = z_obj
                self._z_prev_t = time.time()
            elif (time.time() - self._z_prev_t > 2.0
                    and self._cup_target[2] <= self.surface_z + self.CUP_FLOOR_M + 0.05):
                cup_now, _ = synced_site_pose(client, "cup_site")
                obj_xy = _actor_xy_by_name(client, self.object_name)
                print(f"[place] {self.name}: contact-stall — releasing at "
                      f"object z {z_obj:.3f} xy {np.round(obj_xy, 2)} "
                      f"cup {np.round(cup_now, 2)} (target xy {self.place_xy})",
                      flush=True)
                self._phase = "release"
                self._t0 = None
                return py_trees.common.Status.RUNNING
            if time.time() - self._t0 > self.descend_timeout_s:
                bb.fail_reason = (f"PlaceOn[{self.name}]: descend timeout "
                                  f"(object z {z_obj:.3f})")
                return py_trees.common.Status.FAILURE
            floor = self.surface_z + self.CUP_FLOOR_M
            self._cup_target[2] = max(floor,
                                      self._cup_target[2] - self.DESCEND_STEP_M)
            stream_target(client, bb.name, self._cup_target, self._quat)
            # The TARGET reaches the floor limit long before the REAL cup does
            # (bounded-error streaming: the QP-tracked arm lags the streamed
            # carrot). Failing the instant the target clamped aborted a live
            # descend with the arm still 14 cm up — give the arm a grace
            # window at the floor before declaring the object unsettleable.
            if self._cup_target[2] <= floor:
                if self._floor_t0 is None:
                    self._floor_t0 = time.time()
                elif (time.time() - self._floor_t0 > self.FLOOR_GRACE_S
                        and z_obj - self._rest_z > 0.06):
                    cup_now, _ = synced_site_pose(client, "cup_site")
                    bb.fail_reason = (
                        f"PlaceOn[{self.name}]: target at floor {self.FLOOR_GRACE_S:.0f}s "
                        f"but object z {z_obj:.3f} never settled (cup z {cup_now[2]:.3f})")
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if self._phase == "release":
            if self._t0 is None:
                client.runtime.set_suction(articulation=bb.name, value=0.0)
                self._t0 = time.time()
                return py_trees.common.Status.RUNNING
            if time.time() - self._t0 < self.DWELL_S:
                return py_trees.common.Status.RUNNING
            # Verification happens AFTER retract: a released object can rest
            # TILTED against the cup still hovering over it (live: propped at
            # +0.065 with the cup 4 cm above — the raised cup friction holds
            # the lean) and only lies flat once the cup clears away.
            p0, _ = synced_site_pose(client, "cup_site")
            self._cup_target = p0 + self.RETRACT
            self._t0 = time.time()
            self._phase = "retract"
            return py_trees.common.Status.RUNNING

        # retract, then FINAL settle verification (cup clear of the object)
        a = min(1.0, (time.time() - self._t0) / 2.0)
        p0, _ = synced_site_pose(client, "cup_site")
        pos = (1.0 - a) * p0 + a * self._cup_target
        stream_target(client, bb.name, pos, self._quat)
        if a < 1.0:
            return py_trees.common.Status.RUNNING
        client._rpc_configure_controller(
            articulation=bb.name,
            params={"task_enabled": [False, True, True, True]})
        z_obj = _actor_z_by_name(client, self.object_name)
        if not (-self.settle_tol <= z_obj - self._rest_z <= 0.06):
            bb.fail_reason = (f"PlaceOn[{self.name}]: object z {z_obj:.3f} "
                              f"not at rest height {self._rest_z:.3f} after retract")
            return py_trees.common.Status.FAILURE
        print(f"[place] {self.name}: PLACED — object z {z_obj:.3f} on surface "
              f"{self.surface_z:.3f}", flush=True)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if getattr(self, "_damping_relaxed", False):
            try:  # restore the fallback's damping relax regardless of outcome
                self.bb.client._rpc_configure_controller(
                    articulation=self.bb.name,
                    params={"task_costs": {str(DAMPING_TASK): {"cost": 5.0}}})
            except Exception:
                pass
            self._damping_relaxed = False
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"PlaceOn[{self.name}]: failed"
