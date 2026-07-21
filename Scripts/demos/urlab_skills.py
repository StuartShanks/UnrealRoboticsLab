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


def _object_z(client, object_actor_id: str) -> float:
    """Read the object's LIVE PHYSICS z (its MuJoCo body world pos via the synced
    mirror). find_actors(in_pie=True) must NOT be used here: for a spawned free
    body it returns the actor ROOT transform, which stays FROZEN at the spawn
    location and does not track physics — confirmed live (actor z stuck at 0.66
    while the physics body was on the floor at 0.05), so a lift is invisible to
    it. The body is matched by actor_id suffix (import may prefix the name)."""
    client.runtime.set_mode("direct")
    client.step(n_steps=1)
    m, d = client.model, client.data
    bid = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody, object_actor_id)
    z = float(d.xpos[bid][2]) if bid >= 0 else None
    client.runtime.set_mode("live")
    client.runtime.set_paused(paused=False)
    if bid < 0:
        raise RuntimeError(f"object body {object_actor_id!r} not found in compiled model")
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
    """Base (joint_x, joint_y) world position — the Tracker.base_xy() pattern
    from tidybot_mink_demo.py, resolved by joint suffix off the synced qpos
    mirror."""
    m = client.model
    jx = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, "joint_x")
    jy = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, "joint_y")
    if jx < 0 or jy < 0:
        raise RuntimeError(f"base joint not found")
    d = client.data
    return np.array([d.qpos[m.jnt_qposadr[jx]], d.qpos[m.jnt_qposadr[jy]]])


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

    def __init__(self, name, bb, goal_xy, timeout_s: float = NAV_TIMEOUT_S):
        super().__init__(name)
        self.bb = bb
        self.goal_xy = goal_xy
        self.timeout_s = float(timeout_s)
        self._deadline = None
        self._min_dist = None

    def initialise(self):
        bb = self.bb
        client = bb.client
        client._rpc_configure_controller(
            articulation=bb.name,
            params={"task_costs": {str(bb.damping_task): {"cost": 0.05}}},
        )
        g = client.runtime.set_nav_goal(
            articulation=bb.name, x=self.goal_xy[0], y=self.goal_xy[1],
        )
        self._min_dist = None
        if not g.get("accepted"):
            bb.fail_reason = f"Drive[{self.name}]: goal {self.goal_xy} rejected: {g}"
            self._deadline = -1.0  # sentinel: update() fails immediately
            return
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

    def __init__(self, name, bb):
        super().__init__(name)
        self.bb = bb
        self._plan = None
        self._joints = None
        self._wp_idx = 0
        self._best_err = float("inf")
        self._last_improve_t = None
        self._failed = None

    def initialise(self):
        from urlab_planner import PLANNED_JOINTS, PlanError, plan_reach
        bb = self.bb
        client = bb.client
        self._joints = PLANNED_JOINTS
        self._wp_idx = 0
        self._failed = None
        self._best_err = float("inf")
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
        try:
            self._plan = plan_reach(client, pre_grasp, np.asarray(bb.q_cup, dtype=float))
        except PlanError as e:
            self._plan = None
            bb.fail_reason = f"PlannedReach[{self.name}]: plan {e.stage}: {e}"
            self._failed = bb.fail_reason
            return
        except Exception as e:
            # plan_reach can also raise ValueError (unresolved joint/site), a
            # mink import error, or an RPC error — fail the tick cleanly rather
            # than crash with a traceback. No exec config has been sent yet, so
            # there is nothing to clean up.
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
    lets adhesion pull the box UP to the cup instead. Engages suction once cup_site
    is within ENGAGE_DIST_M, then holds at the hover target until the cup is within
    CONTACT_DIST_M of it or SETTLE_TIMEOUT_S elapses. SUCCESS then (VerifyAttach is
    the real gate); FAILURE only on a read error."""

    ENGAGE_DIST_M = 0.05    # fire suction once this close to the surface
    CONTACT_DIST_M = 0.025  # cup within this of the hover target -> settled, done
    HOVER_M = 0.012         # hover the cup this far ABOVE the surface. Do NOT press
                            # into the object: the cup grabs within the adhesion
                            # margin (3 cm), so pressing only knocks the light box
                            # off the table. Hover + suction -> adhesion pulls it up.
    SETTLE_TIMEOUT_S = 3.0  # hold ceiling for adhesion to grab before VerifyAttach

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
        self._settling = False
        self._settle_t0 = None
        self._init_error = None

    def initialise(self):
        bb = self.bb
        self._init_error = None
        self._engaged = False
        self._settling = False
        self._settle_t0 = None
        if bb.affordance is None:
            self._init_error = "no affordance resolved"
            return
        try:
            # Snapshot the object's affordance pose ONCE, now, while it is
            # sitting still. Do NOT chase it live: the affordance site rides the
            # free body, so a live target lunges after the box the instant it is
            # nudged (the jumping IK marker) and amplifies a knock-off. A fixed
            # snapshot gives a clean straight-down descent to where the box IS.
            surf, quat = synced_site_pose(bb.client, "affordance_suction_top")
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, quat)
            normal = np.asarray(R).reshape(3, 3)[:, 2]
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

        # Engage suction once close, so it's on before contact.
        if not self._engaged and dist_to_surf <= self.ENGAGE_DIST_M:
            client.runtime.set_suction(articulation=bb.name, value=1.0)
            self._engaged = True
        elif self._engaged:
            _hold_suction(client, bb.name)  # the synced read above zeroed it

        if not self._settling:
            # Ramp phase: bounded-error descent from the pre-approach pose.
            a = min(1.0, (time.time() - self._t0) / max(self.ramp_duration, 1e-6))
            stream_target(client, bb.name, (1.0 - a) * self._start_pose + a * pressed, self._quat)
            if a >= 1.0:
                self._settling = True
                self._settle_t0 = time.time()
            return py_trees.common.Status.RUNNING

        # Settle phase: keep pressing the live target until real contact, so the
        # arm converges and adhesion grabs (rather than lifting off a lagging ramp).
        stream_target(client, bb.name, pressed, self._quat)
        client.runtime.set_suction(articulation=bb.name, value=1.0)  # re-assert (read zeroed it)
        self._engaged = True
        if dist_to_surf <= self.CONTACT_DIST_M:
            return py_trees.common.Status.SUCCESS
        if time.time() - self._settle_t0 >= self.SETTLE_TIMEOUT_S:
            # Hand off to VerifyAttach — it decides pass/fail on the actual lift.
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
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
