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

PRE_APPROACH_M = 0.15
ENGAGE_M = 0.02

# Approximate shoulder height above the base plane (world z), used by the
# Reachable condition. Matches the empirically-mapped annulus fixture
# (shoulder = (base_x, base_y, 0.49) in the validated staging geometry).
SHOULDER_HEIGHT_M = 0.49

NAV_TIMEOUT_S = 120.0
# A dist-to-goal jump this large after progress was made matches the
# physics-auto-reset fingerprint called out in tidybot_twist_follow_demo.py's
# module doc — never re-drive across a reset, abort with a diagnostic instead.
RESET_FINGERPRINT_JUMP_M = 0.5


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


def _object_z(client, object_actor_id: str) -> float:
    """Read an object's world z via find_actors(in_pie=True), matched by
    actor_id. find_actors returns the actor ROOT transform (not the MuJoCo
    body) — per tidybot_mobile_manip_demo.py's robot_xy() caveat — but a
    free-floating dynamic body pump-syncs its actor transform every tick, so
    this tracks the live z of the pick object."""
    for row in client.outliner.find_actors(in_pie=True):
        if row.actor_id == object_actor_id:
            return float(row.location[2])
    raise RuntimeError(f"object {object_actor_id!r} not found via find_actors")


def _base_xy(client) -> np.ndarray:
    """Base (joint_x, joint_y) world position — the Tracker.base_xy() pattern
    from tidybot_mink_demo.py, resolved by joint suffix off the synced qpos
    mirror."""
    m = client.model
    jx = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, "joint_x")
    jy = resolve_id_by_suffix(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt, "joint_y")
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
    back up after progress was made (the physics-auto-reset fingerprint named
    in that demo's module doc) rather than silently re-driving across a
    reset."""

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
        client.runtime.set_mode("live")
        client.runtime.set_paused(paused=False)
        clear, dist = corridor_clear(client, p_from, p_to, self.exclude_body_suffixes)
        if clear:
            return py_trees.common.Status.SUCCESS
        bb.fail_reason = (
            f"CorridorClear[{self.name}]: blocked at {dist:.3f} m along "
            f"{np.round(p_from, 3)} -> {np.round(p_to, 3)} "
            "(v1 premise: no motion planner -- abort; RRT-Connect is the named upgrade)"
        )
        return py_trees.common.Status.FAILURE


class ReachRamp(py_trees.behaviour.Behaviour):
    """Seed-then-enable EE-task ramp: seeds the manual target at the CURRENT
    cup_site pose while the EE task is still disabled (enabling against a
    stale bind-pose target yanks — tidybot_twist_follow_demo.py Phase B),
    THEN enables all four tasks, then ramps the EE position linearly through
    bb.<waypoints_key> (a list of world points) over `duration` seconds total
    (split evenly across legs), holding orientation at bb.<quat_key>
    throughout — the wall-clock position ramp is
    tidybot_guarded_reach_demo.py's Phase B pattern."""

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

    def initialise(self):
        bb = self.bb
        client = bb.client
        waypoints = getattr(bb, self.waypoints_key, None)
        if not waypoints:
            bb.fail_reason = f"ReachRamp[{self.name}]: no waypoints under {self.waypoints_key!r}"
            self._legs = []
            return
        self._quat = np.asarray(getattr(bb, self.quat_key), dtype=float)

        p0, q0 = synced_site_pose(client, "cup_site")
        stream_target(client, bb.name, p0, q0)  # seed while disabled -- no yank
        client._rpc_configure_controller(
            articulation=bb.name, params={"task_enabled": [True, True, True, True]},
        )

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
        stream_target(client, bb.name, pos, self._quat)
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


class DescendEngage(py_trees.behaviour.Behaviour):
    """Final-leg ramp onto the affordance point (bb.affordance.waypoints[1:]
    — engage then contact), engaging suction
    (client.runtime.set_suction(articulation=bb.name, value=1.0)) the instant
    cup_site is within 2 cm of the affordance point. Ramp structure from
    tidybot_guarded_reach_demo.py; the engage gate reads cup_site via the
    Tracker pattern (synced_site_pose) every tick."""

    ENGAGE_DIST_M = 0.02

    def __init__(self, name, bb, duration: float = 4.0):
        super().__init__(name)
        self.bb = bb
        self.duration = float(duration)
        self._legs = []
        self._leg_idx = 0
        self._leg_start_pose = None
        self._leg_t0 = None
        self._per_leg = None
        self._quat = None
        self._engaged = False

    def initialise(self):
        bb = self.bb
        aff = bb.affordance
        if aff is None:
            bb.fail_reason = f"DescendEngage[{self.name}]: no affordance resolved"
            self._legs = []
            return
        self._quat = np.asarray(aff.quat_cup_down, dtype=float)
        self._legs = [np.asarray(w, dtype=float) for w in aff.waypoints[1:]]  # engage, contact
        p0, _ = synced_site_pose(bb.client, "cup_site")
        self._leg_start_pose = p0
        self._leg_idx = 0
        self._leg_t0 = time.time()
        self._per_leg = self.duration / max(1, len(self._legs))
        self._engaged = False

    def update(self):
        bb = self.bb
        client = bb.client
        if not self._legs:
            return py_trees.common.Status.FAILURE

        target = self._legs[self._leg_idx]
        a = min(1.0, (time.time() - self._leg_t0) / max(self._per_leg, 1e-6))
        pos = (1.0 - a) * self._leg_start_pose + a * target
        stream_target(client, bb.name, pos, self._quat)

        cup_pos, _ = synced_site_pose(client, "cup_site")
        dist = float(np.linalg.norm(cup_pos - bb.affordance.point))
        if not self._engaged and dist <= self.ENGAGE_DIST_M:
            client.runtime.set_suction(articulation=bb.name, value=1.0)
            self._engaged = True

        if a < 1.0:
            return py_trees.common.Status.RUNNING
        if self._leg_idx + 1 < len(self._legs):
            self._leg_idx += 1
            self._leg_start_pose = target
            self._leg_t0 = time.time()
            return py_trees.common.Status.RUNNING

        if not self._engaged:
            # Final leg complete without crossing the engage threshold (e.g. a
            # shallow approach angle) -- force engage at the contact waypoint
            # rather than silently finishing unattached.
            client.runtime.set_suction(articulation=bb.name, value=1.0)
            self._engaged = True
        return py_trees.common.Status.SUCCESS

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

    def initialise(self):
        bb = self.bb
        client = bb.client
        p0, q0 = synced_site_pose(client, "cup_site")
        self._z_before = _object_z(client, bb.object_actor_id)
        self._start_pose = p0
        self._quat = q0
        self._target = p0 + np.array([0.0, 0.0, self.LIFT_M])
        self._t0 = time.time()
        self._retried = False

    def update(self):
        bb = self.bb
        client = bb.client
        a = min(1.0, (time.time() - self._t0) / self.duration)
        pos = (1.0 - a) * self._start_pose + a * self._target
        stream_target(client, bb.name, pos, self._quat)
        if a < 1.0:
            return py_trees.common.Status.RUNNING

        z_after = _object_z(client, bb.object_actor_id)
        rose = z_after - self._z_before
        if rose >= self.RISE_OK_M:
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
        client.step(n_steps=1)  # let the ramp target land before re-engaging
        client.runtime.set_suction(articulation=bb.name, value=1.0)
        self._z_before = _object_z(client, bb.object_actor_id)
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

    def initialise(self):
        client = self.bb.client
        p0, q0 = synced_site_pose(client, "cup_site")
        self._start_pose = p0
        self._quat = q0
        self._target = p0 + self.OFFSET
        self._t0 = time.time()
        self._stowed = False

    def update(self):
        bb = self.bb
        client = bb.client
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

    def initialise(self):
        bb = self.bb
        client = bb.client
        p0, q0 = synced_site_pose(client, "cup_site")
        stream_target(client, bb.name, p0, q0)  # seed while re-asserting -- no yank
        client._rpc_configure_controller(
            articulation=bb.name, params={"task_enabled": [True, True, True, True]},
        )
        base_xy = _base_xy(client)
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
            cup_pos, _ = synced_site_pose(client, "cup_site")
            obj_z = _object_z(client, bb.object_actor_id)
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
