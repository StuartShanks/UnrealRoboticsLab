#!/usr/bin/env python3
"""Whole-body free-space connector: RRT-Connect over the bridge client's
compiled-model replica (the mjb handshake), goal configs from python-mink IK.

Planner proposes / QP disposes / guards insure: this module only PROPOSES a
geometrically feasible joint-space path. Execution streams it through the mink
controller's posture_target wire (tidybot_planned_reach_probe.py); the QP's
velocity limits + collision guards stay active as the insurance layer.

Spec: docs/superpowers/specs/2026-07-16-whole-body-rrt-connector-design.md
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

# The 10 planned DoF, base first (order matters for goal seeding).
PLANNED_JOINTS = ["joint_x", "joint_y", "joint_th",
                  "joint_1", "joint_2", "joint_3", "joint_4",
                  "joint_5", "joint_6", "joint_7"]
# Execution pace defaults — match the demo QP velocity-limit constants
# (BASE_MAX_VEL/ARM_MAX_VEL in tidybot_mobile_manip_demo.py).
DEFAULT_V_LIMITS = {"joint_x": 0.6, "joint_y": 0.6, "joint_th": 1.0,
                    **{f"joint_{i}": 0.5 for i in range(1, 8)}}


class PlanError(RuntimeError):
    """Planning failed. .stage is 'goal_ik' or 'rrt' — they are fixed
    differently (pose/scene problem vs sampling-budget problem)."""

    def __init__(self, stage: str, msg: str):
        super().__init__(msg)
        self.stage = stage


@dataclass
class Plan:
    """Timed joint-space waypoints: waypoints[k] = (t_sec, {joint: qpos}),
    JSON-ready for the posture_target wire."""
    waypoints: list
    stats: dict = field(default_factory=dict)


def _resolve_id(model, objtype, count, suffix):
    """Compiled MuJoCo object id by exact name, else by name SUFFIX — the UE
    importer prefixes compiled names (e.g. 'tidybot_suction_ue_C_0_joint_x'),
    so a bare 'joint_x' never matches exactly on the live model. Mirrors
    tidybot_mink_demo.resolve_id_by_suffix. Returns -1 if nothing matches.
    (Synthetic offline test scenes use bare names, so the exact branch wins
    there — this is a superset of the old exact-only lookup.)"""
    exact = mujoco.mj_name2id(model, objtype, suffix)
    if exact >= 0:
        return exact
    for i in range(count):
        nm = mujoco.mj_id2name(model, objtype, i)
        if nm and nm.endswith(suffix):
            return i
    return -1


def _resolve_name(model, objtype, count, suffix):
    """The COMPILED name for a bare suffix (mink's FrameTask resolves its
    frame_name by exact string, so it needs the prefixed name). None if unmatched."""
    i = _resolve_id(model, objtype, count, suffix)
    return mujoco.mj_id2name(model, objtype, i) if i >= 0 else None


class PlanContext:
    """Collision-checkable snapshot of the live scene over the client's model
    replica. Start state comes from a SYNCED physics read — never
    find_actors (actor roots are frozen at their spawn pose)."""

    def __init__(self, model, qpos0, joints=PLANNED_JOINTS):
        self.m = model
        self.d = mujoco.MjData(model)
        self.qpos0 = np.asarray(qpos0, dtype=float).copy()
        self.joints = list(joints)
        self.jids = []
        qadr = []
        for name in self.joints:
            jid = _resolve_id(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt, name)
            if jid < 0:
                raise ValueError(f"planned joint {name!r} not in model")
            self.jids.append(jid)
            qadr.append(int(model.jnt_qposadr[jid]))
        self.qadr = np.asarray(qadr, dtype=int)
        # Sampling bounds + a "limited" mask. For LIMITED joints, the actual
        # jnt_range. For UNLIMITED joints there is no real limit, only a sampling
        # convenience: an unlimited HINGE (e.g. base yaw) samples an angle in
        # +-pi, but an unlimited SLIDE (the base x/y translations) moves in world
        # METRES and must sample world-scale — a +-pi bound would wrongly clip the
        # base to ~3.14 m and flag any farther base pose as "out of limits".
        lo, hi, limited = [], [], []
        for jid in self.jids:
            lim = bool(model.jnt_limited[jid])
            limited.append(lim)
            if lim:
                lo.append(float(model.jnt_range[jid][0]))
                hi.append(float(model.jnt_range[jid][1]))
            elif model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_SLIDE:
                lo.append(-50.0)   # world-scale; the base sampler re-centres these
                hi.append(50.0)    # on the start/goal via an annulus/disc anyway
            else:
                lo.append(-np.pi)
                hi.append(np.pi)
        self.lo = np.asarray(lo)
        self.hi = np.asarray(hi)
        # Only limited joints have a range to VIOLATE; unlimited joints never fail
        # the limit gate (their bounds above are sampling hints, not limits).
        self.limited = np.asarray(limited, dtype=bool)
        # Geom pairs already PENETRATING at the start config are pre-existing
        # (resting) contacts, not planner failures — exempt them everywhere.
        self.baseline = self._penetrating_pairs(self.q_start())

    def park_body(self, body_suffix: str):
        """EXCLUDE a free body from this plan's collision world by parking it
        10 m underground in the snapshot. For plans made WHILE HOLDING an
        object: the planner models the held body as STATIC at its held pose,
        which sits exactly where the EE housing sweeps — RRT-Connect could not
        grow a single edge out of the start config (live gate 1). The real
        object's safety is physical (adhesion + QP guards + slow descend), not
        the planner's job. Recomputes the collision baseline."""
        bid = -1
        for b in range(self.m.nbody):
            nm = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if body_suffix in nm:
                bid = b
                break
        if bid < 0:
            raise ValueError(f"park_body: no body matching {body_suffix!r}")
        jadr = self.m.body_jntadr[bid]
        if (jadr < 0 or self.m.body_jntnum[bid] < 1
                or self.m.jnt_type[jadr] != mujoco.mjtJoint.mjJNT_FREE):
            raise ValueError(f"park_body: {body_suffix!r} has no free joint")
        qadr = int(self.m.jnt_qposadr[jadr])
        self.qpos0[qadr + 2] -= 10.0
        self.baseline = self._penetrating_pairs(self.q_start())

    @classmethod
    def from_client(cls, client, joints=PLANNED_JOINTS):
        """Snapshot live physics into a context. v1 constraint: call PRE-suction
        only — a direct-mode step zeroes actuator NetworkValues server-side."""
        client.step(n_steps=1)  # sync the mirror
        return cls(client.model, np.asarray(client.data.qpos), joints)

    def q_start(self):
        return self.qpos0[self.qadr].copy()

    def _set(self, q):
        self.d.qpos[:] = self.qpos0
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_collision(self.m, self.d)

    def _penetrating_pairs(self, q):
        self._set(q)
        pairs = set()
        for i in range(self.d.ncon):
            con = self.d.contact[i]
            # PENETRATION test, not contact test: geoms with margin (the
            # suction cup carries margin=0.03) report positive-distance
            # contacts inside the margin — suction sensing, not collision.
            if con.dist < -1e-6:
                g1, g2 = int(con.geom1), int(con.geom2)
                pairs.add((min(g1, g2), max(g1, g2)))
        return pairs

    def collision_free(self, q):
        q = np.asarray(q, dtype=float)
        # Limit gate applies to LIMITED joints only — unlimited base slides move
        # in world metres and have no range to violate.
        if np.any(self.limited & (q < self.lo - 1e-9)) or \
                np.any(self.limited & (q > self.hi + 1e-9)):
            return False
        return self._penetrating_pairs(q) <= self.baseline

    def site_pose(self, q, site):
        """World (pos, quat_wxyz) of a site at configuration q."""
        self._set(q)
        sid = _resolve_id(self.m, mujoco.mjtObj.mjOBJ_SITE, self.m.nsite, site)
        if sid < 0:
            raise ValueError(f"site {site!r} not in model")
        pos = self.d.site_xpos[sid].copy()
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, self.d.site_xmat[sid].reshape(9))
        return pos, quat


def _edge_free(ctx, qa, qb, resolution=0.05):
    """Every interpolated config on [qa, qb] collision-free. resolution is the
    max per-dimension step (m or rad) between checked configs."""
    qa = np.asarray(qa, dtype=float)
    qb = np.asarray(qb, dtype=float)
    n = int(np.ceil(np.max(np.abs(qb - qa)) / resolution)) + 1
    for a in np.linspace(0.0, 1.0, n + 1):
        if not ctx.collision_free((1.0 - a) * qa + a * qb):
            return False
    return True


def _base_frame_offset_xy(ctx):
    """World-minus-joint xy offset of the base slide frame. joint_x/joint_y
    qpos are relative to the robot's SPAWN frame, not the world: world_xy =
    offset + joint_xy. In-house demos spawned at the origin (offset 0), which
    hid any confusion; a non-origin spawn (HomeInterior: (-10,-18)) fed world
    coords into base joints and scattered goal-IK seeds 10-20 m from the
    target — goal_ik found nothing. Evaluated at the snapshot start config;
    assumes joints[0]/joints[1] are world-aligned x/y slides (true for the
    holonomic base; a spawn-yawed robot would need the rotation too)."""
    q0 = ctx.q_start()
    ctx._set(q0)
    bid = int(ctx.m.jnt_bodyid[ctx.jids[1]])   # joint_y's body: moved by both slides
    return ctx.d.xpos[bid][:2].copy() - q0[:2]


def sample_goal_configs(ctx, pos, quat_wxyz, site="cup_site", n=8, seed=0,
                        iters=200, dt=0.05, pos_tol=5e-3, ori_tol=5e-2):
    """python-mink IK from n seeds -> deduped collision-free goal configs.
    Seed 0 is the current q; later seeds randomize the arm and place the base
    in a disc around the target xy (the whole point: the base participates)."""
    import mink  # deferred import: only goal sampling needs it

    rng = np.random.default_rng(seed)
    pos = np.asarray(pos, dtype=float)
    # Wrist-roll pin: joint_7 is the axisymmetric cup's pure-roll DOF (see below).
    _WRIST_IDX = ctx.joints.index("joint_7") if "joint_7" in ctx.joints else None
    q_start_full = ctx.q_start()
    # mink resolves frame_name by exact string against the compiled model, so
    # pass the prefixed compiled name (the caller gives a bare suffix).
    frame_name = _resolve_name(ctx.m, mujoco.mjtObj.mjOBJ_SITE, ctx.m.nsite, site)
    if frame_name is None:
        raise ValueError(f"site {site!r} not in model")
    # Roll-agnostic orientation: the cup is axisymmetric, so rotation ABOUT its
    # tool axis (cup_site local z) is a don't-care DOF. Zeroing the z-axis
    # orientation cost lets the posture regularizer keep the wrist near its
    # current angle instead of the IK baking in a large arbitrary roll (live: a
    # 208 deg joint_7 spin) — [x, y, z] = [constrain, constrain, free-roll].
    task = mink.FrameTask(frame_name=frame_name, frame_type="site",
                          position_cost=1.0, orientation_cost=[1.0, 1.0, 0.0],
                          lm_damping=1.0)
    rot = mink.SO3(np.asarray(quat_wxyz, dtype=float))
    task.set_target(mink.SE3.from_rotation_and_translation(rot, pos))
    posture_cost = np.zeros(ctx.m.nv)
    for jid in ctx.jids:
        posture_cost[ctx.m.jnt_dofadr[jid]] = 1e-3
    limits = [mink.ConfigurationLimit(ctx.m)]

    # World target -> base joint frame (spawn offset); see _base_frame_offset_xy.
    off_xy = _base_frame_offset_xy(ctx)
    goals = []
    for k in range(n):
        q = ctx.q_start()
        if k > 0:
            q = rng.uniform(ctx.lo, ctx.hi)
            r = rng.uniform(0.3, 0.9)          # base in an annulus around the
            th = rng.uniform(-np.pi, np.pi)    # target xy, roughly arm reach
            q[0] = np.clip(pos[0] - off_xy[0] + r * np.cos(th), ctx.lo[0], ctx.hi[0])
            q[1] = np.clip(pos[1] - off_xy[1] + r * np.sin(th), ctx.lo[1], ctx.hi[1])
        cfg = mink.Configuration(ctx.m)
        qfull = ctx.qpos0.copy()
        qfull[ctx.qadr] = q
        cfg.update(qfull)
        posture = mink.PostureTask(ctx.m, cost=posture_cost)
        posture.set_target_from_configuration(cfg)
        converged = False
        for _ in range(iters):
            v = mink.solve_ik(cfg, [task, posture], dt, "daqp",
                              damping=1e-3, limits=limits)
            cfg.integrate_inplace(v, dt)
            err = task.compute_error(cfg)
            # Score the CONSTRAINED orientation axes only: err[5] is rotation
            # about the site z (tool) axis, which orientation_cost=[1,1,0]
            # deliberately frees (axisymmetric cup). Scoring err[3:] vetoed
            # perfect grasps whenever the uncontrolled roll sat far from the
            # target roll (live: every seed pos_err 0.000, |ori_xy| 0.000,
            # 0.3-1.85 rad of don't-care roll -> goal_ik dry).
            if (np.linalg.norm(err[:3]) <= pos_tol
                    and np.linalg.norm(err[3:5]) <= ori_tol):
                converged = True
                break
        if not converged:
            continue
        qg = np.asarray(cfg.q)[ctx.qadr].copy()
        # Pin the wrist roll to the CURRENT angle. joint_7 is a pure roll DOF
        # about the axisymmetric cup's tool axis (verified: moving it does not
        # translate cup_site), so its value is a don't-care for the grasp — but
        # the roll-free orientation cost lets IK leave it at whatever the random
        # seed had, baking a pointless wrist spin into the plan (live: +3.6 rad).
        # Overwriting it with the start angle costs nothing (cup pose unchanged)
        # and is collision-safe (rotating an axisymmetric cup about its own axis
        # sweeps no new volume); the collision_free check below still runs on it.
        if _WRIST_IDX is not None:
            qg[_WRIST_IDX] = q_start_full[_WRIST_IDX]
        if not ctx.collision_free(qg):
            continue
        if any(np.max(np.abs(qg - g)) < 0.05 for g in goals):
            continue  # dedupe near-identical solutions
        goals.append(qg)
    return goals


def _walk(tree, i):
    """Node chain root -> tree[i]."""
    out = []
    while i >= 0:
        out.append(tree[i][0])
        i = tree[i][1]
    out.reverse()
    return out


def rrt_connect(ctx, q_start, goal_qs, max_iters=2000, step=0.15,
                base_disc=1.2, seed=0, resolution=0.05):
    """Bidirectional RRT-Connect in the planned-joint space. The goal tree is
    seeded with ALL goal configs. Base dims (indices 0,1) sample from discs
    around the start/goal base xy; everything else uniform in limits. Returns
    q_start -> goal waypoints, or None."""
    if not goal_qs:
        return None
    q_start = np.asarray(q_start, dtype=float)
    goal_qs = [np.asarray(g, dtype=float) for g in goal_qs]
    if not ctx.collision_free(q_start):
        return None
    for g in goal_qs:  # trivial connect: straight line already free
        if _edge_free(ctx, q_start, g, resolution):
            return [q_start.copy(), g.copy()]

    rng = np.random.default_rng(seed)
    centers = [q_start[:2]] + [g[:2] for g in goal_qs]

    def sample():
        q = rng.uniform(ctx.lo, ctx.hi)
        c = centers[rng.integers(len(centers))]
        r = base_disc * np.sqrt(rng.uniform())
        th = rng.uniform(-np.pi, np.pi)
        q[0] = np.clip(c[0] + r * np.cos(th), ctx.lo[0], ctx.hi[0])
        q[1] = np.clip(c[1] + r * np.sin(th), ctx.lo[1], ctx.hi[1])
        return q

    ta = [(q_start.copy(), -1)]
    tb = [(g.copy(), -1) for g in goal_qs]
    a_is_start = True

    def nearest(tree, q):
        return int(np.argmin([np.linalg.norm(node[0] - q) for node in tree]))

    def extend(tree, q_target):
        """Greedy-step the nearest node toward q_target; returns the index of
        the last node added (None if the very first step collides)."""
        i = nearest(tree, q_target)
        added = None
        while True:
            q_near = tree[i][0]
            d = q_target - q_near
            dist = float(np.linalg.norm(d))
            q_new = q_target if dist <= step else q_near + d / dist * step
            if not _edge_free(ctx, q_near, q_new, resolution):
                return added
            tree.append((q_new.copy(), i))
            i = len(tree) - 1
            added = i
            if dist <= step:
                return added  # reached q_target exactly

    for _ in range(max_iters):
        q_rand = sample()
        ia = extend(ta, q_rand)
        if ia is not None:
            ib = extend(tb, ta[ia][0])
            if ib is not None and np.linalg.norm(tb[ib][0] - ta[ia][0]) < 1e-9:
                pa = _walk(ta, ia)  # tree-a root -> connect point
                pb = _walk(tb, ib)  # tree-b root -> connect point
                if a_is_start:
                    return pa + pb[::-1][1:]
                return pb + pa[::-1][1:]
        ta, tb = tb, ta  # classic RRT-Connect role swap
        a_is_start = not a_is_start
    return None


def shortcut(ctx, path, attempts=100, seed=0, resolution=0.05):
    """Random-pair shortcutting with full edge re-validation."""
    path = [np.asarray(q, dtype=float) for q in path]
    rng = np.random.default_rng(seed)
    for _ in range(attempts):
        if len(path) <= 2:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j - i < 2:
            continue
        if _edge_free(ctx, path[i], path[j], resolution):
            path = path[: i + 1] + path[j:]
    return path


def time_parameterize(path, joints=PLANNED_JOINTS, v_limits=None, min_dt=0.1):
    """Per-segment dwell = max joint displacement / that joint's velocity limit
    (floored at min_dt). Returns [(t_sec, {joint: qpos})] with plain floats
    (JSON-ready for the posture_target wire)."""
    vl = v_limits or DEFAULT_V_LIMITS
    v = np.asarray([vl[j] for j in joints], dtype=float)

    def named(q):
        return {j: float(x) for j, x in zip(joints, np.asarray(q, dtype=float))}

    t = 0.0
    waypoints = [(0.0, named(path[0]))]
    for qa, qb in zip(path, path[1:]):
        qa = np.asarray(qa, dtype=float)
        qb = np.asarray(qb, dtype=float)
        t += max(min_dt, float(np.max(np.abs(qb - qa) / v)))
        waypoints.append((t, named(qb)))
    return waypoints


def plan_reach(client, pos, quat_wxyz, site="cup_site", joints=PLANNED_JOINTS,
               n_goals=12, max_iters=2000, seed=0, v_limits=None,
               exclude_body_suffix=None):
    """Full pipeline: synced snapshot -> goal IK -> RRT-Connect -> shortcut ->
    timed waypoints. Raises PlanError('goal_ik'|'rrt') on failure.

    Goal selection: sample n_goals IK solutions, then rank them by TRACKABILITY
    (the direct start->goal move time, max_j |dq_j| / v_j) and hand rrt_connect
    the easiest first. A marginal goal can converge in IK yet stall the
    velocity-limited QP at execution (observed live: a lone awkward goal froze
    tracking at err 1.68 where a nearer goal reached +11 cm); ranking closest-
    first makes trivial-connect prefer the trackable one and the extra seeds
    make a good goal far likelier to exist."""
    import time as _time
    t0 = _time.time()
    ctx = PlanContext.from_client(client, joints)
    if exclude_body_suffix:
        ctx.park_body(exclude_body_suffix)   # held object: see park_body
    goals = sample_goal_configs(ctx, pos, quat_wxyz, site=site, n=n_goals, seed=seed)
    if not goals:
        raise PlanError("goal_ik",
                        f"no collision-free IK solution for {site} at {np.round(pos, 3)}")
    # Rank by direct-move time from the current config (same metric as
    # time_parameterize), most-trackable first.
    q0 = ctx.q_start()
    vl = v_limits or DEFAULT_V_LIMITS
    vvec = np.asarray([vl[j] for j in joints], dtype=float)
    goals.sort(key=lambda g: float(np.max(np.abs(np.asarray(g) - q0) / vvec)))
    path = rrt_connect(ctx, q0, goals, max_iters=max_iters, seed=seed)
    if path is None:
        raise PlanError("rrt",
                        f"RRT-Connect exhausted {max_iters} iterations ({len(goals)} goals)")
    path = shortcut(ctx, path, seed=seed)
    wps = time_parameterize(path, joints, v_limits)
    return Plan(waypoints=wps, stats={
        "n_goals": len(goals), "n_waypoints": len(wps),
        "goal_reach_s": round(float(np.max(np.abs(np.asarray(path[-1]) - q0) / vvec)), 2),
        "duration_s": wps[-1][0], "plan_wall_s": _time.time() - t0,
    })
