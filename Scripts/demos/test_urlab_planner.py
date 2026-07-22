#!/usr/bin/env python3
"""Self-tests for the whole-body RRT connector (pure MuJoCo — no editor, no
client). Run: /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python3 test_urlab_planner.py"""
import numpy as np
import mujoco
import sys
sys.path.insert(0, ".")
from urlab_planner import (PlanContext, PlanError, rrt_connect, shortcut,
                           time_parameterize, sample_goal_configs, _edge_free,
                           _base_frame_offset_xy)

# Synthetic scene: planar base (2 slides) + 1 hinge arm + a wall the base must
# route around. Wall at x=1.0, y half-size 0.5 => passable at |y| > ~0.75.
SCENE = """
<mujoco>
  <option timestep="0.005"/>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 1"/>
    <body name="wall" pos="1.0 0 0.5"><geom name="wall_g" type="box" size="0.1 0.5 0.5"/></body>
    <body name="base" pos="0 0 0.35">
      <joint name="joint_x" type="slide" axis="1 0 0" range="-5 5"/>
      <joint name="joint_y" type="slide" axis="0 1 0" range="-5 5"/>
      <geom name="base_g" type="box" size="0.2 0.2 0.1"/>
      <body name="link" pos="0 0 0.2">
        <joint name="joint_1" type="hinge" axis="0 0 1" range="-3 3"/>
        <geom name="arm_g" type="capsule" fromto="0 0 0 0.4 0 0" size="0.05"/>
        <site name="tip" pos="0.4 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
JOINTS = ["joint_x", "joint_y", "joint_1"]
m = mujoco.MjModel.from_xml_string(SCENE)
ctx = PlanContext(m, np.zeros(m.nq), joints=JOINTS)

# --- collision_free: free space ok, inside the wall not -----------------------
assert ctx.collision_free(np.array([0.0, 0.0, 0.0]))
assert not ctx.collision_free(np.array([1.0, 0.0, 0.0]))       # base in the wall
assert not ctx.collision_free(np.array([0.0, 0.0, 10.0]))      # joint limit

# --- margin filter: positive-distance margin contacts are NOT collisions ------
MARGIN_SCENE = """
<mujoco><worldbody>
  <body name="a" pos="0 0 1"><joint name="joint_x" type="slide" axis="1 0 0" range="-5 5"/>
    <geom name="cup_g" type="box" size="0.1 0.1 0.1" margin="0.05" gap="0.05"/></body>
  <body name="b" pos="0.22 0 1"><geom name="b_g" type="box" size="0.1 0.1 0.1"/></body>
</worldbody></mujoco>
"""
mm = mujoco.MjModel.from_xml_string(MARGIN_SCENE)
mctx = PlanContext(mm, np.zeros(mm.nq), joints=["joint_x"])
assert mctx.collision_free(np.array([0.0]))        # 2cm apart, inside 5cm margin: OK
assert not mctx.collision_free(np.array([0.05]))   # overlapping: collision

# --- baseline filter: pre-existing resting penetration is exempt --------------
REST_SCENE = """
<mujoco><worldbody>
  <geom name="floor" type="plane" size="10 10 1"/>
  <body name="base" pos="0 0 0.095">
    <joint name="joint_x" type="slide" axis="1 0 0" range="-5 5"/>
    <geom name="base_g" type="box" size="0.2 0.2 0.1"/>
  </body>
</worldbody></mujoco>
"""
rm = mujoco.MjModel.from_xml_string(REST_SCENE)
rctx = PlanContext(rm, np.zeros(rm.nq), joints=["joint_x"])
assert rctx.collision_free(np.array([0.0]))        # resting penetration at start
assert rctx.collision_free(np.array([2.0]))        # same pair elsewhere: still exempt

# --- rrt_connect: routes around the wall ---------------------------------------
q_start = np.array([-0.5, 0.0, 0.0])
q_goal = np.array([2.2, 0.0, 0.0])
assert not _edge_free(ctx, q_start, q_goal)        # straight line blocked
path = rrt_connect(ctx, q_start, [q_goal], seed=1)
assert path is not None, "planner failed to route around the wall"
assert np.allclose(path[0], q_start) and np.allclose(path[-1], q_goal)
for qa, qb in zip(path, path[1:]):
    assert _edge_free(ctx, qa, qb), "path has a colliding edge"

# --- shortcut: no longer, still valid ------------------------------------------
short = shortcut(ctx, path, seed=1)
assert len(short) <= len(path)
assert np.allclose(short[0], q_start) and np.allclose(short[-1], q_goal)
for qa, qb in zip(short, short[1:]):
    assert _edge_free(ctx, qa, qb)

# --- time_parameterize: monotone, respects per-joint velocity ------------------
wps = time_parameterize(short, joints=JOINTS,
                        v_limits={"joint_x": 0.6, "joint_y": 0.6, "joint_1": 0.5})
assert wps[0][0] == 0.0
for (ta, qa), (tb, qb) in zip(wps, wps[1:]):
    assert tb > ta
    for j, v in (("joint_x", 0.6), ("joint_y", 0.6), ("joint_1", 0.5)):
        assert abs(qb[j] - qa[j]) <= v * (tb - ta) + 1e-9

# --- goal IK (mink): tip reaches a nearby world pose ----------------------------
goals = sample_goal_configs(ctx, pos=[0.3, 0.4, 0.55], quat_wxyz=[1, 0, 0, 0],
                            site="tip", n=4, seed=0, ori_tol=1e9)  # position-only
assert goals, "goal IK found nothing for an easy pose"
p, _ = ctx.site_pose(goals[0], "tip")
assert np.linalg.norm(p - np.array([0.3, 0.4, 0.55])) < 0.02, p

# --- free-roll axis must not veto convergence ----------------------------------
# orientation_cost=[1,1,0] frees rotation about the site z (tool) axis, so the
# convergence check may only score the two constrained axes. In this toy the
# hinge IS the free z-roll: random seeds leave it far from the target roll, and
# scoring err[3:] rejected perfect grasps (live: every HomeInterior seed had
# pos_err 0.000 and |ori_xy| 0.000 but 0.3-1.85 rad of don't-care roll).
# (target far from the wall so collisions don't confound the roll check; the
# 3-DOF toy collapses all converged seeds onto 2 distinct configs via dedupe,
# so >=2 is full yield here — pre-fix the roll veto yields 0)
rgoals = sample_goal_configs(ctx, pos=[-0.6, 0.3, 0.55], quat_wxyz=[1, 0, 0, 0],
                             site="tip", n=6, seed=3, ori_tol=5e-2)
assert len(rgoals) >= 2, f"free roll vetoed convergence: {len(rgoals)} goals"

# --- world->joint base frame: NON-ORIGIN spawn ---------------------------------
# joint_x/joint_y are relative to the spawn frame; the annulus seeding must
# shift world targets into joint frame. Regression for the HomeInterior live
# failure: robot spawned at (-10,-18), seeds landed 10-20 m off, goal_ik dry.
OFF_SCENE = SCENE.replace('<body name="base" pos="0 0 0.35">',
                          '<body name="base" pos="6 -4 0.35">')
mo = mujoco.MjModel.from_xml_string(OFF_SCENE)
octx = PlanContext(mo, np.zeros(mo.nq), joints=JOINTS)
assert np.allclose(_base_frame_offset_xy(octx), [6.0, -4.0], atol=1e-9), \
    _base_frame_offset_xy(octx)
assert np.allclose(_base_frame_offset_xy(ctx), [0.0, 0.0], atol=1e-9)  # origin spawn
# End-to-end: a WORLD target near the offset spawn is found, goals place the
# base (world frame) within annulus+arm reach of the target, and the tip lands
# on the target.
TGT = np.array([6.3, -3.6, 0.55])
ogoals = sample_goal_configs(octx, pos=TGT, quat_wxyz=[1, 0, 0, 0],
                             site="tip", n=8, seed=0, ori_tol=1e9)
assert ogoals, "goal IK found nothing for an offset-spawn world target"
for g in ogoals:
    base_world = np.array([6.0, -4.0]) + g[:2]
    assert np.linalg.norm(base_world - TGT[:2]) < 1.35, (g[:2], base_world)
    p, _ = octx.site_pose(g, "tip")
    assert np.linalg.norm(p - TGT) < 0.02, p

print("urlab_planner self-tests OK")
