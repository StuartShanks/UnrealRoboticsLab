#!/usr/bin/env python3
"""Offline self-tests for the multi-pick skills (stubbed client — no editor,
no physics; live behavior is validated at the live gates). Run:
/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python test_urlab_skills_multi.py"""
import sys, time, types
sys.path.insert(0, ".")
import numpy as np
import py_trees
import urlab_skills as U

# Captured BEFORE any block monkeypatches U._base_xy — the world-frame
# regression test near the bottom must exercise the REAL function.
_REAL_base_xy = U._base_xy


class StubRuntime:
    """Scripted set_nav_goal/get_nav_status; records every payload."""
    def __init__(self, goal_replies, status_replies):
        self.goal_replies = list(goal_replies)
        self.status_replies = list(status_replies)
        self.goal_calls = []
    def set_nav_goal(self, **kw):
        self.goal_calls.append(kw)
        return self.goal_replies.pop(0)
    def get_nav_status(self, **kw):
        return self.status_replies.pop(0)
    def set_suction(self, **kw):
        pass


class StubClient:
    def __init__(self, runtime):
        self.runtime = runtime
        self.configures = []
    def _rpc_configure_controller(self, articulation=None, params=None):
        self.configures.append(params)


def make_bb(runtime):
    bb = U.PickBlackboard()
    bb.client = StubClient(runtime)
    bb.name = "bot"
    return bb


# --- Drive: max_projection reaches the wire; rejection carries the reason ---
rt = StubRuntime(
    goal_replies=[{"accepted": False, "reason": "projection_exceeds_max",
                   "projection_m": 0.33}],
    status_replies=[],
)
bb = make_bb(rt)
d = U.Drive("d", bb, (1.0, 2.0), timeout_s=5.0, max_projection=0.30)
d.initialise()
assert rt.goal_calls[0]["max_projection"] == 0.30, rt.goal_calls[0]
st = d.update()
assert st == py_trees.common.Status.FAILURE
assert "projection_exceeds_max" in bb.fail_reason, bb.fail_reason

# --- Drive: omitted max_projection stays off the wire (back-compat) ---
rt = StubRuntime(
    goal_replies=[{"accepted": True}],
    status_replies=[{"state": "arrived", "distance_to_goal": 0.0}],
)
bb = make_bb(rt)
d = U.Drive("d", bb, (1.0, 2.0), timeout_s=5.0)
d.initialise()
assert "max_projection" not in rt.goal_calls[0], rt.goal_calls[0]
assert d.update() == py_trees.common.Status.SUCCESS

print("task-1 Drive tests OK")

# --- staging_ring: geometry + ordering (pure) --------------------------------
AABB = (-13.20, -12.44, -15.63, -14.87)
ring = U.staging_ring((-12.71, -15.36), AABB, here_xy=np.array([-10.0, -18.0]))
assert len(ring) > 0
for p in ring:
    inside = (AABB[0] - 0.58 <= p[0] <= AABB[1] + 0.58) and \
             (AABB[2] - 0.58 <= p[1] <= AABB[3] + 0.58)
    assert not inside, f"candidate {p} inside the eroded band"
# Ordering: inner radius first (reach quality dominates drive cost — outer
# staging pinned the descend ~3 cm short, live), then drive distance.
keys = [(round(float(np.linalg.norm(p - np.array([-12.71, -15.36]))), 6),
         float(np.linalg.norm(p - np.array([-10.0, -18.0])))) for p in ring]
assert keys == sorted(keys), "candidates not sorted (radius, drive distance)"

# --- StageAt: first candidate rejected -> second verified --------------------
rt = StubRuntime(
    goal_replies=[
        {"accepted": False, "reason": "projection_exceeds_max", "projection_m": 0.4},
        {"accepted": True},
    ],
    status_replies=[{"state": "arrived", "distance_to_goal": 0.0}],
)
bb = make_bb(rt)
stage = U.StageAt("stage", bb, AABB, (-12.71, -15.36))
# Arrival verification: base "lands" exactly on the last requested candidate;
# before any goal was requested (the ring-sort read in initialise) report the
# spawn point.
U._base_xy = lambda client: (
    np.array([rt.goal_calls[-1]["x"], rt.goal_calls[-1]["y"]])
    if rt.goal_calls else np.array([-10.0, -18.0]))
stage.initialise()
status = py_trees.common.Status.RUNNING
for _ in range(10):
    status = stage.update()
    if status != py_trees.common.Status.RUNNING:
        break
assert status == py_trees.common.Status.SUCCESS, bb.fail_reason
assert len(rt.goal_calls) == 2, "should have advanced past the rejected candidate"
assert rt.goal_calls[0]["max_projection"] == 0.30

# --- StageAt: arrival too far from the REQUESTED point -> next candidate -----
rt = StubRuntime(
    goal_replies=[{"accepted": True}, {"accepted": True}],
    status_replies=[
        {"state": "arrived", "distance_to_goal": 0.0},
        {"state": "arrived", "distance_to_goal": 0.0},
    ],
)
bb = make_bb(rt)
calls = {"n": 0}
def _fake_base_xy(client):
    calls["n"] += 1
    if calls["n"] == 1:   # ring-sort read in initialise (no goals yet)
        return np.array([-10.0, -18.0])
    if calls["n"] == 2:   # first arrival verification: 1 m off the request
        return np.array([rt.goal_calls[0]["x"] + 1.0, rt.goal_calls[0]["y"]])
    return np.array([rt.goal_calls[-1]["x"], rt.goal_calls[-1]["y"]])
U._base_xy = _fake_base_xy
stage = U.StageAt("stage", bb, AABB, (-12.71, -15.36))
stage.initialise()
status = py_trees.common.Status.RUNNING
for _ in range(10):
    status = stage.update()
    if status != py_trees.common.Status.RUNNING:
        break
assert status == py_trees.common.Status.SUCCESS, bb.fail_reason
assert len(rt.goal_calls) == 2, "substituted arrival must advance the ring"

print("task-2 StageAt tests OK")

# --- CarryTransit: tuck phase then drive; suction held; drop guard fires -----
class StubOutliner:
    def __init__(self, z_seq):
        self.z_seq = list(z_seq)
    def find_actors(self, class_filter=None, in_pie=False):
        z = self.z_seq.pop(0) if len(self.z_seq) > 1 else self.z_seq[0]
        return [types.SimpleNamespace(name="SM_Book_125",
                                      location=(-12.7, -15.4, z))]

held = {"n": 0}
def _fake_hold(client, name):
    held["n"] += 1
U._hold_suction = _fake_hold
U.synced_site_pose = lambda client, s: (np.array([-12.7, -15.4, 0.7]),  # over the stub book (ride-centering check)
                                        np.array([0.0, 1.0, 0.0, 0.0]))
U.stream_target = lambda client, name, p, q: None

rt = StubRuntime(
    goal_replies=[{"accepted": True}],
    status_replies=[{"state": "navigating", "distance_to_goal": 2.0},
                    {"state": "arrived", "distance_to_goal": 0.0}],
)
bb = make_bb(rt)
bb.client.outliner = StubOutliner([0.94] * 8)
ct = U.CarryTransit("carry", bb, (-11.1, -18.65), "SM_Book_125",
                    tuck_duration=0.01)   # ~instant tuck (0.0 would divide by zero in StowCarry)
ct.initialise()
status = py_trees.common.Status.RUNNING
for _ in range(20):
    status = ct.update()
    if status != py_trees.common.Status.RUNNING:
        break
    time.sleep(0.02)
assert status == py_trees.common.Status.SUCCESS, bb.fail_reason
assert held["n"] >= 2, "suction must be re-asserted through the transit"

# drop guard: book z collapses mid-drive -> FAILURE with reason
rt = StubRuntime(
    goal_replies=[{"accepted": True}],
    status_replies=[{"state": "navigating", "distance_to_goal": 2.0}] * 8,
)
bb = make_bb(rt)
bb.client.outliner = StubOutliner([0.94, 0.94, 0.10])
ct = U.CarryTransit("carry", bb, (-11.1, -18.65), "SM_Book_125",
                    tuck_duration=0.01)
ct.initialise()
status = py_trees.common.Status.RUNNING
for _ in range(20):
    status = ct.update()
    if status != py_trees.common.Status.RUNNING:
        break
    time.sleep(0.02)
assert status == py_trees.common.Status.FAILURE
assert "dropped" in bb.fail_reason, bb.fail_reason

# --- _object_z name fallback + ResolveActorTop -------------------------------
bb = make_bb(StubRuntime([], []))
bb.client.outliner = StubOutliner([0.55])
assert abs(U._object_z(bb.client, "SM_Book_125") - 0.55) < 1e-9, \
    "_object_z must fall back to name matching for Fab actors"
r = U.ResolveActorTop("resolve", bb, "SM_Book_125", 0.011)
r.initialise()
assert r.update() == py_trees.common.Status.SUCCESS
assert abs(bb.affordance.point[2] - 0.55) < 1e-9, bb.affordance.point  # pivot = top

print("task-3 CarryTransit tests OK")

# --- PlaceOn: descend->release->retract phase machine (plan phase stubbed) ---
class StubPlannedReach:
    """Stands in for PlannedReach: immediately SUCCESS. Records ctor kwargs
    so PlaceOn's held-object exclusion pass-through is asserted."""
    last_kwargs = None
    def __init__(self, *a, **k):
        StubPlannedReach.last_kwargs = k
    def initialise(self): pass
    def update(self): return py_trees.common.Status.SUCCESS
    def terminate(self, s): pass

U_PlannedReach_orig = U.PlannedReach
U.PlannedReach = StubPlannedReach

cup_z = {"z": 1.27}
def _fake_site_pose(client, s):
    return (np.array([-11.10, -19.60, cup_z["z"]]),
            np.array([0.0, 1.0, 0.0, 0.0]))
streamed = []
def _fake_stream(client, name, p, q):
    streamed.append(np.array(p))
    cup_z["z"] = max(1.11 + 0.03, cup_z["z"] - 0.02)   # cup tracks down
U.synced_site_pose = _fake_site_pose
U.stream_target = _fake_stream

# book pivot (= TOP face) follows the cup down, then settles at rest height
# surface + 2*half = 1.132
book = {"z": 1.17}
def _fake_actor_z(client, name):
    book["z"] = max(1.11 + 0.022, cup_z["z"] - 0.06)
    return book["z"]
U._actor_z_by_name = _fake_actor_z

suction = {"vals": []}
rt = StubRuntime(goal_replies=[], status_replies=[])
rt.set_suction = lambda **kw: suction["vals"].append(kw.get("value"))
bb = make_bb(rt)
bb.q_cup = U.cup_down_quat(np.array([0.0, 0.0, 1.0]))
place = U.PlaceOn("place", bb, (-11.10, -19.60), 1.11, 0.011, "SM_Book_125")
place.initialise()
status = py_trees.common.Status.RUNNING
for _ in range(600):
    status = place.update()
    if status != py_trees.common.Status.RUNNING:
        break
    time.sleep(0.005)   # release dwell + retract are wall-clock phases
assert status == py_trees.common.Status.SUCCESS, bb.fail_reason
assert 0.0 in suction["vals"], "release must set_suction(0)"
assert StubPlannedReach.last_kwargs.get("exclude_body") == "SM_Book_125", \
    f"PlaceOn must exclude the held object from the plan: {StubPlannedReach.last_kwargs}"
assert bb.affordance is not None and abs(bb.affordance.point[2] - 1.24) < 1e-6, \
    "plan affordance must target surface_z + 0.13"
U.PlannedReach = U_PlannedReach_orig

print("task-4 PlaceOn tests OK")

# --- DescendEngage falls back to bb.affordance when the MJCF site is absent --
def _no_affordance_site(client, site):
    if site == "affordance_suction_top":
        raise RuntimeError("site suffix 'affordance_suction_top' not found in compiled model")
    return (np.array([-12.71, -15.36, 0.60]), np.array([0.0, 1.0, 0.0, 0.0]))  # cup_site
U.synced_site_pose = _no_affordance_site
bb = make_bb(StubRuntime([], []))
bb.affordance = U.Affordance(
    point=np.array([-12.71, -15.36, 0.572]),
    normal=np.array([0.0, 0.0, 1.0]),
    quat_cup_down=U.cup_down_quat(np.array([0.0, 0.0, 1.0])))
de = U.DescendEngage("descend", bb)
de.initialise()
assert de._init_error is None, f"DescendEngage errored on a Fab object: {de._init_error}"
assert np.allclose(de._surf, [-12.71, -15.36, 0.572]), de._surf

print("fix DescendEngage-fallback test OK")

# --- _base_xy returns WORLD base xy (regression: joint-frame bug) ------------
# At a non-origin spawn, base joint qpos are spawn-frame-relative; reading
# them as world put StageAt's ring sort ~14 m off (sorted from the world
# origin — live gate 1 tried only far/blocked candidates). The REAL function
# must return the slide-driven body's world xpos.
import mujoco as _mj
_mjcf = """
<mujoco><worldbody>
  <body name="rig" pos="6 -4 0.1">
    <body name="bx">
      <joint name="r_joint_x" type="slide" axis="1 0 0" range="-5 5"/>
      <geom type="sphere" size="0.02"/>
      <body name="by">
        <joint name="r_joint_y" type="slide" axis="0 1 0" range="-5 5"/>
        <geom type="box" size="0.1 0.1 0.1"/>
      </body>
    </body>
  </body>
</worldbody></mujoco>
"""
_mm = _mj.MjModel.from_xml_string(_mjcf)
_dd = _mj.MjData(_mm)
_dd.qpos[0] = 0.5   # joint_x
_dd.qpos[1] = 0.25  # joint_y
_mj.mj_forward(_mm, _dd)
class _BaseXYClient:
    model = _mm
    data = _dd
    def step(self, n_steps=1):
        pass   # mirror-sync no-op; the toy data is forward'd above
_got = _REAL_base_xy(_BaseXYClient())
assert np.allclose(_got, [6.5, -3.75]), \
    f"_base_xy must be WORLD (spawn + joints), got {_got}"

print("fix base-xy world-frame test OK")

# --- driver: tree assembles with the right legs ------------------------------
import tidybot_multi_pick_demo as M
bb = make_bb(StubRuntime([], []))
t_full = M.build_tree(bb, one_way=False)
t_half = M.build_tree(bb, one_way=True)
names_full = [c.name for c in t_full.children]
assert len(t_full.children) == 14, names_full   # 5 pick + carry + place + 1 stage + 4 re-pick + carry + place
assert len(t_half.children) == 7, [c.name for c in t_half.children]
assert names_full[0].startswith("stage") and "place_table" in names_full[-1]

print("task-6 tree assembly OK")
print("ALL multi-pick offline tests OK")
