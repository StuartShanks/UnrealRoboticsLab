#!/usr/bin/env python3
"""Offline self-tests for the multi-pick skills (stubbed client — no editor,
no physics; live behavior is validated at the live gates). Run:
/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python test_urlab_skills_multi.py"""
import sys, time, types
sys.path.insert(0, ".")
import numpy as np
import py_trees
import urlab_skills as U


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
d0 = [float(np.linalg.norm(p - np.array([-10.0, -18.0]))) for p in ring]
assert d0 == sorted(d0), "candidates not sorted by drive distance"

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
U.synced_site_pose = lambda client, s: (np.array([0.0, 0.0, 0.7]),
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
assert abs(bb.affordance.point[2] - (0.55 + 0.022)) < 1e-9, bb.affordance.point

print("task-3 CarryTransit tests OK")
