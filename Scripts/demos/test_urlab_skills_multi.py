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
