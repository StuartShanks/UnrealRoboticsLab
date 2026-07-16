#!/usr/bin/env python3
"""Self-tests for urlab_skills pure functions (no editor, no client).
Run: /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python test_urlab_skills.py"""
import numpy as np
import sys
sys.path.insert(0, ".")
from urlab_skills import approach_waypoints, cup_down_quat, due_waypoint, reach_annulus_ok

# approach_waypoints: template from an affordance frame.
pt = np.array([1.0, 2.0, 0.5]); n = np.array([0.0, 0.0, 1.0])
w = approach_waypoints(pt, n)
assert np.allclose(w[0], [1.0, 2.0, 0.65]), w[0]   # pre-approach +15cm
assert np.allclose(w[1], [1.0, 2.0, 0.52]), w[1]   # engage +2cm
assert np.allclose(w[2], pt), w[2]                  # contact
# non-unit normals are normalized
w2 = approach_waypoints(pt, np.array([0.0, 0.0, 2.0]))
assert np.allclose(w2[0], [1.0, 2.0, 0.65]), w2[0]

# cup_down_quat: cup axis anti-parallel to the site normal. For n=+z the cup
# points down — matches the pinch/cup frame convention quat (w,x,y,z)=(0,1,0,0).
q = cup_down_quat(np.array([0.0, 0.0, 1.0]))
assert np.allclose(np.abs(q), [0.0, 1.0, 0.0, 0.0], atol=1e-6), q

# reach annulus
shoulder = np.array([4.2, 0.0, 0.49])
assert reach_annulus_ok(shoulder, np.array([4.9, 0.0, 0.5]))          # 0.70 -> ok
assert not reach_annulus_ok(shoulder, np.array([5.3, 0.0, 0.5]))      # 1.10 -> too far
assert not reach_annulus_ok(shoulder, np.array([4.3, 0.0, 0.45]))     # 0.11 -> too close

# suction payload: posture task must span all 10 joints (posture_target wire
# streams whole-body plans through it; SubsetCost makes unlisted joints inert).
from tidybot_suction_pick_demo import suction_pick_controller_payload
_payload = suction_pick_controller_payload([])
assert _payload["tasks"][1]["kind"] == "posture", _payload["tasks"][1]
assert _payload["tasks"][1]["joints"] == [
    "joint_x", "joint_y", "joint_th",
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7",
], _payload["tasks"][1]["joints"]

# due_waypoint: schedule lookup for PlannedReach streaming.
_wps = [(0.0, {"j": 0.0}), (1.0, {"j": 1.0}), (2.5, {"j": 2.0})]
assert due_waypoint(_wps, -0.1) == 0
assert due_waypoint(_wps, 0.0) == 0
assert due_waypoint(_wps, 1.7) == 1
assert due_waypoint(_wps, 99.0) == 2

print("urlab_skills self-tests OK")
