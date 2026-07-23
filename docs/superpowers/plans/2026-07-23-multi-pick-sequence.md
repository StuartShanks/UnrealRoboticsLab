# Multi-Pick Sequence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One py_trees run in HomeInterior: pick the Fab book off the small table, drive while holding it to the kitchen island, place it, re-pick it, drive back, place it back — the roadmap's "nav, grab, nav while holding".

**Architecture:** Behavior tree over skills over controllers. Three new/upgraded skills in `Scripts/demos/urlab_skills.py` (`StageAt`, `CarryTransit`, `PlaceOn`, plus a `max_projection` pass-through on `Drive`), a forked author script, and a tree driver. Fail-fast Sequence; teardown in the driver's `finally`. Pure Python — no engine changes.

**Tech Stack:** Python 3 (bridge venv `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv`), py_trees, numpy, mujoco, `urlab_client`. Offline tests are assert-scripts (the `test_urlab_planner.py` convention), run with the bridge venv python.

**Spec:** `docs/superpowers/specs/2026-07-23-multi-pick-sequence-design.md`

## Global Constraints

- Scenario constants (verbatim from the spec): book `SM_Book_125` half-thickness `0.011`; table AABB `(-13.20, -12.44, -15.63, -14.87)` top `0.55`; island AABB `(-11.99, -10.17, -20.26, -19.38)` top `1.11`; island place point `(-11.10, -19.60)`; table return point `(-12.71, -15.36)`; erosion inflation `0.58` m; ring radii `0.95/1.10` m × 16 bearings; staging verify tolerance `0.30` m; `max_projection 0.30`; `carry_z_min = 0.45` m; nav `max_speed 0.4`.
- Transit doctrine: NO world-frame EE hold while the base moves — tuck via `StowCarry`'s pattern, EE task disabled, posture holds.
- Seed-then-enable: never re-enable the EE frame task without first streaming the current pose.
- All new skills follow the module's existing conventions: `py_trees.behaviour.Behaviour`, `bb.fail_reason` on FAILURE, `terminate()` restores what `initialise()` changed.
- Fail-fast: no new recovery machinery; only StageAt's candidate ring retries.
- No engine/C++ changes.
- Offline tests must not import a live client — stubs only; live behavior is validated at the live gates.
- Run offline tests with: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python <file>` from `Scripts/demos/`.

## File Map

- Modify: `Scripts/demos/urlab_skills.py` — `Drive` upgrade; new `staging_ring()`, `_actor_z_by_name()`, `StageAt`, `CarryTransit`, `PlaceOn`.
- Create: `Scripts/demos/test_urlab_skills_multi.py` — offline assert-script for all of the above (stub client, no physics).
- Create: `Scripts/demos/tidybot_multi_pick_author.py` — authoring (fork of the validated fab author, cup guard emptied, `max_speed=0.4`).
- Create: `Scripts/demos/tidybot_multi_pick_demo.py` — Station config + tree + driver (`--one-way` flag for live gate 1).

---

### Task 1: `Drive` gains `max_projection` + honest reply logging

**Files:**
- Modify: `Scripts/demos/urlab_skills.py` (class `Drive`)
- Test: `Scripts/demos/test_urlab_skills_multi.py` (create)

**Interfaces:**
- Consumes: existing `Drive(name, bb, goal_xy, timeout_s)` contract.
- Produces: `Drive(name, bb, goal_xy, timeout_s=NAV_TIMEOUT_S, max_projection=None)`. When `max_projection` is not None it is added to the `set_nav_goal` payload; the reply's `reason`/`projection_m`/`start_projection_m` are folded into `bb.fail_reason` (rejection) or logged via `self.logger.info` (recovery leg). Existing callers unaffected.

- [ ] **Step 1: Write the failing test (stub-client harness + two Drive cases)**

Create `Scripts/demos/test_urlab_skills_multi.py`:

```python
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd Scripts/demos && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python test_urlab_skills_multi.py`
Expected: `TypeError: __init__() got an unexpected keyword argument 'max_projection'`

- [ ] **Step 3: Implement**

In `urlab_skills.py`, class `Drive`:

```python
    def __init__(self, name, bb, goal_xy, timeout_s: float = NAV_TIMEOUT_S,
                 max_projection: float = None):
        super().__init__(name)
        self.bb = bb
        self.goal_xy = goal_xy
        self.timeout_s = float(timeout_s)
        self.max_projection = max_projection
        self._deadline = None
        self._min_dist = None
```

and in `initialise()` replace the `set_nav_goal` call + rejection handling with:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd Scripts/demos && /home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python test_urlab_skills_multi.py`
Expected: `task-1 Drive tests OK`

- [ ] **Step 5: Commit**

```bash
git add Scripts/demos/urlab_skills.py Scripts/demos/test_urlab_skills_multi.py
git commit -m "feat(skills): Drive passes max_projection + reports rejection reason / recovery leg"
```

---

### Task 2: `staging_ring()` + `StageAt` (staging-policy-as-a-skill)

**Files:**
- Modify: `Scripts/demos/urlab_skills.py`
- Test: `Scripts/demos/test_urlab_skills_multi.py` (append)

**Interfaces:**
- Consumes: Task 1's `Drive(..., max_projection=...)`; existing `_base_xy(client)`.
- Produces:
  - `staging_ring(target_xy, station_aabb, here_xy, erode_m=0.58, radii=(0.95, 1.10), n_bearings=16) -> list[np.ndarray]` — pure; candidates outside `station_aabb` inflated by `erode_m`, sorted by distance from `here_xy`.
  - `StageAt(name, bb, station_aabb, target_xy, erode_m=0.58, verify_m=0.30, max_projection=0.30, max_tries=4, timeout_s=NAV_TIMEOUT_S)` — SUCCESS on the first candidate that Drives to arrival AND lands within `verify_m` of the REQUESTED candidate (`_base_xy`); otherwise advances; FAILURE when the ring/tries are exhausted.

- [ ] **Step 1: Write the failing tests (append to `test_urlab_skills_multi.py`, before the final print — move the print to the file end as each task appends)**

```python
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
```

- [ ] **Step 2: Run to verify failure** — Expected: `AttributeError: module 'urlab_skills' has no attribute 'staging_ring'`

- [ ] **Step 3: Implement in `urlab_skills.py`**

```python
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
    out.sort(key=lambda p: float(np.linalg.norm(p - here)))
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
        self.bb.fail_reason = ""
        self._drive, self._cand = self._next_drive()

    def update(self):
        bb = self.bb
        if self._drive is None:
            if not bb.fail_reason:
                bb.fail_reason = (f"StageAt[{self.name}]: ring exhausted "
                                  f"({self._tries} tries)")
            return py_trees.common.Status.FAILURE
        status = self._drive.update()
        if status == py_trees.common.Status.RUNNING:
            return py_trees.common.Status.RUNNING
        if status == py_trees.common.Status.SUCCESS:
            actual = _base_xy(bb.client)
            miss = float(np.linalg.norm(actual - self._cand))
            if miss <= self.verify_m:
                self.logger.info(f"staged at {np.round(actual, 2)} "
                                 f"({miss:.2f} m from requested)")
                return py_trees.common.Status.SUCCESS
            self.logger.info(f"arrival {miss:.2f} m off requested "
                             f"{np.round(self._cand, 2)} — next candidate")
        # rejected / failed / unverified: advance the ring
        bb.fail_reason = ""
        self._drive, self._cand = self._next_drive()
        if self._drive is None:
            bb.fail_reason = (f"StageAt[{self.name}]: ring exhausted "
                              f"({self._tries} tries)")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"StageAt[{self.name}]: failed"
```

- [ ] **Step 4: Run tests** — Expected: `task-1 Drive tests OK` then `task-2 StageAt tests OK`

- [ ] **Step 5: Commit**

```bash
git add Scripts/demos/urlab_skills.py Scripts/demos/test_urlab_skills_multi.py
git commit -m "feat(skills): staging_ring + StageAt — verified ring staging as a skill"
```

---

### Task 3: Fab-actor helpers + `ResolveActorTop` + `CarryTransit`

**Files:**
- Modify: `Scripts/demos/urlab_skills.py`
- Test: `Scripts/demos/test_urlab_skills_multi.py` (append)

**Interfaces:**
- Consumes: existing `StowCarry(name, bb, duration)`, Task 1 `Drive`, existing `_hold_suction(client, name)`, existing `_object_z(client, object_actor_id)`, `Affordance`, `cup_down_quat`.
- Produces:
  - `_actor_z_by_name(client, ue_name) -> float` — live PIE actor z by `find_actors(class_filter="StaticMeshActor", in_pie=True)` name match (engine truth; the actor pivot for these books is the mesh BOTTOM). Raises `RuntimeError` when not found.
  - `_object_z` gains a NAME fallback: if no `actor_id` match, match `r.name` — the existing skills (`DescendEngage`/`VerifyAttach`/`Release`) read the held object via `_object_z(client, bb.object_actor_id)`, which is actor_id-keyed and never matches a Fab actor (Fab actors have no bridge actor_id). With the fallback, setting `bb.object_actor_id = "SM_Book_125"` (the UE name) makes every existing skill work unchanged.
  - `ResolveActorTop(name, bb, object_name, half_thickness)` — the Fab replacement for `ResolveAffordance` (which requires an MJCF affordance site the Fab book doesn't have): reads the live actor pose, sets `bb.affordance = Affordance(point=(x, y, pivot_z + 2*half), normal=up, quat_cup_down=cup_down_quat(up))` (pivot=bottom ⇒ top = pivot + 2·half) and `bb.q_cup`. SUCCESS always unless the actor is missing.
  - `CarryTransit(name, bb, goal_xy, object_name, carry_z_min=0.45, tuck_duration=3.0, timeout_s=NAV_TIMEOUT_S)` — phase 1 delegates to an internal `StowCarry`; phase 2 an internal `Drive` with a per-tick `_hold_suction` re-assert and a drop guard: FAILURE if `_actor_z_by_name(object_name) < carry_z_min`.

- [ ] **Step 1: Write the failing tests (append)**

```python
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
```

- [ ] **Step 2: Run to verify failure** — Expected: `AttributeError: ... no attribute 'CarryTransit'`

- [ ] **Step 3: Implement in `urlab_skills.py`**

```python
def _actor_z_by_name(client, ue_name: str) -> float:
    """Live PIE actor z (engine truth — the actor follows the MuJoCo body).
    NOTE: for the Fab books the actor pivot is the mesh BOTTOM."""
    for r in client.outliner.find_actors(class_filter="StaticMeshActor",
                                         in_pie=True):
        if r.name == ue_name:
            return float(r.location[2])
    raise RuntimeError(f"actor {ue_name!r} not found in PIE")
```

In the existing `_object_z(client, object_actor_id)`, extend the match so a
Fab actor's UE NAME works wherever an actor_id is expected (existing skills
read the held object through this): where the loop currently matches
`r.actor_id == object_actor_id`, change the condition to

```python
        if getattr(r, "actor_id", None) == object_actor_id or r.name == object_actor_id:
```

(keep everything else in `_object_z` as is), and add:

```python
class ResolveActorTop(py_trees.behaviour.Behaviour):
    """Fab-object affordance: no MJCF affordance site exists on a
    quick-converted actor, so build the Affordance from the live actor pose
    (pivot = mesh bottom for these books => top = pivot_z + 2*half).
    Replaces ResolveAffordance in Fab trees; writes bb.affordance + bb.q_cup."""

    def __init__(self, name, bb, object_name: str, half_thickness: float):
        super().__init__(name)
        self.bb = bb
        self.object_name = object_name
        self.half = float(half_thickness)
        self._error = None

    def initialise(self):
        self._error = None
        try:
            found = None
            for r in self.bb.client.outliner.find_actors(
                    class_filter="StaticMeshActor", in_pie=True):
                if r.name == self.object_name:
                    found = np.array(r.location, dtype=float)
                    break
            if found is None:
                raise RuntimeError(f"actor {self.object_name!r} not found in PIE")
            up = np.array([0.0, 0.0, 1.0])
            q = cup_down_quat(up)
            self.bb.affordance = Affordance(
                point=np.array([found[0], found[1], found[2] + 2.0 * self.half]),
                normal=up, quat_cup_down=q)
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
```

- [ ] **Step 4: Run tests** — Expected all three task prints OK.

- [ ] **Step 5: Commit**

```bash
git add Scripts/demos/urlab_skills.py Scripts/demos/test_urlab_skills_multi.py
git commit -m "feat(skills): CarryTransit — tucked, suction-held loaded transit with drop guard"
```

---

### Task 4: `PlaceOn` (the pick's inverse)

**Files:**
- Modify: `Scripts/demos/urlab_skills.py`
- Test: `Scripts/demos/test_urlab_skills_multi.py` (append)

**Interfaces:**
- Consumes: existing `PlannedReach(name, bb)` (reads `bb.affordance` — `Affordance(point, normal, quat_cup_down)` — and reaches `point + PRE_GRASP_M*normal`), `Affordance`, `cup_down_quat`, `synced_site_pose`, `stream_target`, Task 3's `_actor_z_by_name`.
- Produces: `PlaceOn(name, bb, place_xy, surface_z, half_thickness, object_name, settle_tol=0.02, descend_timeout_s=12.0)`. Phases: `plan` (synthetic affordance at `(place_xy, surface_z + 0.13)` so PlannedReach's +0.03 pre-grasp puts the cup at hover `surface_z + 0.16`) → `descend` (stream down until the OBJECT's bottom `_actor_z_by_name` settles at `surface_z ± settle_tol`, floor-limited at cup `surface_z + 0.03`) → `release` (`set_suction(0)`, dwell 1 s, verify object z stable) → `retract` (cup up 0.15 / back 0.15, restore costs). SUCCESS on verified placement.

- [ ] **Step 1: Write the failing tests (append)**

```python
# --- PlaceOn: descend->release->retract phase machine (plan phase stubbed) ---
class StubPlannedReach:
    """Stands in for PlannedReach: immediately SUCCESS."""
    def __init__(self, *a, **k): pass
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

# book bottom follows the cup down, then settles on the surface at 1.11
book = {"z": 1.17}
def _fake_actor_z(client, name):
    book["z"] = max(1.11, cup_z["z"] - 0.06)
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
assert bb.affordance is not None and abs(bb.affordance.point[2] - 1.24) < 1e-6, \
    "plan affordance must target surface_z + 0.13"
U.PlannedReach = U_PlannedReach_orig

print("task-4 PlaceOn tests OK")
```

- [ ] **Step 2: Run to verify failure** — Expected: `AttributeError: ... no attribute 'PlaceOn'`

- [ ] **Step 3: Implement in `urlab_skills.py`**

```python
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
        self._reach = PlannedReach(f"{self.name}/reach", bb)
        self._reach.initialise()
        self._quat = None
        self._t0 = None

    def update(self):
        bb = self.bb
        client = bb.client

        if self._phase == "plan":
            status = self._reach.update()
            if status == py_trees.common.Status.RUNNING:
                return py_trees.common.Status.RUNNING
            self._reach.terminate(status)
            if status == py_trees.common.Status.FAILURE:
                return py_trees.common.Status.FAILURE
            # seed-then-enable at the landed pose, then descend
            p0, q0 = synced_site_pose(client, "cup_site")
            stream_target(client, bb.name, p0, q0)
            client._rpc_configure_controller(
                articulation=bb.name,
                params={"task_enabled": [True, True, True, False]})
            self._quat = q0
            self._cup_target = p0.copy()
            self._t0 = time.time()
            self._phase = "descend"
            return py_trees.common.Status.RUNNING

        if self._phase == "descend":
            _hold_suction(client, bb.name)
            z_obj = _actor_z_by_name(client, self.object_name)
            if abs(z_obj - self.surface_z) <= self.settle_tol:
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
            if self._cup_target[2] <= floor and z_obj - self.surface_z > 0.06:
                bb.fail_reason = (f"PlaceOn[{self.name}]: cup at floor limit "
                                  f"but object z {z_obj:.3f} never settled")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if self._phase == "release":
            if self._t0 is None:
                client.runtime.set_suction(articulation=bb.name, value=0.0)
                self._t0 = time.time()
                return py_trees.common.Status.RUNNING
            if time.time() - self._t0 < self.DWELL_S:
                return py_trees.common.Status.RUNNING
            z_obj = _actor_z_by_name(client, self.object_name)
            if abs(z_obj - self.surface_z) > self.settle_tol + 0.02:
                bb.fail_reason = (f"PlaceOn[{self.name}]: object z {z_obj:.3f} "
                                  f"not at surface {self.surface_z:.3f} after release")
                return py_trees.common.Status.FAILURE
            p0, _ = synced_site_pose(client, "cup_site")
            self._cup_target = p0 + self.RETRACT
            self._t0 = time.time()
            self._phase = "retract"
            return py_trees.common.Status.RUNNING

        # retract
        a = min(1.0, (time.time() - self._t0) / 2.0)
        p0, _ = synced_site_pose(client, "cup_site")
        pos = (1.0 - a) * p0 + a * self._cup_target
        stream_target(client, bb.name, pos, self._quat)
        if a < 1.0:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.bb.fail_reason:
            self.bb.fail_reason = f"PlaceOn[{self.name}]: failed"
```

- [ ] **Step 4: Run tests** — Expected all four task prints OK. (Note the stubs monkeypatch module attributes; `PlaceOn` must call `synced_site_pose`/`stream_target`/`_actor_z_by_name`/`PlannedReach` as module-level names for the stubs to land — which the code above does.)

- [ ] **Step 5: Commit**

```bash
git add Scripts/demos/urlab_skills.py Scripts/demos/test_urlab_skills_multi.py
git commit -m "feat(skills): PlaceOn — planned hover, object-settles descend, release, retract"
```

---

### Task 5: authoring script

**Files:**
- Create: `Scripts/demos/tidybot_multi_pick_author.py`

**Interfaces:**
- Consumes: `guarded_payload(link_bodies, cup_bodies, base_bodies)` idiom (per-guard obstacle sets — copy the function from `tidybot_fab_book_pick_demo.py`'s authoring sibling, i.e. the payload builder used by the fab arc), `add_nav_stack(..., max_speed=0.4)`.
- Produces: an authored HomeInterior — table+island static hulls, `SM_Book_125` dynamic, robot `sp_tidybot` at `(-10, -18)`, navmesh (agent_radius 55), controller with cup guard EMPTY.

- [ ] **Step 1: Write the script**

```python
#!/usr/bin/env python3
"""Author HomeInterior for the multi-pick round trip (editor IDLE).
Table + island static hulls; SM_Book_125 dynamic (actor must be Movable —
verified via bounds); suction tidybot + nav stack (max_speed 0.4, the carry
doctrine's slow-everywhere v1) + controller. CUP GUARD IS EMPTY: the cup
must approach BOTH surfaces to pick/place; descends are vertical and
reaches planner-checked, while arm+base stay guarded."""
import sys
sys.path.insert(0, "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Scripts/demos")
from tidybot_mobile_manip_demo import AGENT_RADIUS
from tidybot_suction_pick_demo import (ACTOR_ID, MODEL_XML, CUP_STANDOFF_M,
                                       CUP_DETECTION_M, LINK_DETECTION_M)
from tidybot_guarded_reach_demo import LINK_STANDOFF, ROBOT_LINKS
from tidybot_mink_demo import ARM_JOINTS, BASE_JOINTS
from tidybot_twist_follow_demo import twist_follow_controller_payload
from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError

LEVEL = ["/Game/Home_Interior", "/Game/Home_Interior/Home_Interior"]
BOOK = "SM_Book_125"
SPAWN = (-10.0, -18.0)


def log(m): print(f"[multi-author] {m}", flush=True)


def guarded_payload(link_bodies, cup_bodies, base_bodies):
    p = twist_follow_controller_payload()
    p["tasks"][0]["frame"] = "cup_site"
    p["tasks"][1]["joints"] = BASE_JOINTS + ARM_JOINTS
    p["limits"] += [
        {"kind": "collision_avoidance", "geoms_a": ROBOT_LINKS,
         "geoms_b": link_bodies, "min_distance": LINK_STANDOFF,
         "detection_distance": LINK_DETECTION_M},
        {"kind": "collision_avoidance", "geoms_a": ["base"],
         "geoms_b": base_bodies, "min_distance": 0.04,
         "detection_distance": 0.25},
    ]
    if cup_bodies:
        p["limits"].append(
            {"kind": "collision_avoidance", "geoms_a": ["cup"],
             "geoms_b": cup_bodies, "min_distance": CUP_STANDOFF_M,
             "detection_distance": CUP_DETECTION_M})
    return p


c = URLabClient("tcp://localhost")
c.connect()
if c.sim.status().state.value == "ready":
    print("[multi-author] REFUSING: Simulate RUNNING. Stop it first.", flush=True)
    sys.exit(1)

loaded = None
for cand in LEVEL:
    try:
        c.scene.load_level(cand); loaded = cand; break
    except URLabRPCError as e:
        log(f"load_level {cand!r}: {e}")
if loaded is None:
    print("[multi-author] FAIL: no level loaded.", flush=True); sys.exit(1)
log(f"LOADED {loaded!r}")

rows = c.outliner.find_actors(class_filter="StaticMeshActor")
def resolve(pref): return next((r.name for r in rows if r.name.startswith(pref)), None)

table = resolve("SM_Table_01"); island = resolve("SM_Kitchen_island_table")
book = resolve(BOOK)
if not (table and island and book):
    print(f"[multi-author] FAIL: missing actors t={table} i={island} b={book}",
          flush=True); sys.exit(1)
c.outliner.add_quick_convert(target=table, by_name=True, static=True, complex_mesh=False)
c.outliner.add_quick_convert(target=island, by_name=True, static=True, complex_mesh=False)
b = c.outliner.get_actor_bounds(book, by_name=True)
c.outliner.add_quick_convert(target=book, by_name=True, static=False, complex_mesh=False)
log(f"static: {table}, {island}; DYNAMIC {book} (top z={b.max[2]:.3f} — must be Movable)")

bp = c.scene.import_xml(path=str(MODEL_XML))
c.scene.spawn_actor(blueprint=bp, actor_id=ACTOR_ID, location=(SPAWN[0], SPAWN[1], 0.0))
c.scene.ensure_manager()
nav = c.scene.add_nav_stack(target=ACTOR_ID, debug_draw=True, max_speed=0.4)
log(f"nav stack={nav.get('created')} (max_speed 0.4 — loaded-transit doctrine)")
r = c.scene.spawn_nav_bounds(center=(SPAWN[0], SPAWN[1], 0.5), extent=(15.0, 15.0, 2.0),
                             agent_radius=AGENT_RADIUS, timeout_s=60.0)
log(f"navmesh baked={r.get('nav_data_present')}")
ik = c.ik.add_controller(target=ACTOR_ID, **guarded_payload(
    link_bodies=[f"{table}_MjBody", f"{island}_MjBody"],
    cup_bodies=[],                       # cup approaches BOTH surfaces
    base_bodies=[f"{table}_MjBody", f"{island}_MjBody"]))
w = ik.get("warnings") or []
log(f"!! WARNINGS: {w}" if w else "no warnings — cup unguarded, arm/base guarded")
log("AUTHORED. Press SIMULATE, then run tidybot_multi_pick_demo.py.")
```

- [ ] **Step 2: Compile-check**

Run: `/home/stuart/Unreal_Robotics/URLab_Bridge/.venv/bin/python -m py_compile Scripts/demos/tidybot_multi_pick_author.py`
Expected: silent success.

- [ ] **Step 3: Commit**

```bash
git add Scripts/demos/tidybot_multi_pick_author.py
git commit -m "feat(demo): multi-pick authoring — both stations guarded for arm/base, cup unguarded, nav 0.4 m/s"
```

---

### Task 6: tree driver `tidybot_multi_pick_demo.py`

**Files:**
- Create: `Scripts/demos/tidybot_multi_pick_demo.py`
- Test: `Scripts/demos/test_urlab_skills_multi.py` (append tree-assembly check)

**Interfaces:**
- Consumes: every skill above; existing `ResolveAffordance`, `PlannedReach`, `DescendEngage`, `VerifyAttach`, `PickBlackboard`; `resolve_affordance` conventions.
- Produces: `build_tree(bb, one_way: bool) -> py_trees.composites.Sequence` (importable — the assembly test uses it), `Station` dataclass, `main(one_way)` driver with `finally` teardown.

- [ ] **Step 1: Write the failing assembly test (append to `test_urlab_skills_multi.py`)**

```python
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
```

- [ ] **Step 2: Run to verify failure** — Expected: `ModuleNotFoundError: No module named 'tidybot_multi_pick_demo'`

- [ ] **Step 3: Write the driver**

```python
#!/usr/bin/env python3
"""Multi-pick round trip (LIVE, Simulate running, authored by
tidybot_multi_pick_author.py): pick the Fab book off the small table, DRIVE
while holding it to the kitchen island, place it, re-pick it, drive back,
place it back. Fail-fast py_trees Sequence over urlab_skills; teardown in
finally. --one-way stops after the island place (live gate 1)."""
import argparse, sys, time
from dataclasses import dataclass
sys.path.insert(0, "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Scripts/demos")
import numpy as np
import py_trees
from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError
from urlab_client.results import PIEState
import urlab_skills as U

ACTOR_ID = "sp_tidybot"
BOOK = "SM_Book_125"
BOOK_HALF = 0.011
TICK_S = 0.25
RUN_TIMEOUT_S = 600.0


@dataclass
class Station:
    name: str
    aabb: tuple          # xmin, xmax, ymin, ymax
    top_z: float
    place_xy: tuple


TABLE = Station("table", (-13.20, -12.44, -15.63, -14.87), 0.55, (-12.71, -15.36))
ISLAND = Station("island", (-11.99, -10.17, -20.26, -19.38), 1.11, (-11.10, -19.60))
# Loaded-transit goals: a ring-legal point on each station's open side.
ISLAND_STAGING = (-11.10, -18.65)
TABLE_STAGING = (-12.18, -16.47)


def log(m): print(f"[multi-pick] {m}", flush=True)


def build_tree(bb, one_way: bool):
    # NOTE for the implementer: verify the existing skills' __init__
    # signatures (PlannedReach/DescendEngage/VerifyAttach) against
    # urlab_skills.py before wiring — the assembly test instantiates every
    # child, so a signature mismatch fails offline, not live.
    def pick_leg(station, tag):
        return [
            U.StageAt(f"stage_{tag}", bb, station.aabb, station.place_xy),
            U.ResolveActorTop(f"resolve_{tag}", bb, BOOK, BOOK_HALF),
            U.PlannedReach(f"reach_{tag}", bb),
            U.DescendEngage(f"descend_{tag}", bb),
            U.VerifyAttach(f"verify_{tag}", bb),
        ]
    children = (
        pick_leg(TABLE, "table")
        + [U.CarryTransit("carry_to_island", bb, ISLAND_STAGING, BOOK),
           U.PlaceOn("place_island", bb, ISLAND.place_xy, ISLAND.top_z,
                     BOOK_HALF, BOOK)]
    )
    if not one_way:
        children += (
            pick_leg(ISLAND, "island")[0:1]      # StageAt only; then re-pick
            + pick_leg(ISLAND, "island2")[1:]    # Resolve/Reach/Descend/Verify
            + [U.CarryTransit("carry_to_table", bb, TABLE_STAGING, BOOK),
               U.PlaceOn("place_table", bb, TABLE.place_xy, TABLE.top_z,
                         BOOK_HALF, BOOK)]
        )
    seq = py_trees.composites.Sequence("multi_pick_round_trip", memory=True)
    seq.add_children(children)
    return seq


def main(one_way: bool):
    c = URLabClient("tcp://localhost", step_mode="direct", step_port=5559)
    c.connect()
    if c.sim.status().state != PIEState.READY:
        log("FAIL: no READY Simulate session"); return 1
    c.sim.start(timeout_s=60.0)
    name = next((r.name for r in c.outliner.find_actors(
        class_filter="AMjArticulation", in_pie=True)
        if r.actor_id == ACTOR_ID), None)
    if name is None:
        log(f"FAIL: {ACTOR_ID!r} not found"); return 1
    if c.runtime.set_active_controller(
            articulation=name, controller="mink_ik").get("active") != "MjMinkIKController":
        log("FAIL: mink not active"); return 1
    try:
        c.runtime.set_mode("live")
    except URLabRPCError:
        pass
    c.runtime.set_paused(paused=False)
    time.sleep(1.0)

    bb = U.PickBlackboard()
    bb.client = c
    bb.name = name
    bb.object_actor_id = BOOK

    tree = build_tree(bb, one_way)
    log(f"tree: {[ch.name for ch in tree.children]}")
    tree.setup_with_descendants()
    deadline = time.time() + RUN_TIMEOUT_S
    rc = 1
    try:
        last = None
        while time.time() < deadline:
            tree.tick_once()
            cur = tree.tip().name if tree.tip() else "?"
            if cur != last:
                log(f"leg: {cur}")
                last = cur
            if tree.status == py_trees.common.Status.SUCCESS:
                log("ROUND TRIP COMPLETE" if not one_way else "ONE-WAY COMPLETE")
                rc = 0
                break
            if tree.status == py_trees.common.Status.FAILURE:
                log(f"FAILED at leg {cur}: {bb.fail_reason}")
                break
            time.sleep(TICK_S)
        else:
            log("FAIL: run timeout")
    finally:
        # teardown: suction off, posture cleared, EE task seed-then-enabled,
        # damping restored — always.
        try:
            c.runtime.set_suction(articulation=name, value=0.0)
        except Exception:
            pass
        try:
            p0, q0 = U.synced_site_pose(c, "cup_site")
            U.stream_target(c, name, p0, q0)
            c._rpc_configure_controller(articulation=name, params={
                "posture_target": {},
                "task_enabled": [True, True, True, False],
                "task_costs": {str(bb.damping_task): {"cost": 5.0}},
            })
        except Exception as e:
            log(f"teardown note: {e}")
    zf = U._actor_z_by_name(c, BOOK)
    log(f"final book z = {zf:.3f}")
    return rc


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--one-way", action="store_true",
                    help="stop after placing on the island (live gate 1)")
    sys.exit(main(ap.parse_args().one_way))
```

- [ ] **Step 4: Run the full offline suite** — Expected: all task prints + `ALL multi-pick offline tests OK`. (If the child-count assertion fails, count the actual legs — full tree is 5 pick + 2 transit/place + 5 re-pick/transit/place = 12; one-way is 7.)

- [ ] **Step 5: Commit**

```bash
git add Scripts/demos/tidybot_multi_pick_demo.py Scripts/demos/test_urlab_skills_multi.py
git commit -m "feat(demo): multi-pick round-trip tree driver with one-way gate flag"
```

---

### Task 7: live gates (user-coordinated acceptance — NOT a subagent task)

Procedure (requires the user at the editor; follow the session's Simulate
etiquette — never author over a live Simulate):

- [ ] **Gate 0 (author):** editor idle → `tidybot_multi_pick_author.py` → user presses Simulate.
- [ ] **Gate 1 (one-way):** `tidybot_multi_pick_demo.py --one-way`. PASS: book verified resting on the island top (final `book z ≈ 1.11 + 0.011`), no manual intervention. This gate also answers the two recorded risks: island reach at z≈1.11 and the held-object planner caveat. On goal_ik failure at the island: lower `ISLAND.place_xy` toward the edge, retry; on persistent plan failures with the held book: implement PlaceOn's documented fallback (straight streamed approach replacing the plan phase).
- [ ] **Gate 2 (round trip):** full run. PASS: book back on the small table within 0.15 m of `TABLE.place_xy`, all legs verified, ≥1 `start_projection_m > 0` recovery logged.
- [ ] **Gate 3 (repeatability):** gate 2 again without restarting Simulate. PASS: same outcome.
- [ ] **Commit any live-tuning deltas** with a `demo(multi-pick): live-gate tuning` message, then update `docs/roadmap/navigation-and-mobile-manip.md` (milestone delivered) and memory.
