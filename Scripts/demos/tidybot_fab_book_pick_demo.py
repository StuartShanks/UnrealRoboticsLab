#!/usr/bin/env python3
"""Fab-object pick demo: the FULL scripted pipeline against an EXISTING scene
object (HomeInterior's SM_Book_125, quick-converted to a dynamic body) —
verified nav staging -> whole-body RRT plan -> execute -> descend -> suction ->
lift -> slow carry -> drop. First live-validated (user-witnessed) 2026-07-22.

Scene prep (editor idle): author with the HomeInterior book scripts — convert
table+island static-hull, the book dynamic (actor must be Mobility=Movable),
spawn the suction tidybot + nav stack + cup-guard-excludes-table controller.

Hard-won gotchas baked in here:
- STAGING VERIFICATION: SetNavGoal silently projects goals up to ~1m onto the
  navmesh and then honestly reports 'arrived' at the SUBSTITUTE — always
  verify the base landed near the REQUESTED point (nav_to).
- BOOK POSE: read via find_actors(in_pie) (engine truth — the actor follows
  MuJoCo). NOT the mirror: the entity-registration flake can leave the
  handshake entities block empty, freezing all mirror reads of quick-converted
  bodies at handshake values (phantom 'NOT LIFTED' telemetry).
- SUCTION: requires the set_suction ctrl_array mirror fix in urlab_client —
  without it every mirror-sync step clobbers suction back to 0 and the object
  drops off the cup mid-pick.
- CARRY: keep it slow (smoothstep) — fast whole-body carries swing the held
  object off the cup."""
import sys, time
sys.path.insert(0, "/home/stuart/Documents/Unreal Projects/Test/Plugins/UnrealRoboticsLab/Scripts/demos")
import numpy as np, mujoco
from urlab_client import URLabClient
from urlab_client.errors import URLabRPCError
from urlab_client.results import PIEState
from urlab_skills import cup_down_quat, synced_site_pose
from urlab_planner import PLANNED_JOINTS, PlanError, plan_reach
from tidybot_mink_demo import stream_target, resolve_id_by_suffix
from tidybot_twist_follow_demo import POSTURE_TASK, DAMPING_TASK

ACTOR_ID = "sp_tidybot"
BOT = ("SM_Book_125", 0.011)    # (actor name, half-thickness: pivot->center / center->top)
# Table footprint (live get_actor_bounds) + navmesh erosion: AGENT_RADIUS=0.55m
# means no navigable point exists closer than 0.55m to the hull, so staging
# candidates ring the table JUST outside that band; the whole-body planner
# (navmesh-blind) closes the final gap to the book.
TABLE_AABB = (-13.20, -12.44, -15.63, -14.87)   # xmin xmax ymin ymax
NAV_ERODE_M = 0.58                               # agent_radius 0.55 + margin
STAGE_VERIFY_M = 0.30   # nav 'arrived' must land within this of the REQUESTED
                        # goal — catches SetNavGoal's silent ±1m projection
PRE_GRASP_M = 0.03
TRACK_TOL = 0.15; SETTLE_TOL = 0.05; NO_PROGRESS_S = 6.0; PROGRESS_EPS = 0.02


def log(m): print(f"[stacked] {m}", flush=True)
def fail(m): print(f"[stacked] FAIL: {m}", flush=True); sys.exit(1)


c = URLabClient("tcp://localhost", step_mode="direct", step_port=5559)
c.connect()
if c.sim.status().state != PIEState.READY:
    fail("no READY Simulate session")
c.sim.start(timeout_s=60.0)
name = next((r.name for r in c.outliner.find_actors(class_filter="AMjArticulation", in_pie=True)
             if r.actor_id == ACTOR_ID), None)
if name is None:
    fail(f"{ACTOR_ID!r} not found")
if c.runtime.set_active_controller(articulation=name, controller="mink_ik").get("active") != "MjMinkIKController":
    fail("mink not active")
try:
    c.runtime.set_mode("live")
except URLabRPCError:
    pass
c.runtime.set_paused(paused=False)
time.sleep(1.0)


def configure(p): c._rpc_configure_controller(articulation=name, params=p)
def hold():
    try: c.runtime.set_suction(articulation=name, value=1.0)
    except Exception: pass
def release():
    try: c.runtime.set_suction(articulation=name, value=0.0)
    except Exception: pass


def book_loc(name):
    """Book body CENTER, engine-truth: live PIE actor pivot (= mesh bottom for
    these Fab books) + half-thickness. The actor follows the MuJoCo body via
    ApplyRenderState every frame, so this tracks the physics even when the
    entity-registration flake leaves the mirror frozen at handshake values."""
    for r in c.outliner.find_actors(class_filter="StaticMeshActor", in_pie=True):
        if r.name == name:
            p = np.array(r.location, dtype=float)
            p[2] += BOT[1]   # pivot (bottom) -> center
            return p
    return None
if book_loc(BOT[0]) is None:
    fail("book actor not found in PIE")
log(f"book {BOT[0]} @ {np.round(book_loc(BOT[0]),3)}")
def synced_q():
    c.step(n_steps=1); md, dd = c.model, c.data
    q = np.empty(len(PLANNED_JOINTS))
    for i, jn in enumerate(PLANNED_JOINTS):
        jid = resolve_id_by_suffix(md, mujoco.mjtObj.mjOBJ_JOINT, md.njnt, jn)
        q[i] = dd.qpos[md.jnt_qposadr[jid]]
    return q


_BASE_BID = None
def base_pos():
    """Live base world xy from the mirror base_link body (per-articulation sync
    is trustworthy post-qpos-fix; find_actors is frozen for articulations)."""
    global _BASE_BID
    c.step(n_steps=1); md, dd = c.model, c.data
    if _BASE_BID is None:
        for b in range(md.nbody):
            nm = mujoco.mj_id2name(md, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if nm.endswith("_base_link") and "gen3" not in nm:
                _BASE_BID = b; break
        if _BASE_BID is None: fail("base_link body not found")
    return np.array(dd.xpos[_BASE_BID][:2], dtype=float)


def staging_ring(target_xy):
    """Candidate staging points around the table: 16 bearings x 2 radii from
    the target, keeping only points outside the table AABB inflated by the
    navmesh erosion (so the REQUESTED point is itself navigable, not silently
    projected). Sorted by drive distance from the current base position."""
    xmin, xmax, ymin, ymax = TABLE_AABB
    here = base_pos(); out = []
    for r in (0.95, 1.10):
        for k in range(16):
            th = 2.0 * np.pi * k / 16
            p = np.array([target_xy[0] + r * np.cos(th), target_xy[1] + r * np.sin(th)])
            if (xmin - NAV_ERODE_M <= p[0] <= xmax + NAV_ERODE_M) and \
               (ymin - NAV_ERODE_M <= p[1] <= ymax + NAV_ERODE_M):
                continue  # inside the eroded band -> would be projected
            out.append(p)
    out.sort(key=lambda p: float(np.linalg.norm(p - here)))
    return out


def nav_to(requested):
    """Drive to `requested`; True only if nav arrives AND the base actually
    lands within STAGE_VERIFY_M of the REQUESTED point. `max_projection` makes
    the engine reject substituted goals up front (goal-substitution RPC) — the
    post-arrival verification stays as defense-in-depth."""
    configure({"task_enabled": [False, True, True, True], "task_costs": {str(DAMPING_TASK): {"cost": 0.05}}})
    try:
        r = c.runtime.set_nav_goal(articulation=name, x=float(requested[0]), y=float(requested[1]),
                                   max_projection=STAGE_VERIFY_M)
        if not r.get("accepted"):
            log(f"  staging {np.round(requested,2)} rejected"
                + (f" ({r['reason']}, proj {r.get('projection_m', 0):.2f}m)" if r.get("reason") else ""))
            return False
        dl = time.time() + 60.0; s = {"state": "?"}
        while time.time() < dl:
            s = c.runtime.get_nav_status(articulation=name)
            if s["state"] in ("arrived", "failed"): break
            time.sleep(1.0)
        if s["state"] != "arrived":
            log(f"  staging {np.round(requested,2)} nav {s['state']}"); return False
        actual = base_pos(); miss = float(np.linalg.norm(actual - np.asarray(requested)))
        if miss > STAGE_VERIFY_M:
            log(f"  staging {np.round(requested,2)} SUBSTITUTED: base at {np.round(actual,2)} "
                f"({miss:.2f}m off requested) — nav projected the goal; next candidate")
            return False
        log(f"  staged at {np.round(actual,2)} ({miss:.2f}m from requested)")
        return True
    finally:
        configure({"task_costs": {str(DAMPING_TASK): {"cost": 5.0}}})


def reach_and_grab(book_name, half, label):
    """Plan whole-body to the book top, execute, descend, engage suction. Leaves
    the book gripped at the pre-grasp height; caller lifts/drops. (Param is
    book_name so it doesn't shadow the module-global robot `name`.)"""
    ctr = book_loc(book_name)
    surf = np.array([ctr[0], ctr[1], ctr[2] + half])
    normal = np.array([0.0, 0.0, 1.0]); q_cup = cup_down_quat(normal)
    pre = surf + PRE_GRASP_M * normal
    log(f"[{label}] book top {np.round(surf,3)}; planning")
    plan = plan_reach(c, pre, q_cup, site="cup_site")   # PlanError -> caller restages
    log(f"[{label}] PLAN {plan.stats}")
    configure({"task_enabled": [False, True, True, False],
               "task_costs": {str(POSTURE_TASK): {"cost": 5.0}, str(DAMPING_TASK): {"cost": 0.05}}})
    wps = plan.waypoints; idx = 0; best = float("inf"); last = time.time()
    while True:
        _t, wp = wps[idx]; configure({"posture_target": wp})
        q = synced_q(); err = max(abs(q[i] - wp[jn]) for i, jn in enumerate(PLANNED_JOINTS))
        if err < best - PROGRESS_EPS: best = err; last = time.time()
        if err <= TRACK_TOL:
            if idx == len(wps) - 1:
                if err < SETTLE_TOL: break
            else:
                idx += 1; best = float("inf"); last = time.time()
        if time.time() - last > NO_PROGRESS_S:
            configure({"posture_target": {}, "task_costs": {str(POSTURE_TASK): {"cost": 1e-3}, str(DAMPING_TASK): {"cost": 5.0}}})
            fail(f"[{label}] stalled at wp {idx} err {err:.3f}")
        time.sleep(0.25)
    log(f"[{label}] reached pre-grasp")
    # descend + suction, holding the landed orientation
    configure({"posture_target": {},
               "task_costs": {str(POSTURE_TASK): {"cost": 1e-3}, str(DAMPING_TASK): {"cost": 5.0}},
               "task_enabled": [True, True, True, False]})
    p0, q_desc = synced_site_pose(c, "cup_site")
    stream_target(c, name, p0, q_desc); stream_target(c, name, surf, q_desc)
    engaged = False; et = None; t0 = time.time()
    while time.time() - t0 < 8.0:
        time.sleep(0.3)
        cup, _ = synced_site_pose(c, "cup_site"); stream_target(c, name, surf, q_desc)
        d = float(np.linalg.norm(cup - surf))
        if not engaged and d <= 0.05:
            c.runtime.set_suction(articulation=name, value=1.0); engaged = True; et = time.time()
            log(f"[{label}] suction ON at d={d:.3f}")
        if engaged: hold()
        if engaged and time.time() - et > 1.5: break
    if not engaged:
        c.runtime.set_suction(articulation=name, value=1.0); log(f"[{label}] force-engaged (cup d never <=0.05)")
    hold(); time.sleep(0.8); hold()
    return surf, q_desc


def lift_to(z_gain, q_desc, book_name=None):
    """Lift with observability: log cup + book z so 'book did not rise' can be
    attributed (cup rose alone -> adhesion failure; cup never rose -> QP/lift
    failure). hold() every iteration — direct-mode steps zero actuator ctrl
    server-side, so suction must be continuously re-asserted."""
    hold(); cup0, _ = synced_site_pose(c, "cup_site"); hold()
    tgt = cup0 + np.array([0.0, 0.0, z_gain]); t0 = time.time(); last_log = 0.0
    while time.time() - t0 < 4.0:
        a = min(1.0, (time.time() - t0) / 2.5)
        stream_target(c, name, (1 - a) * cup0 + a * tgt, q_desc)
        hold()
        if time.time() - last_log > 0.8:
            cz, _ = synced_site_pose(c, "cup_site"); hold()
            bz = book_loc(book_name)[2] if book_name else float("nan")
            log(f"  lift: cup z={cz[2]:.3f} book z={bz:.3f}")
            last_log = time.time()
        time.sleep(0.05)
    cz, _ = synced_site_pose(c, "cup_site"); hold()
    log(f"  lift done: cup z={cz[2]:.3f} (start {cup0[2]:.3f}, target {tgt[2]:.3f})")
    return tgt


def staged_grab(book_name, half, label, max_tries=4):
    """Stage (verified) then plan+grab; on goal_ik/plan failure, advance to the
    next ring candidate and retry."""
    ctr = book_loc(book_name)
    cands = staging_ring(ctr[:2]); tried = 0
    for cand in cands:
        if tried >= max_tries: break
        if not nav_to(cand): continue
        tried += 1
        try:
            return reach_and_grab(book_name, half, label)
        except PlanError as e:
            log(f"[{label}] plan {e.stage} failed from {np.round(cand,2)}: {e} — restaging")
    fail(f"[{label}] no staging candidate yielded a plan ({tried} plan attempts)")


# ============================== run =========================================
# SINGLE-BOOK pick (user deleted SM_Book_127 + the candles; SM_Book_125 remains).
# Full scripted pipeline: nav-stage (verified) -> plan -> execute -> descend ->
# suction -> lift -> carry (slow, holding) -> drop. Suction now survives the
# interleaved mirror-sync steps (set_suction mirrors into ctrl_array).
z0 = book_loc(BOT[0])[2]
surf_b, qd_b = staged_grab(BOT[0], BOT[1], "BOOK")
lift_to(0.14, qd_b, book_name=BOT[0])
z1 = book_loc(BOT[0])[2]
log(f"book z {z0:.3f} -> {z1:.3f}  rose {z1-z0:+.3f}  "
    f"{'LIFTED' if z1-z0 > 0.04 else 'NOT LIFTED'}")

# carry aside SLOWLY (mid-carry detach seen at higher accel) and drop
hold(); cup, _ = synced_site_pose(c, "cup_site"); hold()
drop_xy = cup + np.array([0.0, 0.35, 0.0])   # 35 cm toward open space, same height
t0 = time.time()
while time.time() - t0 < 6.0:
    a = min(1.0, (time.time() - t0) / 5.0)
    a = a * a * (3 - 2 * a)
    stream_target(c, name, (1 - a) * cup + a * drop_xy, qd_b); hold(); time.sleep(0.1)
zc = book_loc(BOT[0])[2]
log(f"carried: book z={zc:.3f}  {'still held' if zc - z0 > 0.04 else 'DETACHED mid-carry'}")
log("releasing (suction OFF)")
release(); time.sleep(1.5)
zf = book_loc(BOT[0])[2]
log(f"after drop z = {zf:.3f}")
log("DONE (picked+carried+dropped)" if z1 - z0 > 0.04 else "DONE (not lifted)")
