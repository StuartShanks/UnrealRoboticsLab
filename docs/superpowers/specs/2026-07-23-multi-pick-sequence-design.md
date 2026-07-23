# Multi-Pick Sequence — Design

Status: brainstormed + approved 2026-07-23 (scenario: round trip / island
station / fail-fast, approach A). Successor milestone to the delivered
suction-pick v1 (see `docs/roadmap/navigation-and-mobile-manip.md`).

## Goal

One tree run in HomeInterior: the robot picks the Fab book (`SM_Book_125`)
off the small table, **drives while holding it** to the kitchen island,
places it on the island top, re-picks it, drives back, and places it back on
the small table. This converts the single validated pick into repeatable
mobile manipulation and delivers the roadmap's original "nav, grab, nav
while holding".

Composition over new capability: the only genuinely new behavior is the
loaded transit; everything else is promotion/consolidation of logic that is
already live-validated (fab pick demo + the py_trees skills module).

## Context (what exists and is trusted)

- `Scripts/demos/urlab_skills.py`: py_trees skills, live-validated in the
  in-house suction-pick tree — `Drive` (bus nav + damping choreography +
  physics-reset guard), `PlannedReach` (RRT plan + self-paced posture
  streaming), `DescendEngage`, `VerifyAttach`, `StowCarry` (tuck ramp +
  EE-task disable — the transit doctrine), `Release`, plus condition
  helpers (`reach_annulus_ok`, `corridor_clear`) and `PickBlackboard`.
- `Scripts/demos/tidybot_fab_book_pick_demo.py`: the validated Fab pick —
  but its staging/pick logic is INLINE (ring staging, verified nav_to,
  mirror-truthful telemetry), duplicating the skills. This milestone
  consolidates: skills become the single implementation, both demos thin.
- Nav robustness (engine, live-validated): `set_nav_goal` strict mode
  (`max_projection`, reasons, `projection_m`), off-mesh wedge recovery
  (`start_projection_m`), `get_nav_status` honest fields
  (`distance_to_requested_m`).
- Bridge truthfulness: entity mirror live for quick-converted bodies;
  `set_suction` mirrors into the client ctrl buffer (steps no longer
  clobber the grip).

## Scenario constants

- Book: `SM_Book_125` (UI: SM_Book_13), half-thickness 0.011 m, dynamic
  quick-convert, Movable.
- Small table: AABB x [-13.20, -12.44], y [-15.63, -14.87], top z 0.55.
- Kitchen island: AABB x [-11.99, -10.17], y [-20.26, -19.38], top z 1.11.
- Place target on the island: (-11.10, -19.60) — on the top, near the
  table-facing (north) edge, so staging on that side keeps the reach short.
- Return place target on the table: the book's original spot
  (-12.71, -15.36).
- Navmesh erosion: agent_radius 0.55 m ⇒ staging candidates ring each
  station OUTSIDE `AABB inflated by 0.58 m`; radii 0.95/1.10 m from the
  reach target; verified arrival tolerance 0.30 m; `max_projection` 0.30.
- Transit legs ≈ 4.5 m each way between stations.

## Architecture

Behavior tree over skills over controllers (roadmap axis 3). One py_trees
`Sequence` (memory=True), fail-fast: any skill FAILURE aborts the run;
teardown always executes (driver-level `try/finally`, not a tree node —
py_trees has no finally). Skills keep their internal bounded retries
(staging candidate ring; restage-on-plan-dry). No new recovery machinery
(explicitly: no drop-recovery in v1).

Tree:

```
Sequence "multi_pick_round_trip"
├─ StageAt(table, near=book)          # ring + verified nav + max_projection
├─ ResolveAffordance(book)            # live book top via the (now live) mirror
├─ PlannedReach                       # existing skill
├─ DescendEngage                      # existing skill
├─ VerifyAttach                       # existing skill (small lift + z check)
├─ CarryTransit(island staging)       # NEW: StowCarry + suction-held Drive
├─ PlaceOn(island, place_xy)          # NEW: planned reach above point →
│                                     #      descend → release → retract
├─ StageAt(island, near=book)         # re-stage (book now ON the island)
├─ ResolveAffordance(book)            # re-resolve at the island height
├─ PlannedReach → DescendEngage → VerifyAttach
├─ CarryTransit(table staging)
└─ PlaceOn(table, original_xy)
```

Driver: `Scripts/demos/tidybot_multi_pick_demo.py` — connects, activates
mink_ik, `set_mode("live")`, unpauses, builds blackboard + tree, ticks at
~4 Hz, reports per-leg outcomes, teardown (suction off, posture restore,
damping 5.0) in `finally`.

## Components

### 1. Skill upgrades/additions in `urlab_skills.py`

**`StageAt(name, bb, station_aabb, target_xy, erode_m=0.58)`** (NEW;
subsumes the fab demo's `staging_ring` + `nav_to`): generate ring
candidates (16 bearings × radii 0.95/1.10 around `target_xy`, filtered
outside the inflated station AABB, sorted by drive distance from the live
base pose), then for each candidate run the `Drive` logic with
`max_projection=0.30`; on arrival verify the base landed within 0.30 m of
the REQUESTED candidate (defense-in-depth; the strict mode should reject
substitutions up front). SUCCESS on first verified candidate; FAILURE when
the ring is exhausted. Logs rejections with `reason`/`projection_m` and
recovery legs with `start_projection_m`.

**`Drive` upgrade**: `set_nav_goal` call gains optional `max_projection`
pass-through and logs `start_projection_m` when present. Existing users
unaffected (parameter defaults to None → omitted from the payload).

**`CarryTransit(name, bb, goal_xy)`** (NEW, composition): internally runs
StowCarry's tuck ramp (EE to current + [-0.15, 0, +0.25], then EE task off,
posture holds) and then the Drive logic with two additions: re-assert
`set_suction(1.0)` every poll (~1 Hz — belt-and-braces; the ctrl mirror
already protects against clobber), and FAILURE if the held object's z (live
mirror read) drops below `carry_z_min = 0.45` m (a carried book rides at
≥0.9 m after the tuck; a mid-transit drop lands on the floor ≤0.1 m —
0.45 splits them with margin; attach lost ⇒ fail-fast, no recovery). Implementation may compose the existing behaviours rather
than duplicate their bodies (a small internal sub-sequence is acceptable),
but it presents as ONE skill to the tree.

**`PlaceOn(name, bb, place_xy, surface_z, half_thickness)`** (NEW, the
pick's inverse):
1. Pre-place: `PlannedReach`-style planned reach to
   `(place_xy, surface_z + half*2 + PRE_PLACE_M)` with the cup-down quat
   and the HELD-BOOK clearance in mind (target height accounts for the book
   riding ~2–4 cm below the cup on the adhesion margin).
2. Slow descend (frame task, landed-orientation hold, bounded-error
   streaming) until the BOOK's live z settles at
   `surface_z + half ± tolerance` (book contact, not cup contact — the
   book is the thing being placed).
3. Release (`set_suction(0)`), dwell 1 s, verify book z stable at surface.
4. Retract: cup up 0.15 m and 0.15 m back toward the base, restore task
   costs. SUCCESS on verified placement; FAILURE with reason otherwise.

**`ResolveAffordance` note**: the book's top is re-resolved per pick from
the live mirror (post entity-registration fix the mirror is authoritative;
`find_actors` remains the documented fallback). Island-height pick reuses
the same path — nothing station-specific in the skill.

### 2. Station model

A small `Station` dataclass in the demo driver (NOT in urlab_skills —
YAGNI until a third consumer): `name, aabb, top_z, place_xy`. Two
instances (table, island) own every scenario constant listed above. Skills
take plain arguments; stations are driver-side configuration.

### 3. Authoring: `tidybot_multi_pick_author.py`

Fork of the validated fab author with one guard change and no other drift:
- table + island: static quick-convert (hulls).
- book: dynamic quick-convert (actor must be Movable — precondition
  documented in the header, verified by the author via a bounds check).
- controller payload: link guard = [table, island]; base guard =
  [table, island]; **cup guard = [] (empty)** — the cup must approach BOTH
  surfaces to pick/place on them. Rationale: descends are vertical, reaches
  are planner-collision-checked, and the arm/base guards still protect the
  chain; this is the same exclusion argument that let the cup reach the
  table-level book, extended to both stations.
- nav stack with agent_radius 55 cm, `MaxSpeed` authored to 0.4 m/s for
  the WHOLE demo (loaded and unloaded legs — a per-leg runtime speed op is
  out of scope; slow everywhere is acceptable for v1 and matches the carry
  doctrine).

### 4. Failure handling

- Every skill FAILURE sets `bb.fail_reason`; the sequence halts; the driver
  reports the failed leg and runs teardown (suction off, `posture_target`
  cleared, task_costs restored, EE task re-enabled at current pose seed —
  the seed-then-enable rule).
- Internal retries only where they already exist: StageAt's candidate
  ring. StageAt picks ONE verified staging; if PlannedReach then fails its
  plan, the sequence FAILS (v1). The fab demo's restage-on-plan-dry loop is
  intentionally dropped from v1 to keep skills single-purpose; it returns
  as a tree-level retry decorator later if live runs show it's needed (the
  last three live runs never needed it).
- Physics-auto-reset guard: `Drive`'s existing distance-jump abort covers
  transits.

### 5. Telemetry / honesty

All object reads via the live mirror (entity registration fixed); the
driver prints per-leg: staging result (+projection fields), plan stats,
cup/book z through descend+lift, transit start/arrival with
`start_projection_m`, placement verification (book z vs surface), and a
final summary table. Any "arrived"/"placed" claim must be backed by the
honest metric (`distance_to_requested_m`, book-z-at-surface).

## Live-gate plan (testing)

Skills are live-domain; offline coverage is import/assembly only:
1. **Offline**: `py_compile` + a tree-assembly check (build the tree with a
   stub blackboard; assert structure and that no skill raises in
   `__init__`). No physics mocks — they'd test the mock.
2. **Live gate 1 (one-way)**: author → Simulate → run a truncated tree
   (through `PlaceOn(island)`). Success: book verified resting on the
   island top, no manual intervention.
3. **Live gate 2 (round trip)**: full tree. Success: book back on the small
   table within 0.15 m of its original spot, all legs verified, at least
   one wedge-recovery (`start_projection_m > 0`) logged en route.
4. Repeatability: gate 2 twice in a row without editor restarts.

## Out of scope (v1)

Drop recovery (detect + re-pick a dropped book mid-transit); per-leg nav
speed runtime op; restage-on-plan-dry tree decorator; dynamic navmesh /
MPPI (axis 1/2 items, triggers unmet); UE StateTree port; any engine/C++
change (this milestone is expected to be pure Python).

## Risks

- **Island reach at z≈1.11 + held-book clearance**: the plan's goal_ik must
  find configs with the cup at ~1.15–1.20 m near a large hull. Believed
  well inside the envelope (cup reads z=1.63 at stow); the planner will
  answer definitively at live gate 1. Fallback: place nearer the island
  edge or lower `PRE_PLACE_M`.
- **Carry stability while driving**: adhesion held 42 N in static tests and
  through slow arm carries; a 0.4 m/s base transit with the tucked pose is
  the mildest possible loaded motion. Mid-carry loss fails fast by design.
- **Two-station staging near larger furniture**: the island's south side
  neighbors other Fab furniture (un-modeled in MuJoCo but present in the
  navmesh) — the ring naturally routes around via rejected candidates.
