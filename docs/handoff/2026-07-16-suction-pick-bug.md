# Suction Pick v1 — Live-Validation Handoff (2026-07-16)

**Status:** the full pipeline works end-to-end **except the final suction grip**. One open engine bug, instrumented but not yet root-caused.

**Spec/plan:** `docs/superpowers/specs/2026-07-16-suction-pick-v1-design.md`, `docs/superpowers/plans/2026-07-16-suction-pick-v1.md`.

**Commits (branch `feat/nav-stack`, on `fork`):**
- `0374543` fix(suction-model): cup orientation + adhesion margin/gap + heavier box
- `7a161d5` fix(suction-skills): physics-z verify + snapshot descent + front-edge pick
- `9b75c3f` **TEMP DIAG (STRIP BEFORE MERGE)**: pointer/value logs for this bug

## What works (live-verified)
- Drive to an aligned, front-edge staging pose; reach top-down (cup points **down** — the `cup_site` quat orientation fix is confirmed); descend to **contact** (cup geom touching box geom, x/y aligned); `VerifyAttach` now reads the box's real **physics** z.
- The adhesion **config** is correct: a pure-MuJoCo test grips the box with `margin=0.03 gap=0.03` on the cup geom + direct `ctrl=1.0`. Without margin/gap, adhesion produces zero force (the fix in `0374543`).

## The open bug: the suction control value never reaches the actuator server-side
Server log while simulating (from the `9b75c3f` diagnostics):
```
[MinkIK PASSDIAG] adhesion 'tidybot_suction_ue_C_0_suction' ActId=10 Source=0 -> d->ctrl=0.000 (Network=0.000)
```
`Source=0` is correct (the `NetworkValue` path). But `NetworkValue` is **0** even immediately after a successful `set_suction(1.0)` RPC (reply ok, correct actuator name). So `set_suction`'s `SetNetworkControl(1.0)` is **not landing on the `UMjActuator` instance the mink pass-through reads.**

Two hypotheses:
- **(A) Two instances, same name** — a PIE/Simulate world-copy or a manager-vs-physics-engine tracking mismatch: `HandleSetSuction` (`RpcDispatcher.cpp`, `Mgr->GetArticulation(name)->GetActuators()`) sets one instance; the mink's `AllActuatorIdMap` (captured at `Bind`) reads another.
- **(B) Reset every step** — the value is set correctly but overwritten to 0 each step, e.g. by the client's own ZMQ control stream (`ZmqSubscribeTransport.cpp:435` `SetNetworkControl`) zeroing unlisted actuators.

## Next step (instrumented, not yet read)
`9b75c3f` adds `comp=%p` pointer logs to **both** `HandleSetSuction` (`[set_suction DIAG]`) and the mink pass-through (`[MinkIK PASSDIAG]`). To read it:
1. Compile (editor-safe): the Build.sh command in `.superpowers/sdd/…` / the memory `urlab-build-test-workflow`.
2. Restart the editor to load the binary; author the scene; **Simulate**; run the pick.
3. `grep -a "PASSDIAG\|set_suction DIAG" "…/Test/Saved/Logs/Test.log"` and compare the two `comp=%p` pointers.
   - **Same pointer** → hypothesis **B** (overwrite). Fix: stop the client zeroing the suction actuator, or give `set_suction` a persistent set-ctrl path the per-step stream doesn't clobber.
   - **Different pointer** → hypothesis **A** (instance mismatch). Fix: make `set_suction` resolve the *same* actuator the bound controller drives (e.g. via the physics-engine articulation / the controller's actuator set) rather than `Mgr->GetArticulation`.

## Reproduction
The committed demo `Scripts/demos/tidybot_suction_pick_demo.py` authors its own scene and runs the tree. For the collaborative Simulate flow (user presses **Simulate**, not Play — RDP constraint), split it into an author probe (create level + spawn + `add_controller`, `import_xml(..., force_reimport=True)` so a changed model actually re-imports) and a drive probe (attach via `sim.start`, `set_active_controller("mink_ik")`, tick `build_tree(bb)`). Useful facts:
- The client's `d.ctrl` / `d.qfrc_actuator` mirror is **not** synced from the server (always 0) — use the server PASSDIAG log, not the client mirror, to read control values.
- `find_actors(in_pie=True)` returns the actor **root** (frozen at spawn) for the free-body pick object — read the MuJoCo body/site via the mirror for physics truth (already fixed in `_object_z`).

## Before merge
Strip the `9b75c3f` TEMP DIAG commit; re-run `URLab.MinkIK` / `URLab.Nav` / `URLab.Suction` suites green.

## Deferred to v1.1
Deep-table pick via controlled base-assist (the `SetBaseAssist` skill is in `urlab_skills.py`, unused by the demo — free base-assist drifts the base off-center in y; it needs *alignment* control, not just closeness).
