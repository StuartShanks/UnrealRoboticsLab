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

## UPDATE (later same day) — suction DELIVERY root-caused + fixed; one narrower unknown remains

The pointer diagnostic (commit `9b75c3f`) resolved the delivery question:
- **Same instance, correct path.** `set_suction` sets `NetworkValue=1.0` on the *same* `UMjActuator` the mink reads (same `comp=%p`), `Source=0`, `d->ctrl=1.000` — on a clean sim.
- **The real cause: a direct-mode `client.step()` ZEROS actuator NetworkValues server-side** (the step pushes the client's zero ctrl array). So every synced physics read (`synced_site_pose` / `_object_z`, which dip to direct + step) silently un-set an engaged suction. PASSDIAG proof: `Network=1.0` right after `set_suction`, then `0.0` after one synced read.
- **Fixed demo-side** in commit `18a98e7`: a `_hold_suction()` helper re-asserts `set_suction(1.0)` after every stepping read in the grip/verify/carry phases. Verified live that `Network` now holds at `1.0` across the grip window (180 sustained samples).
- **Proper engine fix (follow-up):** a direct-mode step should preserve actuator `NetworkValue`s it isn't explicitly overwriting (in `ZmqStep`/`ZmqSubscribeTransport`/the step handler). That removes the need for the demo-side re-asserts.

**Narrower remaining unknown — does adhesion actually GRIP in the full pick?** After the re-assert fix, a clean run still failed VerifyAttach (`rose -0.012`, i.e. box moved slightly *down*) with suction confirmed held at 1.0. Adhesion is proven to grip in a *minimal* offline model (`margin=0.03 gap=0.03` + `ctrl=1.0`, `adhesion_offline3.py`), but a clean full-model / full-pick grip has NOT yet been observed. Confounders during testing: the light box keeps getting knocked/falling off the table across repeated failed runs (contaminates the next test), and the offline full-model harness wedges the box in the extended default-pose arm. **Next step:** one clean run — restart Simulate for a fresh table box, descend to solid contact, hold suction (fixed), lift, and read the box's PHYSICS z. If it still won't grip, suspects are (a) a contact-param / default-class difference in the full model vs the minimal offline model (check the cup geom's inherited condim/solref), (b) give the pick box its own `margin`/`gap`, (c) bump the adhesion `gain`. `grip_test2.py` (in the job tmp) is the isolated descend+grip+lift probe.

## (original) The open bug: the suction control value never reaches the actuator server-side
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
