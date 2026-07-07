# mink IK on TidyBot in Unreal — end-of-day report

**Infrastructure done; one isolated physics-integration bug left, and it smells
like something you'll recognize.**

## Working & committed (branch `feat/urlabmink-live-ik`, suite 23/23)

- **`UMjMinkIKController`** — the data-driven controller as agreed (spec arrays,
  `UMjComponent` pickers, sliders, writes `d->ctrl`).
- **New `add_controller` editor op** — attach + fully configure over RPC (JSON →
  component refs by MjName). One call, zero warnings on TidyBot.
  *(Follow-up we want: `to_blueprint=true` so the component bakes into the BP —
  instance components die on every actor re-drag.)*
- **Target streaming** through `configure_controller` (`target_pos` / `target_quat`),
  with the manual target taking precedence over the mocap body.
- **`VelocityLimit`** as a spec kind (hard QP cap), **bad-solve discard/counter**,
  and **integration gated on sim-time advance** (the engine calls `ComputeAndApply`
  on idle physics-loop iterations too — the open-loop reference must not integrate
  on those).

## Bugs / gotchas found on the way (each cost real hours — worth tickets)

1. **Import factory fatal-crashes on multi-`<worldbody>` MJCF** — any include-style
   scene wrapper (i.e. Kevin's `scene.xml` pattern): `MjWorldBody` rename collision
   → `Obj.cpp:349` via `MujocoImportFactory.cpp:156`. Workaround: merge into one
   worldbody.
2. **Client `ControllerKind` enum is closed** — an unknown kind (`mink_ik`) makes
   the articulation default `control_mode=raw`, so the server silently *bypasses*
   the UE controller. Also appears to corrupt the client's qpos mirror (we see
   duplicated values across joints for TidyBot — mirror unusable). Suggest a
   permissive fallback.
3. **Mocap bodies are UE-authoritative** — `UMjBody::TickComponent` re-stamps
   `mocap_pos` from the UE transform every tick, so `set_mocap_pose` writes are
   overwritten. Fine for gizmo-driving; surprising over RPC.
4. **Model sim options aren't applied on import/spawn** — TidyBot mandates
   `integrator="implicitfast"` (kp=1e6 base actuators!) but the sim ran Euler until
   we pushed `set_sim_options_from_model()`. (Also `cone=elliptic` / `impratio`
   needed an explicit push.)
5. **Step-request drain rate ≈ one queued request per physics-loop tick** — 50 Hz
   micro-batches silently dropped ~95% of steps for us (replies still return). Few
   big `step(n_steps=N)` batches work perfectly.

## The open bug — precisely isolated

With the controller's **frame task enabled**, the sim NaNs (`QACC at DOF 3/5`)
within ~0.1 s and auto-resets, then loops forever. Discriminators, all verified
live:

| Configuration | Result |
|---|---|
| Controller inert (all tasks off) | **stable 8 s+** |
| Posture task only (ctrl ≈ hold spawn pose) | **stable 6 s+** |
| Frame task on (target = current EE, vel-capped **1 rad/s**, implicitfast, elliptic cone) | **NaN in ~0.1 s** |

At a 1 rad/s cap, nothing physical can move enough in 0.1 s to legitimately
diverge — so either our frame-task `ctrl` write hits something pathological
(units? an actuator index we shouldn't touch? interaction with `ApplyStepCtrl`
staging?) or there's a known landmine here. Ring any bells? Next step otherwise
is dense per-solve logging of the first 50 solves (v, q_ref, ctrl deltas).
