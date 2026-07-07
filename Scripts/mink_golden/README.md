# mink golden fixtures

Golden-fixture harness for the URLabMink module. `gen_golden.py` runs the
pinned upstream Python [mink](https://github.com/kevinzakka/mink) library and
dumps JSON fixtures (`fixtures/<layer>.json`) that the URLabMink C++
automation tests (`URLab.Mink.*`) replay to verify numerical parity with the
Python implementation. Every fixture case stores all of its inputs explicitly;
randomness only ever runs inside this generator, seeded.

## Setup

```bash
cd "Scripts/mink_golden"
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Regenerating fixtures

```bash
source .venv/bin/activate
python gen_golden.py --layers lie   # or omit --layers to regenerate all
```

## Pin policy

Fixtures are generated against **mink v1.2.0** and **mujoco 3.9.0** (see
`requirements.txt`; exact resolved versions are frozen in
`requirements.lock`). The C++ port targets exactly these versions —
regenerating fixtures with a different mink/mujoco version invalidates
parity and must be accompanied by a corresponding port update.

## What is committed

- `fixtures/` (generated JSON) and `requirements.lock` **are committed** so
  the C++ tests are reproducible without a Python environment.
- `.venv/` is **not committed** (ignored via the repo `.gitignore`).

## TidyBot trace

`models/stanford_tidybot/` is vendored verbatim from mink's
[`examples/stanford_tidybot`](https://github.com/kevinzakka/mink/tree/v1.2.0/examples/stanford_tidybot)
at the **v1.2.0 tag** (Apache-2.0; `LICENSE.mink` is the upstream mink
license copied alongside). The MJCF/assets are themselves derived from
Kinova/Robotiq/Stanford `stanford_tidybot` sources bundled in the upstream
mink repo — see `models/stanford_tidybot/README.md` for the original
provenance notes. `models/stanford_tidybot/mobile_tidybot.py.reference` is
mink's `examples/mobile_tidybot.py`, kept as the fidelity reference for
`gen_tidybot_trace.py`.

`gen_tidybot_trace.py` is a headless, deterministic port of
`mobile_tidybot.py`'s `fix_base=False` loop (no viewer, no keyboard, no
`RateLimiter`): a pure function of the step index drives the end-effector
mocap target through a hold → reach → transition → circle script, and every
step's IK solve inputs/outputs plus the resulting physics state are recorded
to `fixtures/tidybot_trace.json`. This is "Rung A" ground truth: a later
URLabMink C++ test replays the trace to verify numerical parity of the
mink → URLabMink port end-to-end (task costs, QP solve, integration, and
`mj_step`).

Regenerate with:

```bash
./.venv/bin/python gen_tidybot_trace.py
```

`tidybot_trace.json` (2500 steps, ~4.3 MB) is committed like the other
fixtures. Solver: `daqp` (matches upstream `mobile_tidybot.py`; `daqp` is
present in this venv, so no fallback to `quadprog` was needed — see
`gen_golden.py`'s `solver="quadprog"` calls for the fallback precedent if a
future regen environment lacks `daqp`).
