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
