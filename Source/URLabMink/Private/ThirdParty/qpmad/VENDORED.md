# Vendored: qpmad

- **Upstream:** https://github.com/asherikov/qpmad
- **Tag:** `1.4.0`
- **Commit:** `b2fd8d57d973cab5d74decc1bbfc3c622d111240`
- **License:** Apache License, Version 2.0 (see `LICENSE` in this directory; verified header text
  `Apache License / Version 2.0, January 2004` matches upstream `LICENSE` before copying).
- **Date vendored:** 2026-07-02.
- **Source subset:** `include/qpmad/*` from the upstream repo (this is a header-only library; no
  `.cpp` sources exist upstream).

## Local modifications

None of the upstream headers were edited.

One file was **added** (not present upstream) because qpmad's build normally generates it via
CMake's `configure_file()` from `config.h.in`, and this vendoring drop does not run qpmad's
CMake build:

- `config.h` — generated from `config.h.in` with all three `#cmakedefine` options left undefined
  (`QPMAD_ENABLE_TRACING`, `QPMAD_USE_HOUSEHOLDER`, `QPMAD_PEDANTIC_LICENSE`), matching upstream's
  default `OFF` for all three corresponding CMake options. Content:

  ```
  #pragma once

  /* #undef QPMAD_ENABLE_TRACING */
  /* #undef QPMAD_USE_HOUSEHOLDER */
  /* #undef QPMAD_PEDANTIC_LICENSE */
  ```

No other files were added, removed, or modified relative to the upstream `include/qpmad/` tree
at the pinned commit.
