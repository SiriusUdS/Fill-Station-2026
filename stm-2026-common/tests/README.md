# stm-2026-common host unit tests

Native (host-run) unit tests for this submodule's **HAL-free logic modules**.
They build and run on your PC, independently of any firmware build — so this
repo is testable on its own (e.g. in its own CI) without the parent project.

```
tests/
├── CMakeLists.txt          # standalone host project (also add_subdirectory'd by the parent)
├── run.cmake               # entry point: cmake -P tests/run.cmake [clean]
├── fetch-googletest.cmake  # downloads a pinned GoogleTest into third_party/ (git-ignored)
├── cmake/
│   └── host_tests.cmake    # shared harness logic (toolchain select + configure/build/ctest)
├── unit/                   # one unit in isolation; mirrors the source layout under logic/
│   └── logic/
│       ├── control/persistent_state_test.cpp
│       └── communication/protocol/command/command_test.cpp
├── integration/            # several units assembled — reserved, empty for now (see its README)
└── support/
    └── fakes.hpp/.cpp      # FakeBus: test doubles for the udp::/can:: interfaces
```

Tests are labelled `unit` / `integration`, so `ctest -L unit` selects a subset.

## Run

```
cmake -P tests/run.cmake          # configure, build, run
cmake -P tests/run.cmake clean    # wipe tests/build first
```

The first run downloads GoogleTest into `tests/third_party/` (needs network);
every run after that is offline.

* **Linux / macOS** — uses the system C++ compiler (GCC ≥ 13 / Clang ≥ 16).
  This is what CI uses; nothing else to install.
* **Windows** — needs a portable MinGW-w64 (the firmware's `arm-none-eabi-g++`
  can't run on the host). When developing inside the parent repo it's found
  automatically in the parent's `tools/`; standalone, pass
  `-DMINGW_DIR=<path-to-mingw64>`.

## Relationship to the parent repo

The harness here is the single source of truth. The parent repo
(`Fill-Station-2026`) does **not** copy it — its test build `add_subdirectory()`s
this directory to reuse these tests, the shared GoogleTest, and the exported
`STMCOMMON_LOGIC_INCLUDE_DIRS`, then adds its own parent-only targets on top.

## Adding a test

Mirror the source tree: for a unit at `logic/<path>/<unit>.cpp`, add
`tests/unit/logic/<path>/<unit>_test.cpp`, then add an `add_executable(...)` +
`gtest_discover_tests(... PROPERTIES LABELS "unit")` block in `CMakeLists.txt`
using `STMCOMMON_LOGIC_INCLUDE_DIRS`. Includes resolve via `-I`, so the test
file's own location doesn't affect them. Submodule-level integration tests (if
any) go under `tests/integration/` with the `"integration"` label.
