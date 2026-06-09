# Logic unit tests (host / GoogleTest)

Native (host-run) unit tests for the **HAL-free logic modules** — the code under
`CM7/app/logic/` that depends only on the `udp::` / `can::` interfaces and never
touches the HAL. These build and run on your PC, separately from the firmware
cross-compile build.

Test files mirror the source tree (a test for `logic/<path>/<unit>.cpp` lives at
`tests/logic/<path>/<unit>_test.cpp`), so a test is easy to locate from the code
it covers.

**The framework is not kept here.** The test harness (GoogleTest fetch,
toolchain selection, the `FakeBus`, and the tests for code that lives in
`stm-2026-common`) lives once in the submodule — see
`stm-2026-common/tests/` — so that submodule is testable on its own (e.g. in its
own CI). This parent build *reuses* it via `add_subdirectory` and adds only the
parent-only targets. `cmake -P tests/run.cmake` runs the **full** suite (parent
+ common).

```
tests/
├── CMakeLists.txt   # parent-only: add_subdirectory(stm-2026-common/tests) + fcu_controller_test
├── run.cmake        # 4-line entry point → reuses the submodule's shared harness
└── logic/
    └── fcu_controller_test.cpp   # logic::fcu (the FCU state machine)

stm-2026-common/tests/            # the shared harness + common-owned tests live here
├── cmake/host_tests.cmake        # toolchain select + configure/build/ctest (single source)
├── fetch-googletest.cmake        # the only copy; one GoogleTest download, one version pin
├── support/fakes.hpp/.cpp        # FakeBus (doubles this submodule's udp::/can:: interfaces)
└── logic/.../*_test.cpp          # persistent_state, command, ...
```

## How it works

The logic exposes only `logic::fcu::init()` / `tick()`, and its state lives in an
anonymous namespace with no getter. That's fine: the state machine is fully
**black-box testable through its interfaces**.

* The `FakeBus` (in `support/`) provides the definitions of the `udp::` and
  `can::` interface functions that the firmware platform normally supplies. Tests
  push inbound CAN frames / UDP datagrams into it, run `tick()`, then assert on
  what the logic sent back.
* The current state is read back from the **heartbeat** the logic emits every
  tick — it carries the state in `UDPPacketHeader.frame.deviceState`.

## Prerequisites: host toolchain

The tests need a C++23-capable **host** compiler. The repo's firmware toolchain
(`arm-none-eabi-g++`) cross-compiles to Cortex-M7 and can't run natively, so the
test build uses a native host compiler instead:

* **Linux / macOS** — the system compiler (GCC ≥ 13 or Clang ≥ 16). Nothing to
  install beyond CMake and a recent compiler; `run.cmake` uses it automatically.
* **Windows** — a portable MinGW-w64 GCC kept in `tools/` (git-ignored, not
  committed), since there's no system host compiler to rely on.

To install the Windows toolchain (one time), download a WinLibs MinGW-w64 build
and extract so that `tools/mingw64/bin/g++.exe` exists:

```powershell
$dir = "tools"; New-Item -ItemType Directory -Force $dir | Out-Null
$url = "https://github.com/brechtsanders/winlibs_mingw/releases/download/16.1.0posix-14.0.0-ucrt-r2/winlibs-x86_64-posix-seh-gcc-16.1.0-mingw-w64ucrt-14.0.0-r2.zip"
Invoke-WebRequest $url -OutFile "$dir/mingw.zip"
Expand-Archive "$dir/mingw.zip" -DestinationPath $dir -Force
Remove-Item "$dir/mingw.zip"
```

(Any host GCC ≥ 13 or Clang ≥ 16 with C++23 works; just point the CMake
`*_COMPILER` variables at it.)

## Run the tests

One command, same on every platform (CMake drives the whole thing):

```sh
cmake -P tests/run.cmake          # configure, build, run
cmake -P tests/run.cmake clean    # wipe tests/build first
```

`run.cmake` picks the toolchain per platform (bundled MinGW on Windows, the
system compiler elsewhere). To drive the steps yourself instead:

```sh
# Linux / macOS — system compiler, default generator:
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

```powershell
# Windows — bundled MinGW toolchain (quote the -D values so PowerShell expands them):
$bin = "$PWD/tools/mingw64/bin"
cmake -S tests -B tests/build -G "MinGW Makefiles" `
    "-DCMAKE_CXX_COMPILER=$bin/g++.exe" `
    "-DCMAKE_C_COMPILER=$bin/gcc.exe" `
    "-DCMAKE_MAKE_PROGRAM=$bin/mingw32-make.exe"
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

## Adding a test — which repo?

Put the test where the code under test lives:

* **Code in `stm-2026-common/`** → add the test in **`stm-2026-common/tests/`**
  (see that README). It runs both standalone and as part of this parent suite.
* **Code in `CM7/app/` (parent-only)** → add it here under `tests/logic/...`
  mirroring the source path, then add an `add_executable(...)` +
  `gtest_discover_tests(...)` block in `tests/CMakeLists.txt` (mirror the
  `fcu_controller_test` block). Use `${STMCOMMON_LOGIC_INCLUDE_DIRS}` (exported
  by the submodule build) plus `${REPO_ROOT}/CM7/app/logic`, and link
  `GTest::gtest_main`. Includes resolve via `-I`, so the test's own path doesn't
  affect them.

GoogleTest is not committed (it's git-ignored to keep the repo light).
`fetch-googletest.cmake` downloads a pinned release into `third_party/googletest`
on the first run — so the first `cmake -P tests/run.cmake` needs network, and
every run after that is offline. To bump the version, edit `GTEST_VERSION` and
`GTEST_SHA256` in that script.
