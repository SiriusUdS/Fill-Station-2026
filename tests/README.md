# Logic unit tests (host / GoogleTest)

Native (host-run) unit tests for the **HAL-free logic modules** — the code under
`CM7/app/logic/` that depends only on the `udp::` / `can::` interfaces and never
touches the HAL. These build and run on your PC, separately from the firmware
cross-compile build.

```
tests/
├── CMakeLists.txt          # host build (separate project from the firmware)
├── run.cmake               # configure + build + test in one step (any platform)
├── fetch-googletest.cmake  # downloads GoogleTest into third_party/ (run.cmake calls it)
├── fcu_controller_test.cpp # tests for logic::fcu (the FCU state machine)
├── support/
│   ├── fakes.hpp/.cpp      # FakeBus: test doubles for the udp::/can:: interfaces
└── third_party/
    └── googletest/         # GoogleTest source (downloaded on demand, git-ignored)
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

## Adding tests for another logic module

1. Create `tests/<module>_test.cpp`.
2. In `tests/CMakeLists.txt`, add an `add_executable(... )` for it (mirror the
   `fcu_controller_test` block), linking `GTest::gtest_main` and listing the
   logic `.cpp` under test plus `support/fakes.cpp` if it uses the interfaces.
3. Keep the same `LOGIC_INCLUDE_DIRS` so headers resolve as in the firmware.

GoogleTest is not committed (it's git-ignored to keep the repo light).
`fetch-googletest.cmake` downloads a pinned release into `third_party/googletest`
on the first run — so the first `cmake -P tests/run.cmake` needs network, and
every run after that is offline. To bump the version, edit `GTEST_VERSION` and
`GTEST_SHA256` in that script.
