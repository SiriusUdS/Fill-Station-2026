# Dual-Board (FCU / ECU) Monorepo Refactor Plan

> Living document. Tracks the migration from the current single-board (FCU) tree to
> a one-repo / two-board (FCU + ECU) structure. Check off steps as we go; revisit the
> "Open decisions" section whenever a new fact lands.

## 1. Goal

One repository builds **two STM32 boards** that share the same MCU (STM32H747IIT) but are
**independent hardware designs** (different pinouts, different peripheral instances) **and**
have **different behavior**:

- **FCU** (Fill/Flight Control Unit): Ethernet → Ground Station, SD logging, local valves + ADC.
- **ECU** (Engine Control Unit): **no Ethernet**; sends its telemetry over **CAN → FCU**, which
  relays it to the Ground Station. Different state machine and command-handler set.

End state: the two boards share as much code as physically possible; everything that differs is
isolated into two clearly-named places — a **hardware-binding seam** and a **per-board logic policy**.

## 2. Guiding principles

1. **Selection by CMake file-set, never `#ifdef`.** Each board target compiles a different *list*
   of files. Shared code contains zero `#ifdef BOARD_*`. This is what preserves "most similar."
2. **Two orthogonal axes of variation, kept separate:**
   - *Hardware binding* (pins, peripheral instances, ISR vectors, clocks) → the `board::` seam.
   - *Application policy* (states, handlers, telemetry routing, peripheral complement) → `logic/{fcu,ecu}`.
3. **Shared code names no generated symbol.** No pin macro (`*_Pin`), no HAL handle (`hspiN`/`htimN`),
   no `MX_*_Init`, no ISR vector name appears outside a board's own `board.cpp`.
4. **Mechanism is shared; policy is split.** `logic/common` holds the *engine* (command dispatch,
   state-machine framework, record pipeline, protocol SSOT). `logic/fcu` and `logic/ecu` hold the
   *content* (which states, which handlers, which routing).
5. **Each step leaves the repo building.** No "big bang." FCU stays green throughout; ECU comes online late.

## 3. Target directory layout

```
repo/
  app/                              # SHARED application code (board-agnostic + per-board-type)
    logic/
      common/                       # shared MECHANISM — compiled into BOTH boards
        (command-dispatch framework, state-machine engine, record pipeline,
         common handlers e.g. ping/set_state)
        protocol/                   # system constants + wire formats (SSOT enums, board IDs,
                                    #   CAN/UDP packet layouts). Kept grouped here on purpose:
                                    #   this is the material that LATER becomes its own submodule.
      fcu/                          # FCU POLICY — controller, states, handlers, run(), eth+SD routing
      ecu/                          # ECU POLICY — controller, states, handlers, run(), CAN-telemetry routing
    platform/                       # SHARED board-agnostic DRIVERS (no pin/handle literals)
      communication/{ethernet,spi,can}/
      acquisition/adc/
      actuation/{servomotor,valve}/
      storage/
      memory/
    board.hpp                       # the board:: contract (declared here, implemented per board)
    app_sources.cmake               # helper: COMMON/ FCU/ ECU source + include lists
  boards/
    fcu/
      FillStationFCU.ioc            # FCU CubeMX project
      CM7/                          # FCU generated Core/ + per-board CMake (isolated). BUILT.
      CM4/                          # emitted by CubeMX (dual-core silicon) but NEVER built; untracked/ignored
      mx-generated.cmake            # CM7 only
      board.cpp                     # FCU halInit() + wireDrivers() + ISR vectors  (FCU pin labels)
    ecu/
      ECU.ioc                       # ECU CubeMX project
      CM7/                          # ECU generated Core/ (isolated — never clobbers FCU). BUILT.
      CM4/                          # emitted but NEVER built; untracked/ignored
      mx-generated.cmake            # CM7 only
      board.cpp                     # ECU halInit() + wireDrivers() + ISR vectors  (ECU pin labels; no eth)
  CMakeLists.txt                    # root: -DBOARD=fcu|ecu selects boards/<board>
  CMakePresets.json                 # fcu-Debug, fcu-Release, ecu-Debug, ecu-Release
```

> **Submodule decision (resolved):** the current `stm-2026-common` submodule is **folded into this
> repo** — its `platform/` drivers move under `app/platform/`, its logic under `app/logic/common`.
> A **new, narrowly-scoped submodule will be created later**, containing **only global system
> constants and protocol wire formats** (the `app/logic/common/protocol/` material). Until then,
> that material is vendored in-repo but kept grouped so the future extraction is a clean lift.

Notes:
- `board.cpp` lives **next to its `.ioc`/`Core/`** so every board-specific fact is co-located.
- The shared `app/` source list is factored into `app_sources.cmake` so the two board CMake files
  don't duplicate the long list; each board adds `common + <its policy> + <its platform subset>`.

## 4. The `board::` contract (the hardware seam)

`app/board.hpp` declares; each `boards/<b>/board.cpp` implements:

```cpp
namespace board {
  // CubeMX MX_*_Init list + SystemClock/PeriphCommonClock + MPU for THIS board.
  void halInit();

  // Hand each driver THIS board's handles/pins via their init() config structs.
  // Receives the board-type's object graph (different per board — FCU has eth, ECU doesn't).
  void wireDrivers(/* board-type driver bundle */);
}
```

- Shared `app/logic/<board>/run()` owns the object graph + tick loop and calls `board::halInit()`
  then `board::wireDrivers(...)`. The ECU's `run()` simply never constructs an Ethernet object.
- ISR vectors (`EXTI9_5_IRQHandler`, `TIM6_DAC_IRQHandler`, …) are board-specific symbols → they
  live in `board.cpp`, dispatching into the shared driver/controller via existing HAL callbacks.
- The per-board entry point shrinks to roughly:
  ```cpp
  int main() { board::halInit(); logic::fcu::run(); }   // ECU: logic::ecu::run();
  ```

## 5. Classification of current files (move-list)

Discriminator: **names a generated symbol → board repo dir; else → shared `app/`.**
Within shared, **mechanism → logic/common; policy → logic/fcu (today's behavior)**.

| Current path | Destination | Notes |
|---|---|---|
| `CM7/app/platform/main.cpp` | **split** | `main()`+loop+objects → `app/logic/fcu/run` (policy); `halInit`/`wireDrivers`/ISR/clocks → `boards/fcu/board.cpp` |
| `CM7/app/logic/fcu_controller.hpp` | `app/logic/fcu/` | FCU policy (header-only template stays) |
| `CM7/app/logic/control/command_handlers/main_handler.*` | `app/logic/common/` | dispatch framework = mechanism |
| `…/command_handlers/ping.*`, `execute_ping.*` | `app/logic/common/` | shared handler — **dedupe vs `stm-2026-common/logic/commands/ping.*`** |
| `…/command_handlers/set_state.*` | `app/logic/common/` if both boards have states; else split | decide during Phase 1 |
| `…/command_handlers/set_state_transitions/activate_igniter.*` | `app/logic/fcu/` (likely) | board-specific transition |
| `…/command_handlers/set_valve_position.*`, `execute_set_valve_position.*` | `app/logic/common/` or split | both boards have valves? confirm |
| `…/command_handlers/synchronise.*`, `execute_synchronise.*` | `app/logic/common/` | likely shared |
| `CM7/app/logic/communication/interfaces/{adc,ethernet}.hpp` | `app/logic/common/` interfaces | ethernet iface used only by FCU policy, but harmless to share |
| `CM7/app/logic/storage/interfaces/storage.hpp` | `app/logic/common/` | concept |
| `CM7/app/platform/communication/ethernet/*` | `app/platform/communication/ethernet/` | shared driver; **ECU target omits from file-list** |
| `CM7/app/platform/communication/spi/*` | `app/platform/communication/spi/` | shared driver |
| `CM7/app/platform/acquisition/adc/ads131m08.*` | `app/platform/acquisition/adc/` | shared driver |
| `CM7/app/platform/storage/sd_card.*` | `app/platform/storage/` | shared driver; ECU may omit |
| `CM7/Core/**`, `FillStation.ioc`, `.mxproject`, `mx-generated.cmake` | `boards/fcu/…` | becomes the FCU CubeMX project |
| `stm-2026-common/platform/**` (can_dil, ball_valve, hbl388, backup_ram) | `app/platform/**` | **fold in** (de-submodule) |
| `stm-2026-common/logic/**` (commands/ping, can/, persistent_state) | `app/logic/common/**` | fold in; dedupe vs CM7 copies |
| `sirius-headers-common/**` (nested submodule) | `app/logic/common/protocol/` (selective) | **migrate only what we use**, purge the rest, then **remove the submodule**. This `protocol/` dir becomes our own SSOT and the **future submodule** (see §8). |

(ECU policy files in `app/logic/ecu/` are authored fresh in Phase 5 — they have no current counterpart.)

## 6. Build-system changes

The genuinely hard part. CubeMX is one-project-per-directory and regenerates `Core/` +
`mx-generated.cmake` + `.mxproject` at fixed paths — two projects at the repo root would
**clobber each other**.

1. **Isolate each board's generated tree** under `boards/<b>/` so regenerating ECU never touches FCU.
2. **Dual-core caveat:** the H747 `.ioc` emits **both CM7 and CM4** trees, but **neither board builds
   CM4**. Keep `BUILD_CONTEXT=CM7` and drop the CM4 `ExternalProject_Add` from each board's
   `mx-generated.cmake`. The emitted `CM4/` is left untracked/`.gitignore`d.
3. **Root `CMakeLists.txt`:** add `-DBOARD=fcu|ecu` (default `fcu`). Based on `BOARD`, include
   `boards/${BOARD}/mx-generated.cmake` (which `ExternalProject_Add`s that board's `boards/${BOARD}/CM7`).
4. **Per-board `CM7/CMakeLists.txt`:** instead of today's inline source list, `include(app_sources.cmake)`
   and add `${COMMON_APP_SOURCES}` + `${${BOARD}_SOURCES}` + `boards/${BOARD}/board.cpp`, plus the
   board's `Core/Inc` and the shared `app/` include dirs. The ethernet/SD sources are added only to
   `FCU_SOURCES` (ECU omits them).
5. **`CMakePresets.json`:** add `fcu-Debug`, `fcu-Release`, `ecu-Debug`, `ecu-Release`, each setting
   `BOARD` and a distinct `binaryDir` (`build/${presetName}`).

## 7. Phased execution (each phase ends with a green build)

- [x] **Phase 0 — Prep & safety** ✅ *DONE — committed `e1f9426` on branch `dual-board-refactor`*
  - [x] Drop CM4: untracked + removed `CM4/`, added `.gitignore` rule. Durable control = `BUILD_CONTEXT=CM7`
        in root `CMakeLists.txt` (CubeMX never regenerates it), which keeps the regenerated
        `mx-generated.cmake` CM4 block inert. Left `mx-generated.cmake` untouched (CubeMX owns it).
  - [x] Branch from `reorganisation` → **`dual-board-refactor`**.
  - [x] Confirm clean FCU build baseline: `cmake --preset Debug` then `cmake --build build/Debug`
        → `FillStation_CM7.elf`, FLASH 97032 B / 9.25%. (toolchain: arm-none-eabi-gcc 12.3, ninja, cmake 3.28)
  - [x] **Fold in `stm-2026-common`** (de-submodule): `git rm --cached stm-2026-common`, removed
        `.gitmodules`, deleted the `.git` pointer files (stm-2026-common + nested sirius), `git add`ed
        the files as normal content. `git submodule status` now empty; no embedded `.git` remains.
  - [ ] **Migrate out of `sirius-headers-common`, then drop it.** Transitive closure of what we actually
        use is exactly **6 headers** (scoped 2026-06-11):
        - `Telecommunication/PacketHeaderVariable.h` (FILLING_STATION_BOARD_ID)
        - `Telecommunication/BoardCommandV2.h`
        - `Telecommunication/InterfaceField.h` (used by `system_state.hpp` in common)
        - `FillingStation/FillingStationState.h`
        - `Ethernet/UDPFrame.h` → includes `Ethernet/UDPDeviceCtrlFlags.h`
        Copy *only those* into `app/logic/common/protocol/`, fix includes, then **remove the
        `sirius-headers-common` submodule** entirely. Everything else (GPS/Gyro/Rocket/… packets) is unused.
        Used at: `main.cpp:36`, `fcu_controller.hpp:24-27`, `stm-2026-common/.../system_state.hpp:5`.
  - [x] **DONE (in place):** sirius-headers-common removed as a submodule and pruned to exactly those 6
        headers, **kept at its current path/name** so includes are unchanged and the build stays identical.
  - [ ] **DEFERRED to Phase 3:** rename/relocate those 6 headers to `app/logic/common/protocol/` and
        update the include sites — rides the main /app relocation (avoids editing includes twice).
  - [ ] ⚠️ **NEVER commit/push anything *inside* `sirius-headers-common`** — it is a shared submodule
        used by other projects. This migration is strictly **copy-out** (files flow into *our* tree).
        Touch its working dir read-only; the only change recorded in *this* repo is the removal of the
        submodule pointer + `.gitmodules` entry. Same rule for `stm-2026-common` while de-submoduling:
        no commits land in the submodule, only the superproject drops the pointer.
  - [ ] Update include paths that pointed at `../stm-2026-common/...` and
        `.../sirius-headers-common/...` to the new `app/...` locations. Build FCU. ✅

- [ ] **Phase 1 — Reshape logic into common/fcu (no behavior change, FCU keeps building)**
  - [ ] Create `app/logic/{common,fcu}` (initially still under `CM7/app/` if not yet relocating the tree).
  - [ ] Classify each command handler as common vs fcu (table §5); move files; fix includes.
  - [ ] Dedupe `ping` against `stm-2026-common/logic/commands/ping.*` — one source of truth.
  - [ ] Update `CM7/CMakeLists.txt` source list. Build FCU. ✅

> **Naming note:** the shared-code root is **`src/`** (not `app/`). `boards/<board>/` holds each board's
> CubeMX project + `board.cpp`. Vendor HAL (`Drivers/Middlewares/Common`) is **duplicated per board**
> under `boards/<board>/` (accepted trade-off — CubeMX projects are self-contained, regen-safe).

- [x] **Phase 2 — Extract the `board::` seam from `main.cpp`** *(halInit done; wireDrivers/run pending)*
  - [x] Add `src/board.hpp` contract.
  - [x] Move `halInit` (clocks/MPU/MX_*_Init) → `boards/fcu/board.cpp`; `main()` calls `board::halInit()`.
  - [ ] Still pending: the composition seam — `board::wireDrivers()` + a shared `run()` (the `g_*`-wiring
        half of `fcuInit`, which still names `hspi4`/`htim1`/pins inside `main.cpp`).

- [~] **Phase 3 — Relocate to target layout + `-DBOARD` plumbing** *(structure done; shared-code move pending)*
  - [x] **FCU CubeMX project moved to `boards/fcu/`**, renamed `FillStation`→`fcu`: `fcu.ioc`, `CM7/Core/`,
        generated cmake, + its own `Drivers/Middlewares/Common` copy. Build byte-identical (`fcu_CM7.elf`).
  - [x] New top-level `CMakeLists.txt` board selector (`-DBOARD=fcu|ecu`, default fcu) builds
        `boards/<board>/CM7` directly, anchoring shared-tree includes via a passed `REPO_ROOT`.
  - [x] Per-board ergonomics: `fcu-*`/`ecu-*` presets; `.clangd` indexes **both** boards at once
        (PathMatch per board); `.vscode/tasks.json` Configure/Build per board (Build FCU = default).
  - [x] **DONE:** shared code relocated into **`src/app/`** (`logic/{common,fcu}` + `platform/` + `board.hpp`);
        board projects under **`src/boards/`**; `stm-2026-common/` fully folded in and deleted. Repo root now
        holds a single code folder `src/`. Firmware builds byte-identical.
  - [x] **Tests consolidated** into `src/app`: one harness (`src/app/tests/` — runner + shared fakes +
        GoogleTest), co-located per-site test dirs `src/app/logic/<site>/tests/` (`common` + `fcu`; `ecu`
        later). The obsolete `stm-2026-common` standalone-submodule test machinery is retired. **75 tests pass.**
        Run via `cmake -P src/app/tests/run.cmake` (VS Code task: "Run logic tests").

- [x] **Phase 2b — Composition seam** ✅ DONE. `main.cpp` (`src/boards/fcu/main.cpp`) is now the
      **handle-free** app composition: it defines the object graph + tick loop and calls
      `board::halInit(); board::wireDrivers();`. `board.cpp` owns **all** hardware naming —
      `wireDrivers()` (the `hspi4`/`htim1`/pin binding + timer/NVIC config) and the ISR vectors.
      `SdCard` gained a default ctor + `bind()` so the card is declared handle-free and bound in `board.cpp`.
      **Layering correction vs the original plan:** the composition instantiates the HAL drivers, so it
      cannot live in the HAL-free `src/app/logic/fcu`; it stays **board-side** under `src/boards/fcu/`
      (with `fcu_objects.hpp` sharing the object graph between `main.cpp` and `board.cpp`). The HAL-free
      FCU logic (Controller, handlers, states) remains in `src/app/logic/fcu`. Firmware builds, 75 tests pass.

- [~] **Phase 4 — Stand up the ECU board** *(clone builds; thinning next)*
  - [x] Created `src/boards/ecu/` by copying `src/boards/fcu/` and renaming the project (`ecu.ioc`
        ProjectName=ecu, `ecu_CM7` build target). Builds via `cmake --preset ecu-Debug` → `ecu_CM7.elf`.
        Currently an **exact FCU clone** (same `Core/`; `board.cpp`/`main.cpp`/`fcu_objects.hpp` still
        reference `logic::fcu` and the FCU drivers). Both boards build side-by-side, isolated build dirs.
  - [x] **Thinned to a do-nothing stub (code track):** ecu's `main.cpp` = idle loop; `board.cpp` = minimal
        `halInit` (clocks/MPU/GPIO) + empty `wireDrivers()`; dropped `fcu_objects.hpp`; ecu `CM7/CMakeLists`
        compiles only `../main.cpp` + `../board.cpp` (no FCU drivers/logic). `src/app/logic/ecu/` placeholder
        added. Builds via `ecu-Debug` → `ecu_CM7.elf`, FLASH 2.21% (was 9.25%). FCU untouched & green.
  - [x] **ECU pinout generated (CubeMX).** Real peripheral complement: 2 servo valves IPA(PE5)/NOS(PE6)
        on TIM15, 4 limit switches (PF0-3), ADS131M08 ADC on **SPI1** (PA4-7, CS PC5), SD on **SDMMC1/hsd1**,
        **FDCAN1** (telemetry→FCU), TIM6 (record timer), CRC. **No Ethernet, no I2C, no SPI6 (no TCs).**
        Synced ecu `board.cpp` (dropped the FCU `PeriphCommonClock_Config` — ECU uses default kernel clocks).
        Builds via `ecu-Debug` → `ecu_CM7.elf`, FLASH 0.94%. Vestigial CubeMX per-board toolchain/preset
        copies gitignored. (NB: generated `Core/Src/main.c` carries leftover USER CODE — harmless, it's excluded.)
  - [x] **ECU drivers wired (logic deferred).** ecu `main.cpp` defines the object graph (2 `BallValve`,
        `Ads131m08`, `Can`, `SdCard`); `board.cpp` `halInit` brings up all peripherals and `wireDrivers`
        binds them: CAN=`ENGINE_BOARD_ID`(0x01), ADS131M08 on SPI1 (DRDY PA4→`EXTI4`, CS PC5), SD on hsd1
        (mounted), IPA valve=TIM15_CH1, NOS valve=TIM15_CH2 (both with open+close limit switches), valves
        closed at boot. Loop ticks the valves; no controller yet. Builds → `ecu_CM7.elf`, FLASH 6.59%. FCU green.
        Shared driver *interfaces* (adc/storage) still live in `logic/fcu` — ECU includes that for now
        (**TODO: move shared interfaces to `logic/common`**).
  - [x] **Bench bring-up confirmed** (BOOT0/VTOR gotcha resolved — see [[ecu-boot0-vtor-gotcha]]; SDMMC flow
        control enabled). Both boards flash + run; ECU has FCU/ECU debug launch configs.
  - [~] **`logic::ecu` controller — E1 done (skeleton).** `logic::ecu::Controller<S,V,A,C>` (FCU mirror minus
        Ethernet) in `src/app/logic/ecu/{ecu_controller.hpp/.tpp,ecu_valves.hpp}`: `init`→`Safe`, `tick`
        (drain CAN + flush telemetry to SD + Init→Safe), `produceRecord` (TIM6 2 kHz → drain ADC → SystemState
        log). Wired into ecu `main.cpp`/`board.cpp` (backup domain, `g_controller.init()`, TIM6 ISR). Builds
        (`ecu_CM7.elf` FLASH 7.25%); FCU green; 75 tests pass. State stays minimal (Init→Safe); engine states later.
  - [ ] **E2 — CAN command routing (next):** `canTick()` dispatches inbound CAN — `CAN_ID_CMD_VALVE`→actuate
        IPA/NOS, ping→pong, set-state — via an ECU command-handler registry.
  - [ ] **E3 — CAN telemetry sink:** `drainTick()` downlinks records over CAN to the FCU (fragment `SystemState`
        or compact `STATUS_VALVE`); likely refactor the record pipeline (LogBuffer + produceRecord + a generic
        sink) into `logic/common` so FCU/ECU share it. Then **E5** — ECU tests under `logic/ecu/tests/`.

- [ ] **Phase 5 — Author ECU logic policy (`app/logic/ecu/`)**
  - [ ] ECU controller composing `logic/common` mechanism.
  - [ ] ECU state machine + command-handler registry.
  - [ ] ECU telemetry routing: produce records → CAN frames toward the FCU (no SD/UDP path).

- [ ] **Phase 6 — ECU board bring-up (`boards/ecu/board.cpp`)**
  - [ ] `halInit()` MX list + clocks for ECU; `wireDrivers()` with ECU handles/pins; ISR vectors.
  - [ ] `ecu-*` presets; build ECU via `-DBOARD=ecu`. ✅

- [ ] **Phase 7 — FCU↔ECU CAN telemetry relay contract**
  - [ ] Define the ECU→FCU telemetry CAN message layout **once** in `logic/common` (single source of truth).
  - [ ] ECU emits it; FCU parses + relays to GS over UDP. Verify round-trip end-to-end.

- [ ] **Phase 8 — CI / tooling**
  - [ ] Build matrix: both boards (× Debug/Release).
  - [ ] Guard: shared code must not reference generated symbols (grep check in CI).
  - [ ] `.clangd`/include-path sanity for both targets.

- [ ] **Phase 9 — Cleanup**
  - [ ] Remove dead duplicated sources; update README/docs.
  - [ ] Confirm `tests/` cover `logic/common` + both policies with fakes.

## 8. Decisions & deferrals

**Resolved:**
1. **`stm-2026-common` folded into the repo** (not kept as a submodule). `app/logic/common` holds this
   family's shared mechanism.
2. **`logic/common` lives in `app/logic/common`** (this repo).
3. **CM4: neither board uses it.** Both FCU and ECU are **CM7-only**. The H747 `.ioc` still *emits* a
   CM4 tree (dual-core silicon), but it is **never built** — `BUILD_CONTEXT=CM7`, no CM4
   `ExternalProject` for either board. The generated `CM4/` can be left untracked/ignored; no `board.cpp`
   or sources target it.
4. **Future narrow submodule** = system constants + protocol wire formats only (`app/logic/common/protocol/`).

**Deferred (do NOT block the migration):**
5. **ECU peripheral complement** — TBD. **Interim: the ECU is a do-nothing buildable stub** — `halInit()`
   (clocks + minimal GPIO) plus an idle loop; no drivers wired, `ECU_SOURCES` minimal. Drivers/interfaces
   get added when the ECU's real peripherals are decided.
6. **Handler common/fcu classification** — postponed. **Interim: all current logic stays as FCU policy
   under `app/logic/fcu`, unchanged.** `app/logic/common` starts with only the obviously-shared infra
   (`protocol/`, base interfaces). Handlers (`set_state`, `set_valve_position`, `synchronise`, …) get
   carved into `common` later, when the ECU actually needs them. Table §5's "common vs split" cells are
   advisory until then.

## 9. Risks & mitigations

- **CubeMX clobbering** — strictly isolate generation dirs per board; never run two projects into one path.
- **Pin-label divergence** (boards won't share labels) — fine *by design*: labels are referenced only
  inside each `board.cpp`. CI grep guards against a label leaking into shared `app/`.
- **`common` accidentally gaining a hardware dependency** — CI grep for `MX_`, `_Pin`, `h[a-z]+[0-9]`
  handle patterns, and `IRQHandler` in `app/logic/` and `app/platform/`.
- **Submodule pointer drift** — only relevant if shared code stays in the submodule; monorepo avoids it.
- **Scope creep during the move** — Phases 1–3 are *pure refactor, no behavior change*; resist mixing
  in ECU feature work until Phase 4+.
- **Accidental writes to shared submodules** — `sirius-headers-common` (and `stm-2026-common` until
  folded) are used by other projects. Migration is **copy-out only**; never `git commit`/`push` inside
  a submodule working dir. The only superproject change is dropping the pointer + `.gitmodules` entry.
  Before any commit, verify no submodule has staged changes (`git submodule foreach git status`).

## 10. Definition of done

- `cmake --preset fcu-Debug && build` → FCU image; `cmake --preset ecu-Debug && build` → ECU image.
- Shared `app/` contains no generated symbol (CI-enforced).
- ECU telemetry reaches the GS via CAN→FCU relay, contract defined once in `logic/common`.
- Both board targets pass their logic tests with fakes.
```
