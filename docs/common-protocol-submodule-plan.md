# `common-protocol` — the official wire-format submodule

Status: **proposed** (2026-06-12). Companion to `dual-board-refactor-plan.md` §8
("Future narrow submodule = system constants + protocol wire formats only").

## 0. Goal

Promote the in-repo `src/app/logic/common/communication/protocol/` material into a
standalone git repository **`common-protocol`**, added as a submodule to **both**
Fill-Station-2026 (FCU + ECU firmware) and the **Ground Station (GS)** repo. It is
the single source of truth for everything that crosses a wire: UDP/CAN packet
layouts, the `SystemState` telemetry record and its sub-structs, board IDs, command
IDs, and the shared enums.

The immediate driver: **the GS must parse the `SystemState` packet** we are already
emitting (the 88 / 1312 / 1384-byte datagrams). Today the GS leans on
`sirius-headers-common` for these definitions; `common-protocol` replaces that
relationship with a repo **we** own.

### Principles
1. **Header-only, HAL-free, language-portable.** Pure C-style structs + fixed-width
   ints + enums. No logic, no STM32 HAL, no `std::` containers in the wire types.
   Must compile in the firmware (arm-none-eabi-gcc, C++23) *and* the GS toolchain
   (same family today — it already eats these headers via sirius).
2. **SSOT, not a dumping ground.** Wire formats and the constants that name them go
   in. Parsers, dispatch, state machines stay in `logic/`.
3. **`common-protocol` is OURS** — unlike `sirius-headers-common`/`stm-2026-common`
   we *do* commit to it. (The copy-out-only rule still applies to sirius during the
   shedding below: we lift sirius headers into `common-protocol` as new files; we
   never push into the sirius working dir.)
4. **No big bang.** Both boards + the test suite stay green at every step.
5. **Wire layout is frozen by `static_assert`.** The existing no-padding asserts
   (`SystemState`, `EthernetHeader`, `SystemStatePacket`, `CanHeader`) move with the
   headers and become the contract the GS relies on.

---

## 1. Scope — what moves in (the lift inventory)

All paths below are under `src/app/logic/common/`.

### Goes into `common-protocol` (wire format + SSOT)
- **`communication/protocol/telemetry/`** — *all* of it:
  `system_state.hpp`, `adc_info|adc_state|adc_status`, `valve_info|valve_state|valve_status`,
  `storage_info|storage_state|storage_status|storage_error`,
  `ethernet_info|ethernet_state|ethernet_status`, `can_info|can_state|can_status`.
- **`communication/protocol/ethernet/ethernet_header.hpp`** (the 12-byte UDP header).
- **`communication/protocol/can/can_header.hpp`** and **`can/system_state_codec.hpp`**
  (the SystemState↔CAN fragment codec — board↔board wire format; ships even though
  the GS won't reassemble, because it's protocol SSOT).
- **`Inc/dil/can_types.h`** (CAN message IDs + `CANHeader` 29-bit ID union).
- **`communication/protocol/command/`** *wire structs only*:
  `command.hpp` (CommandType / payloadID SSOT), `set_state.hpp`, `set_valve_position.hpp`.
- **The 6 headers still vendored under `…/protocol/sirius-headers-common/`** — these
  are rewritten as our own clean files (this is where the sirius name dies):
  - `Telecommunication/PacketHeaderVariable.h` → board IDs (`FILLING_STATION_BOARD_ID`,
    `ENGINE_BOARD_ID`, broadcast) + `payloadID` constants incl. **`GET_SYSTEM`**.
  - `Telecommunication/InterfaceField.h` → `InterfaceField` (used directly by `SystemState`).
  - `Telecommunication/BoardCommandV2.h`.
  - `FillingStation/FillingStationState.h` → the `State` enum.
  - `Ethernet/UDPFrame.h` (`UDPPacketHeader`) + `Ethernet/UDPDeviceCtrlFlags.h`.
- **Dedupe:** there are currently **two** `InterfaceField.h` copies
  (`…/protocol/sirius-headers-common/Telecommunication/` and
  `…/protocol/telemetry/states/`). Collapse to one in `common-protocol`.

### Stays in `logic/common` (consumers, not wire format)
- `communication/protocol/command/parser/command_can_parser.*`,
  `command_ethernet_parser.*` — these *interpret* the wire structs; they `#include`
  `common-protocol` but live with the logic.
- Everything else in `logic/` and `platform/`.

### Decision pending — valve identity maps
`communication/protocol/fcu_valves.hpp` (and `logic/ecu/ecu_valves.hpp`) define which
array index in `SystemState::valve_info[2]` is which physical valve. The GS needs that
mapping to *label* telemetry, which argues for moving the per-board valve enums into
`common-protocol`. They're board-specific, so they'd live under a `boards/` sub-path
there. **Recommendation:** move them in (the GS wants labels); flagged as decision D4.

---

## 2. `common-protocol` repository layout

```
common-protocol/                      # submodule repo root (its own git history)
  include/
    fs_protocol/                      # single umbrella prefix → no include clashes in the GS build
      telemetry/
        system_state.hpp              # SystemState, SystemStatePacket (+ the padding static_asserts)
        adc_info.hpp  adc_state.hpp  adc_status.hpp
        valve_info.hpp valve_state.hpp valve_status.hpp
        storage_*.hpp  ethernet_*.hpp  can_info.hpp can_state.hpp can_status.hpp
        interface_field.hpp           # the deduped InterfaceField
      ethernet/
        ethernet_header.hpp           # EthernetHeader (deviceID, payloadID, payloadLenght, …)
        udp_frame.hpp  udp_device_ctrl_flags.hpp
      can/
        can_types.h                   # CAN message IDs + CANHeader   (was Inc/dil/can_types.h)
        can_header.hpp  system_state_codec.hpp
      command/
        command.hpp  set_state.hpp  set_valve_position.hpp  board_command.hpp
      system/
        board_ids.hpp                 # FILLING_STATION_BOARD_ID, ENGINE_BOARD_ID, GET_SYSTEM, …
        filling_station_state.hpp     # State enum
      boards/                         # (if D4 accepted) fcu_valves.hpp, ecu_valves.hpp
      protocol_version.hpp            # FS_PROTOCOL_VERSION (bump on any wire change)
  CMakeLists.txt                      # INTERFACE target `fs_protocol` exposing include/
  README.md                          # what it is, the copy-out rule, how to bump the version
```

Includes everywhere become `#include "fs_protocol/telemetry/system_state.hpp"` — one
prefix, snake_case, sheds the `sirius-headers-common/…PascalCase…` paths for good.

Mounted in this repo at **`src/common-protocol/`** (keeps the "one code folder = `src/`"
rule).

---

## 3. Phased steps

### Phase A — Stand up the repo (no firmware change yet)
- [ ] Create the `common-protocol` git repo (empty + README + `protocol_version.hpp`,
      `FS_PROTOCOL_VERSION = 1`).
- [ ] Build the `include/fs_protocol/` tree by **copying** the files from §1 in, renamed
      to the §2 layout. Rewrite the 6 sirius headers as clean `fs_protocol` files
      (no `sirius-headers-common` path, no sirius include guards).
- [ ] Fix **intra-protocol includes** to the `fs_protocol/…` form (e.g. `system_state.hpp`
      currently pulls `communication/protocol/telemetry/adc_info.hpp` and
      `sirius-headers-common/Telecommunication/InterfaceField.h` → become
      `fs_protocol/telemetry/adc_info.hpp` and `fs_protocol/telemetry/interface_field.hpp`).
- [ ] Add `CMakeLists.txt` exposing `add_library(fs_protocol INTERFACE)` +
      `target_include_directories(fs_protocol INTERFACE include)`.
- [ ] Sanity-compile the headers standalone (a tiny TU that `#include`s `system_state.hpp`
      and asserts the sizes: `SystemState==72`, `EthernetHeader==12`, `CanHeader==4`).

### Phase B — Wire it into Fill-Station-2026 (firmware + tests green)
- [ ] `git submodule add <url> src/common-protocol`; commit the `.gitmodules` + pointer.
- [ ] **Firmware** (`src/boards/{fcu,ecu}/CM7/CMakeLists.txt`): drop the
      `…/logic/common/communication/protocol` and `…/logic/common/Inc` include dirs;
      `add_subdirectory(src/common-protocol)` at the top level and
      `target_link_libraries(<board>_CM7 PRIVATE fs_protocol)`.
- [ ] **Tests** (`src/app/tests/CMakeLists.txt`): drop the `protocol` entry from
      `LOGIC_COMMON_INCLUDE_DIRS`; link `fs_protocol` into the test targets.
- [ ] **Swap include sites** (8 known files) from `sirius-headers-common/…` and the old
      `telemetry/…` / `dil/can_types.h` paths to `fs_protocol/…`:
      `system_state.hpp`, `Inc/dil/can_types.h` (consumers), `ecu_controller.hpp`,
      `fcu_controller.hpp`, `board.cpp` ×2, and the two FCU tests. Plus the codec,
      command wire-struct, and telemetry cross-includes.
- [ ] **Delete** `src/app/logic/common/communication/protocol/sirius-headers-common/`
      and the now-migrated originals; remove the duplicate `telemetry/states/InterfaceField.h`.
- [ ] Build FCU, build ECU, run the logic tests (currently 80/80). All green.
- [ ] Re-verify on hardware that the live datagrams are byte-identical (sizes unchanged:
      88 / 1312 / 1384) — the lift must not move a single field.

### Phase C — Ground Station consumes it (the actual objective)
- [ ] GS repo: `git submodule add <url> third_party/common-protocol` (or wherever it
      keeps vendored deps); link/include `fs_protocol`.
- [ ] GS UDP receive path parses each datagram against the wire contract (see §4).
- [ ] GS labels records by `EthernetHeader.deviceID` (FCU vs ECU) and by valve map (D4).
- [ ] Round-trip check: a known FCU record + a relayed ECU record decode to the expected
      field values.

### Phase D — Guardrails
- [ ] CI grep in `common-protocol`: reject `#include <stm32`, `HAL_`, `std::vector/string`,
      and any non-fixed-width integer in a wire struct.
- [ ] CI: compile `include/fs_protocol/**` as **both C and C++** (catches bit-field / enum
      portability issues before the GS hits them).
- [ ] `protocol_version.hpp` bump is required by review whenever a wire struct changes;
      note the version in the GS handshake later.

---

## 4. The wire contract the GS implements (from `sendToGs`)

Each UDP datagram is `EthernetHeader (12) + payload (N×SystemState) + CRC32 (4)`:

| Field (EthernetHeader) | Meaning for the GS |
|---|---|
| `deviceID`     | source board — `FILLING_STATION_BOARD_ID` (FCU) or `ENGINE_BOARD_ID` (ECU-relayed) |
| `payloadID`    | `GET_SYSTEM` → payload is a run of `SystemState` records |
| `payloadLenght`| **bytes** of record payload that follow (a multiple of `sizeof(SystemState)=72`) |
| `deviceState`  | source board's fill/engine state at send time |
| `deviceTS_MS`  | sender tick |

GS steps: read 12-byte header → `count = payloadLenght / 72` records → **CRC32 over the
`payloadLenght` payload bytes only** (not the header) and compare to the trailing 4 bytes
→ reinterpret each 72-byte record as `SystemState`. Observed shapes confirm it:
ECU relay = 1 record (88 B), FCU batches = 19 records (1384 B) and the 18-record tail
(1312 B). **Note:** the firmware CRC32 is the reflected zlib variant
(`0xEDB88320`) and is computed over the **records only** — the GS must match both
(there's a `TODO` in `detail::crc32` to confirm the GS variant; close it here).

---

## 5. Risks / watch-items
- **Bit-field portability.** `CanHeader`, `EthernetHeader`, and the `*_status` types use
  C bit-fields, whose layout is compiler-defined. FCU/ECU/GS are all little-endian GCC
  today (so it works), but the C-and-C++ CI compile (Phase D) plus an explicit
  size/offset test is what keeps a future GS compiler from silently re-packing.
- **Packing.** No `#pragma pack` anywhere — correctness rests on the no-padding
  `static_assert`s. They must travel with the headers (they do, in `system_state.hpp` /
  `ethernet_header.hpp`).
- **Two consumers, one history.** A wire change is now a coordinated bump across three
  repos. `protocol_version.hpp` + the submodule pointer make that explicit rather than
  silent drift.
- **Copy-out discipline (sirius).** The migration *reads* sirius and *writes* new files
  into `common-protocol`; never commit inside `sirius-headers-common`. After Phase B it's
  gone from this repo entirely.

---

## 6. Open decisions
- **D1 — Submodule URL/host & name.** Confirm `common-protocol` as the repo name and where
  it lives (same org as the GS repo so both can submodule it).
- **D2 — Include prefix.** Recommend `fs_protocol/`. Alternative: keep the existing
  `telemetry/…`,`can/…` roots with no umbrella (less churn, higher clash risk in GS).
- **D3 — CAN codec in or out.** Recommend **in** (it's protocol SSOT) even though the GS
  doesn't reassemble. Cheap, header-only.
- **D4 — Valve identity maps.** Recommend **in** (`fs_protocol/boards/{fcu,ecu}_valves.hpp`)
  so the GS can label valves. Alternative: leave them as board policy in `logic/`.
- **D5 — Rename vs lift-as-is.** Recommend the clean snake_case rename now (do the include
  churn once, while sheding sirius). Alternative: lift paths verbatim to minimize edits and
  rename later.

## 7. Definition of done
- `common-protocol` exists, versioned, header-only, compiles as C and C++.
- This repo references it as a submodule; `sirius-headers-common` is deleted; FCU + ECU +
  tests build green; live datagrams byte-identical.
- The GS submodules it and decodes FCU and ECU `SystemState` records (CRC-checked) from
  the live downlink.
