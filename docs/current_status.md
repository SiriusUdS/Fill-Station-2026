# Current status — protocol sort + typed-enum standardization

Last updated: 2026-06-12 (end of session). **Branch:** `dual-board-refactor`.
**Last commit:** `2d46b52` (CAN bring-up fixes). **Everything below is UNCOMMITTED.**

## ⚠️ Build state: RED (mid-migration)
Host tests do not compile yet — we are partway through the wire-boundary cast fixes
of the id-consolidation step. The protocol *sort* (below) was verified green
(80/80 tests + both firmware) before the standardization began.

Verify with: `cmake -P src/app/tests/run.cmake`
Firmware: `cmake --build src/boards/fcu/CM7/build` and `.../ecu/CM7/build`

---

## What this work is
Two stacked refactors on top of `2d46b52`:
1. **Sort** `communication/protocol/` into a clean by-meaning taxonomy so it can be
   lifted into the `common-protocol` submodule (see `docs/common-protocol-submodule-plan.md`).
2. **Standardize** the wire types to the project convention: `enum class : uint8_t`
   (not macros), `static_assert` on structs, and **reuse global id SSOTs** instead of
   transport-specific ones.

## DONE (and was green before step 3)
- **Protocol taxonomy** under `communication/protocol/`: `peripherals/{adc,can,ethernet,storage}/`,
  `devices/valve/`, `framing/`, `command/`, `system/`, `telemetry/`. Parsers + `Command`
  intent ejected to `communication/command/`. `sirius-headers-common/` deleted. `State`,
  `crc32_polynomial`, `fcu/ecu` valves pulled into `system/`. 34 git renames.
- **Dead legacy deleted:** `src/app/logic/common/Inc/` and `Src/` removed; stale `Inc`
  include-dir lines removed from `src/app/tests/CMakeLists.txt` + both `boards/*/CM7/CMakeLists.txt`.
- **`State`** is now the single SSOT (`system/state.hpp`, `enum class`); the
  `FILLING_STATION_STATE_*` macros are gone, all ~50 sites use `logic::control::State`.

## DONE in step 3 (typed enums + id consolidation) — definitions
- `system/board_ids.hpp` → `enum class BoardId : uint8_t { Engine=0x01, FillingStation=0x02, GsControl=0x03, Broadcast=0x04 }` + size assert.
- `command/command_type.hpp` → `CommandType` gained `Pong = 0x05` (Ping/SetState/SetValvePosition/Synchronise/Pong).
- `telemetry/telemetry_id.hpp` → **NEW** `enum class TelemetryId : uint8_t { SystemState=0x01 }` (the non-command id space).
- `framing/can_header.hpp` → **kept**, now the single CAN header: `struct FrameCanHeader{...bitfields...}` + `union CanHeader { FrameCanHeader frame; uint32_t code; }` + `static_assert(sizeof==4)`.
- **Deleted:** `framing/can_types.h` (its `CANHeader` union folded into `can_header.hpp`'s `CanHeader`) and `command/board_command.hpp` (superseded by `CommandType` + `TelemetryId`).
- **Struct asserts added:** `udp_frame.hpp` (`==12`), `udp_device_ctrl_flags.hpp` (`==1`), `interface_field.hpp` (`==12`), `can_header.hpp` (`==4`). `typedef struct` → `struct`.
- **CAN-specific enums deleted, reused globals** (the rename already done across the tree):
  - `CAN_ID_CMD_VALVE` → `CommandType::SetValvePosition`
  - `CAN_ID_COMM_PING` → `CommandType::Ping`
  - `CAN_ID_COMM_PONG` → `CommandType::Pong`
  - `CAN_ID_SYSTEM_STATE` → `TelemetryId::SystemState`
  - `CAN_ID_STATUS_VALVE` → **removed** (valve status now lives inside the SystemState record)
  - `CAN_VALVE_1/2` → `EcuValves::IPA/NOS`
  - `CAN_CMD_OPEN/CLOSE` → `ValveCommand::Open/Close` (from `command/set_valve_position.hpp`)
  - `CANHeader` (type) → `CanHeader`
- **Threading rule:** typed enums travel through signatures; cast to `uint8_t` only at the
  wire/bitfield write (and at the platform `Can::init` boundary). Applied to
  `packSystemState(BoardId,BoardId,...)` already.

## DONE in step 3 — compile fixes already applied (green individually)
- `communication/command/parser/command_can_parser.cpp` — `CanHeader` union access (`header.code`, `header.frame.*`).
- `tests/.../command/command_test.cpp` — `makeCanFrame` uses `CanHeader` union.
- `framing/system_state_codec.hpp` — added `telemetry_id.hpp` + `board_ids.hpp` includes; `packSystemState` takes `BoardId`; messageID/senderID/targetID cast; reassembler check casts to `TelemetryId`.
- `tests/.../can/system_state_codec_test.cpp` — `pack()` passes `BoardId::Engine/FillingStation`.
- `fcu_controller.tpp` — added `using logic::communication::command::CommandType;`; `sendValveCmd` casts; `handleDatagram` device→`BoardId` + `switch(static_cast<CommandType>(payloadID))`; `sendToGs` signature → `BoardId` + body casts.

---

## ⏭️ REMAINING to get back to GREEN (do these first tomorrow)
Work the host-test compiler top-down (`cmake -P src/app/tests/run.cmake`); known items:

1. **`fcu_controller.hpp:178`** — `sendToGs` declaration still `uint8_t sourceId`; the edit
   to make it `BoardId sourceId` **FAILED** (file-modified race). Redo it.
2. **`fcu_controller_test.cpp`** (many):
   - `makeCanFrame(uint8_t sender, uint8_t target, ...)` is called with `BoardId` (lines ~299–308) →
     change params to `BoardId`, cast to `uint8_t` at the bitfield writes inside.
   - `requestState(State, uint8_t device=...)` and `makeStateRequest(uint8_t device, ...)` are called
     with `BoardId` (lines ~116, 250, 257) → thread `BoardId device`; cast at `deviceID` write.
   - `payloadID = CommandType::SetState` (line ~50) → `CommandType` not in scope in the test +
     needs `static_cast<uint8_t>` (add a `using` or qualify).
   - `EXPECT_EQ(header.deviceID, BoardId::FillingStation)` (153) and
     `EXPECT_EQ(header.payloadID, TelemetryId::SystemState)` (154) → cast the enum to `uint8_t`.
3. **`ecu_controller.tpp` + `.hpp`** — full pass not done yet:
   - add `using logic::communication::command::CommandType;` (namespace `logic::ecu`).
   - includes: `command/command_type.hpp`, `command/set_valve_position.hpp` (for `ValveCommand`),
     `framing/can_header.hpp` (already via sed), `system/valves/ecu.hpp` (already).
   - `handleCanFrame`: `switch(static_cast<CommandType>(header.frame.messageID))` with
     `case CommandType::SetValvePosition` / `case CommandType::Ping`; targetID compare via `BoardId` cast.
   - `handleValveCmd`: `valve_idx` compare via `static_cast<EcuValves>`; `switch(static_cast<ValveCommand>(deviceState))` with `case ValveCommand::Open/Close`.
   - `handlePing`: cast `senderID`/`messageID` writes (`BoardId::Engine`, `CommandType::Pong`).
4. **`board.cpp` ×2** — `g_can.init(&hfdcan1, BoardId::Engine|FillingStation)` → wrap in
   `static_cast<uint8_t>(...)` (`Can::init` stays `uint8_t` — platform boundary).
5. Re-run host tests to green, then build **both** firmware targets and fix any stragglers.

---

## On-wire value table (for the GS — these are the new SSOT values)
- **BoardId:** Engine 0x01, FillingStation 0x02, GsControl 0x03, Broadcast 0x04.
- **CommandType** (CAN messageID / Ethernet payloadID, command frames): Ping 0x01, SetState 0x02, SetValvePosition 0x03, Synchronise 0x04, Pong 0x05.
- **TelemetryId** (same field, telemetry frames): SystemState 0x01.
- **State:** Init 0x00 … Test 0x06.
- **GS-facing wire changes to propagate:** `REQUEST_STATE 0x83 → SetState 0x02`;
  `GET_SYSTEM 0x03 → SystemState (telemetry) 0x01`. (CommandType/TelemetryId share the
  id field but are disambiguated by direction: boards consume commands, GS consumes telemetry.)

## Open design questions (parked)
- **CanHeader representation.** `FrameCanHeader` uses `uint32_t name:4` bitfields, so every
  wire write casts `BoardId/CommandType → uint8_t`. Consider whether a different layout is
  cleaner. NB: the 29-bit CAN id genuinely needs bit-packed sub-fields, so a flat `uint8_t`
  struct can't replace it without losing the packing — but the *accessor* ergonomics (the
  casts) could be revisited (helper setters, or `enum`-typed bitfields where a field has a
  single meaning — though `deviceState` is overloaded for the valve command, which blocks that).
- **`can_types` split.** Original intent was to split `can_types` across files; it's currently
  collapsed (header → `can_header.hpp`, enums → deleted/reused). Revisit if more CAN-only types appear.
- **Legacy `Inc/can/*`** were deleted as dead — confirm nothing external depended on them.

## After GREEN: commit plan
One commit (one sentence, no trailer) bundling: the protocol sort, the dead-folder deletion,
the typed-enum standardization, the id consolidation, and the updated plan docs
(`common-protocol-submodule-plan.md`, this file). Then the actual `common-protocol` submodule
extraction is the next milestone, followed by bringing the SystemState packet to the GS.
