# `logic/ecu` — ECU board-type logic (placeholder)

HAL-free FCU-counterpart logic for the **ECU**: its controller, state machine,
command handlers, and telemetry routing (records → CAN → FCU → GS; no SD/UDP path).

Empty for now — the ECU currently builds as a do-nothing stub
(`src/boards/ecu/`). This is where the ECU's `logic::ecu::Controller` and friends
land as it is built out, mirroring `logic/fcu` and composing `logic/common`.

Tests will live in `logic/ecu/tests/` (one site per board), wired into the shared
harness at `src/app/tests/`.
