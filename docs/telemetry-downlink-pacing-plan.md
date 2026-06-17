# Telemetry downlink pacing — implementation plan

## Problem (confirmed root cause)

Adopting the 16 KB SD log block (`SD_LOG_BLOCK_BYTES` 4096 → 16384, commit `1cb5f2e`)
quadrupled the telemetry ring slot, because the slot size is hard-tied to the SD block:

```
ecu/telemetry.hpp:50   LOG_HALF_BYTES = logic::telemetry::SD_LOG_BLOCK_BYTES
fcu/telemetry.hpp:58   LOG_HALF_BYTES = logic::telemetry::SD_LOG_BLOCK_BYTES   // log_ AND ecu_log_
```

`drain()` downlinks a **whole slot** of records in one synchronous burst. With 16 KB blocks
that burst is now ~292 records (was ~72):

- `sizeof(EcuSystemState)` = `sizeof(SystemStateBase)` = **56 B**; `SYSTEM_STATE_FRAGMENTS` = 1 frame/record.
- records/slot = `(SD_LOG_BLOCK_BYTES - 16) / 56` → 16 KB: **292**, 4 KB: **72**.

The ECU CAN driver's telemetry software ring is **192 frames** (`can_dil.cpp:68
TX_RING_LO_FRAMES`). drain() pushes 292 frames far faster than the 2 Mbit wire drains, the
192-deep ring saturates mid-burst, and `Can::send()` drops the overflow (best-effort, DAR on).
Effective rate ≈ 192/292 × 2000 ≈ **~1320–1400 states/s** — matches the observed regression.

**Not** caused by the physical card being slower; the block-size bump rode in with the swap.

## STATUS: IMPLEMENTED (backpressure + buffer expansion, not count-pacing)

The originally-planned "fixed records-per-tick cap" was discarded during implementation: the main
loop free-spins (no delay), so a fixed cap just dumps the same ~292-record slot across a handful of
back-to-back iterations and overflows the ring all the same. The as-built fix is **honor
backpressure** + **expand the buffers**, which are complementary (and crucially NOT a throttle —
they prevent drops, never send slower than the link can carry):

- **Backpressure (no-drop safety net).** `drain()` advances a byte cursor into the tail slot and
  sends records until the link *rejects* one (CAN TX ring full / Ethernet TX busy), then yields,
  holding the cursor so the next tick resumes the same record. Nothing is dropped; the cursor only
  advances on accepted frames. A separate `tail_recorded_` flag SD-logs each slot exactly once,
  independent of downlink pacing.
- **Buffer expansion (raise the burst ceiling so backpressure rarely engages).** ECU CAN TX LO
  ring `TX_RING_LO_FRAMES` 192 → **320** (holds one whole 16 KB slot's ~292 records + margin), so a
  drain burst flushes in one go at full rate. Expand further / expand the FCU CAN-RX + ETH paths if
  a deeper backlog must ride without pacing — the buffers are the knob, throttling is off the table.

The only hard ceiling is raw wire bandwidth (~3400 fps for 64 B FD frames at the 1/2 Mbit timing;
2000/s ≈ 60 % load — verify on the bench). memcpy cost is irrelevant (telemetry < 1 MB/s vs a
~50 MB/s budget); backpressure adds zero copies.

### As-built `drain(now)` (both boards)
1. If `!ready[tail]` → return (FCU loops over ready slots).
2. If `!tail_recorded_` → `recorder_.recordSystemState(...)` once; set `tail_recorded_`. (Footer
   stamping writes `[used, end)`; downlinked records are `[0, used)`, so order is independent.)
3. Send whole records from `downlink_off_` until the link rejects one → return (cursor held); else
   advance the cursor.
4. Slot fully sent → `ready[tail] = false`, advance `tail`, reset `downlink_off_` + `tail_recorded_`.

The ECU threads `comm_.sendFrame` → `optional<CanError>` and `sendRecordCan` → bool; the FCU threads
`comm_.sendToGs` → `optional<NetError>` and `downlink(..., start_off)` → reached offset. Both rings
on the FCU (`log_` via `drain`, `ecu_log_` via `drainRelayedEcu`) carry their own cursor.

## ECU changes

- `src/app/logic/ecu/telemetry.hpp`
  - Add `MAX_DOWNLINK_RECORDS_PER_TICK` (in `detail`), retire/repurpose `MAX_DRAIN_SLOTS_PER_TICK`.
  - Add `std::size_t downlink_off_ = 0;` member; reset it in `init()`.
  - Rewrite `drain()` per the design above.
- No CAN-driver change needed — the 192-deep ring + ISR pump is correct once we stop over-feeding it.

## FCU changes (do at the same time — same coupling)

The FCU shares the slot=SD-block coupling and has **two** drained rings: `log_` (own telemetry,
`drain()`) and `ecu_log_` (relayed ECU, `drainRelayedEcu()`); both currently `while (ready)`
**unbounded**. Its transport differs — `downlink()` batches records into ~1.4 KB UDP datagrams
(GS_PAYLOAD_CAPACITY ≈ 1432 − header − CRC), so a 16 KB slot is ~12 datagrams/drain (was ~3).

- `src/app/logic/fcu/telemetry.hpp`
  - Pace both `drain()` and `drainRelayedEcu()` — bound datagrams (or records) per tick, with a
    cursor per ring (`downlink_off_`, `ecu_downlink_off_`). Reset both in `init()/clearLog`.
- **Open decision (confirm by measurement):** `Ethernet::send()` (`ethernet.cpp:616`) is
  *single-in-flight* — one shared `tx_udp_data` buffer + `HAL_ETH_Transmit_IT`; returns
  `HAL_BUSY` when a transmit is still in flight, and `downlink()` ignores the return
  (`(void)eth_.send(...)`), so a burst that outruns the ~115 µs/frame TX silently drops the tail.
  Pacing reduces the burst, but the real fix may be to **honor the Busy return**: on Busy, stop
  draining this tick and retry the same datagram next tick (do NOT advance the cursor). Decide
  between (a) pacing only, (b) pacing + honor-Busy, or (c) a small TX datagram ring, after
  measuring the FCU's actual GS rate. Lean (b) — cheap and correct.

## Tests to update

- `src/app/logic/ecu/tests/unit/logic/ecu_controller_test.cpp` — already pumps `tick()` N times
  and was scaled to the block size in `1cb5f2e`. Paced draining means a full slot now needs
  ⌈records/slot ÷ N⌉ ticks to flush; update the pump counts / assertions accordingly. Add a test
  asserting no single `tick()` enqueues more than `MAX_DOWNLINK_RECORDS_PER_TICK` to the CAN fake.
- `src/app/logic/fcu/tests/unit/logic/fcu_controller_test.cpp` — same: bump pump counts for the
  paced drain; if honoring Busy, add a fake-eth Busy path test (datagram retried, not dropped).
- Check the SystemState codec / recorder tests still pass unchanged (no format change here).

## Stale comments to fix (independent of the logic, do in the same PR)

- `can_dil.cpp:52-54` — "~73 records = ~73 FD frames" (4 KB era) → 16 KB numbers.
- `ecu/telemetry.hpp:66` `MAX_DRAIN_SLOTS_PER_TICK` block — "4096 B slot holds ~60 records"; the
  whole rationale rewrites with the new pacing.
- `sd_block_footer.hpp:22-24, 71-74` — comments still say "4096" / "fixed 4096-byte fast block".
- `fcu/telemetry.hpp` drain/relay comments referencing whole-half downlink.

## Verification

1. Build both boards (`ecu_CM7`, `fcu_CM7`) + run the host unit tests.
2. Bench: confirm ECU GS rate back to ~2000 states/s; watch `CanInfo` tx-error / LO-lane-full
   count → ~0. Confirm SD logging throughput unaffected (engine overrun count steady).
3. Bench: measure FCU GS rate (was unverified) before/after; watch `EthernetInfo.tx_error` /
   tx_busy. Decide the FCU Busy-handling option from this data.
4. Sweep `MAX_DOWNLINK_RECORDS_PER_TICK` if needed; keep the deep telemetry ring (LOG_BUFFER_COUNT=4)
   as the stall absorber — pacing handles the burst, the ring handles the stall.

## Quick facts to re-confirm at start (don't trust from memory)

- `sizeof(EcuSystemState)` = 56, `sizeof(FcuSystemState)` (= 56 + `sizeof(EthernetInfo)`),
  `GS_PAYLOAD_CAPACITY`, `SYSTEM_STATE_FRAGMENTS` (= 1).
- `TX_RING_LO_FRAMES` = 192 (ECU CAN), ETH TX in-flight depth (FCU).
- That nothing else reads `LOG_HALF_BYTES`/`MAX_DRAIN_SLOTS_PER_TICK` we'd break.
</content>
</invoke>
