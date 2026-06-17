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

## Chosen fix: pace the per-tick downlink burst (no buffer decoupling, zero extra copies)

Bound how many records `drain()` hands to the wire per main-loop iteration, carrying a
byte-offset cursor into the tail slot across ticks. The loop runs thousands of times/sec, so a
small per-tick bound keeps up with the 2 kHz producer while never exceeding the ring depth in
one tick. Rejected alternatives: enlarging `TX_RING_LO_FRAMES` (papers over the burst, costs
~23 KB AXI-SRAM, still fragile to future block bumps); shrinking the telemetry slot below the SD
block (forces a separate SD accumulator — works, but more moving parts than pacing).

memcpy cost of pacing is **zero** — it reorders *when* the existing per-record copies happen, not
how many. (Even a true size-decouple would add only ~112 KB/s of copies vs a ~50 MB/s budget;
irrelevant. The constraint is the 2 Mbit CAN wire, not the CPU.)

## Design

The slot stays one buffer with two readers. SD recording is unchanged (whole-slot, once). Only
the downlink is paced.

Per drain ring, add a `downlink_off_` cursor (bytes already downlinked from the tail slot) and a
per-tick record bound. `drain(now)` becomes:

1. If `!ready[tail]` → return.
2. If `downlink_off_ == 0` (first touch of this slot) → `recorder_.recordSystemState(...)` once.
   (Footer stamping writes `[used, end)`; the downlinked records are `[0, used)`, so order is
   independent — SD-record first or after, doesn't matter.)
3. Send up to `MAX_DOWNLINK_RECORDS_PER_TICK` whole records starting at `downlink_off_`; advance
   the cursor.
4. When the cursor reaches `used` (no whole record left) → `ready[tail] = false`,
   advance `tail`, reset `downlink_off_ = 0`.

This replaces the current `MAX_DRAIN_SLOTS_PER_TICK = 1` slot-granularity bound with a
record-granularity bound (strictly more responsive — also subsumes the "yield to command
handling" rationale in that comment).

### Picking `MAX_DOWNLINK_RECORDS_PER_TICK`
- Upper bound: must not overflow the ring in one tick → comfortably `< TX_RING_LO_FRAMES` (192).
- Lower bound: must keep up with 2 kHz → `N × min_loop_rate ≥ 2000`. The loop runs well above
  1 kHz, so even N=16 keeps up; bigger N just drains backlog faster after a stall.
- Proposal: **N = 64** (≈ ⅓ of the ring; ample headroom both ways). Tune on the bench.
- Sanity: average downlink can't exceed the 2 kHz produce rate anyway — pacing only kills the
  *burst*, it doesn't throttle steady-state.

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
