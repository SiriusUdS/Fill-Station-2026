# Plan: SD-card block footers, extended-state SD status, and a deeper log-buffer ring

## Goal

Three related changes to the SD telemetry-logging path, on **both boards** (FCU + ECU):

1. **Block footer** — append a footer to every block saved to the card, carrying the **saving
   timestamp**, a **MAGIC** marker (so a reader can re-find block boundaries after local
   corruption), the **payload length**, and a **CRC-32**. The footer format **and the reader-side
   helper** live in the `common-protocol` submodule so offline tooling can parse the files without
   any firmware/platform dependency.
2. **SD status in the extended state** — surface `StorageInfo` (SD state + status + overrun count)
   in the low-rate `FcuExtendedSystemState` / `EcuExtendedSystemState`, not only in the 2 kHz record.
3. **Deeper buffer ring** — the telemetry double buffer (currently **2** halves) overruns under SD
   sync-stall bursts. Generalise it to an **N-buffer ring** to absorb the bursts.

---

## Background facts (verified in the code)

- **Sync cadence:** `SYNC_PERIOD_WRITES = 16` (`src/app/platform/storage/sd_card.cpp:37`, used at
  `:140`). `f_sync()` (FAT + dir-entry flush) runs once per **16** `write()` calls; the first write
  always syncs. Each `write()` = one block (4096 B on `data_fast`, variable on `data_slow`/`data_ext`).
  A sync is the latency-spiky part of a save — a sync stall is the main overrun trigger.
- **Block unit:** `SD_LOG_BLOCK_BYTES = 4096` (`src/app/logic/common/telemetry/sd_recorder.hpp:37`),
  sector-aligned; the whole half is handed to `recordSystemState()` and written verbatim for
  `data_fast`.
- **CRC seam:** `logic::data_integrity::crc32(const uint8_t*, size_t)` — zlib/Ethernet variant,
  HW-backed on firmware, software impl linked in host tests. Foreground-only today, so a footer CRC
  computed in `drain()` / `produceExtended()` has no ISR contention. The reflected polynomial
  constant (`CRC32_POLYNOMIAL_REFLECTED = 0xEDB88320`) already lives in the submodule
  (`.../protocol/system/crc32_polynomial.hpp`).
- **`StorageInfo`** (4 B: `StorageState` + `StorageStatus` + `uint16 overrun_count`) already rides
  the high-rate `SystemStateBase.storage_info`, produced from `recorder_.health()`; it is **not** in
  the extended state yet.
- **Double buffer** (`src/app/logic/fcu/telemetry.hpp`, mirrored in `logic/ecu/telemetry.hpp`):
  `LogBuffer{ data[2][4096], used[2], ready[2], active, overrun_count }`. `logAppend` (record-timer
  ISR) fills `active`; on full it finalises and flips; if the other half is still `ready` (undrained)
  the record is dropped and `overrun_count` saturates. `drain` (foreground tick) flushes ready halves.
  The FCU has two of these: `log_` (its own records → SD) and `ecu_log_` (relayed ECU records → GS,
  not SD).

## Current state

| Concern | Today |
|---|---|
| Footer / magic on disk | none — raw record stream in `data_fast/slow/ext.bin` |
| SD status in extended state | absent (only in the 2 kHz `SystemStateBase`) |
| Log buffers | fixed **2** (double buffer), overruns under sync stalls |
| Footer owner | n/a |
| Submodule reader | n/a |

---

## Part A — `StorageInfo` in the extended system state

1. **Submodule** (`telemetry/fcu_extended_system_state.hpp`, `telemetry/ecu_extended_system_state.hpp`):
   add `StorageInfo storage_info;` and extend the `sizeof` layout guard by `+ sizeof(StorageInfo)`.
   `#include "peripherals/storage/storage_info.hpp"` (already in the submodule).
2. **FCU `telemetry.hpp::produceExtended`** (and ECU equivalent): set
   `ext.storage_info = recorder_.health();` then `ext.storage_info.overrun_count = log_.overrun_count;`
   (mirrors `buildSystemState`).
3. Redundant with the high-rate record by design — the extended record is the low-rate "dashboard"
   stream and is where the GS reads slow-changing status.

---

## Part B — Per-block footer (timestamp + MAGIC + payload length + CRC)

### On-disk format (writer)

Footer = **MAGIC fill** + a **12-byte trailer**, anchored at the **end** of each SD write:

```
trailer (last 12 B):  uint32 saved_ms · uint32 payload_bytes · uint32 crc32
magic fill (before):  uint32 SD_BLOCK_MAGIC, repeated to consume the slack
```

- **Fast 4096 block:** `[records = payload_bytes][MAGIC × k][trailer 12B]` summing to **exactly
  4096** — the magic fill absorbs the leftover between the last whole record and the trailer, so the
  block is fully used (no dead/zero padding). `k = (4096 − payload_bytes − 12) / 4`. With fixed-size
  records and full-half draining, `payload_bytes` (and `k`) are constant per board in steady state.
- **slow / ext (variable writes):** one magic word + trailer = **16 B** appended (no block to fill).
- `crc32` = zlib CRC over everything written except the crc word itself
  (`payload + magic-fill + saved_ms + payload_bytes`), so the trailer metadata is integrity-checked.
- `SD_BLOCK_MAGIC = 0x5D10F007` (arbitrary-but-fixed, little-endian on disk).
- **SD-only:** the GS downlink still sends `payload_bytes` of records — wire format unchanged.

### Capacity reservation

- Add `SD_BLOCK_TRAILER_BYTES = 12`, `SD_BLOCK_MIN_FOOTER_BYTES = 16` (≥1 magic word + trailer),
  `SD_BLOCK_PAYLOAD_CAP = SD_LOG_BLOCK_BYTES − SD_BLOCK_MIN_FOOTER_BYTES` (= 4080).
- `logAppend` flips the buffer at `SD_BLOCK_PAYLOAD_CAP` instead of `LOG_HALF_BYTES`, guaranteeing
  `used ≤ 4080` so the footer always fits inside the 4096 block.

### Ownership / wiring

- The **`SdRecorder`** computes and stamps the footer (it owns write policy, `now_ms`, and the CRC
  seam). Platform `SdCard::write()` stays an untouched byte sink.
- Thread `now_ms` into `recordSystemState(...)` and `recordExtended(...)`; `drain` passes the half as
  a **mutable** span so the recorder can stamp the magic-fill + trailer in place for the fast path.
  `slow` appends into its existing `out` buffer (sized `+16`); `ext` concats into a small scratch
  member then writes once (one write = one block keeps sync accounting clean).

### Reader (in the submodule — "what is needed to read")

New submodule header, e.g. `telemetry/sd_block_footer.hpp`, self-contained so offline readers need
nothing from firmware/platform:

- `struct SdBlockTrailer { uint32_t saved_ms, payload_bytes, crc32; }` + `SD_BLOCK_MAGIC` +
  `SD_BLOCK_TRAILER_BYTES` / `SD_BLOCK_MIN_FOOTER_BYTES` / `SD_LOG_BLOCK_BYTES` shared constants.
- A header-only **software** CRC-32 (zlib, using the submodule's `CRC32_POLYNOMIAL_REFLECTED`) and a
  verify/parse helper, e.g.:
  - `sdBlockReadTrailer(span) -> std::optional<SdBlockTrailer>` (checks magic run + CRC),
  - `sdBlockPayload(span) -> std::span<const uint8_t>` (the `payload_bytes` record region),
  - `sdBlockScanForMagic(span, from) -> offset` for resync after corruption.
- Writer (firmware, HW CRC) and reader (software CRC) must agree on the zlib variant — they already
  do for the telemetry frame CRC, so the equivalence is established.

---

## Part C — Deeper log-buffer ring (fix overruns)

Generalise the 2-slot double buffer to an **N-slot ring** (single-producer ISR / single-consumer
foreground), in both boards' `telemetry.hpp`:

```cpp
inline constexpr std::size_t LOG_BUFFER_COUNT = 4;   // was 2; tune (see sizing)

struct LogBuffer {
    uint8_t           data[LOG_BUFFER_COUNT][LOG_HALF_BYTES];
    volatile uint16_t used [LOG_BUFFER_COUNT];
    volatile bool     ready[LOG_BUFFER_COUNT];
    uint8_t           head;          // producer fills data[head]
    uint8_t           tail;          // consumer drains from data[tail]
    volatile uint16_t overrun_count;
};
```

- **Producer (`logAppend`):** fill `head`; when `used[head] + record > SD_BLOCK_PAYLOAD_CAP`, mark
  `ready[head]`, advance `head = (head+1) % N`; if the new `head` is still `ready` (consumer behind),
  drop the record and saturate `overrun_count`. Order-preserving via head/tail (vs the old XOR flip).
- **Consumer (`drain`):** while `ready[tail]`, flush `data[tail]`, clear `ready[tail]`, advance
  `tail = (tail+1) % N`. Drains in fill order (records stay chronological on disk).
- `LOG_HALF_BYTES` stays `SD_LOG_BLOCK_BYTES` (4096) per slot.
- Apply to `log_` (SD path) and `ecu_log_` (relay path) — both overrun under load. `clearLog`
  zeroes the new index fields.

### Sizing

- RAM cost: `N × 4096` per `LogBuffer`. FCU holds two (`log_` + `ecu_log_`) → `2 × N × 4096`,
  pinned in D1 AXI-SRAM (the controller instance's `.axisram` placement). `N = 4` → 32 KB on FCU;
  **confirm against the AXI-SRAM budget** before raising further.
- N should cover the longest expected drain gap (a worst-case `f_sync` stall) worth of fills:
  `N ≳ ceil(sync_stall_ms × fill_rate_blocks_per_ms) + 1`. Start at **4**, measure `overrun_count`
  via the new extended-state `storage_info`, raise if still nonzero. (A complementary lever is the
  sync cadence / batching, out of scope here.)

---

## Touch list

**Submodule (`common-protocol`):**
- `telemetry/sd_block_footer.hpp` — new: trailer struct, magic, size constants, software-CRC +
  reader/verify helpers.
- `telemetry/fcu_extended_system_state.hpp`, `telemetry/ecu_extended_system_state.hpp` —
  `storage_info` field + layout guard.

**Parent (`Fill-Station-2026`):**
- `logic/common/telemetry/sd_recorder.hpp` — footer stamping for all three files; mutable 4096 block
  param; `now_ms` params; CRC over payload+footer.
- `logic/fcu/telemetry.hpp` + `logic/ecu/telemetry.hpp` — N-slot ring (`LogBuffer`, `logAppend`,
  `drain`, `clearLog`); `logAppend` cap → `SD_BLOCK_PAYLOAD_CAP`; pass mutable block + `now_ms` to
  the recorder; set `storage_info` in `produceExtended`.
- `platform/storage/sd_card.*` — unchanged (footer is above the byte sink).

---

## Testing

- Submodule: layout `static_assert`s; reader round-trips a stamped block (magic-fill makes 4096
  exactly; trailer fields correct; CRC validates and flips on a mutated byte; `slow`/`ext` 16 B
  footer; `sdBlockScanForMagic` resyncs past a corrupted prefix).
- Recorder unit tests over `FakeStorage` (capture written bytes): fast block is exactly 4096 with
  footer; slow/ext append 16 B; payload/CRC correct.
- Per-board controller tests: `storage_info` rides the extended record; ring absorbs more than 2
  outstanding blocks before `overrun_count` increments.

## Scope / sequencing

- Two-commit change (submodule + parent), like the heater work: commit + push the submodule first,
  then bump the pointer in the parent.
- Suggested order: Part C (ring) → Part B (footer + reader) → Part A (extended status), so the ring
  and footer capacity reservation land together before wiring telemetry.
