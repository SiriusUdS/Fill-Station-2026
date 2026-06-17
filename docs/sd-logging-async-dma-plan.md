# Plan: Non-blocking SD logging via pre-allocated raw-sector DMA (level 3)

## Goal

Make SD saves faster and get the **control loop off the SD entirely** (no `f_write`/`f_sync` stall
on the foreground), **without an RTOS**. The 2 kHz producer keeps filling the telemetry ring; the
consumer must drain to the card without ever blocking the valve/control work or overrunning the ring.

This supersedes the batched-`f_sync` mitigation from
[`sd-logging-footer-buffers-plan.md`](sd-logging-footer-buffers-plan.md): instead of *reducing* the
FAT-flush stall, level 3 *removes* it — the FAT is written once up front by `f_expand`, and the run
writes raw sectors by DMA.

**Status: core implemented + both boards (FCU/ECU) build + link clean. CubeMX NVIC done on both.
Space-reclaim trigger DONE (GS `DisableLogging` flag → recorder finalize, host-tested). Engine-health
telemetry DONE (in ExtendedSystemStateBase, both boards, host-tested). ECU port DONE (links + wired).
The main-loop engine advance is `tick()` (project convention), not `service()`. Remaining: pre-alloc
sizing + on-target HW bring-up validation.**

---

## Why blocking was unavoidable in the old design (verified)

- Write path was **polled**: `SdCard::write()` → `f_write` → `disk_write` → `SD_write` (`sd_diskio.c`)
  → `BSP_SD_WriteBlocks` (`HAL_SD_WriteBlocks`, polling) → busy-spin on `BSP_SD_GetCardState()`.
- Two stalls: (1) CPU copies every byte into the SDMMC FIFO; (2) the **`f_sync`** FAT/dir flush —
  the multi-ms latency spike that overran the ring.
- **FatFs `disk_write` is synchronous by contract** — it cannot return before the sectors commit. So
  no amount of DMA/queueing *inside* FatFs makes `f_write` non-blocking. Truly non-blocking requires
  bypassing FatFs on the hot path → pre-allocate + raw-sector DMA. That is this plan.

## Gating facts (verified in the code, 2026-06-17)

- **CM7 D-cache is OFF** — only `SCB_EnableICache()` runs (`board.cpp:53`); no `SCB_EnableDCache`.
  → **No DMA cache-coherency work needed.** (If D-cache is ever enabled, `enqueue()` must
  `SCB_CleanDCache_by_Addr` the copied block before the DMA, or the queue must be in a non-cacheable
  MPU region. The MPU today has only region 0, a 4 GB background — `board.cpp:414`.)
- **H7 SDMMC uses internal IDMA** — `HAL_SD_WriteBlocks_DMA` drives it; no separate DMA stream to
  configure. IDMA reaches D1 AXI-SRAM (where the buffers live), not DTCM.
- **`f_expand` / `f_truncate` / `f_lseek` all available**: `_USE_EXPAND 1` (set via CubeMX),
  `_FS_MINIMIZE 0`, `_FS_READONLY 0` (`ffconf.h`).
- **Cluster→LBA formula** (FatFs R0.12, `_FS_EXFAT 0`): `base_lba = (sclust-2)*csize + database`,
  fields on `FIL.obj.fs` / `FIL.obj.sclust` (`ff.c:990`, `ff.h:133-149`).
- **One physical card, three files** (`data_fast/slow/ext`) on one SDMMC2 → only one DMA write in
  flight; needs a single shared arbiter.
- **Block unit = 4096 B = 8 sectors**; every `write()` is exactly one footer-stamped block
  (`SD_LOG_BLOCK_BYTES`, the on-disk SSOT). Drain (`telemetry.hpp:172`) and the slow/ext accumulators
  (`sd_recorder.hpp`) all hand whole 4096-B blocks.
- **Buffer ownership**: drain frees the ring slot right after `write()` returns, and the recorder
  reuses its slow/ext accumulators immediately → a deferred (async) write must **copy** the block.
  (Decision below.)

---

## Architecture (implemented)

```
2 kHz ISR ─ produce() ─► telemetry ring (log_)                       [unchanged]
main loop ─ drain() ──► recorder.recordSystemState(block,4096)       [unchanged policy]
                          └─► SdCard::write(block)
                                ├─ compute LBA = base_lba_ + sector_cursor_
                                ├─ engine.enqueue(LBA, block)   ← COPIES into AXI-SRAM ring, returns
                                └─ sector_cursor_ += 8          (only if enqueued)
main loop ─ sd_write_engine().tick()  ← if !busy && card ready: HAL_SD_WriteBlocks_DMA(next)
SDMMC2 ISR ─ HAL_SD_IRQHandler ─► TxCplt ─► BSP_SD_WriteCpltCallback ─► engine.onComplete() (free slot)
                                  └─ Error ─► HAL_SD_ErrorCallback ─► engine.onError()
```

- **No `f_write`/`f_sync` during the run.** FAT/dir written once by `f_expand` at init.
- Control loop cost per iteration: one quick `HAL_SD_GetCardState` (CMD13) poll + maybe one DMA kick.
  No byte copying in the loop (copy happens in `enqueue`, which is on the drain path), no busy-spin.
- **Concurrency**: single-producer (`enqueue`+`tick`, main loop) / single-completer
  (`onComplete`/`onError`, ISR). Exactly one block in flight, gated by `busy_`: while `busy_==false`
  no DMA is outstanding so the ISR can't fire and the producer runs lock-free; while `busy_==true`
  `tick()` no-ops. `head_` touched only by producer; `tx_` only by completer.

### Decisions taken

- **Level 3** (raw-sector DMA), not the cheaper "DMA + decouple" — user chose true non-blocking.
- **Copy into the engine queue** (not ring-slot handoff) — keeps the `Storage` seam clean and
  telemetry/recorder untouched; costs `SD_WRITE_QUEUE_DEPTH × 4096` AXI-SRAM (8 × 4096 = 32 KB) + a
  ~1–2 µs memcpy per block. (Ring-slot handoff would be zero-copy but rewrites `telemetry.hpp`
  drain/release and the recorder accumulators — rejected as fragile.)
- **CubeMX for generated-file config** (NVIC, `_USE_EXPAND`) so regen doesn't clobber it — see memory
  `cubemx-config-over-hand-edit`.

---

## What is DONE (this session)

**New files (`src/app/platform/storage/`):**
- `sd_write_engine.hpp` / `.cpp` — `SdWriteEngine`: the single shared async arbiter.
  - `init(handle)`, `enqueue(lba, block)` (copy + return; drops + counts on full ring),
    `tick()` (kick next when idle + card-ready), `onComplete()` / `onError()` (ISR),
    `idle()`, `overrun_count()`, `errored()`.
  - Constants: `SD_WRITE_BLOCK_BYTES (=SD_LOG_BLOCK_BYTES)`, `SD_SECTOR_BYTES 512`,
    `SECTORS_PER_BLOCK 8`, `SD_WRITE_QUEUE_DEPTH 8`.
  - Single instance `s_engine` in `.axisram` (NOLOAD, trivial ctor → no static init); accessor
    `sd_write_engine()`.
  - C-linkage overrides at file end: strong `BSP_SD_WriteCpltCallback` (overrides weak in
    `bsp_driver_sd.c`) → `onComplete`; strong `HAL_SD_ErrorCallback` (overrides weak HAL) → `onError`.

**Rewritten `sd_card.hpp` / `.cpp`:**
- `bind(handle, drive, filename, prealloc_bytes)` — **replaced** `sync_period_writes` with
  `prealloc_bytes` (default 64 MiB).
- `init()` — `beginSession` (mount + per-boot folder, unchanged) → `f_open(FA_CREATE_ALWAYS|FA_WRITE)`
  → `f_expand(reserve, 1)` → resolve `base_lba_`, `capacity_sectors_`, `sector_cursor_=0`. Errors →
  `fail(...)`. (`beginSession`/`nextSessionNumber` unchanged.)
- `write(span)` — must be exactly 4096 B; stop if extent full; else `enqueue` and advance cursor.
  Never touches FatFs, never blocks.
- `finalize()` — **new**: spin `tick()` until `engine.idle()`, then `f_lseek(written)` +
  `f_truncate()` + `f_sync()` to reclaim the unused pre-alloc tail. **Not called anywhere yet.**
- `static_assert(logic::storage::Storage<SdCard>)` still holds (init/write/info unchanged signatures;
  `finalize` is extra). Host `FakeStorage` needs no change.

**Wiring:**
- `board.cpp` (`wireDrivers`): `platform::storage::sd_write_engine().init(&hsd2)` **before** the
  binds + `g_controller.init()`; three `bind()`s now pass pre-alloc sizes (fast 512 MiB / slow 32 /
  ext 8 MiB).
- `main.cpp`: `platform::storage::sd_write_engine().tick()` at the **end** of the for(;;) loop
  (after valve + controller ticks, so SD never delays actuation). Updated the stale "double buffer →
  DMA" comment.
- `CM7/CMakeLists.txt`: added `sd_write_engine.cpp` next to `sd_card.cpp`.
- **CubeMX (done by user):** `_USE_EXPAND = 1`; **SDMMC2 global interrupt enabled in NVIC** (generates
  the NVIC enable in `sdmmc.c` MspInit + `SDMMC2_IRQHandler` → `HAL_SD_IRQHandler(&hsd2)` in
  `stm32h7xx_it.c`). Recommended preemption priority **8** (below TIM6=6 and ADC DRDY EXTI).

**Build:** `cmake --build` on FCU CM7 links clean. RAM_D1 (AXI-SRAM) 16.58 % of 512 KB (incl. the new
32 KB write ring). No warnings surfaced.

---

## What is LEFT (resume here)

1. **Space reclaim / `finalize()` trigger — DONE (GS `DisableLogging` flag).**
   Chose the **GS "stop logging" command** option. A new BASE control flag
   `ControlFlagBase::DisableLogging` (id 1, default clear → logging active) drives it, set via the
   existing `SetControlFlag` path (no new command type). The **shared recorder** reads the flag in
   both `recordSystemState`/`recordExtended`: the first record seen with it set flushes the partial
   slow/ext blocks, then calls `finalize()` on all three files (truncate the pre-alloc tail back to
   bytes written) and latches `finalized_`. It is **one-way for the session** — once finalized the
   tail is reclaimed and the raw-sector cursor cannot safely advance into freed space, so clearing
   the flag does NOT resume logging (a reboot does). `finalize()` is now part of the
   `logic::storage::Storage` concept (added to `FakeStorage` too). Both boards get this for free
   (shared logic); the operator stops a board's logging by addressing the flag to it (or Broadcast).
   Host-tested (`SdRecorderTest.DisableLoggingFinalizesAllFilesAndStops`); FCU firmware links clean.
   Note a power-cut still skips finalize (file left at full pre-alloc, recoverable via footers) — a
   boot-time orphan reclaim could be added later if power-cut tail reclaim becomes a concern.

2. **Surface engine health in telemetry — DONE (ExtendedSystemStateBase).** Added a 4-byte
   `SdWriteEngineInfo { overrun_count, errored, reserved }` wire struct and placed it in the SHARED
   `ExtendedSystemStateBase` (so both boards carry it, decoded identically). The engine is a platform
   object, so it crosses the platform→logic seam through the store: `engineInfo()` is now part of the
   `logic::storage::Storage` concept; `SdCard::engineInfo()` reads the shared `sd_write_engine()`,
   `FakeStorage` returns a scriptable value. The recorder surfaces it via `engineHealth()` (reads the
   fast store — the engine is shared so all three agree), and each board's `produceExtended` writes
   `ext.base.sd_write_engine_info = recorder_.engineHealth()`. Distinct from the high-rate
   `StorageInfo.overrun_count` (the telemetry double-buffer ring overrun — unchanged, no regression):
   this is the engine write-ring drop count + sticky DMA error. Host-tested
   (`SdRecorderTest.EngineHealthIsSurfacedFromTheStore`); both boards link clean.

3. **ECU port — DONE (links + wired).** Mirrored the FCU on SDMMC1 (`hsd1`): `sd_write_engine.cpp`
   added to ECU `CM7/CMakeLists.txt`; `sd_write_engine().init(&hsd1)` + the three pre-alloc `bind()`s
   (512/32/8 MiB, replacing the stale `sync_period_writes` args) in ECU `board.cpp`;
   `sd_write_engine().tick()` at the end of the ECU `main.cpp` loop. The SDMMC1 NVIC + ISR were
   ALREADY in the generated files (`sdmmc.c` `HAL_NVIC_EnableIRQ(SDMMC1_IRQn)`, `stm32h7xx_it.c`
   `SDMMC1_IRQHandler → HAL_SD_IRQHandler(&hsd1)`), so the CubeMX toggle is done. **`ecu_CM7.elf`
   now links.** Caveat for bring-up: the ECU SDMMC1 IRQ preemption priority is **0** (`sdmmc.c:114`),
   which preempts TIM6/ADC — the FCU plan recommended **8**; revisit in CubeMX if it disturbs the
   record/ADC cadence (regen-safe per `cubemx-config-over-hand-edit`).

4. **Tune pre-alloc sizes.** 512/32/8 MiB are placeholders. Compute fast-stream rate from
   `sizeof(FcuSystemState)`: bytes/s ≈ 2000 × sizeof(record); size to the longest expected run.
   Per-stream via `bind()`.

5. **Hardware bring-up validation (cannot be unit-tested):**
   - Confirm `SDMMC2_IRQHandler` fires and `onComplete` advances `tx_` (else pipe wedges after 1 block
     — the symptom if the NVIC toggle didn't take).
   - Confirm `f_expand` succeeds on the real card (contiguous free space) and `base_lba_` math yields
     readable files (pull card, verify footers/CRC offline).
   - Confirm card-busy gating: `HAL_SD_GetCardState` poll in `tick()` defers the next kick
     correctly under sustained load; watch `engine.overrun_count()`.
   - Stuck-card watchdog: a DMA that never completes leaves `busy_` stuck (logging stalls, control
     loop fine). Consider a timeout → `onError()` reset. Not implemented.

---

## Touch list

**Done — parent (`Fill-Station-2026`):**
- `src/app/platform/storage/sd_write_engine.hpp` / `.cpp` — new.
- `src/app/platform/storage/sd_card.hpp` / `.cpp` — rewritten (pre-alloc + raw-sector + finalize).
- `src/boards/fcu/board.cpp` — engine init + pre-alloc binds.
- `src/boards/fcu/main.cpp` — `tick()` in loop.
- `src/boards/fcu/CM7/CMakeLists.txt` — add engine source.
- CubeMX `.ioc` (FCU): `_USE_EXPAND`, SDMMC2 NVIC.

**Done — finalize trigger (GS `DisableLogging` flag):**
- `communication/protocol/command/set_control_flag.hpp` — `ControlFlagBase::DisableLogging` (id 1)
  + `toControlFlagBase`.
- `storage/interfaces/storage.hpp` — `finalize()` added to the `Storage` concept.
- `tests/support/fake_storage.hpp` — `finalize()` + `finalize_calls`.
- `telemetry/sd_recorder.hpp` — flag check + `finalizeAll()` + `finalized_` latch.
- `tests/.../sd_recorder_test.cpp` — `DisableLoggingFinalizesAllFilesAndStops`.

**Left:**
- `sd_recorder.hpp` / `fcu telemetry.hpp` — surface engine overrun/error.
- ECU engine wiring (`boards/ecu` board.cpp/main.cpp/CMake, ECU `.ioc` SDMMC2 NVIC). The
  finalize trigger itself is already shared logic (no ECU-specific work).

**Untouched (correct):** `sd_diskio.c` (still polled — only used by the one-time `f_expand`/`f_mkdir`
/`finalize` FatFs calls, never the hot path), telemetry ring + footer format (the footer plan's work
is upstream of this and unchanged).

---

## Testing

- **Host (`FakeStorage`)**: `Storage` concept unaffected; existing recorder/controller tests should
  pass unchanged (write is still "hand a 4096 block to the store"). Add: `write` rejects non-4096
  spans; cursor stops at capacity.
- **Engine unit test (host-able if `HAL_SD_*` is faked)**: enqueue past `SD_WRITE_QUEUE_DEPTH` →
  drops + `overrun_count` saturates; `onComplete` frees slots in order; `idle()` after drain.
- **On-target** (see "Hardware bring-up" above) — the parts that can't be faked.

## Sequencing to resume

1. ~~Pick + wire the **finalize trigger** (item 1)~~ — DONE (GS `DisableLogging` flag).
2. **Surface engine health** (item 2) — cheap, improves observability for bring-up. ← NEXT
3. **On-target validation** on FCU (item 5).
4. **ECU port** (item 3) once FCU is proven.
