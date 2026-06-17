# Plan: Run without an SD card (graceful SD-absent recovery)

## Goal

Boot and run the full system **with no SD card present** (or a dead/failed card) instead of getting
locked in `Error_Handler()`. Surface two distinct, already-existing telemetry flags so the GS knows
why nothing is logging:

- **not connected** → `StorageStatus::plugged_in = 0`
- **not initialized** → `StorageStatus::initialized = 0`

Logging is simply off; everything else (valves, state machine, ADC, telemetry downlink, CAN) runs
normally. Optionally (phase 2) support **hot-plug**: detect a card inserted after boot and bring
logging online without a reboot.

---

## Root cause (verified 2026-06-17)

The **only** hard lock on the SD path is the CubeMX peripheral init:

- `board.cpp halInit()` calls `MX_SDMMC1_SD_Init()` (ECU `board.cpp:69`) / `MX_SDMMC2_SD_Init()`
  (FCU `board.cpp:72`).
- That generated function (`sdmmc.c:47-49`) does `if (HAL_SD_Init(&hsdN) != HAL_OK) Error_Handler();`.
- With no card, `HAL_SD_Init` → `HAL_SD_InitCard` fails to identify a card and returns non-OK →
  `Error_Handler()` → `__disable_irq(); while(1){}` (`board.cpp:235` ECU / `:306` FCU). **Locked.**

Everything past that point is **already graceful** (no other change strictly required to "not crash"):

- `SdCard::init()` already routes f_mount / f_open / f_expand failures through `fail(code)` →
  `state = Error` (`sd_card.cpp:85-127`). It never calls `Error_Handler`.
- `SdCard::write()` no-ops unless `state == Active` (`sd_card.cpp:131-133`), so a failed/absent card
  silently drops records.
- `SdWriteEngine::tick()` returns at the **ring-empty** check (`state_[tx_] != Pending`) *before*
  calling `HAL_SD_GetCardState` (`sd_write_engine.cpp:65-74`). Since an absent card never gets
  anything enqueued (write no-ops), the engine **never touches the HAL** — no per-loop poll on a
  dead peripheral, no spin.
- `SdCard::finalize()` (the `DisableLogging` path) returns immediately when `state != Active`
  (`sd_card.cpp:155-159`), so the drain spin is unreachable when absent.
- The control loop is SD-decoupled by design (async engine), so it does not depend on the card.

**Conclusion:** the fix is small and contained — make the peripheral init non-fatal, gate
`SdCard::init()` to skip the mount path when the card is absent, and drive the two status bits.

---

## Gating facts

- **No card-detect (CD) GPIO is wired.** `HAL_SD_MspInit` configures only D0–D3 / CK / CMD
  (`sdmmc.c:90-111`); there is no CD pin. So "present" must be inferred from `HAL_SD_Init`'s result
  (and, for hot-plug, a periodic re-`HAL_SD_Init` / `HAL_SD_GetCardState` probe), not a level read.
- **The status bits already exist and are unused**: `StorageStatus { initialized:1; plugged_in:1;
  error:2; reserved:4 }` (`storage_status.hpp`). No wire-format change needed to report absence.
- **The 2-bit `error` field is full** (`None/MountFail/FileOpenFail/FileWriteFail`,
  `storage_error.hpp`). "Absent" should therefore be encoded via `plugged_in = 0` (with `error =
  None`), NOT a new `StorageError` value (which would need a 3rd bit → wire change). A present-but-
  broken card still uses the existing `error` causes.
- `Error_Handler()` is **shared by every peripheral init** (SPI/TIM/I2C/CRC/FDCAN all call it), so it
  must NOT be globally weakened — only the SD init's use of it should become non-fatal.
- Both boards differ only in the handle: **FCU = `hsd2` (SDMMC2)**, **ECU = `hsd1` (SDMMC1)**. The
  `hsdN.Init` config (clock edge, 4-bit bus, HW flow control, `ClockDiv`) lives per-board in each
  `sdmmc.c`.

---

## Design

### 1. Make the SDMMC peripheral init non-fatal (the core change)

`Error_Handler()` sits **outside** the `USER CODE` regions of `sdmmc.c`, so it cannot be neutralised
by a regen-safe hand-edit. Options:

- **(Recommended) Stop calling `MX_SDMMCx_SD_Init()` from `halInit()`; call our own non-fatal init.**
  `halInit()` is our code (`board.cpp`) and already owns the explicit list of `MX_*_Init()` calls, so
  we simply swap one line:
  ```cpp
  // was: MX_SDMMC2_SD_Init();   // FCU   (MX_SDMMC1_SD_Init for ECU)
  const bool sd_ok = platform::storage::tryInitSd(&hsd2);   // non-fatal: HAL_SD_Init, no Error_Handler
  ```
  `tryInitSd()` sets `hsdN.Instance` + the 6 `hsdN.Init.*` fields (copied from `sdmmc.c`) and calls
  `HAL_SD_Init`, returning its `HAL_OK` result. `HAL_SD_Init` still invokes `HAL_SD_MspInit`
  internally (GPIO / clock / NVIC unchanged). The generated `MX_SDMMCx_SD_Init` stays in `sdmmc.c`
  (now unused, harmless, still regen-clean).
  - **Tradeoff:** duplicates the 6 `Init` fields. They are stable (bus width / clock div rarely
    change); add a comment pointing at `sdmmc.c` as the reference. If CubeMX ever changes them, update
    the copy. (Per memory `cubemx-config-over-hand-edit`, we are NOT hand-editing generated files —
    we stop calling one and add our own in board code.)
  - Where: a small `platform::storage` helper (board passes its own handle), or inline in each
    `board.cpp` (keeps the board-specific `Init` config in the board). Inline is simplest; a shared
    helper that takes the already-filled `SD_HandleTypeDef&` and just calls `HAL_SD_Init` + records
    the result avoids logic duplication while leaving the per-board `Init` in `board.cpp`.

- **(Rejected) Make `Error_Handler()` return instead of looping.** It is shared by all peripherals;
  a genuine clock/bus fault would then silently continue. Too broad, unsafe.

- **(Rejected) Guard inside `sdmmc.c`.** The `Error_Handler()` call is outside `USER CODE`; any edit
  is clobbered on regen.

### 2. Thread an `sd_present` fact to the storage layer

`tryInitSd()`'s result is a board-level fact. Store it where the cards can read it at `init()`:

- Simplest: a `platform::storage` flag set by `tryInitSd()`, exposed as `bool sdPresent()`.
  Alternatively pass it into `SdWriteEngine::init(handle, present)` and expose `engine.present()`
  (the engine is already the shared SD owner). Either works; the engine is the natural home.

### 3. Gate `SdCard::init()` on presence (skip the FatFs path when absent)

```cpp
void SdCard::init() {
    info_.state = StorageState::Init;
    if (!sdPresent()) {                 // peripheral init failed / no card
        info_.status.plugged_in  = 0;   // "not connected"
        info_.status.initialized = 0;   // "not inited"
        info_.status.error       = StorageError::None;   // absent by-design, not an op failure
        info_.state              = StorageState::Error;  // -> write()/finalize() no-op; health() surfaces it
        return;                         // do NOT touch f_mount / diskio with no card
    }
    info_.status.plugged_in = 1;
    ... existing mount + open + f_expand ...
    info_.status.initialized = 1;       // already set on success today
}
```

This avoids exercising the FatFs/`sd_diskio` path with a dead peripheral entirely (see Risks). On a
present-but-failing card (mount/open/expand fails), keep today's behaviour: `plugged_in = 1`,
`initialized = 0`, `error = MountFail/...`, `state = Error`.

### 4. Surface the flags (already plumbed)

`recorder_.health()` already returns a `StorageInfo` (state + status incl. `plugged_in` /
`initialized`) onto the high-rate `SystemStateBase.storage_info` (`fcu/telemetry.hpp buildSystemState
:309`). So once `init()` sets the bits, the GS sees them with **no telemetry change**. The
`SdWriteEngineInfo` on the extended record is orthogonal (drop/error counts) and untouched.

### 5. (Phase 2, optional) Hot-plug

If running with no card and the operator inserts one, bring logging up without a reboot:

- In the main loop, when `!sdPresent()`, every ~1–2 s call `tryInitSd()` again (it is a quick CMD
  sequence; `HAL_SD_Init` returns on its internal timeout if still absent — bounded, no hang).
- On success: set present, then re-run the storage bring-up (`recorder_.init()` → each
  `SdCard::init()` → mount + per-boot folder + open + `f_expand`) and flip `plugged_in/initialized`.
- Caveats: `recorder_.init()` resets the slow/ext accumulators (fine); the session-folder counter
  picks the next free number (fine); must run from the foreground (not ISR); add a small backoff so a
  permanently-absent card does not re-probe every loop. Defer until phase 1 is proven.

---

## Touch list (phase 1)

- `src/boards/fcu/board.cpp` — replace `MX_SDMMC2_SD_Init();` with the non-fatal `tryInitSd(&hsd2)`;
  store the result.
- `src/boards/ecu/board.cpp` — same for `MX_SDMMC1_SD_Init();` / `&hsd1`.
- `src/app/platform/storage/` — `tryInitSd()` + `sdPresent()` (or fold into `SdWriteEngine::init` +
  `present()`).
- `src/app/platform/storage/sd_card.cpp` — presence short-circuit in `init()` + drive
  `plugged_in` / `initialized` on both the absent and present paths.
- **Untouched:** `sdmmc.c` (still regenerable; `MX_SDMMCx_SD_Init` just no longer called),
  telemetry/recorder (the health bits already ride `StorageInfo`), the engine hot path (already
  safe with an empty ring), the control layer (SD-independent).

No wire-format change. No submodule change.

---

## Risks / edge cases

- **Does `f_mount`/`sd_diskio` hang or merely fail with no card?** The recommended design **never
  calls f_mount when absent** (step 3 short-circuits), so this path is not exercised — the safest
  choice. (If we ever let it through, `disk_initialize` should return `FR_NOT_READY`, but that is
  unverified on this `sd_diskio`; avoid relying on it.)
- **Peripheral left half-initialized after a failed `HAL_SD_Init`.** MspInit (GPIO/clock/NVIC) has
  already run; the card is just unidentified. Harmless as long as nothing issues a transfer — which
  holds, because `write()` no-ops and the engine never kicks on an empty ring.
- **`HAL_SD_Init` latency when absent.** It returns on its internal command timeout, so boot is
  delayed by that bounded timeout once, not hung.
- **Brown-out / marginal card** that enumerates then fails mid-run: already covered by the existing
  `error` causes + `state = Error`; the engine's sticky `errored` flag also surfaces on the extended
  record (`SdWriteEngineInfo`).

---

## Testing

- **Host (`FakeStorage`)**: add a "card absent" script — `init()` leaves `plugged_in = 0`,
  `initialized = 0`, `state = Error`; assert `recordSystemState`/`recordExtended` no-op and
  `health()` surfaces the absent bits. (FakeStorage models the seam; the real `tryInitSd` is
  on-target only.)
- **On-target (cannot be unit-tested):**
  - Boot with **no card** → reaches the main loop, heartbeat blinks, valves/state/telemetry work;
    GS shows `plugged_in = 0`, `initialized = 0`. No `Error_Handler` lock.
  - Boot with a **good card** → unchanged: `plugged_in = 1`, `initialized = 1`, logging works.
  - (Phase 2) Insert a card after boot → logging comes online within the retry interval.

---

## Sequencing

1. Phase 1 (non-fatal init + presence gate + status bits) — directly removes the lock; small,
   contained, no wire change.
2. On-target validation (the three boot scenarios above).
3. Phase 2 (hot-plug) only if running-then-inserting is a real operational need.
