# `-O3` + LTO release-readiness audit

**Status:** Not ready. Build is not configured for `-O3`/LTO, and the code has several
miscompile hazards that aggressive optimization + LTO will expose. None of these throw a
build error — they fail silently on hardware (dropped GS commands, boot into the default
handler). Fixes are small and mechanical; act on them before flipping the switch.

Audited the **FCU** board path in depth (`src/app/**`, `src/boards/fcu/{main,board}.cpp`).
The **ECU** board was covered more lightly — give `src/boards/ecu/board.cpp` the same
ISR-handler / section-placement check before shipping its release build.

Verified at audit time: D-cache is **disabled** (only `SCB_EnableICache()` is called), so
the usual Cortex-M7 DMA-coherency landmine is moot. App headers are ODR-clean (header-scope
vars are `inline`/`constexpr`). No removable busy-wait delays. No fall-off-end functions,
bad shifts, or signed overflow. The hazards below are the exceptions.

---

## 0. Build is not configured for `-O3`/LTO

- "Release" = `-Os -g0` (GCC, `gcc-arm-none-eabi.cmake:36`) / `-Oz` (clang toolchain).
  **No `-flto` anywhere** in the project config.
- To enable: add `-O3 -flto=auto -ffat-lto-objects` to the release C/CXX flags and the
  matching `-flto` to `CMAKE_EXE_LINKER_FLAGS`.
- Release flags do **not** set `-DNDEBUG`, but that's harmless: nothing relies on `assert()`
  and `USE_FULL_ASSERT` is never defined, so there is no Debug→Release behavior change.

---

## 1. CRITICAL — will miscompile at `-O3`/LTO

### 1a. Ethernet RX ring indices are not `volatile`
- `src/app/platform/communication/ethernet/ethernet.cpp:159-160` —
  `rx_ring_head` / `rx_ring_tail` are plain `std::size_t`.
- Producer: `HAL_ETH_RxCpltCallback` ISR (→ `process_eth_frame` → … → `enqueue_rx`).
  Consumer: `receive()` in the main loop.
- At `-O3` the head index can be cached in a register → **ground-station commands silently
  dropped**. The ADC and CAN rings already use `volatile` indices; this ring was missed.
- Fix: make both `volatile std::size_t`; add `__DMB()` between filling the slot and advancing
  `rx_ring_head`, and between the `tail != head` test and reading the slot in `receive()`
  (mirror the ADC ring at `ads131m08.cpp:159/169`).

### 1b. Byte-buffer → struct `reinterpret_cast` then dereference (strict-aliasing UB)
`-fstrict-aliasing` is on at `-O2`+; LTO makes the assumption cross-TU. Reading multi-byte
struct members through a pointer cast from a `uint8_t` array is UB the optimizer can act on
(also misaligned reads). Sites:
- `src/app/logic/fcu/control.hpp:364` (`SetStateFrame*` over align-1 `std::array<uint8_t,8>`),
  and `:427`, `:451`, `:523` (`SetValvePositionFrame*`, `SetControlFlagFrame*` — `flag` is `uint16_t`).
- `src/app/platform/communication/ethernet/ethernet.cpp` — 7 sites: `:235, 251, 262, 270,
  278, 319, 328` (`EthHeader`/`Ipv4Header`/`UdpHeader`/`IcmpEchoHeader`/`ArpPacket` over the
  RX pool; `:262` also writes through the cast).
- Fix: `std::memcpy` (or `std::bit_cast`) into a local struct — exactly what the inbound
  parsers (`command_can_parser.cpp`, `command_ethernet_parser.cpp`) already do.

---

## 2. HIGH — fix before enabling LTO

### 2a. ISR vector overrides risk being garbage-collected by LTO
- `src/boards/fcu/board.cpp:261, 276, 284, 289` (`TIM6_DAC_IRQHandler`, `EXTI9_5_IRQHandler`,
  `I2C4_EV_IRQHandler`, `I2C4_ER_IRQHandler`) + `HAL_TIM_PeriodElapsedCallback`.
- `src/boards/ecu/board.cpp:194, 208, 215, 220` (equivalent set; `EXTI4_IRQHandler`).
- `src/app/platform/communication/spi/spi_dil.cpp:53, 61` (`HAL_SPI_TxRxCpltCallback`,
  `HAL_SPI_ErrorCallback` weak overrides).
- These are referenced only from the assembly vector table, which the LTO plugin doesn't
  parse → LTO+`--gc-sections` may drop them or rebind the vector to `Default_Handler`
  (infinite loop), **with no link error**.
- Fix: `__attribute__((used))` on each handler/callback override, or `KEEP()` the vector
  symbols in the linker script.

### 2b. `section(...)`-placed DMA / persistent objects need `used` / `KEEP`
- `.axisram`: `main.cpp:63-66` (`g_card_*`, `g_controller`), `spi_dma.cpp:30` (`s_bus`).
- `.RxBuffSection`/`.TxBuffSection`: `ethernet.cpp:147, 164-179` (`rx_pool`, `tx_*`).
- `.backup_sram`: `persistent_state.cpp:26` (`persistent_state`) — most at risk.
- Linker sections in `stm32h747xx_flash_CM7.ld:226-274` use plain `*(.axisram)` with no
  `KEEP()`. LTO can prove an object "dead" and drop it even though DMA/hardware still uses it.
- Fix: `KEEP(*(.axisram*))` etc. in the linker script, or `__attribute__((used, section(...)))`.

### 2c. `rx_buf_status[]` not `volatile` + missing telemetry barriers
- `ethernet.cpp:148` — `rx_buf_status[]` is written in ISR, read in `tick()`; make it `volatile`.
- `src/app/logic/fcu/telemetry.hpp` and `src/app/logic/ecu/telemetry.hpp` `LogBuffer`: add a
  `__DMB()` between the payload `memcpy`/`used` write and the `ready[h] = true` publish
  (producer runs in the TIM6 ISR), and after the `ready[tail]` test in `drain()`. Without it
  the memcpy can sink past the publish at `-O3`. (head/tail non-volatile is OK here — the
  handshake goes through the `volatile ready[]` flag with single-writer-per-context.)

---

## 3. LOW / latent

- `command_can_parser.cpp` / `command_ethernet_parser.cpp` clamp the copy length to the
  **source** buffer (64 B) rather than the **destination** `cmd.payload` capacity (8 B). Safe
  today (max payload = 2 B) but a silent overflow if any command payload ever exceeds 8 B.
  Fix: clamp `n` to `cmd.payload.size()`.
- `sd_recorder.hpp:146` arithmetic right-shift of signed `int32_t` — implementation-defined,
  GCC-ARM defines it as arithmetic. Portability note only.

---

## 4. Confirmed clean (no action)

- D-cache disabled → no SCB clean/invalidate needed around DMA; buffers correctly kept out of
  DTCM (in AXI / D2 SRAM). If anyone enables `SCB_EnableDCache()` later, this becomes CRITICAL:
  add non-cacheable MPU regions or cache-maintenance calls first.
- ADC ring (`ads131m08.hpp:517-518`) — `volatile` indices + `__DMB()`, exemplary SPSC.
- SPI/I2C completion flags (`spi_dil.hpp:120-121`, `ina3221.hpp:124-125`) — properly `volatile`.
- App headers ODR-clean; no inline asm / naked functions in app or board code.
- Serialization that uses `memcpy` / `char*`-span / aligned union punning — all safe.

---

## Suggested order of work

1. `volatile` + `__DMB()` on the Ethernet RX ring (1a) and `rx_buf_status` (2c).
2. `memcpy`/`bit_cast` the ~11 byte-buffer→struct casts (1b).
3. `used`/`KEEP` on ISR handlers (2a) and section-placed DMA/persistent objects (2b).
4. Repeat the 2a/2b check on the ECU board.
5. Add the `-O3 -flto` release preset, trial-build, then **flash-and-run** — these bugs pass
   the build and only fail on hardware.
