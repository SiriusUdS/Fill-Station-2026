#pragma once

/* ------------------------------------------------------------------------- *
 * logic::ecu::Controller<S, V, A, C> template definitions. Included at the end of
 * ecu_controller.hpp; not a standalone TU. The CAN-side sibling of the FCU
 * controller: no Ethernet, telemetry downlinked over CAN.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

/* ---- CAN (commands from the FCU) ----------------------------------------- */

/* Drain the CAN RX ring every tick so it cannot back up. Parsing FCU commands
   (CAN_ID_CMD_VALVE -> actuate IPA/NOS, ping -> pong, set-state) is E2. */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::canTick()
{
    while (auto frame = can_.receive()) {
        engine_.last_cmd_ms = engine_.current_tick_ms;
        (void)frame;  // TODO(E2): dispatch CAN_ID_CMD_VALVE / ping / set-state
    }
}

/* ---- Telemetry production (per ADC sample) -------------------------------- */

// Build a SystemState from the ADC's info record (a fresh conversion on the sample
// path, or a synthesized Faulted record on the silent-ADC fallback path).
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
SystemState Controller<S, V, A, C>::buildSystemState(const AdcInfo& adc, uint32_t now_ms)
{
    SystemState state = {};
    state.frameTs_MS         = now_ms;
    state.lastHandshakeTs_MS = engine_.last_cmd_ms;   // last command from the FCU

    // Each peripheral OWNS its telemetry record; the controller only reads it.
    state.adc_info     = adc;
    state.storage_info = storage_.info();
    state.can_info     = can_.info();
    // No Ethernet on the ECU: state.eth_info stays zero-initialised.

    // The double-buffer overrun (a half filled before it could be flushed, so
    // records were dropped) is a logging-pipeline fault — surface it as the
    // sdCard interface's writingError.
    InterfaceFieldFlags sd = {};
    sd.bits.writingError = log_.overrun ? 1 : 0;
    state.interfaces.frame.sdCardFlags = sd;

    // The ECU's two valves report their own info, indexed by the EcuValves SSOT.
    state.valve_info[static_cast<std::size_t>(EcuValves::IPA)] = ipa_valve_.info();
    state.valve_info[static_cast<std::size_t>(EcuValves::NOS)] = nos_valve_.info();

    return state;
}

// Append one record to the active half (single-producer: produceRecord() in the
// record-timer ISR). When a half fills it is marked ready for drainTick(); if the
// other half is still unflushed the record is dropped and overrun is flagged.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::logAppend(const SystemState& record)
{
    uint8_t a = log_.active;
    if (log_.used[a] + sizeof(SystemState) > detail::LOG_HALF_BYTES) {
        log_.ready[a] = true;        // finalize this half
        a ^= 1;
        if (log_.ready[a]) {         // consumer hasn't drained it yet
            log_.overrun = true;
            return;
        }
        log_.active  = a;
        log_.used[a] = 0;
    }
    std::memcpy(&log_.data[a][log_.used[a]], &record, sizeof(SystemState));
    log_.used[a] = static_cast<uint16_t>(log_.used[a] + sizeof(SystemState));
}

// The comms/save cadence — driven by the record timer, NOT the ADC. Drain every
// conversion the ADC has queued (each a fresh record); if the ring is empty and the
// ADC has timed out, emit one filler carrying the last-known data flagged invalid.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::produceRecord(uint32_t now_ms)
{
    bool produced = false;
    while (auto sample = adc_.pop()) {     // every queued conversion, exactly once
        last_adc_ms_ = now_ms;
        logAppend(buildSystemState(*sample, now_ms));
        produced = true;
    }
    if (!produced && (now_ms - last_adc_ms_) > detail::ADC_TIMEOUT_MS) {
        AdcInfo info = adc_.info();
        info.state             = AdcState::Faulted;
        info.status.data_valid = 0u;
        logAppend(buildSystemState(info, now_ms));
    }
}

/* ---- Telemetry drain (SD now; CAN downlink in E3) ------------------------- */

// Flush any full half: write the 4096-byte block to SD (sector-aligned). The CAN
// downlink of the records (fragmented to the FCU) is added in E3.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::drainTick()
{
    for (uint8_t h = 0; h < 2; ++h) {
        if (!log_.ready[h]) {
            continue;
        }
        storage_.write(std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES));
        // TODO(E3): sendBatchedCan(std::span(log_.data[h], log_.used[h]));
        log_.ready[h] = false;
    }
}

/* ---- Public surface ------------------------------------------------------- */

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::init()
{
    engine_ = detail::Engine{};

    // The telemetry double buffer lives in NOLOAD .axisram — clear it and its
    // indices before the producer starts.
    std::memset(log_.data, 0, sizeof(log_.data));
    log_.used[0]  = 0;
    log_.used[1]  = 0;
    log_.ready[0] = false;
    log_.ready[1] = false;
    log_.active   = 0;
    log_.overrun  = false;
    last_adc_ms_  = 0;

    // The state machine state lives in Backup SRAM (shared mechanism with the FCU).
    // Resume it across a reset; on a cold/corrupt boot, commit a fresh INIT so the
    // blob is valid from here on.
    logic::control::persistent_state.saveState(
        logic::control::persistent_state.loadState().value_or(logic::control::State::Init));

    // Bring the backing store online. Platform bring-up starts the record timer
    // after this init() returns, so records start flowing only once the SD is ready.
    storage_.init();
}

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::tick(uint32_t now_ms)
{
    engine_.current_tick_ms = now_ms;

    canTick();
    drainTick();         // flush full halves to SD (record production is on the timer)

    // Minimal state machine for now: Init -> Safe on the first tick (engine states
    // are added once routing + telemetry are in place).
    if (logic::control::persistent_state.fill_state == logic::control::State::Init) {
        logic::control::persistent_state.saveState(logic::control::State::Safe);
    }
}

} // namespace logic::ecu
