#pragma once

/* ------------------------------------------------------------------------- *
 * logic::ecu::Controller<S, V, A, C> template definitions. Included at the end of
 * ecu_controller.hpp; not a standalone TU. The CAN-side sibling of the FCU
 * controller: no Ethernet, telemetry downlinked over CAN.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

/* ---- CAN (commands from the FCU) ----------------------------------------- */

/* Drain the CAN RX ring every tick (so it cannot back up) and dispatch each frame
   addressed to us. The FCU sends CommandType::SetValvePosition to drive the ECU's valves and may
   ping; the ECU replies pong and actuates. */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::canTick()
{
    while (auto frame = can_.receive()) {
        engine_.last_cmd_ms = engine_.current_tick_ms;
        handleCanFrame(*frame);
    }
}

// Decode the 29-bit identifier into the shared CanHeader and route by messageID,
// ignoring frames not addressed to this node (or broadcast).
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::handleCanFrame(const logic::communication::CanFrame& frame)
{
    CanHeader header;
    header.code = frame.id;

    const auto target = static_cast<BoardId>(header.frame.targetID);
    if (target != BoardId::Engine && target != BoardId::Broadcast) {
        return;  // not for us
    }

    switch (static_cast<CommandType>(header.frame.messageID)) {
        case CommandType::SetValvePosition: handleValveCmd(frame, header); break;
        case CommandType::Ping: handlePing(frame, header);     break;
        default:               break;
    }
}

// Drive one of the ECU's valves from a CommandType::SetValvePosition frame: the valve index is at
// data[4] (EcuValves::IPA = IPA, EcuValves::NOS = NOS); the open/close action is in the
// header's deviceState (ValveCommand::Open / ValveCommand::Close).
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::handleValveCmd(const logic::communication::CanFrame& frame,
                                            const CanHeader& header)
{
    if (frame.length <= detail::CMD_VALVE_INDEX_OFFSET) {
        return;  // frame too short to carry a valve index
    }
    const auto valve_id = static_cast<EcuValves>(frame.data[detail::CMD_VALVE_INDEX_OFFSET]);

    V* valve = (valve_id == EcuValves::IPA) ? &ipa_valve_
             : (valve_id == EcuValves::NOS) ? &nos_valve_
             : nullptr;
    if (valve == nullptr) {
        return;  // unknown valve id
    }

    switch (static_cast<ValveCommand>(header.frame.deviceState)) {
        case ValveCommand::Open:  (void)valve->open();  break;
        case ValveCommand::Close: (void)valve->close(); break;
        default:            break;
    }
}

// Reply to a ping with a pong back to the sender, echoing the payload.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::handlePing(const logic::communication::CanFrame& frame,
                                        const CanHeader& header)
{
    CanHeader reply  = {};
    reply.frame.senderID  = static_cast<uint8_t>(BoardId::Engine);
    reply.frame.targetID  = header.frame.senderID;
    reply.frame.messageID = static_cast<uint8_t>(CommandType::Pong);

    logic::communication::CanFrame out;
    out.id     = reply.code;
    out.length = frame.length;
    out.data   = frame.data;   // echo payload
    (void)can_.send(out);
}

/* ---- Telemetry production (per ADC sample) -------------------------------- */

// Build an EcuSystemState from the ADC's info record (a fresh conversion on the
// sample path, or a synthesized Faulted record on the silent-ADC fallback path).
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
EcuSystemState Controller<S, V, A, C>::buildSystemState(const AdcInfo& adc, uint32_t now_ms)
{
    EcuSystemState state = {};
    state.base.creation_timestamp_ms = now_ms;

    // Each peripheral OWNS its telemetry record; the controller only reads it.
    state.base.adc_info     = adc;
    state.base.storage_info = storage_.info();
    state.base.can_info     = can_.info();
    // The ECU has no Ethernet peripheral, so EcuSystemState carries no eth_info.

    // The double-buffer overrun (a half filled before it could be flushed, so
    // records were dropped) is a logging-pipeline fault — surface it on the SD
    // card's own status record (it owns its interface flags now).
    state.base.storage_info.status.write_overrun = log_.overrun ? 1 : 0;

    // The ECU's two valves report their own info, indexed by the EcuValves SSOT.
    state.base.valve_info[static_cast<std::size_t>(EcuValves::IPA)] = ipa_valve_.info();
    state.base.valve_info[static_cast<std::size_t>(EcuValves::NOS)] = nos_valve_.info();

    return state;
}

// Append one record to the active half (single-producer: produceRecord() in the
// record-timer ISR). When a half fills it is marked ready for drainTick(); if the
// other half is still unflushed the record is dropped and overrun is flagged.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::logAppend(const EcuSystemState& record)
{
    uint8_t a = log_.active;
    if (log_.used[a] + sizeof(EcuSystemState) > detail::LOG_HALF_BYTES) {
        log_.ready[a] = true;        // finalize this half
        a ^= 1;
        if (log_.ready[a]) {         // consumer hasn't drained it yet
            log_.overrun = true;
            return;
        }
        log_.active  = a;
        log_.used[a] = 0;
    }
    std::memcpy(&log_.data[a][log_.used[a]], &record, sizeof(EcuSystemState));
    log_.used[a] = static_cast<uint16_t>(log_.used[a] + sizeof(EcuSystemState));
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

/* ---- Telemetry drain (SD + CAN downlink) ---------------------------------- */

// Flush any full half: write the 4096-byte block to SD (sector-aligned), and
// downlink the half's most recent record to the FCU over CAN. The SD gets the
// full-rate log; the CAN bus cannot carry every record (a EcuSystemState is many
// frames and the record rate is far above the bus), so the CAN downlink is a
// downsampled live-telemetry stream (one record per drained half) alongside it.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::drainTick()
{
    for (uint8_t h = 0; h < 2; ++h) {
        if (!log_.ready[h]) {
            continue;
        }
        const uint16_t bytes = log_.used[h];

        storage_.write(std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES));
        if (bytes >= sizeof(EcuSystemState)) {
            EcuSystemState latest;
            std::memcpy(&latest, &log_.data[h][bytes - sizeof(EcuSystemState)], sizeof(EcuSystemState));
            sendRecordCan(latest);
        }
        log_.ready[h] = false;
    }
}

// Fragment one EcuSystemState into CAN frames (shared codec) and send them to the FCU.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
void Controller<S, V, A, C>::sendRecordCan(const EcuSystemState& record)
{
    namespace codec = logic::communication::can;
    std::array<logic::communication::CanFrame, codec::SYSTEM_STATE_FRAGMENTS> frames;
    codec::packSystemState(
        record, BoardId::Engine, BoardId::FillingStation, telemetry_seq_,
        std::span<logic::communication::CanFrame, codec::SYSTEM_STATE_FRAGMENTS>(frames));
    for (auto& f : frames) {
        (void)can_.send(f);
    }
    ++telemetry_seq_;
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
