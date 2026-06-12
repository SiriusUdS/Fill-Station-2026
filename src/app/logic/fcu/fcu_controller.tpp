#pragma once

/* ------------------------------------------------------------------------- *
 * Controller<S> template definitions. Included at the end of fcu_controller.hpp;
 * not a standalone TU. Bodies are the former free-function controller verbatim,
 * with file-scope state now members (fill_/log_/last_adc_ms_/storage_) and the
 * storage seam called directly through storage_.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

using logic::communication::command::CommandType;

/* ---- CAN ----------------------------------------------------------------- */

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::sendValveCmd(uint8_t valve, uint8_t cmd)
{
    CanHeader header = {};
    header.frame.senderID    = static_cast<uint8_t>(BoardId::FillingStation);
    header.frame.targetID    = static_cast<uint8_t>(BoardId::Engine);
    header.frame.deviceState = cmd;
    header.frame.messageID   = static_cast<uint8_t>(CommandType::SetValvePosition);

    logic::communication::CanFrame frame;
    frame.id     = header.code;
    frame.length = static_cast<uint8_t>(frame.data.size());
    std::memcpy(frame.data.data(), &fill_.current_tick_ms, sizeof(uint32_t));  // ValveCmd timestamp
    frame.data[sizeof(uint32_t)] = valve;                                       // ValveCmd valveIndex

    (void)can_.send(frame);
}

/* The FCU receives ECU telemetry over CAN: EcuSystemState records fragmented by the
   shared codec. Reassemble each and relay it to the GS through the single egress,
   tagged as BoardId::Engine so the GS demuxes it from the FCU's own records. (The
   FCU never receives commands over CAN — those always arrive over Ethernet.) */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::canTick()
{
    while (auto frame = can_.receive()) {
        if (auto record = ecu_reassembler_.accept(*frame)) {
            // One reassembled ECU record, relayed through the shared egress. Its byte
            // span and the per-record stride are the same here (a single record).
            const EcuSystemState ecu = *record;
            const std::span<const uint8_t> one(reinterpret_cast<const uint8_t*>(&ecu), sizeof(ecu));
            sendToGs(BoardId::Engine, /*sourceState (ECU state not yet in the record)*/ 0,
                     one, one.size());
        }
    }
}

/* ---- UDP (ground-station commands) --------------------------------------- */

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::handleStateRequest(std::span<const uint8_t> payload)
{
    if (payload.size() <= detail::REQUEST_STATE_OFFSET_BYTES) {
        return;
    }
    const auto requested =
        static_cast<logic::control::State>(payload[detail::REQUEST_STATE_OFFSET_BYTES]);

    switch (logic::control::persistent_state.fill_state) {
        case logic::control::State::Safe:
            if (requested != logic::control::State::Test &&
                requested != logic::control::State::Unsafe) return;
            break;
        case logic::control::State::Test:
            if (requested != logic::control::State::Safe) return;
            break;
        case logic::control::State::Unsafe:
            if (requested != logic::control::State::Safe &&
                requested != logic::control::State::Ignite &&
                requested != logic::control::State::Abort) return;
            break;
        case logic::control::State::Ignite:
            if (requested != logic::control::State::Safe &&
                requested != logic::control::State::Abort) return;
            break;
        case logic::control::State::Abort:
            if (requested != logic::control::State::Safe) return;
            break;
        default:
            return;
    }
    logic::control::persistent_state.saveState(requested);
}

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::handleDatagram(std::span<const uint8_t> payload)
{
    if (payload.size() < sizeof(UDPPacketHeader)) {
        return;
    }

    UDPPacketHeader header;
    std::memcpy(header.bytes, payload.data(), sizeof(UDPPacketHeader));
    fill_.last_rx_ms = fill_.current_tick_ms;

    const auto device = static_cast<BoardId>(header.frame.deviceID);
    if (device != BoardId::FillingStation && device != BoardId::Broadcast) {
        return;  /* not addressed to us (CAN-bridge routing TODO) */
    }

    // Dispatch on the payloadID so exactly the matching handler runs.
    switch (static_cast<CommandType>(header.frame.payloadID)) {
        case CommandType::SetState:
            handleStateRequest(payload);
            break;
        case CommandType::SetValvePosition:
            handleSetValvePosition(payload);
            break;
        default:
            break;
    }
}

// Drive one of the FCU's local valves from a SetValvePosition command. The frame
// follows the 12-byte UDP header. Open/Close ignore value; SetOpenedPct uses it.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::handleSetValvePosition(std::span<const uint8_t> payload)
{
    constexpr std::size_t frame_offset = sizeof(UDPPacketHeader);
    if (payload.size() < frame_offset + sizeof(SetValvePositionFrame)) {
        return;
    }
    SetValvePositionFrame frame;
    std::memcpy(&frame, payload.data() + frame_offset, sizeof(frame));

    V* valve = (frame.valve == FcuValves::Fill) ? &fill_valve_
             : (frame.valve == FcuValves::Dump) ? &dump_valve_
             : nullptr;
    if (valve == nullptr) {
        return;  // unknown valve id
    }

    switch (frame.action) {
        case ValveCommand::Open:         (void)valve->open();  break;
        case ValveCommand::Close:        (void)valve->close(); break;
        case ValveCommand::SetOpenedPct: (void)valve->setOpenPercent(static_cast<float>(frame.value)); break;
    }
}

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::rxTick()
{
    eth_.tick();  // service the link so receive() can return inbound datagrams
    while (auto datagram = eth_.receive()) {
        handleDatagram(datagram->payload);
    }
}

/* ---- Telemetry production (per ADC sample) -------------------------------- */

// Build an FcuSystemState from the ADC's info record (a fresh conversion on the
// sample path, or a synthesized Faulted record on the watchdog fallback path).
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
FcuSystemState Controller<S, V, A, E, C>::buildSystemState(const AdcInfo& adc, uint32_t now_ms)
{
    FcuSystemState state = {};
    state.base.creation_timestamp_ms = now_ms;

    // The ADC owns its record (state + status + channels) — the silent/fault
    // condition now lives in adc_info, not in sdCardFlags.
    state.base.adc_info = adc;

    // The SD card owns its record too: state (Init/Active/Error) + status bits
    // (incl. the last error cause). The GS reads health straight from here.
    state.base.storage_info = storage_.info();

    // The CAN bus too: state + status + dropped-frame count.
    state.base.can_info = can_.info();

    // The Ethernet link reports its own state + status + drop count. This is the
    // FCU-specific field, alongside the common base (the ECU record has none).
    state.eth_info = eth_.info();

    // The double-buffer overrun (a half filled before it could be flushed, so
    // records were dropped) is a logging-pipeline fault, not the card's own —
    // surface it on the SD card's status record (it owns its interface flags now).
    state.base.storage_info.status.write_overrun = log_.overrun ? 1 : 0;

    // The FCU's own valves report their own info, indexed by the FcuValves SSOT.
    state.base.valve_info[static_cast<std::size_t>(FcuValves::Fill)] = fill_valve_.info();
    state.base.valve_info[static_cast<std::size_t>(FcuValves::Dump)] = dump_valve_.info();

    return state;
}

// Append one record to the active half. Single-producer: every record comes from
// produceRecord() in the record-timer ISR. When a half fills it is marked ready
// for drainTick(); if the other half is still unflushed the record is dropped and
// overrun is flagged.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::logAppend(const FcuSystemState& record)
{
    uint8_t a = log_.active;
    if (log_.used[a] + sizeof(FcuSystemState) > detail::LOG_HALF_BYTES) {
        log_.ready[a] = true;        // finalize this half
        a ^= 1;
        if (log_.ready[a]) {         // consumer hasn't drained it yet
            log_.overrun = true;
            return;
        }
        log_.active  = a;
        log_.used[a] = 0;
    }
    std::memcpy(&log_.data[a][log_.used[a]], &record, sizeof(FcuSystemState));
    log_.used[a] = static_cast<uint16_t>(log_.used[a] + sizeof(FcuSystemState));
}

/* ---- Watchdogs ------------------------------------------------------------ */

// Abort if the ground-station link goes quiet while armed.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::watchdogTick()
{
    const auto state = logic::control::persistent_state.fill_state;
    if (state == logic::control::State::Unsafe || state == logic::control::State::Ignite) {
        if ((fill_.current_tick_ms - fill_.last_rx_ms) >= detail::RX_WATCHDOG_MS) {
            logic::control::persistent_state.saveState(logic::control::State::Abort);
        }
    }
}

// The comms/save cadence — driven by the record timer, NOT the ADC. Drain every
// conversion the ADC has queued (each a fresh record); if the ring is empty and
// the ADC has timed out, emit one filler carrying the last-known data flagged
// invalid, so the downstream packet rate is unchanged when the ADC is silent.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::produceRecord(uint32_t now_ms)
{
    bool produced = false;
    while (auto sample = adc_.pop()) {     // every queued conversion, exactly once
        last_adc_ms_ = now_ms;
        logAppend(buildSystemState(*sample, now_ms));
        produced = true;
    }
    if (!produced && (now_ms - last_adc_ms_) > detail::ADC_TIMEOUT_MS) {
        // ADC silent: filler with the actual last-known record, flagged bogus.
        AdcInfo info = adc_.info();
        info.state             = AdcState::Faulted;
        info.status.data_valid = 0u;
        logAppend(buildSystemState(info, now_ms));
    }
}

/* ---- Telemetry drain (SD + Ethernet) -------------------------------------- */

// The single GS egress: split a run of fixed-size telemetry records into UDP
// datagrams (EthernetHeader + whole records + CRC) tagged with their source board
// id + state, and send them to the GS. Both the FCU's own records (drainTick,
// FcuSystemState) and the reassembled ECU records (canTick, EcuSystemState) flow
// through here — the two differ in size, so the per-record stride is passed in and
// datagrams are batched on record boundaries so the GS never sees a split record.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::sendToGs(BoardId sourceId, uint8_t sourceState,
                                         std::span<const uint8_t> records, std::size_t record_size)
{
    static std::array<uint8_t, detail::UDP_MAX_PAYLOAD_BYTES> packet;

    // Whole records per datagram, and the byte run they occupy.
    const std::size_t per_packet =
        (detail::UDP_MAX_PAYLOAD_BYTES - sizeof(EthernetHeader) - sizeof(uint32_t)) / record_size;
    const std::size_t batch_bytes = per_packet * record_size;
    for (std::size_t off = 0; off < records.size(); off += batch_bytes) {
        const std::size_t chunk =
            records.size() - off < batch_bytes ? records.size() - off : batch_bytes;

        EthernetHeader header = {};
        header.deviceID      = static_cast<uint8_t>(sourceId);
        header.payloadID     = static_cast<uint8_t>(TelemetryId::SystemState);
        header.payloadLenght = static_cast<uint16_t>(chunk);
        header.deviceState   = sourceState;
        header.deviceTS_MS   = fill_.current_tick_ms;

        const uint32_t crc = logic::data_integrity::crc32(records.data() + off, chunk);
        std::memcpy(packet.data(), &header, sizeof(header));
        std::memcpy(packet.data() + sizeof(header), records.data() + off, chunk);
        std::memcpy(packet.data() + sizeof(header) + chunk, &crc, sizeof(crc));

        (void)eth_.send(fill_.gs,
            std::span<const uint8_t>(packet.data(), sizeof(header) + chunk + sizeof(crc)));
    }
}

// Flush any full half: write the 4096-byte block to SD (sector-aligned) and
// stream its records to the GS, then release the half.
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::drainTick()
{
    for (uint8_t h = 0; h < 2; ++h) {
        if (!log_.ready[h]) {
            continue;
        }
        const uint16_t bytes = log_.used[h];

        storage_.write(std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES));
        sendToGs(BoardId::FillingStation,
                 static_cast<uint8_t>(logic::control::persistent_state.fill_state),
                 std::span<const uint8_t>(log_.data[h], bytes), sizeof(FcuSystemState));
        log_.ready[h] = false;
    }
}

/* ---- Public surface ------------------------------------------------------- */

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::init()
{
    fill_ = detail::FillStation{};
    fill_.gs.mac  = detail::GS_MAC;
    fill_.gs.ipv4 = detail::make_ipv4(192, 168, 0, 111);
    fill_.gs.port = detail::GS_PORT;

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

    // The state machine state lives in Backup SRAM. Resume it across a reset; on
    // a cold or corrupt boot, commit a fresh INIT so the blob is valid from here
    // on. (The platform inspects the same persistent_state before bringing the
    // valves up, to decide whether to skip the normal valve-moving init.)
    logic::control::persistent_state.saveState(
        logic::control::persistent_state.loadState().value_or(logic::control::State::Init));

    // Bring the backing store online. Platform bring-up starts the record timer
    // after this init() returns, so records start flowing only once the logic
    // (and SD) are ready.
    storage_.init();
}

template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
void Controller<S, V, A, E, C>::tick(uint32_t now_ms)
{
    fill_.current_tick_ms = now_ms;

    canTick();
    rxTick();
    drainTick();         // flush full halves to SD + the GS (record production is on the timer)
    watchdogTick();      // GS-link abort watchdog

    if (logic::control::persistent_state.fill_state == logic::control::State::Init) {
        logic::control::persistent_state.saveState(logic::control::State::Safe);
    }
}

} // namespace logic::fcu
