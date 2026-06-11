#pragma once

/* ------------------------------------------------------------------------- *
 * Controller<S> template definitions. Included at the end of fcu_controller.hpp;
 * not a standalone TU. Bodies are the former free-function controller verbatim,
 * with file-scope state now members (fill_/log_/last_adc_ms_/storage_) and the
 * storage seam called directly through storage_.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

/* ---- CAN ----------------------------------------------------------------- */

template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::sendValveCmd(uint8_t valve, uint8_t cmd)
{
    CANHeader header = {};
    header.frame.senderID    = FILLING_STATION_BOARD_ID;
    header.frame.targetID    = ENGINE_BOARD_ID;
    header.frame.deviceState = cmd;
    header.frame.messageID   = CAN_ID_CMD_VALVE;

    logic::communication::CanFrame frame;
    frame.id     = header.code;
    frame.length = static_cast<uint8_t>(frame.data.size());
    std::memcpy(frame.data.data(), &fill_.current_tick_ms, sizeof(uint32_t));  // ValveCmd timestamp
    frame.data[sizeof(uint32_t)] = valve;                                       // ValveCmd valveIndex

    (void)logic::communication::can::send(frame);
}

/* The FCU receives only status/telemetry from the ECU over CAN — never commands,
   which always arrive over Ethernet. Drain the RX ring each tick so it cannot
   back up; feeding ECU valve status into the state machine is a later step. */
template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::canTick()
{
    while (auto frame = logic::communication::can::receive()) {
        (void)frame;  // TODO: consume ECU valve status / telemetry
    }
}

/* ---- UDP (ground-station commands) --------------------------------------- */

template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::handleStateRequest(std::span<const uint8_t> payload)
{
    if (payload.size() <= detail::REQUEST_STATE_OFFSET_BYTES) {
        return;
    }
    const uint8_t requested = payload[detail::REQUEST_STATE_OFFSET_BYTES];

    switch (static_cast<uint8_t>(logic::control::persistent_state.fill_state)) {
        case FILLING_STATION_STATE_SAFE:
            if (requested != FILLING_STATION_STATE_TEST &&
                requested != FILLING_STATION_STATE_UNSAFE) return;
            break;
        case FILLING_STATION_STATE_TEST:
            if (requested != FILLING_STATION_STATE_SAFE) return;
            break;
        case FILLING_STATION_STATE_UNSAFE:
            if (requested != FILLING_STATION_STATE_SAFE &&
                requested != FILLING_STATION_STATE_IGNITE &&
                requested != FILLING_STATION_STATE_ABORT) return;
            break;
        case FILLING_STATION_STATE_IGNITE:
            if (requested != FILLING_STATION_STATE_SAFE &&
                requested != FILLING_STATION_STATE_ABORT) return;
            break;
        case FILLING_STATION_STATE_ABORT:
            if (requested != FILLING_STATION_STATE_SAFE) return;
            break;
        default:
            return;
    }
    logic::control::persistent_state.saveState(static_cast<logic::control::State>(requested));
}

template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::handleDatagram(std::span<const uint8_t> payload)
{
    if (payload.size() < sizeof(UDPPacketHeader)) {
        return;
    }

    UDPPacketHeader header;
    std::memcpy(header.bytes, payload.data(), sizeof(UDPPacketHeader));
    fill_.last_rx_ms = fill_.current_tick_ms;

    const uint8_t device = header.frame.deviceID;
    if (device != FILLING_STATION_BOARD_ID && device != BOARD_BROADCAST_ID) {
        return;  /* not addressed to us (CAN-bridge routing TODO) */
    }

    // Dispatch on the payloadID so exactly the matching handler runs.
    switch (header.frame.payloadID) {
        case REQUEST_STATE:
            handleStateRequest(payload);
            break;
        case static_cast<uint8_t>(logic::communication::command::CommandType::SetValvePosition):
            handleSetValvePosition(payload);
            break;
        default:
            break;
    }
}

// Drive one of the FCU's local valves from a SetValvePosition command. The frame
// follows the 12-byte UDP header. Open/Close ignore value; SetOpenedPct uses it.
template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::handleSetValvePosition(std::span<const uint8_t> payload)
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

template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::rxTick()
{
    logic::communication::udp::tick();  // service the link so receive() can return inbound datagrams
    while (auto datagram = logic::communication::udp::receive()) {
        handleDatagram(datagram->payload);
    }
}

/* ---- Telemetry production (per ADC sample) -------------------------------- */

// Build a SystemState from a set of channel counts. adc_ok == false (the
// fallback path) leaves the channels zero and flags the ADC fault in erno.
template <logic::storage::Storage S, logic::actuation::Valve V>
SystemState Controller<S, V>::buildSystemState(std::span<const int32_t> channels, bool adc_ok)
{
    constexpr std::size_t SLOTS = std::extent_v<decltype(SystemState::raw_adc_values)>;

    SystemState state = {};
    state.frameTs_MS         = fill_.current_tick_ms;
    state.lastHandshakeTs_MS = fill_.last_rx_ms;

    const std::size_t n = channels.size() < SLOTS ? channels.size() : SLOTS;
    for (std::size_t i = 0; i < n; ++i) {
        state.raw_adc_values[i] = static_cast<uint32_t>(channels[i]);  // bit-for-bit
    }

    // Logging / data health into sdCardFlags so the GS sees it per record:
    //   initialized  — SD mounted and ready
    //   writingError — the double buffer overran, so some records were dropped
    //   readingError — this record is a fallback (ADC silent; channels stale/zero)
    // TODO: readingError is the ADC fault parked here for now; move it to a
    // dedicated ADC/device flag set when InterfaceField gets per-device flags.
    InterfaceFieldFlags sd = {};
    sd.bits.initialized  = (storage_.status().bits.state == STORAGE_STATE_ACTIVE) ? 1 : 0;
    sd.bits.writingError = log_.overrun ? 1 : 0;
    sd.bits.readingError = adc_ok ? 0 : 1;
    state.interfaces.frame.sdCardFlags = sd;

    // The FCU's own valves report their own info, indexed by the FcuValves SSOT.
    state.valve_info[static_cast<std::size_t>(FcuValves::Fill)] = fill_valve_.info();
    state.valve_info[static_cast<std::size_t>(FcuValves::Dump)] = dump_valve_.info();

    return state;
}

// Append one record to the active half. Single-producer: the ADC ISR, or the
// fallback while the ADC is down — never both at once. When a half fills it is
// marked ready for drainTick(); if the other half is still unflushed the record
// is dropped and overrun is flagged.
template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::logAppend(const SystemState& record)
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

/* ---- Watchdogs ------------------------------------------------------------ */

// Abort if the ground-station link goes quiet while armed.
template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::watchdogTick()
{
    const uint8_t state = static_cast<uint8_t>(logic::control::persistent_state.fill_state);
    if (state == FILLING_STATION_STATE_UNSAFE || state == FILLING_STATION_STATE_IGNITE) {
        if ((fill_.current_tick_ms - fill_.last_rx_ms) >= detail::RX_WATCHDOG_MS) {
            logic::control::persistent_state.saveState(logic::control::State::Abort);
        }
    }
}

// If the ADC has gone silent, keep the pipeline alive: produce fallback records
// (no fresh ADC data, ADC fault flagged) at a gated cadence from the main loop.
template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::adcWatchdogTick()
{
    if ((fill_.current_tick_ms - last_adc_ms_) <= detail::ADC_TIMEOUT_MS) {
        return;  // ADC producing normally
    }
    if ((fill_.current_tick_ms - fill_.last_fallback_ms) < detail::FALLBACK_PERIOD_MS) {
        return;
    }
    fill_.last_fallback_ms = fill_.current_tick_ms;
    logAppend(buildSystemState({}, /*adc_ok=*/false));
}

/* ---- Telemetry drain (SD + Ethernet) -------------------------------------- */

// Split a run of records into UDP datagrams (EthernetHeader + records + CRC) and
// send them to the GS.
template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::sendBatched(std::span<const uint8_t> records)
{
    static std::array<uint8_t,
        sizeof(EthernetHeader) + detail::ETH_RECORDS_PER_PACKET * sizeof(SystemState) + sizeof(uint32_t)>
        packet;

    constexpr std::size_t batch_bytes = detail::ETH_RECORDS_PER_PACKET * sizeof(SystemState);
    for (std::size_t off = 0; off < records.size(); off += batch_bytes) {
        const std::size_t chunk =
            records.size() - off < batch_bytes ? records.size() - off : batch_bytes;

        EthernetHeader header = {};
        header.deviceID      = FILLING_STATION_BOARD_ID;
        header.payloadID     = GET_SYSTEM;
        header.payloadLenght = static_cast<uint16_t>(chunk);
        header.deviceState   = static_cast<uint8_t>(logic::control::persistent_state.fill_state);
        header.deviceTS_MS   = fill_.current_tick_ms;

        const uint32_t crc = detail::crc32(records.data() + off, chunk);
        std::memcpy(packet.data(), &header, sizeof(header));
        std::memcpy(packet.data() + sizeof(header), records.data() + off, chunk);
        std::memcpy(packet.data() + sizeof(header) + chunk, &crc, sizeof(crc));

        (void)logic::communication::udp::send(fill_.gs,
            std::span<const uint8_t>(packet.data(), sizeof(header) + chunk + sizeof(crc)));
    }
}

// Flush any full half: write the 4096-byte block to SD (sector-aligned) and
// stream its records to the GS, then release the half.
template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::drainTick()
{
    for (uint8_t h = 0; h < 2; ++h) {
        if (!log_.ready[h]) {
            continue;
        }
        const uint16_t bytes = log_.used[h];

        // TEMPORARY (remove later): stamp a marker in the tail of the 4096-byte
        // block (past the records, so it is not in the Ethernet send) to make SD
        // chunk boundaries easy to spot when reading runtime.bin.
        static constexpr char SD_CHUNK_MARKER[] = "END_OF_SD_CHUNK";
        constexpr std::size_t MARKER_LEN = sizeof(SD_CHUNK_MARKER) - 1;  // drop the NUL
        std::memcpy(&log_.data[h][detail::LOG_HALF_BYTES - MARKER_LEN], SD_CHUNK_MARKER, MARKER_LEN);

        storage_.write(std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES));
        sendBatched(std::span<const uint8_t>(log_.data[h], bytes));
        log_.ready[h] = false;
    }
}

/* ---- Public surface ------------------------------------------------------- */

template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::init()
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

    // Bring the backing store online. Platform bring-up connects onAdcSample to
    // the streaming ADC after this init() returns, so records start flowing only
    // once the logic (and SD) are ready.
    storage_.init();
}

template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::onAdcSample(std::span<const int32_t> channels)
{
    last_adc_ms_ = fill_.current_tick_ms;
    logAppend(buildSystemState(channels, /*adc_ok=*/true));
}

template <logic::storage::Storage S, logic::actuation::Valve V>
void Controller<S, V>::tick(uint32_t now_ms)
{
    fill_.current_tick_ms = now_ms;

    canTick();
    rxTick();
    adcWatchdogTick();   // fallback production if the ADC is silent
    drainTick();         // flush full halves to SD + the GS
    watchdogTick();      // GS-link abort watchdog

    if (logic::control::persistent_state.fill_state == logic::control::State::Init) {
        logic::control::persistent_state.saveState(logic::control::State::Safe);
    }
}

} // namespace logic::fcu
