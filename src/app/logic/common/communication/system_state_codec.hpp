#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "communication/interfaces/can.hpp"                     // logic::communication::CanFrame
#include "communication/protocol/telemetry/ecu_system_state.hpp"  // EcuSystemState (high-rate record fragmented over CAN)
#include "communication/protocol/telemetry/ecu_extended_system_state.hpp"  // EcuExtendedSystemState (low-rate record)
#include "communication/protocol/framing/can_header.hpp"                               // CanHeader
#include "communication/protocol/framing/payload_type.hpp"                             // PayloadType::Telemetry
#include "communication/protocol/telemetry/telemetry_type.hpp"  // TelemetryType::SystemState / ExtendedSystemState
#include "communication/protocol/system/board_id.hpp"          // BoardId

/* ------------------------------------------------------------------------- *
 * ECU-telemetry-record <-> CAN fragment codec (HAL-free).
 *
 * The ECU's telemetry records (the high-rate EcuSystemState and the low-rate
 * EcuExtendedSystemState) are downlinked to the FCU over CAN — the ECU packs each
 * record into a fixed run of fragments and sends; the FCU reassembles it (to relay
 * to the GS). The fragment mechanism is record-agnostic, so it is a template
 * parameterised on the record type, with thin SystemState / ExtendedSystemState
 * aliases below. The packer/unpacker live here in common so both ends agree. Only
 * the ECU's records travel over CAN; the FCU downlinks its own records straight over
 * Ethernet, so it never needs this codec.
 *
 * Wire format, per fragment (one CAN frame):
 *   - 29-bit id  = CanHeader { sender_id, target_id, sender_state = the ECU's state-machine
 *                              state, payload_id = the record's TelemetryType,
 *                              seq = 4-bit record sequence }
 *   - data[0]    = fragment index (0 .. fragmentCount-1)
 *   - data[1..]  = up to FRAGMENT_PAYLOAD_BYTES of the record's bytes
 * sender_state carries the engine board's state on every fragment, so the FCU relays it to the
 * ground station (in EthernetHeader.sender_state) exactly as it does its own. seq is the record
 * sequence: it lets the receiver detect record boundaries and drop a record cleanly if a
 * fragment is lost (a new sequence resets the in-progress record). payload_id discriminates the
 * record streams, so a reassembler for one record type ignores the other's fragments.
 * ------------------------------------------------------------------------- */

namespace logic::communication::can {

inline constexpr std::size_t FRAGMENT_PAYLOAD_BYTES = MAX_PAYLOAD_LENGTH_BYTES - 1;  // frame data minus the 1 index byte (63 on CAN-FD)

/* Number of CAN fragments one telemetry record splits into (1 each now, on CAN-FD). */
template <typename Record>
inline constexpr std::size_t fragmentCountFor =
    (sizeof(Record) + FRAGMENT_PAYLOAD_BYTES - 1) / FRAGMENT_PAYLOAD_BYTES;

/* Split one telemetry record into fragmentCountFor<Record> CAN frames addressed
   sender -> target, tagging every fragment with the record's TelemetryType, the sender's
   state-machine state senderState, and the 4-bit record sequence seq. Fills `out`. */
template <typename Record>
inline void packTelemetryRecord(const Record& record, TelemetryType payloadId,
                                BoardId sender, BoardId target,
                                uint8_t senderState, uint8_t seq,
                                std::span<CanFrame, fragmentCountFor<Record>> out)
{
    const auto* bytes = reinterpret_cast<const uint8_t*>(&record);

    CanHeader header = {};
    header.frame.sender_id    = static_cast<uint8_t>(sender);
    header.frame.target_id    = static_cast<uint8_t>(target);
    header.frame.sender_state = senderState & 0x0F;  // the sender board's state-machine state
    header.frame.payload_type = static_cast<uint8_t>(PayloadType::Telemetry);
    header.frame.payload_id   = static_cast<uint8_t>(payloadId);
    header.frame.seq          = seq & 0x0F;          // record sequence: boundary / drop detection
    header.frame.priority     = canBusPriority(PayloadType::Telemetry);  // low: yields to commands/responses

    for (std::size_t i = 0; i < fragmentCountFor<Record>; ++i) {
        CanFrame& f = out[i];
        f.id   = header.code;
        f.data = {};                            // zero all payload bytes
        f.data[0] = static_cast<uint8_t>(i);    // fragment index
        const std::size_t off = i * FRAGMENT_PAYLOAD_BYTES;
        const std::size_t n   = std::min(FRAGMENT_PAYLOAD_BYTES, sizeof(Record) - off);
        std::memcpy(&f.data[1], bytes + off, n);
        f.length = static_cast<uint8_t>(1 + n);
    }
}

/* Reassembles records of one type from inbound fragments tagged PayloadId, one record at a
   time. Feed every frame; fragments of a different TelemetryType are ignored, so two
   reassemblers can share the same CAN ingress. A completed record is returned exactly once. */
template <typename Record, TelemetryType PayloadId>
class TelemetryReassembler {
public:
    std::optional<Record> accept(const CanFrame& frame)
    {
        CanHeader header;
        header.code = frame.id;
        if (static_cast<PayloadType>(header.frame.payload_type) != PayloadType::Telemetry
            || static_cast<TelemetryType>(header.frame.payload_id) != PayloadId
            || frame.length < 1) {
            return std::nullopt;
        }
        const uint8_t seq = static_cast<uint8_t>(header.frame.seq) & 0x0F;
        const uint8_t idx = frame.data[0];
        if (idx >= FRAGMENTS) {
            return std::nullopt;
        }

        if (seq != seq_ || got_mask_ == 0) {   // a new record (or the first ever)
            seq_      = seq;
            got_mask_ = 0;
        }

        const std::size_t off = static_cast<std::size_t>(idx) * FRAGMENT_PAYLOAD_BYTES;
        const std::size_t n   = std::min(FRAGMENT_PAYLOAD_BYTES, sizeof(Record) - off);
        std::memcpy(buffer_ + off, &frame.data[1], n);
        got_mask_ |= static_cast<uint16_t>(1u << idx);

        if (got_mask_ == ALL_FRAGMENTS) {
            Record record;
            std::memcpy(&record, buffer_, sizeof(Record));
            got_mask_ = 0;   // ready for the next record
            return record;
        }
        return std::nullopt;
    }

private:
    static constexpr std::size_t FRAGMENTS = fragmentCountFor<Record>;
    static constexpr uint16_t    ALL_FRAGMENTS =
        static_cast<uint16_t>((1u << FRAGMENTS) - 1u);

    uint8_t  buffer_[FRAGMENTS * FRAGMENT_PAYLOAD_BYTES];
    uint16_t got_mask_ = 0;
    uint8_t  seq_      = 0xFF;
};

/* ---- The two ECU telemetry record streams over CAN ---------------------- */

inline constexpr std::size_t SYSTEM_STATE_FRAGMENTS   = fragmentCountFor<EcuSystemState>;
inline constexpr std::size_t EXTENDED_STATE_FRAGMENTS = fragmentCountFor<EcuExtendedSystemState>;

static_assert(SYSTEM_STATE_FRAGMENTS <= 16,
              "fragment index/mask scheme assumes <=16 fragments; SystemState too large");
static_assert(EXTENDED_STATE_FRAGMENTS <= 16,
              "fragment index/mask scheme assumes <=16 fragments; ExtendedSystemState too large");

/* Pack the high-rate EcuSystemState (tagged TelemetryType::SystemState). */
inline void packSystemState(const EcuSystemState& record, BoardId sender, BoardId target,
                            uint8_t senderState, uint8_t seq,
                            std::span<CanFrame, SYSTEM_STATE_FRAGMENTS> out)
{
    packTelemetryRecord(record, TelemetryType::SystemState, sender, target, senderState, seq, out);
}

/* Pack the low-rate EcuExtendedSystemState (tagged TelemetryType::ExtendedSystemState). */
inline void packExtendedSystemState(const EcuExtendedSystemState& record, BoardId sender, BoardId target,
                                    uint8_t senderState, uint8_t seq,
                                    std::span<CanFrame, EXTENDED_STATE_FRAGMENTS> out)
{
    packTelemetryRecord(record, TelemetryType::ExtendedSystemState, sender, target, senderState, seq, out);
}

using SystemStateReassembler   = TelemetryReassembler<EcuSystemState, TelemetryType::SystemState>;
using ExtendedStateReassembler = TelemetryReassembler<EcuExtendedSystemState, TelemetryType::ExtendedSystemState>;

} // namespace logic::communication::can
