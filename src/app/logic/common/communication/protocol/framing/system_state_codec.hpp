#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "communication/interfaces/can.hpp"                     // logic::communication::CanFrame
#include "communication/protocol/telemetry/system_state.hpp"    // SystemState
#include "framing/can_header.hpp"                               // CanHeader
#include "communication/protocol/telemetry/telemetry_id.hpp"    // TelemetryId::SystemState
#include "communication/protocol/system/board_ids.hpp"          // BoardId

/* ------------------------------------------------------------------------- *
 * SystemState <-> CAN fragment codec (HAL-free, shared by both boards).
 *
 * A SystemState is larger than a CAN frame's 8 data bytes, so it is downlinked as a
 * fixed run of fragments — the ECU packs them and sends; the FCU reassembles them
 * (to relay to the GS). The packer/unpacker live here in common so both ends agree.
 *
 * Wire format, per fragment (one CAN frame):
 *   - 29-bit id  = CanHeader { senderID, targetID, messageID = TelemetryId::SystemState,
 *                              deviceState = 4-bit record sequence }
 *   - data[0]    = fragment index (0 .. SYSTEM_STATE_FRAGMENTS-1)
 *   - data[1..]  = up to FRAGMENT_PAYLOAD_BYTES of the record's bytes
 * The record sequence lets the receiver detect record boundaries and drop a record
 * cleanly if a fragment is lost (a new sequence resets the in-progress record).
 * ------------------------------------------------------------------------- */

namespace logic::communication::can {

inline constexpr std::size_t FRAGMENT_PAYLOAD_BYTES = 7;   // 8 data bytes - 1 index byte
inline constexpr std::size_t SYSTEM_STATE_FRAGMENTS =
    (sizeof(SystemState) + FRAGMENT_PAYLOAD_BYTES - 1) / FRAGMENT_PAYLOAD_BYTES;

static_assert(SYSTEM_STATE_FRAGMENTS <= 16,
              "fragment index/mask scheme assumes <=16 fragments; SystemState too large");

/* Split one SystemState into SYSTEM_STATE_FRAGMENTS CAN frames addressed
   sender -> target, tagged with the 4-bit record sequence seq. Fills `out`. */
inline void packSystemState(const SystemState& record, BoardId sender, BoardId target,
                            uint8_t seq, std::span<CanFrame, SYSTEM_STATE_FRAGMENTS> out)
{
    const auto* bytes = reinterpret_cast<const uint8_t*>(&record);

    CanHeader header = {};
    header.frame.senderID    = static_cast<uint8_t>(sender);
    header.frame.targetID    = static_cast<uint8_t>(target);
    header.frame.deviceState = seq & 0x0F;
    header.frame.messageID   = static_cast<uint8_t>(TelemetryId::SystemState);

    for (std::size_t i = 0; i < SYSTEM_STATE_FRAGMENTS; ++i) {
        CanFrame& f = out[i];
        f.id   = header.code;
        f.data = {};                            // zero all 8 bytes
        f.data[0] = static_cast<uint8_t>(i);    // fragment index
        const std::size_t off = i * FRAGMENT_PAYLOAD_BYTES;
        const std::size_t n   = std::min(FRAGMENT_PAYLOAD_BYTES, sizeof(SystemState) - off);
        std::memcpy(&f.data[1], bytes + off, n);
        f.length = static_cast<uint8_t>(1 + n);
    }
}

/* Reassembles SystemState records from inbound TelemetryId::SystemState fragments, one
   record at a time. Feed every frame; a completed record is returned exactly once. */
class SystemStateReassembler {
public:
    std::optional<SystemState> accept(const CanFrame& frame)
    {
        CanHeader header;
        header.code = frame.id;
        if (static_cast<TelemetryId>(header.frame.messageID) != TelemetryId::SystemState || frame.length < 1) {
            return std::nullopt;
        }
        const uint8_t seq = static_cast<uint8_t>(header.frame.deviceState) & 0x0F;
        const uint8_t idx = frame.data[0];
        if (idx >= SYSTEM_STATE_FRAGMENTS) {
            return std::nullopt;
        }

        if (seq != seq_ || got_mask_ == 0) {   // a new record (or the first ever)
            seq_      = seq;
            got_mask_ = 0;
        }

        const std::size_t off = static_cast<std::size_t>(idx) * FRAGMENT_PAYLOAD_BYTES;
        const std::size_t n   = std::min(FRAGMENT_PAYLOAD_BYTES, sizeof(SystemState) - off);
        std::memcpy(buffer_ + off, &frame.data[1], n);
        got_mask_ |= static_cast<uint16_t>(1u << idx);

        if (got_mask_ == ALL_FRAGMENTS) {
            SystemState record;
            std::memcpy(&record, buffer_, sizeof(SystemState));
            got_mask_ = 0;   // ready for the next record
            return record;
        }
        return std::nullopt;
    }

private:
    static constexpr uint16_t ALL_FRAGMENTS =
        static_cast<uint16_t>((1u << SYSTEM_STATE_FRAGMENTS) - 1u);

    uint8_t  buffer_[SYSTEM_STATE_FRAGMENTS * FRAGMENT_PAYLOAD_BYTES];
    uint16_t got_mask_ = 0;
    uint8_t  seq_      = 0xFF;
};

} // namespace logic::communication::can
