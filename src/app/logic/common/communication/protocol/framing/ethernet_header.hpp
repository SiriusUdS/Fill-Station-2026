#pragma once
#include <stdint.h>
/**
 * UDP PACKET:
 *   EthernetHeader (12 bytes)
 *   Payload (length is a multiple of 4 bytes)
 *   CRC (4 bytes)
 */

/// @brief Fixed-length header: the first bytes read to interpret the payload. It
/// routes the frame (sender/target) and carries the sender's real-time data.
/// sender_id and target_id carry BoardId values (mirroring FrameCanHeader so the
/// CAN and Ethernet routing fields line up). The payload length MUST be a multiple
/// of 4 bytes (CRC not included).
typedef struct {
    uint32_t sender_id: 4;          /**< BoardId that emitted the frame. */
    uint32_t target_id: 4;          /**< Destination BoardId (or Broadcast). */
    uint32_t payload_type: 2;       /**< Command vs telemetry class. */
    uint32_t payload_id: 6;         /**< A CommandType or a TelemetryType. */
    uint32_t payload_size_bytes: 16;
    uint8_t  sender_state;
    uint8_t  seq;                   /**< Command/response sequence: tags a command so its reply is
                                         matched to it and retried; the receiver echoes it. 0 for
                                         telemetry. Mirrors FrameCanHeader::seq. */
    uint8_t  reserved[2];           /**< Explicit padding bytes — keeps the layout padding-free. */
    uint32_t sender_timestamp_ms;   /**< Empty if sender is GS (Command). */
} EthernetHeader;

// Guard the wire layout: the struct must be exactly 12 bytes with no implicit
// padding. If a future field change reintroduces padding, the sum-of-members
// assert fails — add an explicit reservedN byte to close the gap.
static_assert(sizeof(EthernetHeader) == 12,
              "EthernetHeader must be exactly 12 bytes (UDP packet header wire format)");
static_assert(sizeof(EthernetHeader) == sizeof(uint32_t)   // sender/target/payload_type/payload_id/payload_size bit-fields
                                      + sizeof(uint8_t)     // sender_state
                                      + sizeof(uint8_t)     // seq
                                      + sizeof(uint8_t[2])  // reserved
                                      + sizeof(uint32_t),   // sender_timestamp_ms
              "EthernetHeader has implicit padding — add explicit reserved bytes to close the gap");

