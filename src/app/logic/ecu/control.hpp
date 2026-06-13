#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "actuation/interfaces/valve.hpp"   // logic::actuation::Valve
#include "communication/interfaces/can.hpp" // logic::communication::CanFrame

#include "communication/protocol/framing/can_header.hpp"      // CanHeader
#include "communication/protocol/framing/payload_type.hpp"    // PayloadType
#include "communication/protocol/command/command_type.hpp"    // CommandType
#include "communication/protocol/response/response_type.hpp"  // ResponseType (Pong)
#include "communication/protocol/command/set_valve_position.hpp"  // ValveCommand
#include "system/valves/ecu.hpp"                              // EcuValves
#include "system/board_id.hpp"

/* ------------------------------------------------------------------------- *
 * ECU control layer (HAL-free) — command handling + execution, the receive side
 * of the ECU. It decodes an inbound CAN command, checks it is addressed to us,
 * dispatches, and runs the action — actuating a propellant valve, or answering a
 * ping. The CAN-only sibling of logic::fcu::Control.
 *
 * It speaks to the wire only through the injected Communication layer; it never
 * touches can_ directly. Commands arrive frame-encoded (the action rides in the
 * header's sender_state, the valve index in the payload), so this decodes the
 * frame directly rather than via the Command parser.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

namespace detail {

/* Byte offset of the valve index in a CommandType::SetValvePosition frame (mirrors the
   FCU's sendValveCmd: data[0..3] = timestamp, data[4] = valve index). */
inline constexpr std::size_t CMD_VALVE_INDEX_OFFSET = sizeof(uint32_t);

} // namespace detail

/**
 * @brief The ECU control layer, parameterised on the valve type it actuates and
 *        the communication layer it replies through.
 * @tparam V    logic::actuation::Valve (a BallValve in firmware; both IPA and NOS).
 * @tparam Comm The ECU Communication layer (replies to the FCU).
 */
template <logic::actuation::Valve V, typename Comm>
class Control {
public:
    /** @brief Construct over the propellant valves + the communication layer. */
    Control(V& ipa_valve, V& nos_valve, Comm& comm)
        : ipa_valve_(ipa_valve), nos_valve_(nos_valve), comm_(comm) {}

    /** @brief Boot-init the control layer. (The ECU does not boot-safe its valves
     *         today; that policy, if wanted, hooks in here as on the FCU.) */
    void init() {}

    /**
     * @brief Handle one inbound CAN frame: decode the header, check it is addressed
     *        to us, then dispatch the command. Telemetry/response frames and frames
     *        for other boards are ignored.
     */
    void onCommand(const logic::communication::CanFrame& frame, uint32_t now_ms)
    {
        (void)now_ms;
        CanHeader header;
        header.code = frame.id;

        const auto target = static_cast<BoardId>(header.frame.target_id);
        if (target != BoardId::Engine && target != BoardId::Broadcast) {
            return;  // not for us
        }
        if (static_cast<PayloadType>(header.frame.payload_type) != PayloadType::Command) {
            return;  // telemetry/response or unset — only commands route here
        }

        switch (static_cast<logic::communication::command::CommandType>(header.frame.payload_id)) {
            case logic::communication::command::CommandType::SetValvePosition:
                handleValveCmd(frame, header);
                break;
            case logic::communication::command::CommandType::Ping:
                handlePing(frame, header);
                break;
            default:
                break;
        }
    }

private:
    // Drive one of the ECU's valves from a SetValvePosition frame: the valve index is at
    // data[4] (EcuValves::IPA / NOS); the open/close action is in the header's
    // sender_state (ValveCommand::Open / ValveCommand::Close).
    void handleValveCmd(const logic::communication::CanFrame& frame, const CanHeader& header)
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

        switch (static_cast<ValveCommand>(header.frame.sender_state)) {
            case ValveCommand::Open:         (void)valve->open();  break;
            case ValveCommand::Close:        (void)valve->close(); break;
            case ValveCommand::SetOpenedPct: break;  // not used for the ECU's on/off propellant valves
        }
    }

    // Answer a ping with a pong back to the FCU, echoing the payload AND the command's
    // seq so the FCU can match the reply to the ping it sent and stop retrying.
    void handlePing(const logic::communication::CanFrame& frame, const CanHeader& header)
    {
        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Pong),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>(frame.data.data(), frame.length));
    }

    V&    ipa_valve_;   // injected IPA / NOS valves, commanded over CAN
    V&    nos_valve_;
    Comm& comm_;        // injected communication layer; replies to the FCU
};

} // namespace logic::ecu
