#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "actuation/interfaces/valve.hpp"   // logic::actuation::Valve
#include "communication/interfaces/can.hpp" // logic::communication::CanFrame

#include "communication/protocol/framing/can_header.hpp"      // CanHeader
#include "communication/protocol/framing/payload_type.hpp"    // PayloadType
#include "communication/protocol/command/command_type.hpp"    // CommandType
#include "communication/protocol/response/response_type.hpp"  // ResponseType (Pong, Ack)
#include "communication/protocol/command/set_valve_position.hpp"  // ValveCommand
#include "communication/protocol/command/set_control_flag.hpp"    // ControlFlag, SetControlFlagFrame, toControlFlag
#include "communication/protocol/command/set_state.hpp"           // SetStateFrame
#include "control/control_flags.hpp"                          // logic::control::control_flags
#include "control/persistent_state.hpp"                       // logic::control::persistent_state
#include "control/state_machine.hpp"                          // toState, isTransitionAllowed (shared)
#include "control/refused_transition.hpp"                     // logic::control::last_refused_transition
#include "system/state.hpp"                                   // logic::control::State
#include "system/valves/ecu.hpp"                              // EcuValves
#include "system/board_id.hpp"

/* ------------------------------------------------------------------------- *
 * ECU control layer (HAL-free) — command handling + execution, the receive side
 * of the ECU. It decodes an inbound CAN command, checks it is addressed to us,
 * dispatches, and runs the action — actuating a propellant valve, or answering a
 * ping. The CAN-only sibling of logic::fcu::Control.
 *
 * It speaks to the wire only through the injected Communication layer; it never
 * touches can_ directly. Commands arrive as the verbatim GS command payload bridged
 * by the FCU over CAN (the same bytes the GS sent), so this decodes each payload
 * frame directly rather than via the Command parser.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

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

    /** @brief THE single point every ECU state change passes through: reject the change if the
     *         shared transition table does not permit it from the current state (recording the
     *         refused transition for telemetry); otherwise run the board's per-transition action
     *         (onTransition), commit, and report acceptance. Returns true if applied, false if
     *         refused. Used by the bridged SetState handler and the boot Init -> Safe. */
    bool transitionTo(logic::control::State to)
    {
        const auto from = logic::control::persistent_state.fill_state;
        if (!logic::control::isTransitionAllowed(from, to)) {
            logic::control::last_refused_transition = {from, to};  // surfaced in ExtendedSystemState
            return false;
        }
        onTransition(from, to);
        logic::control::persistent_state.saveState(to);
        return true;
    }

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
            case logic::communication::command::CommandType::SetControlFlag:
                handleSetControlFlag(frame, header);
                break;
            case logic::communication::command::CommandType::SetState:
                handleSetState(frame, header);
                break;
            case logic::communication::command::CommandType::Ping:
                handlePing(frame, header);
                break;
            default:
                break;
        }
    }

private:
    // Drive one of the ECU's valves from a SetValvePosition command bridged from the GS
    // (relayed by the FCU over CAN). The 3-byte SetValvePositionFrame rides verbatim in the
    // payload (data[0] = valve index, data[1] = action, data[2] = value), decoded exactly as
    // the FCU encoded it; the valve byte selects EcuValves::IPA / NOS. After driving the valve,
    // Ack back to the FCU echoing the seq so the reliable relay matches the reply and stops
    // retrying (Gs->Fcu->Ecu, Ack: Ecu->Fcu->Gs).
    void handleValveCmd(const logic::communication::CanFrame& frame, const CanHeader& header)
    {
        if (frame.length < sizeof(SetValvePositionFrame)) {
            return;  // frame too short to carry the valve frame
        }
        SetValvePositionFrame payload;
        std::memcpy(&payload, frame.data.data(), sizeof(payload));

        const auto valve_id = static_cast<EcuValves>(static_cast<uint8_t>(payload.valve));
        V* valve = (valve_id == EcuValves::IPA) ? &ipa_valve_
                 : (valve_id == EcuValves::NOS) ? &nos_valve_
                 : nullptr;
        if (valve == nullptr) {
            return;  // unknown valve id — do not Ack a command we did not apply
        }

        switch (payload.action) {
            case ValveCommand::Open:         (void)valve->open();  break;
            case ValveCommand::Close:        (void)valve->close(); break;
            case ValveCommand::SetOpenedPct: break;  // not used for the ECU's on/off propellant valves
        }

        // Generic acknowledgement: the command was received AND applied. Echoes the seq.
        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Ack),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>{});
    }

    // Apply a SetControlFlag command bridged from the GS (relayed by the FCU over CAN):
    // the 2-byte SetControlFlagFrame rides in the payload (data[0] = flag, data[1] = value).
    // Set the named runtime flag, then Ack back to the FCU echoing the command's seq so the
    // reliable relay can match the reply and stop retrying (Gs->Fcu->Ecu, Ack: Ecu->Fcu->Gs).
    void handleSetControlFlag(const logic::communication::CanFrame& frame, const CanHeader& header)
    {
        if (frame.length < sizeof(SetControlFlagFrame)) {
            return;  // frame too short to carry the flag + value
        }
        SetControlFlagFrame payload;
        std::memcpy(&payload, frame.data.data(), sizeof(payload));

        const std::optional<ControlFlag> flag = toControlFlag(static_cast<uint8_t>(payload.flag));
        if (!flag) {
            return;  // unknown flag id — do not Ack a command we did not apply
        }
        logic::control::control_flags.set(*flag, payload.value != 0);

        // Generic acknowledgement: the command was received AND applied. Echoes the seq.
        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Ack),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>{});
    }

    // Apply a SetState command bridged from the GS (relayed by the FCU over CAN): the 2-byte
    // SetStateFrame rides verbatim in the payload (data[0] = flags, data[1] = requested id).
    // The legal transitions are the SHARED table (logic::control::isTransitionAllowed) — the
    // same on both boards; the ECU's own per-transition action is onTransition(). On success,
    // Ack to the FCU echoing the seq so the reliable relay matches the reply (Gs->Fcu->Ecu,
    // Ack: Ecu->Fcu->Gs).
    void handleSetState(const logic::communication::CanFrame& frame, const CanHeader& header)
    {
        if (frame.length < sizeof(SetStateFrame)) {
            return;  // frame too short to carry the requested state
        }
        SetStateFrame payload;
        std::memcpy(&payload, frame.data.data(), sizeof(payload));

        const std::optional<logic::control::State> requested =
            logic::control::toState(payload.requestedID);
        if (!requested) {
            return;  // unknown requested state id — do not Ack
        }
        if (!transitionTo(*requested)) {
            return;  // not a permitted transition (recorded) — do not Ack a command we did not apply
        }

        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Ack),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>{});
    }

    // The ECU's per-transition action hook. The legal edges are shared with the FCU (see
    // logic::control::isTransitionAllowed); the SIDE EFFECTS are board-specific:
    //   - Any transition INTO Safe drives both propellant valves closed (people may approach).
    //   - Ignite -> Launch drives both propellant valves fully open (the FCU does nothing here).
    void onTransition(logic::control::State from, logic::control::State to)
    {
        if (to == logic::control::State::Safe) {
            (void)ipa_valve_.close();
            (void)nos_valve_.close();
        }
        if (from == logic::control::State::Ignite && to == logic::control::State::Launch) {
            (void)ipa_valve_.open();
            (void)nos_valve_.open();
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
