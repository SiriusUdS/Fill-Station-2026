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
#include "communication/protocol/command/set_control_flag.hpp"    // ControlFlagBase / FcuControlFlag, SetControlFlagFrame, toControlFlagBase
#include "communication/protocol/command/set_state.hpp"           // SetStateFrame
#include "control/control_flags.hpp"                          // logic::control::base_control_flags
#include "control/persistent_state.hpp"                       // logic::control::persistent_state
#include "control/state_machine.hpp"                          // toState, isTransitionAllowed, isTransitionLockedOut (shared)
#include "control/state_timing.hpp"                           // logic::control::state_entered_ms
#include "control/last_ping.hpp"                              // logic::control::last_ping_ms (GS-heartbeat liveness clock)
#include "control/watchdog.hpp"                               // logic::control::watchdog::kick (IWDG feed seam)
#include "control/refused_transition.hpp"                     // logic::control::last_refused_transition (+ count)
#include "control/refused_control_flag.hpp"                   // logic::control::last_refused_control_flag (+ count)
#include "control/refused_valve.hpp"                          // logic::control::last_refused_valve (+ count)
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

    /** @brief Boot-init the control layer, driving the state machine INTO the boot state.
     *
     * The controller resumes persistent_state BEFORE calling this:
     *   - a RELOAD that resumes an in-progress state (Abort / Launch / Ignite) RE-EXECUTES the
     *     transition into that state, BYPASSING the transition-table rules, so the propellant
     *     valves are driven to match the resumed state — Launch OPENS both valves, Abort CLOSES
     *     both. onTransition is edge-keyed, so we replay the canonical arming edge
     *     (reloadEntryFrom). This is intentional: a reload restores the live valve positions.
     *   - any other boot (a fresh Init, i.e. cold or corrupt Backup SRAM) transitions into
     *     Safe, which closes both propellant valves.
     * @param now_ms boot tick (the board passes 0). */
    void init(uint32_t now_ms)
    {
        const logic::control::State boot = logic::control::persistent_state.fill_state;  // resumed by the controller
        if (boot != logic::control::State::Init) {
            // Reload: re-run the entry actuation for the resumed state (rules bypassed).
            onTransition(logic::control::reloadEntryFrom(boot), boot);
            logic::control::state_entered_ms = now_ms;  // start the dwell clock for the resumed state
            // Restore the recording rate to match the resumed state (flags are not battery-backed,
            // so a warm reboot into Ignite/Launch/Abort must re-arm fast logging).
            logic::control::base_control_flags.set(
                ControlFlagBase::FastRecording, logic::control::isFastRecordingState(boot));
            return;
        }
        (void)transitionTo(logic::control::State::Safe, now_ms);  // cold boot: enter Safe (closes IPA + NOS)
    }

    /** @brief THE single point every ECU state change passes through: reject the change if the
     *         shared transition table does not permit it from the current state (recording the
     *         refused transition for telemetry); otherwise run the board's per-transition action
     *         (onTransition), commit, and report acceptance. Returns true if applied, false if
     *         refused. Used by the bridged SetState handler and the boot Init -> Safe.
     *         now_ms feeds the time-gated transition policy (e.g. the Launch -> Safe
     *         dwell lockout) and stamps the new state's entry time; the boot edge passes 0. */
    bool transitionTo(logic::control::State to, uint32_t now_ms)
    {
        const auto from = logic::control::persistent_state.fill_state;
        if (!logic::control::isTransitionAllowed(from, to) ||
            logic::control::isTransitionLockedOut(
                from, to, now_ms - logic::control::state_entered_ms)) {
            logic::control::last_refused_transition = {from, to};  // surfaced in ExtendedSystemState
            ++logic::control::refused_transition_count;
            return false;
        }
        onTransition(from, to);
        logic::control::persistent_state.saveState(to);
        logic::control::state_entered_ms = now_ms;  // start the dwell clock for the new state
        // The state machine OWNS the SD recording rate: arm fast logging for the burn window
        // (Ignite/Launch/Abort), fall back to the slow default for every resting state. This
        // overrides any manual GS FastRecording setting on each transition (see isFastRecordingState).
        logic::control::base_control_flags.set(
            ControlFlagBase::FastRecording, logic::control::isFastRecordingState(to));
        return true;
    }

    /** @brief Feed the independent watchdog each tick WHILE IN SAFE, so losing the FCU->ECU CAN
     *         bridge does not reset the board there. Safe is the state in which people are allowed
     *         near the system, and it is entered precisely for the periods (assembly) when the link
     *         is expected to be gone for minutes at a time — silence is normal there, not a dead
     *         board. In EVERY OTHER state the bridge is part of the safety case and the only feed is
     *         a serviced Ping (see handlePing), so ~30 s of silence lets the IWDG reset the board.
     *         The ECU mirrors fill_state from the FCU's bridged SetState, so both boards apply the
     *         relaxed feed over the same window. */
    void serviceWatchdog()
    {
        if (logic::control::persistent_state.fill_state == logic::control::State::Safe) {
            logic::control::watchdog::kick();
        }
    }

    /**
     * @brief Handle one inbound CAN frame: decode the header, check it is addressed
     *        to us, then dispatch the command. Telemetry/response frames and frames
     *        for other boards are ignored.
     */
    void onCommand(const logic::communication::CanFrame& frame, uint32_t now_ms)
    {
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
                handleSetState(frame, header, now_ms);
                break;
            case logic::communication::command::CommandType::Ping:
                handlePing(frame, header, now_ms);
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
            return;  // malformed: too short to carry the valve frame — silently dropped, not a refusal
        }
        SetValvePositionFrame payload;
        std::memcpy(&payload, frame.data.data(), sizeof(payload));

        // Each invariant below REFUSES-and-RECORDS (the command is addressed to us; it is invalid,
        // not malformed), so the ground station sees the denial in the ExtendedSystemState.

        // Single-board only: a Broadcast-targeted valve command is refused (the FCU never bridges
        // one, but reject it here too so the ECU never hand-drives a valve off a broadcast).
        if (static_cast<BoardId>(header.frame.target_id) != BoardId::Engine) {
            recordRefusedValve(payload);
            return;
        }
        // Operator per-valve actuation is permitted only in Unsafe — defense in depth behind the
        // FCU's own gate, since the ECU's state mirrors the FCU's. Transition-driven valve moves
        // (Safe/Abort/Launch) bypass this: they run in onTransition, not here.
        if (logic::control::persistent_state.fill_state != logic::control::State::Unsafe) {
            recordRefusedValve(payload);
            return;
        }

        if (!isValidAction(payload.action)) {
            recordRefusedValve(payload);   // unknown action — refused, not Acked
            return;
        }
        const auto valve_id = static_cast<EcuValves>(static_cast<uint8_t>(payload.valve));
        V* valve = (valve_id == EcuValves::IPA) ? &ipa_valve_
                 : (valve_id == EcuValves::NOS) ? &nos_valve_
                 : nullptr;
        if (valve == nullptr) {
            recordRefusedValve(payload);   // unknown valve id — refused, not Acked
            return;
        }

        // A non-zero force byte makes the actuation forced: the valve bypasses its limit switches
        // for FORCED_VALVE_ACTUATION_MS then reverts. For Open/Close it drives hard to the stop
        // ignoring the switch; for SetOpenedPct it holds the percent without idling or faulting on
        // a stray switch read (a forced proportional hold off both limits).
        const uint32_t bypass_ms = payload.force ? logic::control::FORCED_VALVE_ACTUATION_MS : 0;
        switch (payload.action) {
            case ValveCommand::Open:         (void)valve->open(bypass_ms);  break;
            case ValveCommand::Close:        (void)valve->close(bypass_ms); break;
            case ValveCommand::SetOpenedPct: (void)valve->setOpenPercent(static_cast<float>(payload.value), bypass_ms); break;
        }

        // Generic acknowledgement: the command was received AND applied. Echoes the seq.
        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Ack),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>{});
    }

    // Apply a SetControlFlag command bridged from the GS (relayed by the FCU over CAN): the
    // SetControlFlagFrame (16-bit flag id + value) rides verbatim in the payload. The ECU honours
    // only BASE flags (the per-board byte is FCU-specific); set the named base flag, then Ack back
    // to the FCU echoing the seq so the reliable relay matches the reply (Gs->Fcu->Ecu, Ack back).
    void handleSetControlFlag(const logic::communication::CanFrame& frame, const CanHeader& header)
    {
        if (frame.length < sizeof(SetControlFlagFrame)) {
            return;  // frame too short to carry the flag + value
        }
        SetControlFlagFrame payload;
        std::memcpy(&payload, frame.data.data(), sizeof(payload));

        // The ECU honours only BASE flags (ids 0..7); it has no per-board flags. A per-board id
        // (>= CONTROL_FLAG_BOARD_OFFSET) or an unknown base id is refused and recorded, not Acked.
        const std::optional<ControlFlagBase> flag =
            payload.flag < CONTROL_FLAG_BOARD_OFFSET ? toControlFlagBase(payload.flag)
                                                     : std::nullopt;
        if (!flag) {
            recordRefusedFlag(payload);
            return;  // unknown / per-board flag — do not Ack a command we did not apply
        }
        logic::control::base_control_flags.set(*flag, payload.value != 0);

        // Generic acknowledgement: the command was received AND applied. Echoes the seq.
        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Ack),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>{});
    }

    // Record a refused SetControlFlag (the 16-bit flag id + value + the state we were in) for the
    // ground station, surfaced in the ExtendedSystemState. Mirrors the FCU's recordRefusedFlag.
    static void recordRefusedFlag(const SetControlFlagFrame& frame)
    {
        logic::control::last_refused_control_flag = {
            frame.flag, frame.value, logic::control::persistent_state.fill_state};
        ++logic::control::refused_control_flag_count;
    }

    [[nodiscard]] static bool isValidAction(ValveCommand action)
    {
        return action == ValveCommand::Open ||
               action == ValveCommand::Close ||
               action == ValveCommand::SetOpenedPct;
    }

    // Record a refused SetValvePosition (the valve id + action + value + the state we were in) for
    // the ground station, surfaced in the ExtendedSystemState. Mirrors the FCU's recordRefusedValve.
    static void recordRefusedValve(const SetValvePositionFrame& frame)
    {
        logic::control::last_refused_valve = {
            static_cast<uint8_t>(frame.valve), static_cast<uint8_t>(frame.action), frame.value,
            logic::control::persistent_state.fill_state};
        ++logic::control::refused_valve_count;
    }

    // Apply a SetState command bridged from the GS (relayed by the FCU over CAN): the 2-byte
    // SetStateFrame rides verbatim in the payload (data[0] = flags, data[1] = requested id).
    // The legal transitions are the SHARED table (logic::control::isTransitionAllowed) — the
    // same on both boards; the ECU's own per-transition action is onTransition(). On success,
    // Ack to the FCU echoing the seq so the reliable relay matches the reply (Gs->Fcu->Ecu,
    // Ack: Ecu->Fcu->Gs).
    void handleSetState(const logic::communication::CanFrame& frame, const CanHeader& header,
                        uint32_t now_ms)
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
        if (!transitionTo(*requested, now_ms)) {
            return;  // not a permitted transition (recorded) — do not Ack a command we did not apply
        }

        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Ack),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>{});
    }

    // The ECU's per-transition action hook. The legal edges are shared with the FCU (see
    // logic::control::isTransitionAllowed); the SIDE EFFECTS are board-specific:
    //   - INTO Safe FORCE-closes both propellant valves (people may approach). Like every
    //     transition-driven actuation it is forced — limit switches bypassed for
    //     FORCED_VALVE_ACTUATION_MS, then each valve reverts to normal switch-monitoring.
    //   - INTO Abort FORCE-closes both propellant valves the same way, so the propellant shuts
    //     for certain even if a switch is flaky.
    //   - Ignite -> Launch FORCE-opens both propellant valves the same way (the FCU does nothing).
    // The force is self-contained in each valve command — it carries its own bypass window and the
    // valve auto-reverts — so a later command (e.g. the Abort close superseding a Launch open)
    // simply replaces it; there is no separate force state to unwind here.
    void onTransition(logic::control::State from, logic::control::State to)
    {
        if (to == logic::control::State::Safe) {
            // Force both propellant valves shut: a transition-driven actuation must take effect for
            // certain even if a limit switch is flaky, so it bypasses the switches for
            // FORCED_VALVE_ACTUATION_MS then each valve reverts to a normal move on its own.
            (void)ipa_valve_.close(logic::control::FORCED_VALVE_ACTUATION_MS);
            (void)nos_valve_.close(logic::control::FORCED_VALVE_ACTUATION_MS);
        }
        if (to == logic::control::State::Abort) {
            (void)ipa_valve_.close(logic::control::FORCED_VALVE_ACTUATION_MS);
            (void)nos_valve_.close(logic::control::FORCED_VALVE_ACTUATION_MS);
        }
        if (from == logic::control::State::Ignite && to == logic::control::State::Launch) {
            (void)ipa_valve_.open(logic::control::FORCED_VALVE_ACTUATION_MS);
            (void)nos_valve_.open(logic::control::FORCED_VALVE_ACTUATION_MS);
        }
    }

    // Answer a ping with a pong back to the FCU, echoing the payload AND the command's seq so the
    // FCU can match the reply to the ping it forwarded. Answered in EVERY state (Ping is never
    // state-gated) — the engine board's half of the ~1 Hz network heartbeat.
    void handlePing(const logic::communication::CanFrame& frame, const CanHeader& header,
                    uint32_t now_ms)
    {
        // --- Heartbeat received --------------------------------------------------------------
        // Stamp the GS-heartbeat liveness clock; telemetry publishes seconds_since_last_ping from it.
        logic::control::last_ping_ms = now_ms;
        // The FCU bridges the GS's ~1 Hz Ping over CAN; a serviced Ping here proves the FCU->ECU CAN
        // bridge is alive, so this is also where the ECU feeds its independent watchdog. If the bridge
        // drops, the ECU stops seeing pings and the IWDG (~30 s) resets it: a board that cannot service
        // a ping for the timeout is treated as dead.
        logic::control::watchdog::kick();
        // -------------------------------------------------------------------------------------
        comm_.sendToFcu(PayloadType::Response, static_cast<uint8_t>(ResponseType::Pong),
                        /*senderState=*/0, /*seq=*/static_cast<uint8_t>(header.frame.seq),
                        std::span<const uint8_t>(frame.data.data(), frame.length));
    }

    V&    ipa_valve_;   // injected IPA / NOS valves, commanded over CAN
    V&    nos_valve_;
    Comm& comm_;        // injected communication layer; replies to the FCU
};

} // namespace logic::ecu
