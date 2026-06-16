#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "actuation/interfaces/valve.hpp"   // logic::actuation::Valve

#include "communication/command/command.hpp"                   // Command + CommandType
#include "communication/command/parser/command_ethernet_parser.hpp"  // fromEthernet
#include "communication/protocol/command/set_state.hpp"        // SetStateFrame
#include "communication/protocol/command/set_valve_position.hpp"  // SetValvePositionFrame, ValveCommand
#include "communication/protocol/command/set_control_flag.hpp"    // ControlFlagBase / FcuControlFlag, SetControlFlagFrame, toControlFlagBase
#include "communication/protocol/framing/payload_type.hpp"     // PayloadType
#include "communication/protocol/response/response_type.hpp"   // ResponseType (Pong, Ack)
#include "system/valves/fcu.hpp"                               // FcuValves
#include "system/board_id.hpp"
#include "system/state.hpp"                                    // logic::control::State
#include "control/persistent_state.hpp"                        // logic::control::persistent_state
#include "control/state_machine.hpp"                           // toState, isTransitionAllowed, isTransitionLockedOut (shared)
#include "control/state_timing.hpp"                            // logic::control::state_entered_ms
#include "control/refused_transition.hpp"                      // logic::control::last_refused_transition
#include "control/refused_control_flag.hpp"                     // logic::control::last_refused_control_flag
#include "control/refused_valve.hpp"                            // logic::control::last_refused_valve
#include "control/control_flags.hpp"                           // logic::control::base_control_flags / fcu_control_flags
#include "actuation/interfaces/ematch.hpp"                     // logic::actuation::Ematch (the igniter seam)
#include "actuation/interfaces/solenoid.hpp"                   // logic::actuation::Solenoid (the solenoid-valve seam)
#include "actuation/interfaces/heater.hpp"                     // logic::actuation::Heater (the heater seam)

/* ------------------------------------------------------------------------- *
 * FCU control layer (HAL-free) — command handling + execution, the receive side
 * of the FCU. It parses an inbound datagram into a transport-agnostic Command,
 * gates it on the current state, dispatches to the matching handler, and runs the
 * action — committing a state change, actuating a local valve, or forwarding a
 * command to the ECU through the communication layer.
 *
 * This folds in the former free-function logic::control::command_handlers module:
 * the validate-then-act handlers and the state transition table now live here as
 * methods, which finally gives them the transport access (Comm) they need to
 * actually emit — e.g. forwarding a Ping to the ECU. It speaks to the wire only
 * through the injected Communication layer; it never touches eth_/can_ directly.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

namespace detail {

/* A reliable FCU->ECU command (CAN auto-retransmission is off) is resent if no matching
   response arrives within COMMAND_TIMEOUT_MS, up to MAX_COMMAND_RETRIES times; after that
   it is abandoned and the FCU<->ECU link is treated as down. */
inline constexpr uint32_t COMMAND_TIMEOUT_MS  = 50;
inline constexpr uint8_t  MAX_COMMAND_RETRIES = 3;

} // namespace detail

/**
 * @brief The FCU control layer, parameterised on the valve type it actuates and
 *        the communication layer it forwards through.
 * @tparam V    logic::actuation::Valve (a BallValve in firmware; both Fill and Dump).
 * @tparam Comm The FCU Communication layer (forwards commands to the ECU, the pong to the GS).
 * @tparam EM   logic::actuation::Ematch (the igniter line, energised in the Ignite state).
 * @tparam SOL  logic::actuation::Solenoid (the solenoid valve, open only in the Unsafe state).
 * @tparam HTR  logic::actuation::Heater (the heater, driven straight from its control flag).
 */
template <logic::actuation::Valve V, typename Comm, logic::actuation::Ematch EM,
          logic::actuation::Solenoid SOL, logic::actuation::Heater HTR>
class Control {
public:
    /** @brief Construct over the local valves, the communication layer, the e-match, the solenoid,
     *         and the heater. */
    Control(V& fill_valve, V& dump_valve, Comm& comm, EM& ematch, SOL& solenoid, HTR& heater)
        : fill_valve_(fill_valve), dump_valve_(dump_valve), comm_(comm), ematch_(ematch),
          solenoid_(solenoid), heater_(heater) {}

    /** @brief Boot-init the control layer: reset the GS-link liveness clock and drive
     *         the local valves to a safe (closed) position.
     *
     * Actuation is the control layer's authority, so boot-safing the valves lives here
     * rather than in board bring-up. The controller resumes persistent_state before
     * calling this, so it CAN be made state-aware — e.g. skip safing on a warm reboot
     * that resumes into an armed state mid-operation. TODO: decide that policy; for now
     * it unconditionally safes, matching the previous board-level behaviour. */
    void init()
    {
        (void)fill_valve_.close();
        (void)dump_valve_.close();
        ematch_.deenergise(0);  // boot-safe the firing line through the logic seam (like the valves); t=0
        solenoid_.close(0);     // boot-safe the solenoid closed through the logic seam; t=0
        heater_.off(0);         // boot-safe the heater off through the logic seam; t=0
    }

    /** @brief Sample the e-match detect line and mirror it onto the continuity LED. Called
     *         every controller tick; cheap and non-blocking (a GPIO read + a GPIO write). */
    void serviceEmatch() { (void)ematch_.poll(); }

    /** @brief Service the solenoid valve each tick: sample its detect line onto the continuity
     *         LED, then enforce its actuation policy — open ONLY while the SolenoidValve flag is
     *         set AND the board is in Unsafe; closed otherwise (so it auto-closes on leaving
     *         Unsafe). Cheap and non-blocking; the open/close edge ticks are stamped by the
     *         solenoid only on an actual state change. */
    void serviceSolenoid(uint32_t now_ms)
    {
        (void)solenoid_.poll();
        const bool want_open =
            logic::control::fcu_control_flags.get(FcuControlFlag::SolenoidValve) &&
            logic::control::persistent_state.fill_state == logic::control::State::Unsafe;
        if (want_open) {
            solenoid_.open(now_ms);
        } else {
            solenoid_.close(now_ms);
        }
    }

    /** @brief Service the heater each tick: drive it on/off straight from the Heater control
     *         flag. Unlike the solenoid the heater is NOT state-gated — it follows its flag in
     *         any state. Cheap and non-blocking; the on/off edge ticks are stamped by the heater
     *         only on an actual state change. */
    void serviceHeater(uint32_t now_ms)
    {
        if (logic::control::fcu_control_flags.get(FcuControlFlag::Heater)) {
            heater_.on(now_ms);
        } else {
            heater_.off(now_ms);
        }
    }

    /**
     * @brief Handle one inbound ground-station datagram: parse it into a Command,
     *        check it is addressed to us, then gate + dispatch.
     */
    void onDatagram(std::span<const uint8_t> payload, uint32_t now_ms)
    {
        const auto cmd = logic::communication::command::fromEthernet(payload);
        if (!cmd) {
            return;  // not a command / malformed / unknown id
        }
        if (!addressedToUs(static_cast<BoardId>(cmd->target))) {
            return;  // addressed to a board we neither act for nor bridge to
        }
        (void)handleCommand(*cmd, now_ms);
    }

    /** @brief A response (Pong or Ack) arrived from the ECU over CAN carrying the seq it
     *         answers: if it matches the in-flight relayed command, stop retrying it; then
     *         relay it to the GS CARRYING that seq AND its response id, so the GS matches it
     *         to the command it sent (Ecu->Fcu->Gs). One reliable command is outstanding at
     *         a time, so the seq alone identifies which command this answers. */
    void onResponse(uint8_t responseId, uint8_t seq, uint32_t now_ms)
    {
        // The response answers the command we relayed: if it echoes the seq of our
        // outstanding command, the ECU got it — clear the pending slot so servicePending()
        // stops resending. Pong answers Ping, Ack answers any other relayed command alike.
        if (pending_.active && pending_.seq == static_cast<uint8_t>(seq & 0x0F)) {
            pending_.active = false;
        }
        // Relay to the GS, propagating the seq + response id so the GS matches command<->response.
        comm_.sendToGs(BoardId::Engine, PayloadType::Response, responseId, /*sourceState=*/0,
                       /*seq=*/seq, std::span<const uint8_t>{}, now_ms);
    }

    /** @brief Resend the in-flight reliable command if its response has not arrived in
     *         time, up to MAX_COMMAND_RETRIES; after that abandon it (FCU<->ECU down). */
    void servicePending(uint32_t now_ms)
    {
        if (!pending_.active) {
            return;
        }
        if ((now_ms - pending_.sent_ms) < detail::COMMAND_TIMEOUT_MS) {
            return;  // still within the response window
        }
        if (pending_.retries < detail::MAX_COMMAND_RETRIES) {
            ++pending_.retries;
            pending_.sent_ms = now_ms;
            comm_.sendToEcu(PayloadType::Command, pending_.payload_id, pending_.sender_state,
                            pending_.seq,
                            std::span<const uint8_t>(pending_.payload.data(), pending_.payload_len));
        } else {
            // Gave up: the ECU never answered after MAX_COMMAND_RETRIES. This is the seam
            // for the FCU-local reaction when GS<->FCU works but FCU<->ECU does NOT.
            // TODO(FCU-local ping reaction): act on the dead FCU<->ECU link here.
            pending_.active = false;
        }
    }

    /** @brief THE single point every FCU state change passes through: reject the change if the
     *         shared transition table does not permit it from the current state; otherwise run
     *         the board's per-transition action (onTransition), commit the new state, and report
     *         acceptance. Returns true if applied, false if refused. Every path uses this —
     *         commanded SetState, the boot Init -> Safe. now_ms timestamps
     *         the per-transition side effects (e.g. the e-match energise/deenergise edges). */
    bool transitionTo(logic::control::State to, uint32_t now_ms)
    {
        const State from = logic::control::persistent_state.fill_state;
        if (!logic::control::isTransitionAllowed(from, to) ||
            logic::control::isTransitionLockedOut(
                from, to, now_ms - logic::control::state_entered_ms)) {
            logic::control::last_refused_transition = {from, to};  // surfaced in ExtendedSystemState
            ++logic::control::refused_transition_count;
            return false;
        }
        onTransition(from, to, now_ms);
        logic::control::persistent_state.saveState(to);
        logic::control::state_entered_ms = now_ms;  // start the dwell clock for the new state
        return true;
    }

private:

    using Command     = logic::communication::command::Command;
    using CommandType = logic::communication::command::CommandType;
    using State       = logic::control::State;

    /* ---- Addressing + gate + dispatch ---------------------------------------- */

    // Should the FCU act on a command with this target? Yes for FillingStation / Broadcast
    // (we are, or are among, the addressees) and yes for Engine: every command targeted at the
    // ECU is bridgeable by definition — the ECU is reachable only through us, so we accept it
    // solely to relay it on over CAN. A target of any other board is not ours.
    [[nodiscard]] static bool addressedToUs(BoardId target)
    {
        return target == BoardId::FillingStation
            || target == BoardId::Broadcast
            || target == BoardId::Engine;
    }

    // May a command of `type` run at all (is it a known command)? This is the dispatch gate;
    // per-state validity is enforced inside each handler, which also RECORDS the refusal for
    // telemetry (SetState in transitionTo, SetControlFlag in applyControlFlagLocally,
    // SetValvePosition in handleSetValvePosition — the operator per-valve gate to Unsafe). Keeping
    // those gates in the handlers, not here, is what lets each record a typed refusal.
    [[nodiscard]] static bool canExecute(CommandType type, State current)
    {
        (void)current;
        switch (type) {
            case CommandType::Ping:
            case CommandType::SetState:
            case CommandType::SetValvePosition:
            case CommandType::SetControlFlag:
            case CommandType::Synchronise:
                return true;
        }
        return false;  // unknown command type
    }

    // Gate on the current state, then dispatch to the matching handler.
    bool handleCommand(const Command& cmd, uint32_t now_ms)
    {
        if (!canExecute(cmd.type, logic::control::persistent_state.fill_state)) {
            return false;
        }
        switch (cmd.type) {
            case CommandType::Ping:             return handlePing(cmd, now_ms);
            case CommandType::SetState:         return handleSetState(cmd, now_ms);
            case CommandType::SetValvePosition: return handleSetValvePosition(cmd, now_ms);
            case CommandType::SetControlFlag:   return handleSetControlFlag(cmd, now_ms);
            case CommandType::Synchronise:      return handleSynchronise(cmd, now_ms);
        }
        return false;  // unknown command type
    }

    // Ack the GS directly for a command the FCU handled itself (no ECU hop), echoing the GS's
    // seq so the ground station matches the reply to the command it sent. Every locally-applied
    // command Acks through here on success; a refused command is NOT Acked (the GS times out and
    // can read the refusal in the ExtendedSystemState). Bridged (Engine) commands are instead
    // Acked by the ECU, relayed to the GS by onResponse.
    void ackGs(uint8_t seq, uint32_t now_ms)
    {
        comm_.sendToGs(BoardId::FillingStation, PayloadType::Response,
                       static_cast<uint8_t>(ResponseType::Ack), /*sourceState=*/0,
                       /*seq=*/seq, std::span<const uint8_t>{}, now_ms);
    }

    /* ---- Ping (Gs->Fcu->Ecu) ------------------------------------------------- */

    // Reaching here means a Ping from the GS arrived: the GS<->FCU leg works. Forward it
    // to the ECU over CAN as a RELIABLE command, PROPAGATING the GS's seq — the ECU echoes
    // it in its Pong, onResponse matches it (stopping retries) and relays the Pong to the GS
    // carrying that same seq, so the GS can match its own ping. The "FCU<->ECU is down"
    // seam lives in servicePending's give-up branch, not here.
    bool handlePing(const Command& cmd, uint32_t now_ms)
    {
        sendReliable(CommandType::Ping, /*sender_state=*/0, std::span<const uint8_t>{}, cmd.seq, now_ms);
        return true;
    }

    // Send a command to the ECU and record it as the single in-flight reliable command:
    // it carries @p seq (the GS's seq for forwarded commands, or a freshly generated one
    // for FCU-originated commands) and is resent by servicePending() until a response
    // echoes that seq. At most one reliable command is outstanding at a time, which is
    // all Ping needs; a second call overwrites the slot.
    void sendReliable(CommandType type, uint8_t sender_state, std::span<const uint8_t> payload,
                      uint8_t seq, uint32_t now_ms)
    {
        pending_.active       = true;
        pending_.payload_id   = static_cast<uint8_t>(type);
        pending_.sender_state = sender_state;
        pending_.seq          = static_cast<uint8_t>(seq & 0x0F);
        pending_.sent_ms      = now_ms;
        pending_.retries      = 0;
        pending_.payload_len  = static_cast<uint8_t>(
            payload.size() < pending_.payload.size() ? payload.size() : pending_.payload.size());
        if (pending_.payload_len != 0) {
            std::memcpy(pending_.payload.data(), payload.data(), pending_.payload_len);
        }
        comm_.sendToEcu(PayloadType::Command, pending_.payload_id, sender_state, pending_.seq, payload);
    }

    /* ---- SetState (commits through persistent_state) ------------------------- */

    // Route by target like handleSetControlFlag: apply to the FCU's own state machine when we
    // are an addressee (FillingStation / Broadcast) and Ack the GS directly, and bridge to the
    // ECU over CAN when the Engine is addressed (Engine / Broadcast) so both boards' state
    // machines follow the GS command. The 2-byte SetStateFrame rides verbatim in the bridged CAN
    // payload; the ECU applies it and Acks, and onResponse relays that Ack to the GS.
    bool handleSetState(const Command& cmd, uint32_t now_ms)
    {
        const auto target = static_cast<BoardId>(cmd.target);
        bool ok = true;

        if (target == BoardId::FillingStation || target == BoardId::Broadcast) {
            ok = applyStateLocally(cmd, now_ms);
        }
        if (target == BoardId::Engine || target == BoardId::Broadcast) {
            sendReliable(CommandType::SetState, /*sender_state=*/0,
                         std::span<const uint8_t>(cmd.payload.data(), sizeof(SetStateFrame)),
                         cmd.seq, now_ms);
        }
        return ok;
    }

    // Apply a SetState to the FCU's own (global) state machine: validate the requested id and
    // the transition, run any per-transition action, then commit through persistent_state.
    bool applyStateLocally(const Command& cmd, uint32_t now_ms)
    {
        const auto* frame = reinterpret_cast<const SetStateFrame*>(cmd.payload.data());
        const std::optional<State> requested = logic::control::toState(frame->requestedID);
        if (!requested) {
            return false;  // unknown requested state id
        }
        if (!transitionTo(*requested, now_ms)) {   // validates, routes the action + commit, records refusals
            return false;  // refused transition (recorded) — do not Ack a command we did not apply
        }
        ackGs(cmd.seq, now_ms);
        return true;
    }

    // The FCU's per-transition action hook. The legal edges are shared with the ECU (see
    // logic::control::isTransitionAllowed); the SIDE EFFECTS are board-specific:
    //   - Any transition INTO Safe drives the local Fill/Dump valves closed (people may
    //     approach the system, so it must hold no flow).
    //   - Any transition INTO Abort closes Fill and OPENS Dump (vent the line), independent
    //     of the operator valve-command gate, which is the abort side effect for the FCU.
    //   - On Unsafe -> Ignite the FCU energises the e-match firing line; on leaving Ignite
    //     by ANY path (Launch, Abort, Safe) it de-energises it, so the igniter is never left
    //     hot once Ignite is exited. (Only Unsafe reaches Ignite per the transition table.)
    void onTransition(State from, State to, uint32_t now_ms)
    {
        if (to == State::Safe) {
            (void)fill_valve_.close();
            (void)dump_valve_.close();
        }
        if (to == State::Abort) {
            (void)fill_valve_.close();  // stop the fill
            (void)dump_valve_.open();   // vent the line
        }
        if (from == State::Unsafe) {
            // The solenoid valve is only armable in Unsafe; clear its flag on the way out so it
            // can't re-open on a later re-entry into Unsafe without a fresh command. serviceSolenoid
            // closes the valve itself once the flag is clear / the state is no longer Unsafe.
            logic::control::fcu_control_flags.set(FcuControlFlag::SolenoidValve, false);
        }
        if (from == State::Unsafe && to == State::Ignite) {
            ematch_.energise(now_ms);    // hold the e-match firing line high for the Ignite state
        } else if (from == State::Ignite) {
            ematch_.deenergise(now_ms);  // leaving Ignite (Launch / Abort / Safe): drop the firing line
        }
    }

    /* ---- SetValvePosition (gated to Unsafe; single-board actuation OR bridge to the ECU) ------ */

    // Drive ONE valve from an operator command. Two invariants, each REFUSED-and-RECORDED (not
    // silently dropped) when violated, since the command IS addressed to us:
    //   1. State gate: per-valve actuation is permitted only in Unsafe (the one state an operator
    //      may hand-drive a valve). Outside Unsafe -> refuse + record, neither actuate nor bridge.
    //   2. Single-board: unlike SetState / SetControlFlag, a valve command is NEVER fanned out to
    //      both boards. Routed by target:
    //        FillingStation -> actuate our own Fill/Dump valve locally and Ack the GS directly.
    //        Engine         -> bridge to the ECU over CAN as a reliable command; the ECU drives
    //                          its IPA/NOS valve and Acks, and onResponse relays the Ack.
    //        Broadcast      -> refuse + record: it would drive Fill on the FCU and IPA/NOS on the
    //                          ECU at once, never wanted for a hand-driven valve.
    // The 3-byte SetValvePositionFrame rides verbatim in the payload, so the bridged CAN frame
    // carries the same bytes the GS sent (the valve byte is read as EcuValves on the ECU).
    bool handleSetValvePosition(const Command& cmd, uint32_t now_ms)
    {
        const auto* frame = reinterpret_cast<const SetValvePositionFrame*>(cmd.payload.data());

        if (logic::control::persistent_state.fill_state != State::Unsafe) {
            recordRefusedValve(*frame);   // operator per-valve actuation is Unsafe-only
            return false;
        }
        switch (static_cast<BoardId>(cmd.target)) {
            case BoardId::FillingStation:
                return actuateLocalValve(cmd, now_ms);
            case BoardId::Engine:
                sendReliable(CommandType::SetValvePosition, /*sender_state=*/0,
                             std::span<const uint8_t>(cmd.payload.data(), sizeof(SetValvePositionFrame)),
                             cmd.seq, now_ms);
                return true;
            default:
                recordRefusedValve(*frame);   // Broadcast / other: valve commands are single-board only
                return false;
        }
    }

    // Actuate the FCU's own Fill/Dump valve from a SetValvePosition frame, then Ack the GS on
    // success. An invalid action or unknown valve is refused, RECORDED, and NOT Acked.
    bool actuateLocalValve(const Command& cmd, uint32_t now_ms)
    {
        const auto* frame = reinterpret_cast<const SetValvePositionFrame*>(cmd.payload.data());
        if (!isValidAction(frame->action)) {
            recordRefusedValve(*frame);
            return false;
        }
        V* valve = (frame->valve == FcuValves::Fill) ? &fill_valve_
                 : (frame->valve == FcuValves::Dump) ? &dump_valve_
                 : nullptr;
        if (valve == nullptr) {
            recordRefusedValve(*frame);   // unknown valve id
            return false;
        }
        switch (frame->action) {
            case ValveCommand::Open:         (void)valve->open();  break;
            case ValveCommand::Close:        (void)valve->close(); break;
            case ValveCommand::SetOpenedPct: (void)valve->setOpenPercent(static_cast<float>(frame->value)); break;
        }
        ackGs(cmd.seq, now_ms);
        return true;
    }

    [[nodiscard]] static bool isValidAction(ValveCommand action)
    {
        return action == ValveCommand::Open ||
               action == ValveCommand::Close ||
               action == ValveCommand::SetOpenedPct;
    }

    // Record a refused SetValvePosition (the valve id + action + value + the state we were in) for
    // the ground station, surfaced in the ExtendedSystemState. Mirrors recordRefusedFlag.
    static void recordRefusedValve(const SetValvePositionFrame& frame)
    {
        logic::control::last_refused_valve = {
            static_cast<uint8_t>(frame.valve), static_cast<uint8_t>(frame.action), frame.value,
            logic::control::persistent_state.fill_state};
        ++logic::control::refused_valve_count;
    }

    /* ---- SetControlFlag (local apply and/or bridge to the ECU) --------------- */

    // Set a runtime control flag, routed by the command's target board:
    //   FillingStation -> apply our own flag and Ack the GS directly (no ECU hop).
    //   Engine         -> bridge the command to the ECU over CAN as a reliable command;
    //                     the ECU applies it and Acks, and onResponse relays that Ack to the GS.
    //   Broadcast      -> both: apply locally (Ack as the FCU) AND bridge (the ECU Acks too).
    // The 2-byte SetControlFlagFrame rides verbatim in the payload, so the bridged CAN frame
    // carries the same bytes the GS sent.
    bool handleSetControlFlag(const Command& cmd, uint32_t now_ms)
    {
        const auto target = static_cast<BoardId>(cmd.target);
        bool ok = true;

        if (target == BoardId::FillingStation || target == BoardId::Broadcast) {
            ok = applyControlFlagLocally(cmd, now_ms) && ok;
        }
        if (target == BoardId::Engine || target == BoardId::Broadcast) {
            sendReliable(CommandType::SetControlFlag, /*sender_state=*/0,
                         std::span<const uint8_t>(cmd.payload.data(), sizeof(SetControlFlagFrame)),
                         cmd.seq, now_ms);
        }
        return ok;
    }

    // Apply a SetControlFlag to the FCU's own control-flag state and Ack the GS directly
    // (we handled it ourselves; no ECU round-trip). Echoes the GS's seq in the Ack. A refused
    // command (unknown flag, or a state-gated flag commanded in the wrong state) is recorded in
    // last_refused_control_flag and NOT Acked.
    bool applyControlFlagLocally(const Command& cmd, uint32_t now_ms)
    {
        const auto* frame = reinterpret_cast<const SetControlFlagFrame*>(cmd.payload.data());
        const uint16_t id = frame->flag;

        if (id < CONTROL_FLAG_BOARD_OFFSET) {
            // A BASE flag (common to every board): low-byte id, applied to base_control_flags.
            const std::optional<ControlFlagBase> flag = toControlFlagBase(id);
            if (!flag) {
                recordRefusedFlag(*frame);
                return false;  // unknown base flag — do not Ack a command we did not apply
            }
            logic::control::base_control_flags.set(*flag, frame->value != 0);
        } else {
            // A PER-BOARD (FCU) flag: high-byte id, applied to fcu_control_flags.
            const std::optional<FcuControlFlag> flag =
                toFcuControlFlag(static_cast<uint16_t>(id - CONTROL_FLAG_BOARD_OFFSET));
            if (!flag) {
                recordRefusedFlag(*frame);
                return false;  // unknown per-board flag
            }
            // The solenoid valve is hazardous unless the area is clear: its flag is only honoured
            // in Unsafe. Reject (and record) a SetControlFlag(SolenoidValve) in any other state.
            if (*flag == FcuControlFlag::SolenoidValve &&
                logic::control::persistent_state.fill_state != logic::control::State::Unsafe) {
                recordRefusedFlag(*frame);
                return false;
            }
            logic::control::fcu_control_flags.set(*flag, frame->value != 0);
        }

        ackGs(cmd.seq, now_ms);
        return true;
    }

    // Record a refused SetControlFlag (the 16-bit flag id + value + the state we were in) for the
    // ground station, surfaced in the ExtendedSystemState. Mirrors last_refused_transition.
    static void recordRefusedFlag(const SetControlFlagFrame& frame)
    {
        logic::control::last_refused_control_flag = {
            frame.flag, frame.value, logic::control::persistent_state.fill_state};
        ++logic::control::refused_control_flag_count;
    }

    /* ---- Synchronise --------------------------------------------------------- */

    bool handleSynchronise(const Command& cmd, uint32_t now_ms)
    {
        (void)cmd.timestamp_ms;
        // TODO: set the clock offset from the network time. Inert for now, but Ack receipt so the
        // GS sees the command landed (the FCU handles Synchronise itself; it is not bridged).
        ackGs(cmd.seq, now_ms);
        return true;
    }

    /* One outstanding reliable command awaiting its response. A single slot — at most one
       reliable command in flight at a time, which is all Ping needs. The FCU is a bridge:
       every reliable command here is a GS command being RELAYED to the ECU (the seq is the
       GS's, propagated), never one the FCU originates. */
    struct Pending {
        bool                   active       = false;
        uint8_t                payload_id   = 0;   // the CommandType awaiting a response
        uint8_t                sender_state = 0;
        uint8_t                seq          = 0;   // the GS's seq (4-bit on CAN), echoed in the response
        uint32_t               sent_ms      = 0;
        uint8_t                retries      = 0;
        std::array<uint8_t, 8> payload{};          // bytes to resend (commands carry <= 8)
        uint8_t                payload_len  = 0;
    };

    V&       fill_valve_;   // injected Fill / Dump valves, actuated by SetValvePosition
    V&       dump_valve_;
    Comm&    comm_;         // injected communication layer; forwards to ECU / GS
    EM&      ematch_;       // injected e-match: energised in Ignite, polled each tick for the cont LED
    SOL&     solenoid_;     // injected solenoid valve: open only while its flag is set AND in Unsafe
    HTR&     heater_;       // injected heater: driven on/off straight from its flag, any state
    Pending  pending_{};       // the in-flight relayed GS->ECU command (Ping for now)
};

} // namespace logic::fcu
