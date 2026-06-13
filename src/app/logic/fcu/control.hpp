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
#include "communication/protocol/command/set_control_flag.hpp"    // ControlFlag, SetControlFlagFrame, toControlFlag
#include "communication/protocol/framing/payload_type.hpp"     // PayloadType
#include "communication/protocol/response/response_type.hpp"   // ResponseType (Pong, Ack)
#include "system/valves/fcu.hpp"                               // FcuValves
#include "system/board_id.hpp"
#include "system/state.hpp"                                    // logic::control::State
#include "control/persistent_state.hpp"                        // logic::control::persistent_state
#include "control/control_flags.hpp"                           // logic::control::control_flags

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

/* Abort if the ground-station link goes quiet for this long while armed. */
inline constexpr uint32_t RX_WATCHDOG_MS = 500;

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
 */
template <logic::actuation::Valve V, typename Comm>
class Control {
public:
    /** @brief Construct over the local valves + the communication layer. */
    Control(V& fill_valve, V& dump_valve, Comm& comm)
        : fill_valve_(fill_valve), dump_valve_(dump_valve), comm_(comm) {}

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
        last_rx_ms_ = 0;
        (void)fill_valve_.close();
        (void)dump_valve_.close();
    }

    /**
     * @brief Handle one inbound ground-station datagram: parse it into a Command,
     *        check it is addressed to us, then gate + dispatch. Any received datagram
     *        feeds the GS-link watchdog (the GS is talking to us).
     */
    void onDatagram(std::span<const uint8_t> payload, uint32_t now_ms)
    {
        last_rx_ms_ = now_ms;

        const auto cmd = logic::communication::command::fromEthernet(payload);
        if (!cmd) {
            return;  // not a command / malformed / unknown id
        }
        if (!addressedToUs(cmd->type, static_cast<BoardId>(cmd->target))) {
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

    /** @brief Abort if the ground-station link has gone quiet while armed. */
    void watchdog(uint32_t now_ms)
    {
        const auto state = logic::control::persistent_state.fill_state;
        if (state == logic::control::State::Unsafe || state == logic::control::State::Ignite) {
            if ((now_ms - last_rx_ms_) >= detail::RX_WATCHDOG_MS) {
                logic::control::persistent_state.saveState(logic::control::State::Abort);
            }
        }
    }

private:
    using Command     = logic::communication::command::Command;
    using CommandType = logic::communication::command::CommandType;
    using State       = logic::control::State;

    /* ---- Addressing + gate + dispatch ---------------------------------------- */

    // Is a command bridgeable to the ECU? The FCU is a bridge for these: the GS may target
    // them at the Engine and the FCU relays them over CAN (Ping tests the link; SetControlFlag
    // sets an ECU-side flag). Every other command is FCU-local only.
    [[nodiscard]] static bool isBridgeable(CommandType type)
    {
        return type == CommandType::Ping || type == CommandType::SetControlFlag;
    }

    // Should the FCU act on a command with this target? Always for FillingStation / Broadcast
    // (we are, or are among, the addressees). For an Engine target only if the command is
    // bridgeable — then we accept it solely to relay it on; otherwise it is for another board.
    [[nodiscard]] static bool addressedToUs(CommandType type, BoardId target)
    {
        if (target == BoardId::FillingStation || target == BoardId::Broadcast) {
            return true;
        }
        if (target == BoardId::Engine) {
            return isBridgeable(type);
        }
        return false;
    }

    // May a command of `type` run in `current`? Skeleton: permissive for every known
    // command in every state for now — the single place to add per-command, per-state
    // gating (e.g. SetValvePosition only in TEST/UNSAFE) as the policy firms up.
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
            case CommandType::SetState:         return handleSetState(cmd);
            case CommandType::SetValvePosition: return handleSetValvePosition(cmd);
            case CommandType::SetControlFlag:   return handleSetControlFlag(cmd, now_ms);
            case CommandType::Synchronise:      return handleSynchronise(cmd);
        }
        return false;  // unknown command type
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

    bool handleSetState(const Command& cmd)
    {
        const auto* frame = reinterpret_cast<const SetStateFrame*>(cmd.payload.data());
        const std::optional<State> requested = toState(frame->requestedID);
        if (!requested) {
            return false;  // unknown requested state id
        }
        const State current = logic::control::persistent_state.fill_state;
        if (!isAllowed(current, *requested)) {
            return false;  // transition not permitted from `current`
        }
        // Run the per-transition action first, then commit so the change lands
        // atomically from the caller's view.
        runTransitionAction(current, *requested);
        logic::control::persistent_state.saveState(*requested);
        return true;
    }

    // Map a raw on-wire state id to the typed State, or nullopt if unknown. State's
    // underlying values ARE the wire encoding, so a command can only name a defined state.
    [[nodiscard]] static std::optional<State> toState(uint8_t id)
    {
        switch (static_cast<State>(id)) {
            case State::Init:
            case State::Safe:
            case State::Unsafe:
            case State::Abort:
            case State::Error:
            case State::Ignite:
            case State::Launch:
            case State::Test:
                return static_cast<State>(id);
        }
        return std::nullopt;
    }

    // The filling-station transition table: is `requested` a legal operator-commanded
    // transition from `current`? Self-transitions and any pair not listed are rejected.
    [[nodiscard]] static bool isAllowed(State current, State requested)
    {
        switch (current) {
            case State::Safe:
                return requested == State::Test || requested == State::Unsafe;
            case State::Test:
                return requested == State::Safe;
            case State::Unsafe:
                return requested == State::Safe || requested == State::Ignite ||
                       requested == State::Abort;
            case State::Ignite:
                return requested == State::Safe || requested == State::Abort;
            case State::Abort:
                return requested == State::Safe;
            case State::Init:
            case State::Error:
            case State::Launch:
                // No operator-commanded transitions out of these. (Launch's own
                // transition policy is not wired yet — and nothing transitions INTO
                // Launch above, so it stays unreachable by command for now.)
                return false;
        }
        return false;
    }

    // Run the action bound to this exact from -> to transition, if any. Most
    // transitions have none.
    void runTransitionAction(State from, State to)
    {
        if (from == State::Unsafe && to == State::Ignite) {
            // TODO: energise the igniter (HAL-free — e.g. a CAN command to the ECU
            // through comm_). Inert stub for now, so the transition is wired but has
            // no physical effect yet.
        }
    }

    /* ---- SetValvePosition (actuates a local valve) --------------------------- */

    bool handleSetValvePosition(const Command& cmd)
    {
        const auto* frame = reinterpret_cast<const SetValvePositionFrame*>(cmd.payload.data());
        if (!isValidAction(frame->action)) {
            return false;
        }
        V* valve = (frame->valve == FcuValves::Fill) ? &fill_valve_
                 : (frame->valve == FcuValves::Dump) ? &dump_valve_
                 : nullptr;
        if (valve == nullptr) {
            return false;  // unknown valve id
        }
        switch (frame->action) {
            case ValveCommand::Open:         (void)valve->open();  break;
            case ValveCommand::Close:        (void)valve->close(); break;
            case ValveCommand::SetOpenedPct: (void)valve->setOpenPercent(static_cast<float>(frame->value)); break;
        }
        return true;
    }

    [[nodiscard]] static bool isValidAction(ValveCommand action)
    {
        return action == ValveCommand::Open ||
               action == ValveCommand::Close ||
               action == ValveCommand::SetOpenedPct;
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
    // (we handled it ourselves; no ECU round-trip). Echoes the GS's seq in the Ack.
    bool applyControlFlagLocally(const Command& cmd, uint32_t now_ms)
    {
        const auto* frame = reinterpret_cast<const SetControlFlagFrame*>(cmd.payload.data());
        const std::optional<ControlFlag> flag = toControlFlag(static_cast<uint8_t>(frame->flag));
        if (!flag) {
            return false;  // unknown flag id — do not Ack a command we did not apply
        }
        logic::control::control_flags.set(*flag, frame->value != 0);
        comm_.sendToGs(BoardId::FillingStation, PayloadType::Response,
                       static_cast<uint8_t>(ResponseType::Ack), /*sourceState=*/0,
                       /*seq=*/cmd.seq, std::span<const uint8_t>{}, now_ms);
        return true;
    }

    /* ---- Synchronise --------------------------------------------------------- */

    bool handleSynchronise(const Command& cmd)
    {
        (void)cmd.timestamp_ms;
        // TODO: set the clock offset from the network time. Inert for now.
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
    uint32_t last_rx_ms_ = 0;  // last tick a datagram arrived; feeds the GS-link watchdog
    Pending  pending_{};       // the in-flight relayed GS->ECU command (Ping for now)
};

} // namespace logic::fcu
