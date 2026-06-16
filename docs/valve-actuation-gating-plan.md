# Plan: gate per-valve actuation commands to the UNSAFE state

## Goal

Make the **operator valve-actuation command** (`CommandType::SetValvePosition` — open / close /
set-open-% of an individual valve) succeed **only while the board is in the `Unsafe` state**, on
both the FCU and the ECU.

The gate must live in the **command-handling layer**, NOT in the valve drivers, because several
state transitions must still actuate valves while in states other than `Unsafe`:

- **→ Abort**: FCU sets Fill to 0 % (closed) and Dump to 100 % (open); ECU closes both propellant
  valves.
- **Ignite → Launch**: ECU opens both propellant valves fully; FCU does nothing.
- **→ Safe** (from any state that can reach Safe): close **all** valves on **both** boards.

Those are per-transition side effects (`onTransition`) that call the valve methods directly, so they
are unaffected by the command gate.

## Principles

1. **Gate the command, not the valve.** `BallValve` / the `Valve` concept stay unconditionally
   actuatable. Only `SetValvePosition` command dispatch is state-gated.
2. **Transitions actuate valves directly** via `valve.open()/close()/setOpenPercent()` inside each
   board's `onTransition`, bypassing the command path entirely.
3. **Each board owns its own valves.** "Close all valves on both boards" is achieved by each board
   closing its own valves in `onTransition`, given the two state machines are kept in sync (see
   [Cross-board synchronization](#cross-board-state-synchronization)).

---

## Current state (what exists today)

| Concern | FCU (`logic/fcu/control.hpp`) | ECU (`logic/ecu/control.hpp`) |
|---|---|---|
| Valve command | `handleSetValvePosition` → `actuateLocalValve` (local) + bridge to ECU | `handleValveCmd` (drives IPA/NOS) |
| Command gate | `canExecute(type, state)` returns `true` for **all** commands | none (dispatch switch has no state gate) |
| `onTransition` → Safe | closes Fill + Dump | closes IPA + NOS |
| `onTransition` → Abort | **none** | **none** |
| `onTransition` Ignite→Launch | none (e-match only) | opens IPA + NOS |
| State table (`control/state_machine.hpp`) | `Launch → {Safe, Abort}` (allows Launch→Safe) | shared table (same) |

So: the gate is missing on both boards, the Abort valve actions are missing on both boards, and the
state table currently permits `Launch → Safe` (which the requirement says must not exist).

---

## Changes

### 1. Gate `SetValvePosition` to UNSAFE — FCU

In `logic::fcu::Control::canExecute(CommandType, State)` (the designed per-command/per-state gate),
return `true` for every command **except** `SetValvePosition`, which returns `true` only when
`current == State::Unsafe`:

```cpp
[[nodiscard]] static bool canExecute(CommandType type, State current)
{
    switch (type) {
        case CommandType::SetValvePosition:
            return current == State::Unsafe;   // per-valve actuation is operator-only, in Unsafe
        case CommandType::Ping:
        case CommandType::SetState:
        case CommandType::SetControlFlag:
        case CommandType::Synchronise:
            return true;
    }
    return false;
}
```

Effect: outside `Unsafe`, `handleCommand` returns `false` before `handleSetValvePosition`, so the FCU
neither actuates its local valve **nor bridges** the command to the ECU.

### 2. Gate `SetValvePosition` to UNSAFE — ECU (defense in depth)

The ECU is reachable only through the FCU bridge, and its state mirrors the FCU's, so the FCU gate
already blocks it in practice. Add an explicit gate in `logic::ecu::Control::handleValveCmd` (or a
shared `canExecute` on the ECU) so the ECU never actuates a bridged valve command unless its own
`persistent_state.fill_state == State::Unsafe`:

```cpp
void handleValveCmd(const CanFrame& frame, const CanHeader& header)
{
    if (logic::control::persistent_state.fill_state != logic::control::State::Unsafe) {
        return;  // per-valve actuation only in Unsafe — do not actuate or Ack
    }
    ...existing decode + actuate + Ack...
}
```

> The transition-driven valve moves (Safe/Abort/Launch) live in `onTransition`, NOT in
> `handleValveCmd`, so this gate does not affect them.

### 3. Transition valve actions — FCU `onTransition`

Add the Abort case; keep the Safe case. All direct valve calls:

```cpp
void onTransition(State from, State to, uint32_t now_ms)
{
    if (to == State::Safe) {
        (void)fill_valve_.close();
        (void)dump_valve_.close();
    }
    if (to == State::Abort) {
        (void)fill_valve_.setOpenPercent(0.0F);     // fill closed
        (void)dump_valve_.setOpenPercent(100.0F);   // dump open (vent)
    }
    // ... existing e-match / solenoid-flag logic ...
}
```

### 4. Transition valve actions — ECU `onTransition`

Add the Abort case; keep Safe + Ignite→Launch:

```cpp
void onTransition(State from, State to)
{
    if (to == State::Safe || to == State::Abort) {
        (void)ipa_valve_.close();
        (void)nos_valve_.close();
    }
    if (from == State::Ignite && to == State::Launch) {
        (void)ipa_valve_.open();
        (void)nos_valve_.open();
    }
}
```

### 5. State-machine table — remove `Launch → Safe`

The requirement states Launch cannot transition to Safe. In
`logic::control::isTransitionAllowed` (`control/state_machine.hpp`), change the Launch row to allow
**Abort only**:

```cpp
case State::Launch:
    return requested == State::Abort;   // no direct Launch -> Safe; come down via Abort
```

Consequence: the way down from Launch is `Launch → Abort → Safe`. Valves close along the way
(Launch→Abort: fill closed / dump open + ECU closed; Abort→Safe: all closed). This is a **shared**
table change affecting both boards.

---

## Cross-board state synchronization

"Close all valves on **both** boards on a transition to Safe" works **only if both boards actually
transition to Safe**. Today the FCU bridges a `SetState` to the ECU based on the command's target
(`Engine` / `Broadcast`), and the watchdog-driven `Abort` is FCU-local (not bridged). So there are
paths where one board changes state and the other does not — leaving the other board's valves open.

This must be resolved for the safety guarantee to hold. Options (pick one — **open question**):

- **A. Always bridge safety transitions.** When the FCU commits a transition to `Safe` or `Abort`
  (including the watchdog abort), it sends a reliable `SetState(<to>)` to the ECU so the ECU runs its
  own `onTransition` and closes its valves. Cleanest guarantee; keeps the gate clean (uses `SetState`,
  not the gated `SetValvePosition`). Requires care to avoid a bridge loop (don't re-bridge a state
  the FCU entered *because of* an ECU/GS message already addressed to both).
- **B. Operating convention.** Require the GS to address `Safe`/`Abort` `SetState` to `Broadcast`, so
  both boards transition. Simple, but no firmware guarantee — a `FillingStation`-only Safe leaves ECU
  valves open.

Recommendation: **A** for the safety-critical Safe/Abort edges; the existing per-target bridging
remains for normal operation.

> Note: the FCU must **not** close ECU valves via a `SetValvePosition` command — that command is now
> gated to `Unsafe`, and a Safe/Abort transition is by definition not `Unsafe`, so it would be
> rejected by the ECU gate. Propagate the **state** (`SetState`) and let the ECU's `onTransition` act.

---

## Affected files

- `src/app/logic/fcu/control.hpp` — `canExecute` gate; `onTransition` Abort case; (option A) bridge
  Safe/Abort to the ECU.
- `src/app/logic/ecu/control.hpp` — `handleValveCmd` gate; `onTransition` Abort case.
- `src/app/logic/common/control/state_machine.hpp` — remove `Launch → Safe`.
- Tests (below).

No protocol/submodule or wire-format changes — this is pure control-logic behavior.

---

## Edge cases & decisions

- **Test state:** the requirement is "ONLY in Unsafe", so `SetValvePosition` is rejected in `Test`
  too (an earlier code comment had speculated "TEST/UNSAFE" — superseded).
- **Bridged valve command in Unsafe:** operator commands an ECU valve via `SetValvePosition` targeted
  at `Engine`; FCU (Unsafe) bridges; ECU (Unsafe, synced) actuates. Consistent.
- **Refused valve commands in telemetry:** currently only refused `SetState` / `SetControlFlag` are
  surfaced (`RefusedCommandInfo`). Extending it to refused `SetValvePosition` is **out of scope**
  here — flag as a possible follow-up (open question).
- **Fill=0 / Dump=100 semantics:** modeled as `setOpenPercent(0/100)`; `close()/open()` are the same
  endpoints for a ball valve — confirm which to use.
- **Watchdog abort:** under option A this also closes ECU valves; under option B it does not.

## Testing plan

- **FCU control / controller tests:** `SetValvePosition` is rejected (no valve calls, no bridge) in
  Safe/Ignite/Launch/Abort/Error; accepted (valve driven + bridged) in Unsafe.
- **ECU controller tests:** a bridged `SetValvePosition` is ignored unless the ECU is in Unsafe.
- **Transition tests:** `→ Safe` closes Fill/Dump (FCU) and IPA/NOS (ECU); `→ Abort` sets Fill 0 /
  Dump 100 (FCU) and closes IPA/NOS (ECU); `Ignite → Launch` opens IPA/NOS (ECU), FCU valves
  untouched. Assert these fire **even though** `SetValvePosition` would be rejected in those states
  (proving the gate is command-only).
- **State table:** `Launch → Safe` is now refused; `Launch → Abort` allowed.
- (Option A) **Sync test:** an FCU transition to Safe/Abort drives the ECU to the same state and
  closes its valves.

## Open questions (need a decision before implementing)

1. **Cross-board sync:** option **A** (firmware bridges Safe/Abort to the ECU) or **B** (broadcast
   convention)?
2. **`Launch → Safe` removal** — confirm this shared state-table change is intended.
3. **Refused-`SetValvePosition` telemetry** — add to `RefusedCommandInfo`, or leave out for now?
4. **Fill/Dump abort positions** — `setOpenPercent(0/100)` vs `close()/open()`?
