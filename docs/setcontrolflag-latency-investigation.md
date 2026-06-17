# SetControlFlag latency investigation

## Symptom

From the ground station, a `SetState` command takes effect near-instantaneously, while a
`SetControlFlag` ("set flag") command can take **multiple seconds** to physically take effect.
The lag was confirmed on an **oscilloscope** — the actuation itself is late, so this is not a
telemetry-display or Ack-confirmation artifact: the command genuinely lands at the FCU late.

Observed specifics:
- `SetState` is fast; `SetControlFlag` is slow.
- `SetControlFlag` is slow whether targeted at `Broadcast` **or** the local `FillingStation`.
- The ground station sends each command **10 times without waiting for an Ack**.

## Topology

- **`SetState`** originates **directly** on the Raspberry Pi ground station
  ([SiriusUdS/rpi-gs](https://github.com/SiriusUdS/rpi-gs)).
- **`SetControlFlag`** originates on the **operator gs** (a separate program) and **transits
  through** the rpi-gs relay on its way to the FCU.

So the flag path has an extra hop (operator gs → rpi-gs → FCU) that the state path does not.

## What the FCU does (this repo)

The two command handlers are **structurally symmetric** and both Ack in the same tick through the
same path, so the FCU does **not** make one command slower than the other by type:

- `handleSetState` / `applyStateLocally` — `src/app/logic/fcu/control.hpp:344,362`
- `handleSetControlFlag` / `applyControlFlagLocally` — `src/app/logic/fcu/control.hpp:501,521`
- Both Ack via `ackGs()` → `sendToGs(..., PayloadType::Response, ...)` — `control.hpp:295`

A local `SetControlFlag` is in fact *less* work than a `SetState` (a single bit flip in
`ControlFlags::set`, `control_flags.hpp:26`, vs. `onTransition` actuation + a software CRC-32 +
a Backup-SRAM write in `saveState`, `persistent_state.cpp:81`). So the FCU cannot be the source
of a *type-dependent* slowdown on the local Ack path.

### Two FCU facts that shape what the operator perceives

1. **State is echoed in every high-rate telemetry packet; flags are not.** `fill_state` rides the
   header (`sourceState`) of every ~2 kHz `FcuSystemState` packet (`telemetry.hpp:184`), so a
   state change is confirmable almost immediately. The control-flags byte appears **only** in the
   ~10 Hz `ExtendedSystemState` (`telemetry.hpp:207`, `EXTENDED_INTERVAL_MS = 100`,
   `telemetry.hpp:72`). This makes a flag change *look* laggy even when it isn't — but it does not
   explain a scope-confirmed actuation delay.

2. **Responses go to a separate, hardcoded `command_endpoint_` than telemetry**, and the commander
   address is flagged as a placeholder/TODO (`communication.hpp:64-74,139`). Telemetry → sink
   `192.168.0.111`; responses → commander `192.168.0.101`. Fire-and-forget UDP, no retransmit on
   the FCU→GS response path.

### FCU per-flag actuation gating (matters depending on which flag is scoped)

- **Heater** — ungated; follows its flag in any state (`serviceHeater`, `control.hpp:142`).
- **SolenoidValve** — the FCU **refuses** `SetControlFlag(SolenoidValve)` unless in `Unsafe`
  (`applyControlFlagLocally`, `control.hpp:544`), and `serviceSolenoid` only actuates when the flag
  is set **and** the state is `Unsafe` (`control.hpp:128`). Setting the solenoid flag around a state
  change can therefore actuate seconds late: early (fire-once) flag packets are refused, and it only
  takes effect once both conditions line up.

## What rpi-gs actually does (read from the local checkout)

Pure **C++** relay; single worker thread looping every 5 ms:
`processReceiving(); processStateMachine(); processSending(); sleep(5ms)` (`GroundStation.cpp:646`).
Two UDP sockets: `server_` faces the operator gs, `client_` faces the FCU.

**Theories falsified by the code:**

- **No internal queue backlog.** Every stage drains its socket/queue in a `while`-loop, not one
  item per tick (`GroundStation.cpp:461,473,516,569,617,630`). Internal queues empty every tick, so
  10×-duplicated commands + high-rate telemetry cannot pile into seconds; worst internal latency is
  ~one tick (≤10 ms). A true backlog could only form in the kernel UDP socket buffer under sustained
  CPU saturation, and UDP's behaviour there is to **drop**, not to accumulate seconds of latency.
- **No long sleep / timeout / batching / rate-limit on the relay path.**

**Findings that matter:**

1. **The relay is fire-once; only local `SetState` is reliable.** A relayed `SetControlFlag` is
   forwarded a single time (`enqueueClientSend(server_data)`, `GroundStation.cpp:607-610`) with **no
   ack-tracking and no retransmit**. Locally-originated `SetState` is authored fresh on the Pi *and*
   resent every 100 ms until the FCU acks (`pending_cmd_`, `GroundStation.cpp:494-501`). So SetState
   punches through packet loss; a relayed flag does not. The gs blasting 10× is the **only**
   reliability the flag path has, and nothing resends after that burst.

2. **The relay re-originates the packet** — the FCU sees the **Pi's** source IP, not the operator
   gs's (`UdpClient.cpp:52,109-117`). It also **re-stamps the CRC** over the bytes it holds before
   sending to the FCU (`GroundStation.cpp:631-634`).

## Most likely root cause

A **best-effort, no-retry relay losing the command burst under load, with nothing to resend it.**
`SetState` survives the same conditions because it is retransmitted until acked; the relayed flag
does not, so it only "takes" when a later packet happens to get through — seconds later. This fits
the scope-confirmed late actuation and the eventual (jittery) success.

Caveat: best-effort loss explains *eventual, jittery* delivery but does not by itself pin the exact
multi-second magnitude. If the scoped flag is **SolenoidValve**, the FCU state-gate above can fully
account for a clean multi-second lag; if it is **Heater**, the cause is purely the relay reliability.
**Open question: which flag was scoped?**

## Recommended fixes

1. **Pi-side (the real latency fix):** give relayed commands — especially `SetControlFlag` — the
   same `pending_cmd_`-style retry-until-acked treatment that local `SetState` already has
   (`GroundStation.cpp:494-501`). Best-effort relay of safety-relevant flags is the root problem.
2. **FCU-side (good hygiene, does NOT fix this latency):** reply to the inbound command's source
   (`Datagram::source`, `ethernet.hpp:53-56`) instead of the hardcoded `command_endpoint_` — thread
   it through `onDatagram → ackGs → sendToGs`. This retires the placeholder-commander TODO, but
   because the relay **re-originates**, `datagram.source` is the Pi, so it would not bypass the
   transit hop.
3. **Reconcile the addressing triangle:** FCU acks → commander `192.168.0.101`, FCU telemetry → sink
   `192.168.0.111` (`communication.hpp:64-74`); Pi `client_` → `192.168.0.111`, Pi `server_` →
   `192.168.0.100`. These three configs do not form a consistent triangle — confirm the FCU's Ack is
   actually reaching the Pi.

## Status

- [ ] Confirm which flag was scoped (Heater vs SolenoidValve vs base flag).
- [ ] Pi-side: add retry/ack tracking to relayed commands.
- [ ] FCU-side: reply-to-emitter cleanup + retire `command_endpoint_` placeholder.
- [ ] Verify the FCU↔Pi command/ack/telemetry address triangle.
