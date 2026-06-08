#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * FCU state-machine logic (HAL-free).
 *
 * Drives the filling-station state machine over the logic interfaces only
 * (communication/interfaces/ethernet.hpp + can.hpp): it never touches the HAL.
 * The platform brings the peripherals up (eth::init, can::init) and pumps RX
 * (eth::process); this layer polls udp::receive() / can::receive(), runs the
 * state machine, emits the heartbeat and commands valves over CAN.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

/**
 * @brief  Initialise the FCU logic: starting state and ground-station endpoint.
 *         Call once after the platform peripherals (Ethernet, CAN) are up.
 */
void init();

/**
 * @brief  Advance the FCU one step: drain CAN and UDP, run the state machine,
 *         emit the heartbeat and service the receive watchdog.
 * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
 */
void tick(uint32_t now_ms);

} // namespace logic::fcu
