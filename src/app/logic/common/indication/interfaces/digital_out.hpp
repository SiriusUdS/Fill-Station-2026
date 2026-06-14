#pragma once

#include <concepts>

/* ------------------------------------------------------------------------- *
 * Class-based digital-output contract for the logic layer (C++23 concept).
 *
 * The minimal seam for a single on/off line the logic can drive (a status LED
 * today; any GPIO output tomorrow). The logic depends ONLY on set(bool); no HAL
 * type, port or pin leaks in. A platform GPIO driver models it for firmware and a
 * fake models it for host tests — same shape as the Valve / Adc / Storage seams.
 * ------------------------------------------------------------------------- */

namespace logic::indication {

/**
 * @brief The contract a drivable on/off output must satisfy.
 *
 * set(on) — drive the line on (true) or off (false). Mapping on/off to a physical
 *           level (active-high vs active-low) is the driver's concern, not the
 *           logic's.
 */
template <typename T>
concept DigitalOut = requires(T out, bool on) {
    { out.set(on) } -> std::same_as<void>;
};

} // namespace logic::indication
