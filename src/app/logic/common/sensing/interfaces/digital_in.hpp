#pragma once

#include <concepts>

/* ------------------------------------------------------------------------- *
 * Class-based digital-input contract for the logic layer (C++23 concept).
 *
 * The input counterpart to logic::indication::DigitalOut: the minimal seam for a
 * single on/off line the logic can read (a presence/continuity sense today; any
 * GPIO input tomorrow). The logic depends ONLY on read(); no HAL type, port or pin
 * leaks in. A platform GPIO driver models it for firmware and a fake models it for
 * host tests — same shape as the DigitalOut / Valve / Adc seams.
 * ------------------------------------------------------------------------- */

namespace logic::sensing {

/**
 * @brief The contract a readable on/off input must satisfy.
 *
 * read() — true when the line is ACTIVE, false otherwise. Mapping the active level
 *          to a physical pin level (active-high vs active-low) is the driver's
 *          concern, not the logic's.
 */
template <typename T>
concept DigitalIn = requires(T in) {
    { in.read() } -> std::same_as<bool>;
};

} // namespace logic::sensing
