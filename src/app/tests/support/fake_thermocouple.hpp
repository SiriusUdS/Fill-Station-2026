#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::communication::ThermocoupleBank contract.
 *
 * Stands in for the 4-channel MAX31856 bank: service() is a no-op that just
 * records it was advanced (the real bank drives interrupt-driven SPI there), and
 * info() returns the scriptable per-channel snapshot a test sets up. Because the
 * contract is structural, logic templates instantiate on this directly.
 * ------------------------------------------------------------------------- */

#include "communication/interfaces/thermocouple.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

/** @brief In-memory thermocouple bank (models logic::communication::ThermocoupleBank). */
struct FakeThermocoupleBank {
    std::array<ThermocoupleInfo, THERMOCOUPLE_COUNT> infos{};  /**< Returned by info(); script per channel. */
    uint32_t service_calls = 0;   /**< Number of service() calls. */
    uint32_t last_now_ms   = 0;   /**< now_ms of the last service() call. */

    void service(uint32_t now_ms)
    {
        ++service_calls;
        last_now_ms = now_ms;
    }

    [[nodiscard]] std::array<ThermocoupleInfo, THERMOCOUPLE_COUNT> info() const { return infos; }

    /** @brief Test helper: set one channel's info that info() returns. */
    void set(std::size_t ch, const ThermocoupleInfo& v) { infos[ch] = v; }
};

static_assert(logic::communication::ThermocoupleBank<FakeThermocoupleBank>);
