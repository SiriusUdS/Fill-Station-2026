#pragma once

/* ------------------------------------------------------------------------- *
 * Host test doubles for the logic ADC contracts.
 *
 * FakeAdc          models the base Adc (info() only)      — stands in for a polled
 *                  device (e.g. the SPI6 ADCs): the latest info is all it offers.
 * FakeStreamingAdc models the StreamingAdc refinement     — stands in for the
 *                  continuous ADS131M08: push() queues one conversion in the ring,
 *                  pop() drains it (exactly as the driver's DRDY ISR / controller
 *                  do), and info() returns the latest pushed conversion.
 *
 * Because the contracts are structural, neither needs inheritance and tests
 * instantiate logic templates on these directly — no separate link.
 * ------------------------------------------------------------------------- */

#include "communication/interfaces/adc.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

/** @brief Pull-only ADC double (models logic::communication::Adc). */
struct FakeAdc {
    AdcInfo latest{};

    [[nodiscard]] AdcInfo info() const { return latest; }

    /** @brief Test helper: set the info that info() returns. */
    void set(const AdcInfo& info) { latest = info; }
};

/** @brief Continuous/streaming ADC double (models logic::communication::StreamingAdc). */
struct FakeStreamingAdc {
    AdcInfo            latest{};
    std::deque<AdcInfo> ring;

    [[nodiscard]] AdcInfo info() const { return latest; }

    [[nodiscard]] std::optional<AdcInfo> pop()
    {
        if (ring.empty()) return std::nullopt;
        const AdcInfo out = ring.front();
        ring.pop_front();
        return out;
    }

    /** @brief Test helper: simulate one hardware conversion — queue it in the ring
     *         and make it the latest, exactly as the driver's DRDY ISR would. */
    void push(const AdcInfo& info)
    {
        latest = info;
        ring.push_back(info);
    }
};

// The doubles really model the contracts the logic is written against — and the
// pull-only one is NOT a StreamingAdc, so logic that needs the stream can't be
// instantiated on it by mistake.
static_assert(logic::communication::Adc<FakeAdc>);
static_assert(!logic::communication::StreamingAdc<FakeAdc>);
static_assert(logic::communication::StreamingAdc<FakeStreamingAdc>);
