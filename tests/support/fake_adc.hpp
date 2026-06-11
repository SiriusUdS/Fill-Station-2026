#pragma once

/* ------------------------------------------------------------------------- *
 * Host test doubles for the logic ADC contracts.
 *
 * FakeAdc        models the base Adc (pull only)        — stands in for a polled
 *                device (e.g. the SPI6 ADCs).
 * FakeStreamingAdc models the StreamingAdc refinement   — stands in for the
 *                continuous ADS131M08: feed() simulates one hardware conversion,
 *                updating the latest sample AND firing the registered callback.
 *
 * Because the contracts are structural, neither needs inheritance and tests
 * instantiate logic templates on these directly — no separate link.
 * ------------------------------------------------------------------------- */

#include "communication/interfaces/adc.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

/** @brief Pull-only ADC double (models logic::communication::Adc). */
struct FakeAdc {
    static constexpr std::size_t channel_count = 4;

    std::array<int32_t, channel_count> latest{};
    bool has_new = false;

    [[nodiscard]] std::optional<std::span<const int32_t>> samples()
    {
        if (!has_new) return std::nullopt;
        has_new = false;
        return std::span<const int32_t>(latest.data(), latest.size());
    }

    /** @brief Test helper: stage a conversion the next samples() call returns. */
    void push(std::span<const int32_t> values)
    {
        const std::size_t n = values.size() < channel_count ? values.size() : channel_count;
        for (std::size_t i = 0; i < n; ++i) latest[i] = values[i];
        has_new = true;
    }
};

/** @brief Continuous/streaming ADC double (models logic::communication::StreamingAdc). */
struct FakeStreamingAdc {
    static constexpr std::size_t channel_count = 8;

    std::array<int32_t, channel_count> latest{};
    bool has_new = false;
    logic::communication::SampleCallback cb = nullptr;

    [[nodiscard]] std::optional<std::span<const int32_t>> samples()
    {
        if (!has_new) return std::nullopt;
        has_new = false;
        return std::span<const int32_t>(latest.data(), latest.size());
    }

    void set_sample_callback(logic::communication::SampleCallback c) { cb = c; }

    /** @brief Test helper: simulate one hardware conversion — update the latest
     *         sample and fire the per-sample callback, exactly as the ISR would. */
    void feed(std::span<const int32_t> values)
    {
        const std::size_t n = values.size() < channel_count ? values.size() : channel_count;
        for (std::size_t i = 0; i < n; ++i) latest[i] = values[i];
        has_new = true;
        if (cb) cb(std::span<const int32_t>(latest.data(), latest.size()));
    }
};

// The doubles really model the contracts the logic is written against — and the
// pull-only one is NOT a StreamingAdc, so logic that needs the stream can't be
// instantiated on it by mistake.
static_assert(logic::communication::Adc<FakeAdc>);
static_assert(!logic::communication::StreamingAdc<FakeAdc>);
static_assert(logic::communication::StreamingAdc<FakeStreamingAdc>);
