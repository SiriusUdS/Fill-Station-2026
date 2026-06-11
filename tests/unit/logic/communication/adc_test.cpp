/* ------------------------------------------------------------------------- *
 * Unit test for the class-based ADC seam (logic::communication::Adc /
 * StreamingAdc).
 *
 * Demonstrates the refinement: generic logic templated on the BASE Adc works
 * against both a polled double and a streaming one (StreamingAdc subsumes Adc),
 * while logic that needs the high-rate PUSH is templated on StreamingAdc and can
 * only bind to a device that offers a callback. No HAL, no separate link.
 * ------------------------------------------------------------------------- */

#include "communication/interfaces/adc.hpp"
#include "support/fake_adc.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "gtest/gtest.h"

using logic::communication::Adc;
using logic::communication::StreamingAdc;

namespace {

/* Base-contract consumer: pull the latest first-channel reading from ANY ADC. */
template <Adc A>
std::optional<int32_t> firstChannel(A& adc)
{
    if (auto s = adc.samples()) return (*s)[0];
    return std::nullopt;
}

TEST(AdcSeam, BaseConsumerPullsFromAPolledAdc)
{
    FakeAdc adc;
    const std::array<int32_t, 4> conv{11, 22, 33, 44};
    adc.push(conv);

    EXPECT_EQ(firstChannel(adc), 11);
    EXPECT_EQ(firstChannel(adc), std::nullopt);  // consumed; nothing new
}

TEST(AdcSeam, BaseConsumerAlsoWorksOnAStreamingAdc)
{
    // StreamingAdc subsumes Adc: the same generic code binds to it unchanged.
    FakeStreamingAdc adc;
    const std::array<int32_t, 8> conv{7, 0, 0, 0, 0, 0, 0, 0};
    adc.feed(conv);

    EXPECT_EQ(firstChannel(adc), 7);
}

/* Streaming-only consumer: register a per-sample sink. Constrained on
   StreamingAdc, so it will not compile against a pull-only device. */
std::vector<int32_t> g_sink;  // the test's stand-in for the telemetry pipeline
void sampleSink(std::span<const int32_t> channels) { g_sink.push_back(channels[0]); }

template <StreamingAdc A>
void attachSink(A& adc) { adc.set_sample_callback(&sampleSink); }

TEST(AdcSeam, StreamingConsumerReceivesEveryPushedSample)
{
    g_sink.clear();
    FakeStreamingAdc adc;
    attachSink(adc);

    adc.feed(std::array<int32_t, 8>{1, 0, 0, 0, 0, 0, 0, 0});
    adc.feed(std::array<int32_t, 8>{2, 0, 0, 0, 0, 0, 0, 0});
    adc.feed(std::array<int32_t, 8>{3, 0, 0, 0, 0, 0, 0, 0});

    EXPECT_EQ(g_sink, (std::vector<int32_t>{1, 2, 3}));
}

/* Compile-time guarantees that back the runtime behaviour above. */
static_assert(Adc<FakeAdc>);
static_assert(!StreamingAdc<FakeAdc>);          // a polled ADC is not a streaming one
static_assert(StreamingAdc<FakeStreamingAdc>);  // and the continuous one is

} // namespace
