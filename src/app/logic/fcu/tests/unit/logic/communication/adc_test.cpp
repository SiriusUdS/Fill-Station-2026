/* ------------------------------------------------------------------------- *
 * Unit test for the class-based ADC seam (logic::communication::Adc /
 * StreamingAdc).
 *
 * Demonstrates the refinement: generic logic templated on the BASE Adc works
 * against both a polled double and a streaming one (StreamingAdc subsumes Adc) by
 * reading info(), while logic that needs every conversion is templated on
 * StreamingAdc and drains the ring through pop() — which the pull-only device does
 * not offer, so it will not compile against it. No HAL, no separate link.
 * ------------------------------------------------------------------------- */

#include "communication/interfaces/adc.hpp"
#include "support/fake_adc.hpp"

#include <cstdint>
#include <optional>
#include <vector>

#include "gtest/gtest.h"

using logic::communication::Adc;
using logic::communication::StreamingAdc;

namespace {

// Build an AdcInfo whose first channel carries `ch0` (the rest zero).
AdcInfo make_info(int32_t ch0)
{
    AdcInfo info{};
    info.state              = AdcState::Streaming;
    info.status.initialized = 1u;
    info.status.data_valid  = 1u;
    info.channels[0]        = ch0;
    return info;
}

/* Base-contract consumer: read the latest first-channel reading from ANY ADC. */
template <Adc A>
int32_t firstChannel(A& adc)
{
    return adc.info().channels[0];
}

TEST(AdcSeam, BaseConsumerReadsInfoFromAPolledAdc)
{
    FakeAdc adc;
    adc.set(make_info(11));

    EXPECT_EQ(firstChannel(adc), 11);
}

TEST(AdcSeam, BaseConsumerAlsoWorksOnAStreamingAdc)
{
    // StreamingAdc subsumes Adc: the same generic code binds to it unchanged.
    FakeStreamingAdc adc;
    adc.push(make_info(7));

    EXPECT_EQ(firstChannel(adc), 7);
}

/* Streaming-only consumer: drain the ring. Constrained on StreamingAdc, so it
   will not compile against a pull-only device. */
template <StreamingAdc A>
std::vector<int32_t> drain(A& adc)
{
    std::vector<int32_t> out;
    while (auto s = adc.pop()) out.push_back(s->channels[0]);
    return out;
}

TEST(AdcSeam, StreamingConsumerDrainsEveryQueuedConversion)
{
    FakeStreamingAdc adc;
    adc.push(make_info(1));
    adc.push(make_info(2));
    adc.push(make_info(3));

    EXPECT_EQ(drain(adc), (std::vector<int32_t>{1, 2, 3}));
    EXPECT_EQ(drain(adc), std::vector<int32_t>{});  // ring emptied; nothing left
}

/* Compile-time guarantees that back the runtime behaviour above. */
static_assert(Adc<FakeAdc>);
static_assert(!StreamingAdc<FakeAdc>);          // a polled ADC is not a streaming one
static_assert(StreamingAdc<FakeStreamingAdc>);  // and the continuous one is

} // namespace
