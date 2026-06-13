#pragma once

#include <cstdint>

#include "peripherals/adc/adc_info.hpp"
#include "devices/valve/valve_info.hpp"
#include "peripherals/storage/storage_info.hpp"
#include "peripherals/can/can_info.hpp"

/* The fields common to every board's periodic telemetry record. Both
 * FcuSystemState and EcuSystemState embed this as their leading `base` member, so
 * the shared prefix decodes identically on the ground regardless of source board.
 *
 * Board-specific peripherals (e.g. the FCU's Ethernet link) are added ALONGSIDE
 * `base` in the per-board record — that is where the two records diverge, and the
 * divergence is expected to grow. A field that stops being common (e.g. if one
 * board's valve count changes) moves out of here into the per-board records. */

struct SystemStateBase {
    uint32_t    creation_timestamp_ms;  /**< Time this record was assembled. */
    AdcInfo     adc_info;               /**< Streaming ADC: state + status + channels. */
    ValveInfo   valve_info[2];          /**< 2 x 3 bytes (board-specific valve identities). */
    StorageInfo storage_info;           /**< SD card: state + status. */
    CanInfo     can_info;               /**< CAN bus: state + status + dropped-frame count. */
};

// Wire layout guard: the common prefix must be packed with no implicit padding so
// the ground station decodes it byte-for-byte. If a field change introduces a
// gap, this fails — add an explicit reserved field to close it.
static_assert(sizeof(SystemStateBase) == sizeof(uint32_t)        // creation_timestamp_ms
                                       + sizeof(AdcInfo)          // adc_info
                                       + 2 * sizeof(ValveInfo)    // valve_info
                                       + sizeof(StorageInfo)      // storage_info
                                       + sizeof(CanInfo),         // can_info
              "SystemStateBase has implicit padding — add explicit reserved bytes");
