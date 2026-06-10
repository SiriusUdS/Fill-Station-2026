#pragma once

#include <cstdint>
#include <span>
#include <type_traits>

#include "sirius-headers-common/Storage/StorageStatus.h"   // StorageStatus
#include "sirius-headers-common/Storage/StorageState.h"    // STORAGE_STATE_* (to read status())

/* ------------------------------------------------------------------------- *
 * Statically-linked storage interface for the logic layer.
 *
 * Storage is a concrete, non-polymorphic class: its methods are DECLARED here
 * and DEFINED by the platform (platform/storage/storage.cpp, backed by the SD
 * card driver on SDMMC2), resolved at link time. There are no virtual
 * functions, so no vtable and no indirect dispatch — calls bind directly to the
 * linked implementation. The class is stateless: instances are interchangeable
 * handles to the one backing store, so the controller holds a Storage by value.
 *
 * This mirrors the free-function seams (spi/adc/udp) in class form, to evaluate
 * the class shape for the other peripheral interfaces.
 * ------------------------------------------------------------------------- */

namespace logic::storage {

class Storage {
public:
    /** @brief Bring the backing store online (mount). Sets status ACTIVE/ERROR. */
    void init();

    /** @brief Persist @p data. No-op unless the store is ready. */
    void write(std::span<const uint8_t> data);

    /** @brief Coarse status in the shared storage protocol. */
    StorageStatus status() const;
};

// Hard requirement: no vtable. A virtual member would make Storage polymorphic
// and add a vtable pointer — keep it a plain, statically-dispatched class.
static_assert(!std::is_polymorphic_v<Storage>, "Storage must have no vtable");

} // namespace logic::storage
