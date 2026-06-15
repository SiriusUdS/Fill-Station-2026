#pragma once

/* ------------------------------------------------------------------------- *
 * The standard set of host test doubles — one include for all of them.
 *
 * Pull this into a test instead of listing each support/fake_*.hpp by hand: it
 * bundles the FakeBus (CAN + UDP transports) and every peripheral double the logic
 * templates are instantiated on. Adding a new fake means updating THIS header, not
 * every test's include list. The individual fake_*.hpp headers remain usable
 * directly for a focused single-double test.
 * ------------------------------------------------------------------------- */

#include "support/fake_bus.hpp"          // FakeBus + FakeEthernet + FakeCan + bus()
#include "support/fake_storage.hpp"      // FakeStorage
#include "support/fake_valve.hpp"        // FakeValve
#include "support/fake_adc.hpp"          // FakeAdc + FakeStreamingAdc
#include "support/fake_thermocouple.hpp" // FakeThermocoupleBank
#include "support/fake_power_monitor.hpp" // FakePowerMonitor
#include "support/fake_digital_out.hpp"  // FakeDigitalOut
