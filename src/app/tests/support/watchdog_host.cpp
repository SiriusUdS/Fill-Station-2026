/* ------------------------------------------------------------------------- *
 * Host definition of the watchdog feed seam (logic::control::watchdog::kick).
 * On firmware this refreshes the STM32 IWDG (platform DIL); in host tests there
 * is no peripheral, so feeding the watchdog is a no-op. Linked by every test
 * executable whose control layer services a Ping (Control::handlePing).
 * ------------------------------------------------------------------------- */

#include "control/watchdog.hpp"

void logic::control::watchdog::kick()
{
    // No watchdog on the host; nothing to refresh.
}
