/* ------------------------------------------------------------------------- *
 * Platform watchdog DIL: the STM32H7 IWDG behind the logic-layer watchdog feed
 * seam (logic::control::watchdog::kick).
 *
 * MX_IWDG_Init() (CubeMX) creates AND starts the watchdog; init() binds the handle so
 * kick() can refresh it. The logic layer decides WHEN to kick: on every serviced Ping
 * in any state, and every tick while in Safe. Outside Safe, no Ping within the IWDG
 * timeout (~30 s) therefore resets the board.
 * ------------------------------------------------------------------------- */

#include "system/watchdog/watchdog.hpp"
#include "control/watchdog.hpp"   // the logic seam we define here

namespace {

/* The board's single IWDG, bound by init(). */
IWDG_HandleTypeDef* g_hiwdg = nullptr;

} // namespace

namespace platform::system::watchdog {

void init(IWDG_HandleTypeDef* hiwdg)
{
    g_hiwdg = hiwdg;
}

} // namespace platform::system::watchdog

// ---- The logic-layer watchdog feed seam, backed by the hardware IWDG --------
void logic::control::watchdog::kick()
{
    // Before init() binds the handle there is nothing to refresh: the IWDG, once
    // started by MX_IWDG_Init, simply runs until it is bound and first kicked.
    if (g_hiwdg != nullptr) {
        (void)HAL_IWDG_Refresh(g_hiwdg);
    }
}
