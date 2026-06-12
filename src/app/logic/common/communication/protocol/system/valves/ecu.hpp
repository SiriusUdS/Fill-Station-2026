#pragma once

#include <cstdint>

/* The ECU's two propellant ball valves, as an array-index SSOT (mirrors FcuValves).
   These index into EcuSystemState::base.valve_info[2] and map the CAN valve indices
   (EcuValves::IPA / EcuValves::NOS). */
enum class EcuValves : uint8_t {
    IPA = 0x00,   // isopropyl-alcohol valve (PE5 / TIM15_CH1)
    NOS = 0x01,   // nitrous-oxide valve   (PE6 / TIM15_CH2)
};
