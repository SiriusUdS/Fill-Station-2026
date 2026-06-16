# FCU-logic test site (logic::fcu): controller, command handlers, ADC seam, plus
# integration tests that assemble the real units. Shared infra (GoogleTest,
# support/, LOGIC_* vars, FAKES_SRC) comes from src/app/tests.

set(FCU_INCLUDE_DIRS ${TEST_SUPPORT} ${LOGIC_FCU} ${LOGIC_COMMON_INCLUDE_DIRS})

# The Ethernet -> Command parser TU that the control layer's onDatagram() uses.
# Header-only otherwise, so every controller/control target links just this.
set(COMMAND_PARSER_SRC ${LOGIC_COMMON}/communication/command/parser/command_ethernet_parser.cpp)

# --- FCU controller logic (unit) ------------------------------------------
add_executable(fcu_controller_test
    unit/logic/fcu_controller_test.cpp
    ${FAKES_SRC}
    ${CRC_HOST_SRC}
    ${COMMAND_PARSER_SRC}
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(fcu_controller_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(fcu_controller_test PRIVATE GTest::gtest_main)
gtest_discover_tests(fcu_controller_test PROPERTIES LABELS "unit")

# --- ADC seam (unit) ------------------------------------------------------
add_executable(adc_test
    unit/logic/communication/adc_test.cpp
)
target_include_directories(adc_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(adc_test PRIVATE GTest::gtest_main)
gtest_discover_tests(adc_test PROPERTIES LABELS "unit")

# --- E-match (GpioEmatch over the digital seams) (unit) -------------------
add_executable(gpio_ematch_test
    unit/logic/gpio_ematch_test.cpp
)
target_include_directories(gpio_ematch_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(gpio_ematch_test PRIVATE GTest::gtest_main)
gtest_discover_tests(gpio_ematch_test PROPERTIES LABELS "unit")

# --- Solenoid valve (GpioSolenoid over the digital seams) (unit) ----------
add_executable(gpio_solenoid_test
    unit/logic/gpio_solenoid_test.cpp
)
target_include_directories(gpio_solenoid_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(gpio_solenoid_test PRIVATE GTest::gtest_main)
gtest_discover_tests(gpio_solenoid_test PROPERTIES LABELS "unit")

# --- Heater (GpioHeater over the digital seam) (unit) ---------------------
add_executable(gpio_heater_test
    unit/logic/gpio_heater_test.cpp
)
target_include_directories(gpio_heater_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(gpio_heater_test PRIVATE GTest::gtest_main)
gtest_discover_tests(gpio_heater_test PROPERTIES LABELS "unit")

# --- Control layer (unit) -------------------------------------------------
add_executable(control_test
    unit/logic/control/control_test.cpp
    ${FAKES_SRC}
    ${CRC_HOST_SRC}
    ${COMMAND_PARSER_SRC}
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(control_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(control_test PRIVATE GTest::gtest_main)
gtest_discover_tests(control_test PROPERTIES LABELS "unit")

# --- UDP SetValvePosition -> valve actuation (integration) ----------------
add_executable(valve_actuation_test
    integration/valve_actuation_test.cpp
    ${FAKES_SRC}
    ${CRC_HOST_SRC}
    ${COMMAND_PARSER_SRC}
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(valve_actuation_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(valve_actuation_test PRIVATE GTest::gtest_main)
gtest_discover_tests(valve_actuation_test PROPERTIES LABELS "integration")

# --- Ping routing (Gs<->Fcu<->Ecu) (integration) --------------------------
add_executable(ping_routing_test
    integration/ping_routing_test.cpp
    ${FAKES_SRC}
    ${CRC_HOST_SRC}
    ${COMMAND_PARSER_SRC}
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(ping_routing_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(ping_routing_test PRIVATE GTest::gtest_main)
gtest_discover_tests(ping_routing_test PROPERTIES LABELS "integration")
