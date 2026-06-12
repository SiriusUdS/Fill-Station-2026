# FCU-logic test site (logic::fcu): controller, command handlers, ADC seam, plus
# integration tests that assemble the real units. Shared infra (GoogleTest,
# support/, LOGIC_* vars, FAKES_SRC) comes from src/app/tests.

set(FCU_INCLUDE_DIRS ${TEST_SUPPORT} ${LOGIC_FCU} ${LOGIC_COMMON_INCLUDE_DIRS})

# The FCU command-handler translation units, reused by the targets that link them.
set(HANDLERS ${LOGIC_FCU}/control/command_handlers)
set(COMMAND_HANDLER_SOURCES
    ${HANDLERS}/main_handler.cpp
    ${HANDLERS}/ping.cpp
    ${HANDLERS}/execute_ping.cpp
    ${HANDLERS}/set_state.cpp
    ${HANDLERS}/set_state_transitions/activate_igniter.cpp
    ${HANDLERS}/set_valve_position.cpp
    ${HANDLERS}/execute_set_valve_position.cpp
    ${HANDLERS}/synchronise.cpp
    ${HANDLERS}/execute_synchronise.cpp
)

# --- FCU controller logic (unit) ------------------------------------------
add_executable(fcu_controller_test
    unit/logic/fcu_controller_test.cpp
    ${FAKES_SRC}
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

# --- Command handlers (unit) ----------------------------------------------
add_executable(command_handlers_test
    unit/logic/control/command_handlers/command_handlers_test.cpp
    ${COMMAND_HANDLER_SOURCES}
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(command_handlers_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(command_handlers_test PRIVATE GTest::gtest_main)
gtest_discover_tests(command_handlers_test PROPERTIES LABELS "unit")

# --- Ethernet -> state change (integration) -------------------------------
add_executable(ethernet_state_change_test
    integration/ethernet_state_change_test.cpp
    ${COMMAND_HANDLER_SOURCES}
    ${LOGIC_COMMON}/communication/command/parser/command_ethernet_parser.cpp
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(ethernet_state_change_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(ethernet_state_change_test PRIVATE GTest::gtest_main)
gtest_discover_tests(ethernet_state_change_test PROPERTIES LABELS "integration")

# --- UDP SetValvePosition -> valve actuation (integration) ----------------
add_executable(valve_actuation_test
    integration/valve_actuation_test.cpp
    ${FAKES_SRC}
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(valve_actuation_test PRIVATE ${FCU_INCLUDE_DIRS})
target_link_libraries(valve_actuation_test PRIVATE GTest::gtest_main)
gtest_discover_tests(valve_actuation_test PROPERTIES LABELS "integration")
