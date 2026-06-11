# Common-logic test site (logic::control, logic::actuation, protocol/command).
# Shared infra (GoogleTest, support/, LOGIC_* vars) comes from src/app/tests.

add_executable(persistent_state_test
    unit/logic/control/persistent_state_test.cpp
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(persistent_state_test PRIVATE ${TEST_SUPPORT} ${LOGIC_COMMON_INCLUDE_DIRS})
target_link_libraries(persistent_state_test PRIVATE GTest::gtest_main)
gtest_discover_tests(persistent_state_test PROPERTIES LABELS "unit")

# Header-only on the logic side (the Valve concept + FakeValve double).
add_executable(valve_test
    unit/logic/actuation/valve_test.cpp
)
target_include_directories(valve_test PRIVATE ${TEST_SUPPORT} ${LOGIC_COMMON_INCLUDE_DIRS})
target_link_libraries(valve_test PRIVATE GTest::gtest_main)
gtest_discover_tests(valve_test PROPERTIES LABELS "unit")

set(COMMAND_DIR ${LOGIC_COMMON}/communication/protocol/command)
add_executable(command_test
    unit/logic/communication/protocol/command/command_test.cpp
    ${COMMAND_DIR}/parser/command_can_parser.cpp
    ${COMMAND_DIR}/parser/command_ethernet_parser.cpp
)
target_include_directories(command_test PRIVATE ${TEST_SUPPORT} ${LOGIC_COMMON_INCLUDE_DIRS})
target_link_libraries(command_test PRIVATE GTest::gtest_main)
gtest_discover_tests(command_test PROPERTIES LABELS "unit")
