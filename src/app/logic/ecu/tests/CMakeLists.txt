# ECU-logic test site (logic::ecu): the controller and its Communication / Telemetry /
# Control collaborators. Shared infra (GoogleTest, support/, LOGIC_* vars, FAKES_SRC)
# comes from src/app/tests. The controller + collaborators are header-only templates.

set(ECU_INCLUDE_DIRS ${TEST_SUPPORT} ${LOGIC_ECU} ${LOGIC_COMMON_INCLUDE_DIRS})

# --- ECU controller logic (unit) ------------------------------------------
add_executable(ecu_controller_test
    unit/logic/ecu_controller_test.cpp
    ${FAKES_SRC}
    ${CRC_HOST_SRC}   # the SD recorder now stamps a block-footer CRC via logic::data_integrity::crc32
    ${LOGIC_COMMON}/control/persistent_state.cpp
)
target_include_directories(ecu_controller_test PRIVATE ${ECU_INCLUDE_DIRS})
target_link_libraries(ecu_controller_test PRIVATE GTest::gtest_main)
gtest_discover_tests(ecu_controller_test PROPERTIES LABELS "unit")
