cmake_minimum_required(VERSION 3.10)

# Only the validated open-loop foundations are active. The old optimizers,
# orchestration, solver tests and closed-loop entry points have been deleted.
add_library(mpc_wbc INTERFACE)
target_include_directories(mpc_wbc INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${THIRD_PARTY}/eigen
  ${WORKSPACE_DIR}/support/types
  ${WORKSPACE_DIR}/support/utils)

if(BUILD_MPC_WBC_SIM)
  add_executable(mpc_wbc_position_loop_sim apps/position_loop_sim.cpp)
  target_link_libraries(mpc_wbc_position_loop_sim mpc_wbc interface pthread)
endif()

if(BUILD_MPC_WBC_TESTS)
  foreach(component kinematics dynamics gait_trajectory state_estimator)
    add_executable(lite3_${component}_test test/lite3_${component}_test.cpp)
    target_link_libraries(lite3_${component}_test PRIVATE mpc_wbc)
  endforeach()
  add_test(NAME mpc_wbc_gait_trajectory COMMAND lite3_gait_trajectory_test)

  add_executable(lite3_foundation_test test/lite3_foundation_test.cpp)
  target_link_libraries(lite3_foundation_test PRIVATE mpc_wbc)
  add_executable(lite3_simulation_interface_test test/lite3_simulation_interface_test.cpp)
  target_include_directories(lite3_simulation_interface_test PRIVATE ${WORKSPACE_DIR}/interface/robot)
  target_link_libraries(lite3_simulation_interface_test PRIVATE mpc_wbc pthread)
  add_test(NAME mpc_wbc_foundation_contracts COMMAND lite3_foundation_test)
  add_test(NAME mpc_wbc_simulation_interface COMMAND lite3_simulation_interface_test)
  set_tests_properties(mpc_wbc_simulation_interface PROPERTIES TIMEOUT 15)

  find_program(PYTHON3_EXECUTABLE python3)
  if(PYTHON3_EXECUTABLE)
    add_test(NAME mpc_wbc_simulation_execution
      COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_execution.py)
    set_tests_properties(mpc_wbc_simulation_execution PROPERTIES TIMEOUT 15)
    foreach(component kinematics dynamics)
      if(component STREQUAL "kinematics")
        set(generator gen_mujoco_cases.py)
        set(count 500)
      else()
        set(generator gen_mujoco_dynamics_cases.py)
        set(count 200)
      endif()
      set(cases ${CMAKE_BINARY_DIR}/mpc_wbc_${component}_cases.txt)
      add_test(NAME mpc_wbc_gen_${component}_cases
        COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/${generator} ${cases} ${count})
      add_test(NAME mpc_wbc_${component} COMMAND lite3_${component}_test ${cases})
      set_tests_properties(mpc_wbc_gen_${component}_cases PROPERTIES
        WORKING_DIRECTORY ${WORKSPACE_DIR} FIXTURES_SETUP ${component}_cases)
      set_tests_properties(mpc_wbc_${component} PROPERTIES FIXTURES_REQUIRED ${component}_cases)
    endforeach()

    set(est_cases ${CMAKE_BINARY_DIR}/mpc_wbc_estimator_rollout.csv)
    add_test(NAME mpc_wbc_gen_estimator_rollout
      COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/gen_estimator_rollout.py ${est_cases} 10)
    add_test(NAME mpc_wbc_state_estimator COMMAND lite3_state_estimator_test ${est_cases})
    set_tests_properties(mpc_wbc_gen_estimator_rollout PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR} FIXTURES_SETUP estimator_cases)
    set_tests_properties(mpc_wbc_state_estimator PROPERTIES FIXTURES_REQUIRED estimator_cases)

    set(effective_cases ${CMAKE_BINARY_DIR}/closed_loop_dynamics_cases.txt)
    add_test(NAME mpc_wbc_gen_effective_dynamics
      COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/gen_effective_dynamics.py ${effective_cases})
    add_test(NAME mpc_wbc_effective_dynamics COMMAND lite3_foundation_test ${effective_cases})
    set_tests_properties(mpc_wbc_gen_effective_dynamics PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR} FIXTURES_SETUP effective_cases)
    set_tests_properties(mpc_wbc_effective_dynamics PROPERTIES FIXTURES_REQUIRED effective_cases)
  endif()
endif()
