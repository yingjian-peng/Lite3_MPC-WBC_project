cmake_minimum_required(VERSION 3.10)

# Lite3 MPC+WBC 算法库。
# 当前 00_lite3_model.hpp 为 header-only 运动学；S2 起会加入动力学实现（.cpp），
# 届时把 INTERFACE 改为 STATIC 并列出源文件即可。
add_library(mpc_wbc INTERFACE)
target_include_directories(mpc_wbc INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${THIRD_PARTY}/eigen
  ${WORKSPACE_DIR}/support/types
  ${WORKSPACE_DIR}/support/utils
)

if(BUILD_MPC_WBC_TESTS)
  add_executable(lite3_kinematics_test test/lite3_kinematics_test.cpp)
  target_link_libraries(lite3_kinematics_test PRIVATE mpc_wbc)
  add_executable(lite3_dynamics_test test/lite3_dynamics_test.cpp)
  target_link_libraries(lite3_dynamics_test PRIVATE mpc_wbc)

  find_program(PYTHON3_EXECUTABLE python3)
  if(PYTHON3_EXECUTABLE)
    set(MPC_WBC_KI_CASES ${CMAKE_BINARY_DIR}/mpc_wbc_kinematics_cases.txt)
    set(MPC_WBC_DYN_CASES ${CMAKE_BINARY_DIR}/mpc_wbc_dynamics_cases.txt)
    add_test(NAME mpc_wbc_gen_kinematics_cases
      COMMAND ${PYTHON3_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/test/gen_mujoco_cases.py
              ${MPC_WBC_KI_CASES} 500)
    add_test(NAME mpc_wbc_kinematics
      COMMAND lite3_kinematics_test ${MPC_WBC_KI_CASES})
    add_test(NAME mpc_wbc_gen_dynamics_cases
      COMMAND ${PYTHON3_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/test/gen_mujoco_dynamics_cases.py
              ${MPC_WBC_DYN_CASES} 200)
    add_test(NAME mpc_wbc_dynamics
      COMMAND lite3_dynamics_test ${MPC_WBC_DYN_CASES})
    set_tests_properties(mpc_wbc_gen_kinematics_cases PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_SETUP mpc_wbc_ki_cases)
    set_tests_properties(mpc_wbc_kinematics PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_REQUIRED mpc_wbc_ki_cases)
    set_tests_properties(mpc_wbc_gen_dynamics_cases PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_SETUP mpc_wbc_dyn_cases)
    set_tests_properties(mpc_wbc_dynamics PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_REQUIRED mpc_wbc_dyn_cases)
  else()
    message(WARNING "python3 not found: MPC/WBC test executables are built, "
                    "but the MuJoCo table-check tests are not registered")
  endif()
endif()
