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

# S4 位置环跑步基线（通过 UDP 与 mujoco_simulation.py 通信）
if(BUILD_MPC_WBC_SIM)
  add_executable(mpc_wbc_position_loop_sim apps/position_loop_sim.cpp)
  target_link_libraries(mpc_wbc_position_loop_sim mpc_wbc interface pthread)
endif()

if(BUILD_MPC_WBC_TESTS)
  add_executable(lite3_kinematics_test test/lite3_kinematics_test.cpp)
  target_link_libraries(lite3_kinematics_test PRIVATE mpc_wbc)
  add_executable(lite3_dynamics_test test/lite3_dynamics_test.cpp)
  target_link_libraries(lite3_dynamics_test PRIVATE mpc_wbc)
  add_executable(lite3_gait_trajectory_test test/lite3_gait_trajectory_test.cpp)
  target_link_libraries(lite3_gait_trajectory_test PRIVATE mpc_wbc)
  add_executable(lite3_state_estimator_test test/lite3_state_estimator_test.cpp)
  target_link_libraries(lite3_state_estimator_test PRIVATE mpc_wbc)
  add_executable(lite3_srbd_mpc_test test/lite3_srbd_mpc_test.cpp)
  target_link_libraries(lite3_srbd_mpc_test PRIVATE mpc_wbc)
  add_executable(qp_ipm_test test/qp_ipm_test.cpp)
  target_link_libraries(qp_ipm_test PRIVATE mpc_wbc)
  add_executable(lite3_wbc_test test/lite3_wbc_test.cpp)
  target_link_libraries(lite3_wbc_test PRIVATE mpc_wbc)
  add_test(NAME mpc_wbc_gait_trajectory COMMAND lite3_gait_trajectory_test)
  # S6 SRBD-MPC 离线自检：不需要 MuJoCo，纯数值/物理检查
  add_test(NAME mpc_wbc_srbd_mpc COMMAND lite3_srbd_mpc_test)
  # S7：自研 QP 求解器单测 + Weighted-QP WBC 离线自检（不需要 MuJoCo）
  add_test(NAME mpc_wbc_qp_ipm COMMAND qp_ipm_test)

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

    # S7 WBC：接触偏置加速度真值 → WBC 自检（导出 q/qvel/qdd/f/τ/stance）→ MuJoCo 正动力学复核
    set(MPC_WBC_CA_CASES ${CMAKE_BINARY_DIR}/wbc_contact_accel_cases.txt)
    set(MPC_WBC_WBC_CASES ${CMAKE_BINARY_DIR}/wbc_mujoco_cases.csv)
    add_test(NAME mpc_wbc_gen_contact_accel_cases
      COMMAND ${PYTHON3_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/test/gen_mujoco_contact_accel_cases.py
              ${MPC_WBC_CA_CASES} 100)
    add_test(NAME mpc_wbc_wbc
      COMMAND lite3_wbc_test ${MPC_WBC_CA_CASES} ${MPC_WBC_WBC_CASES})
    add_test(NAME mpc_wbc_wbc_mujoco
      COMMAND ${PYTHON3_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/test/check_wbc_hover_mujoco.py
              ${MPC_WBC_WBC_CASES})
    set_tests_properties(mpc_wbc_gen_contact_accel_cases PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_SETUP mpc_wbc_ca_cases)
    set_tests_properties(mpc_wbc_wbc PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_REQUIRED mpc_wbc_ca_cases
      FIXTURES_SETUP mpc_wbc_wbc_csv)
    set_tests_properties(mpc_wbc_wbc_mujoco PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_REQUIRED mpc_wbc_wbc_csv)

    set(MPC_WBC_EST_CASES ${CMAKE_BINARY_DIR}/mpc_wbc_estimator_rollout.csv)
    add_test(NAME mpc_wbc_gen_estimator_rollout
      COMMAND ${PYTHON3_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/test/gen_estimator_rollout.py
              ${MPC_WBC_EST_CASES} 10)
    add_test(NAME mpc_wbc_state_estimator
      COMMAND lite3_state_estimator_test ${MPC_WBC_EST_CASES})
    set_tests_properties(mpc_wbc_gen_estimator_rollout PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_SETUP mpc_wbc_est)
    set_tests_properties(mpc_wbc_state_estimator PROPERTIES
      WORKING_DIRECTORY ${WORKSPACE_DIR}
      FIXTURES_REQUIRED mpc_wbc_est)
  else()
    message(WARNING "python3 not found: MPC/WBC test executables are built, "
                    "but the MuJoCo table-check tests are not registered")
  endif()
endif()
