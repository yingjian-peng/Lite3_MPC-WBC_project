# MPC/WBC 算法为 header-only；主程序只链接该接口库。
add_library(mpc_wbc INTERFACE)
target_include_directories(mpc_wbc INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${THIRD_PARTY}/eigen
  ${WORKSPACE_DIR}/support/types
  ${WORKSPACE_DIR}/support/utils)

# 离线调参 harness（不属于主程序，只产出独立可执行文件）：
#   mpc_wbc_mpc_wbc_sim      : 闭环 MPC+WBC，经 UDP 与 MuJoCo 通信（apps/mpc_wbc_sim.cpp）
#   mpc_wbc_position_loop_sim: 开环位置环基线（apps/position_loop_sim.cpp）
# 主程序默认构建不依赖它们，可用 -DBUILD_MPC_WBC_SIM=OFF 关掉。
option(BUILD_MPC_WBC_SIM "Build the MPC+WBC MuJoCo harness" ON)

if(BUILD_MPC_WBC_SIM)
  foreach(app mpc_wbc_sim position_loop_sim)
    add_executable(mpc_wbc_${app} apps/${app}.cpp)
    target_include_directories(mpc_wbc_${app} PRIVATE
      ${WORKSPACE_DIR}/interface/robot
      ${WORKSPACE_DIR}/interface/user_command
      ${WORKSPACE_DIR}/support/types)
    target_link_libraries(mpc_wbc_${app} PRIVATE mpc_wbc pthread m)
  endforeach()
endif()
