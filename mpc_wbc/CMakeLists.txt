#===============================================================================================
# MPC+WBC 核心算法构建配置
# 本文件由工程根目录 add_subdirectory(mpc_wbc) 引入；第三方和工程路径由上层提供。
#===============================================================================================


# 阶段 1：创建核心算法接口库
# INTERFACE 表示算法实现位于头文件中，此目标本身不生成 .a 或 .so 文件。
# 主控制程序链接 mpc_wbc 后，会继承下面的头文件搜索路径与核心头文件列表。
add_library(mpc_wbc INTERFACE)


# 阶段 2：向使用者提供头文件搜索目录
# CMAKE_CURRENT_SOURCE_DIR 为当前 mpc_wbc 目录；其余变量由工程根 CMakeLists.txt 设置。
target_include_directories(mpc_wbc INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}            # 00～07 核心算法头文件
  ${THIRD_PARTY}/eigen                   # Eigen 矩阵、向量及线性方程求解
  ${WORKSPACE_DIR}/support/types         # 工程共用数据类型
  ${WORKSPACE_DIR}/support/utils)        # 工程共用数学与辅助工具


# 阶段 3：按依赖顺序列出核心头文件，便于在 IDE 中定位和维护
# 头文件通过主程序的 #include 编译，不会在此分别生成独立的算法可执行程序。
target_sources(mpc_wbc INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/00_lite3_model.hpp          # 几何、运动学、动力学与质心模型
  ${CMAKE_CURRENT_SOURCE_DIR}/01_state_estimator.hpp      # 基座速度、高度和有效性估计
  ${CMAKE_CURRENT_SOURCE_DIR}/02_gait_trajectory.hpp      # 速度选段、相位、足端轨迹与接触计划
  ${CMAKE_CURRENT_SOURCE_DIR}/03_box_qp.hpp               # MPC 的盒约束 QP 求解器
  ${CMAKE_CURRENT_SOURCE_DIR}/04_qp_ipm.hpp               # WBC 的一般约束 QP 内点求解器
  ${CMAKE_CURRENT_SOURCE_DIR}/05_srbd_mpc.hpp             # 单刚体接触力滚动预测控制
  ${CMAKE_CURRENT_SOURCE_DIR}/06_weighted_wbc.hpp         # 全身任务与动力学 QP 控制
  ${CMAKE_CURRENT_SOURCE_DIR}/07_mpc_wbc_controller.hpp)  # 控制链编排、关节命令与 MPC 工作线程



  
