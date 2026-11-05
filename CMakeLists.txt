cmake_minimum_required(VERSION 3.10)
project(lite3_control LANGUAGES CXX)

# 阶段 1：设置工程使用的 C++ 标准和默认构建模式
set(CMAKE_CXX_STANDARD 17)                              # 所有 C++ 源文件使用 C++17;
set(CMAKE_CXX_STANDARD_REQUIRED ON)                     # REQUIRED 表示编译器不支持时直接报错
if(NOT CMAKE_BUILD_TYPE)                                # 用户没有指定构建类型时默认使用 Release 优化配置
  set(CMAKE_BUILD_TYPE Release)
endif()



# 阶段 2：定位第三方依赖，并配置头文件和库文件的搜索路径
# PROJECT_SOURCE_DIR 是本 CMakeLists.txt 所在的工程根目录
set(THIRD_PARTY ${PROJECT_SOURCE_DIR}/support/third_party)  # THIRD_PARTY 指向随仓库提供的依赖;
set(WORKSPACE_DIR ${PROJECT_SOURCE_DIR})                # WORKSPACE_DIR 会被 mpc_wbc 子目录使用;
set(ONNX_RUNTIME_ROOT ${THIRD_PARTY}/onnxruntime/x86)   # 当前配置使用仓库中 x86 版本的 ONNX Runtime

include_directories(                                    # 添加头文件搜索目录
  ${THIRD_PARTY}/eigen
  ${ONNX_RUNTIME_ROOT}/include
  ${PROJECT_SOURCE_DIR}/support/types
  ${PROJECT_SOURCE_DIR}/support/utils
  ${PROJECT_SOURCE_DIR}/interface/robot
  ${PROJECT_SOURCE_DIR}/interface/user_command
  ${PROJECT_SOURCE_DIR}/state_machine
  ${PROJECT_SOURCE_DIR}/onnx/run_policy
)

link_directories(${ONNX_RUNTIME_ROOT}/lib)              # 添加 ONNX Runtime 库的搜索目录；



# 阶段 3：加入 MPC/WBC 子工程，并编译 ONNX 策略运行器供控制程序使用
# 加入 MPC/WBC 子工程：通过 mpc_wbc 接口库提供核心算法头文件
add_subdirectory(mpc_wbc)

# 编译 ONNX 策略运行器
add_library(run_policy OBJECT 
  onnx/run_policy/lite3_policy_runner.cpp
)

# PUBLIC 目录会传递给链接 run_policy 的目标，保证使用者也能找到这些头文件
target_include_directories(run_policy PUBLIC
  ${PROJECT_SOURCE_DIR}/onnx/run_policy
  ${THIRD_PARTY}/eigen
  ${PROJECT_SOURCE_DIR}/support/utils
  ${PROJECT_SOURCE_DIR}/support/types
  ${ONNX_RUNTIME_ROOT}/include
)

target_link_libraries(run_policy PUBLIC onnxruntime)    # ONNX Runtime 提供策略模型的加载和推理实现



# 阶段 4：生成两个程序入口，共用状态机、源文件和链接依赖
# 两个程序均支持站立后按 c 进入 RL、按 v 进入 MPC+WBC，按 x 躺下。
# 保留 mpc_control / rl_control 两个名称，控制模式由运行时按键选择。
foreach(control_target IN ITEMS mpc_control rl_control)
  add_executable(${control_target}
    main.cpp
    state_machine/parameters/control_parameters_lite3.cpp
  )
  set_target_properties(${control_target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )

  # 策略推理、MPC/WBC，以及线程、数学、系统运行时和栈回溯依赖
  target_link_libraries(${control_target} PRIVATE
    run_policy mpc_wbc pthread m rt dl stdc++ dw
  )
endforeach()
