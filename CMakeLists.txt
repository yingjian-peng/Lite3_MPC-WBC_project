cmake_minimum_required(VERSION 3.10)
project(lite3_mpc_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(THIRD_PARTY ${PROJECT_SOURCE_DIR}/support/third_party)
set(WORKSPACE_DIR ${PROJECT_SOURCE_DIR})
set(ONNX_RUNTIME_ROOT ${THIRD_PARTY}/onnxruntime/x86)

include_directories(
  ${THIRD_PARTY}/eigen
  ${ONNX_RUNTIME_ROOT}/include
  ${PROJECT_SOURCE_DIR}/support/types
  ${PROJECT_SOURCE_DIR}/support/utils
  ${PROJECT_SOURCE_DIR}/interface/robot
  ${PROJECT_SOURCE_DIR}/interface/user_command
  ${PROJECT_SOURCE_DIR}/state_machine
  ${PROJECT_SOURCE_DIR}/onnx/run_policy)
link_directories(${ONNX_RUNTIME_ROOT}/lib)

add_subdirectory(mpc_wbc)

# 默认构建只生成这一个可执行文件，位置固定为 build/mpc_control。
add_library(run_policy OBJECT onnx/run_policy/lite3_test_policy_runner_onnx.cpp)
target_include_directories(run_policy PUBLIC
  ${PROJECT_SOURCE_DIR}/onnx/run_policy
  ${THIRD_PARTY}/eigen
  ${PROJECT_SOURCE_DIR}/support/utils
  ${PROJECT_SOURCE_DIR}/support/types
  ${ONNX_RUNTIME_ROOT}/include)
target_link_libraries(run_policy PUBLIC onnxruntime)

add_executable(mpc_control main.cpp state_machine/parameters/lite3_control_parameters.cpp)
set_target_properties(mpc_control PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_compile_definitions(mpc_control PRIVATE BUILD_SIMULATION)
target_link_libraries(mpc_control PRIVATE run_policy mpc_wbc pthread m rt dl stdc++ dw)
