cmake_minimum_required(VERSION 3.10)

set(BUILD_PLATFORM "x86" CACHE STRING "Build platform: x86 or arm")
if(BUILD_PLATFORM STREQUAL "arm")
  set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
elseif(NOT BUILD_PLATFORM STREQUAL "x86")
  message(FATAL_ERROR "BUILD_PLATFORM must be x86 or arm")
endif()

project(lite3_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-fPIC)

option(BUILD_SIM "Build the MuJoCo UDP simulation controller" OFF)
option(SEND_REMOTE "Copy the built controller to Lite3" OFF)
option(USE_MOLD_LINKER "Use mold for faster linking" OFF)

if(BUILD_PLATFORM STREQUAL "arm" OR SEND_REMOTE)
  set(BUILD_SIM OFF)
endif()

message(STATUS "BUILD_PLATFORM: ${BUILD_PLATFORM}")
message(STATUS "BUILD_SIM: ${BUILD_SIM}")
message(STATUS "SEND_REMOTE: ${SEND_REMOTE}")

get_filename_component(WORKSPACE_DIR . ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/support/third_party)
set(ONNX_RUNTIME_ROOT ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM})

include_directories(
  ${THIRD_PARTY}/eigen
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/Lite3_MotionSDK/include/common
  ${THIRD_PARTY}/Lite3_MotionSDK/include
  ${ONNX_RUNTIME_ROOT}/include
  support/types
  support/utils
  interface/robot
  interface/user_command
  state_machine
  onnx/run_policy
)
link_directories(${ONNX_RUNTIME_ROOT}/lib)

add_subdirectory(interface)

if(SEND_REMOTE)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  set(CMAKE_INSTALL_RPATH "../lib/onnxruntime/${BUILD_PLATFORM}/lib")
endif()

if(USE_MOLD_LINKER)
  find_program(MOLD_EXECUTABLE mold)
  if(MOLD_EXECUTABLE)
    add_link_options(-fuse-ld=mold)
  else()
    message(WARNING "mold not found; using the default linker")
  endif()
endif()

add_library(run_policy STATIC onnx/run_policy/lite3_test_policy_runner_onnx.cpp)
target_include_directories(run_policy PUBLIC
  onnx/run_policy
  ${THIRD_PARTY}/eigen
  support/utils
  support/types
  ${ONNX_RUNTIME_ROOT}/include
)
target_link_libraries(run_policy onnxruntime)

add_executable(rl_deploy main.cpp state_machine/parameters/lite3_control_parameters.cpp)
target_link_libraries(rl_deploy run_policy interface pthread m rt dl stdc++)

if(BUILD_SIM)
  target_compile_definitions(rl_deploy PRIVATE BUILD_SIMULATION)
  target_link_libraries(rl_deploy dw)
endif()

if(BUILD_PLATFORM STREQUAL "arm")
  target_link_libraries(rl_deploy
    ${THIRD_PARTY}/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so)
else()
  target_link_libraries(rl_deploy
    ${THIRD_PARTY}/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_x86_64.so)
endif()

if(SEND_REMOTE)
  add_custom_target(push_to_robot ALL DEPENDS rl_deploy)
  add_custom_command(TARGET push_to_robot POST_BUILD
    COMMAND ./support/scripts/sftp_to_remote.sh
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Copy controller to Lite3")
endif()
