cmake_minimum_required(VERSION 3.14)
project(pnd_cpp_examples)
set(CMAKE_CXX_STANDARD 14)
set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../)
get_filename_component(ROOT_DIR "${ROOT_DIR}" REALPATH)


if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(PND_DIR ${ROOT_DIR})

set(pnd_cpp_build_dir pnd-cpp-${CMAKE_BUILD_TYPE})
string(TOLOWER ${pnd_cpp_build_dir} pnd_cpp_build_dir)
get_filename_component(pnd_cpp_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${pnd_cpp_build_dir}" REALPATH)

add_subdirectory(${PND_DIR} ${pnd_cpp_build_dir})

include_directories(${ROOT_DIR}/src)
include_directories(${ROOT_DIR}/pnd/include)

link_directories(
    ${ROOT_DIR}/pnd/lib/win_x64
    ${ROOT_DIR}/pnd/lib/darwin_arm64
    ${ROOT_DIR}/pnd/lib/darwin_x86_64
    ${ROOT_DIR}/pnd/lib/linux_armv7l
    ${ROOT_DIR}/pnd/lib/linux_x86_64
)

SET(AIOS_SOURCES
  ${ROOT_DIR}/examples/src/01_lookup.cpp
  ${ROOT_DIR}/examples/src/01_lookup_from_ips.cpp
  ${ROOT_DIR}/examples/src/reboot.cpp
  ${ROOT_DIR}/examples/src/get_error.cpp
  ${ROOT_DIR}/examples/src/get_device_list.cpp
  ${ROOT_DIR}/examples/src/clear_error.cpp
  ${ROOT_DIR}/examples/src/enable_disable.cpp
  ${ROOT_DIR}/examples/src/03_feedback.cpp
  ${ROOT_DIR}/examples/src/04_command.cpp
  ${ROOT_DIR}/examples/src/05_feedback_command_async.cpp
  ${ROOT_DIR}/examples/src/06_latency_test.cpp
  ${ROOT_DIR}/examples/src/reset_linear_count.cpp
  ${ROOT_DIR}/examples/src/motion/position_pt.cpp
  ${ROOT_DIR}/examples/src/motion/position_pt_test.cpp
  ${ROOT_DIR}/examples/src/motion/double_group_position_pt.cpp
  ${ROOT_DIR}/examples/src/motion/velocity_pt.cpp
  ${ROOT_DIR}/examples/src/motion/velocity_pt_2p.cpp
  ${ROOT_DIR}/examples/src/motion/velocity.cpp
  ${ROOT_DIR}/examples/src/motion/trapezoidal_pos.cpp
  ${ROOT_DIR}/examples/src/motion/current.cpp
  ${ROOT_DIR}/examples/src/motion/torque_pt.cpp
  ${ROOT_DIR}/examples/src/config/motion_control_config.cpp
  ${ROOT_DIR}/examples/src/encoder/encoder.cpp
  ${ROOT_DIR}/examples/src/encoder/motor_rotor_abs_pos.cpp
  ${ROOT_DIR}/examples/src/rcu/rcu.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../bin)
foreach (EXAMPLE ${AIOS_SOURCES})
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  add_executable(${EX_NAME} ${EXAMPLE})

  target_compile_features(${EX_NAME} PUBLIC cxx_std_14)
  
  target_link_libraries(${EX_NAME} pndc++-static)
  target_link_libraries(${EX_NAME} pnd)

  add_custom_command(TARGET ${EX_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory
                         $<TARGET_FILE_DIR:${EX_NAME}>/log)

endforeach (EXAMPLE ${AIOS_SOURCES})

