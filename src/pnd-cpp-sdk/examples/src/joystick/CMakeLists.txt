cmake_minimum_required(VERSION 3.14)
project(pnd_cpp_examples)
set(CMAKE_CXX_STANDARD 17)
set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../)
get_filename_component(ROOT_DIR "${ROOT_DIR}" REALPATH)


if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Debug)
endif()


find_package(SDL2 REQUIRED)

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
  ${ROOT_DIR}/examples/src/joystick/joystick_vel_pt.cpp
)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../bin)
foreach (EXAMPLE ${AIOS_SOURCES})
  get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
  add_executable(${EX_NAME} ${EXAMPLE})

  target_compile_features(${EX_NAME} PUBLIC cxx_std_17)
  
  target_link_libraries(${EX_NAME} pndc++-static)
  target_link_libraries(${EX_NAME} pnd ${SDL2_LIBRARIES})

  add_custom_command(TARGET ${EX_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory
                         $<TARGET_FILE_DIR:${EX_NAME}>/log)

  target_include_directories(${EX_NAME} PRIVATE ${SDL2_INCLUDE_DIRS})

endforeach (EXAMPLE ${AIOS_SOURCES})


