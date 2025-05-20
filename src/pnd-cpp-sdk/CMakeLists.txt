
cmake_minimum_required (VERSION 3.14)

cmake_policy(SET CMP0048 NEW)
project (pnd_cpp_api VERSION 1.5.1 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release")

# add_definitions(-DDEBUG)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/pnd/include)

IF (WIN32)
    link_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/pnd/lib/win_x64
    )
ELSEIF (APPLE)
    EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
    message( STATUS "Architecture: ${ARCHITECTURE}" )
    if( ${ARCHITECTURE} STREQUAL  "arm64" )
        link_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/pnd/lib/darwin_arm64
        )
    elseif( ${ARCHITECTURE} STREQUAL  "x86_64" )
        link_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/pnd/lib/darwin_x86_64
        )
    else()
        message( "ERROR: Platform architecture mismatch in darwin" )
    endif()
    ELSEIF (UNIX)
    EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
    if( ${ARCHITECTURE} STREQUAL  "armv7l" )
        link_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/pnd/lib/linux_armv7l
        )
    elseif(${ARCHITECTURE} STREQUAL "aarch64")
        link_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/pnd/lib/linux_aarch64
        )
    elseif( ${ARCHITECTURE} STREQUAL  "x86_64" )
        link_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/pnd/lib/linux_x86_64
        )
    else()
        message( "ERROR: Platform architecture mismatch" )
    endif()
ENDIF ()

# 编译静态库
set(pnd_cpp_src
    src/lookup.cpp
    src/groupCommand.cpp
    src/groupFeedback.cpp
    src/group.cpp
    src/version.cpp
)

# 编译库
add_library(_pndc++-obj OBJECT ${pnd_cpp_src})
add_library(pndc++ SHARED $<TARGET_OBJECTS:_pndc++-obj>)
add_library(pndc++-static STATIC $<TARGET_OBJECTS:_pndc++-obj>)

# 链接动态库pnd
target_link_libraries(pndc++ PUBLIC pnd)
target_link_libraries(pndc++-static PUBLIC pnd)

target_include_directories(_pndc++-obj
    PUBLIC src include pnd
)
target_include_directories(pndc++ INTERFACE $<TARGET_PROPERTY:_pndc++-obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(pndc++-static INTERFACE $<TARGET_PROPERTY:_pndc++-obj,INTERFACE_INCLUDE_DIRECTORIES>)

# 项目支持C++14
set_target_properties(_pndc++-obj pndc++ pndc++-static PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${pnd_cpp_api_VERSION}
  SOVERSION 1
)

if (WIN32)
    set_target_properties(pndc++ PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

set_property(TARGET pndc++ pndc++-static PROPERTY INTERFACE_LINK_LIBRARIES
    $<$<BOOL:${UNIX}>:m pthread>
    $<$<BOOL:${WIN32}>:kernel32>
)