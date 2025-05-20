# pnd-cpp-sdk

## Requirements

- cmake
- gcc
- [spdlog](https://github.com/gabime/spdlog)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [json](https://github.com/nlohmann/json)

## Example

Add the following to your CMakeLists.txt

```C++
set(PND_DIR pnd-cpp-sdk)
set(pnd_cpp_sdk_build_dir pnd-cpp-sdk-${CMAKE_BUILD_TYPE})
string(TOLOWER ${pnd_cpp_sdk_build_dir} pnd_cpp_sdk_build_dir)
get_filename_component(pnd_cpp_sdk_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${pnd_cpp_sdk_build_dir}" REALPATH)

add_subdirectory(${PND_DIR} ${pnd_cpp_build_dir})


IF (WIN32)
    link_directories(
        ${PROJECT_SOURCE_DIR}/${PND_DIR}/pnd-cpp-sdk/pnd/lib/win_x64
    )
ELSEIF (APPLE)
    EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
    message( STATUS "Architecture: ${ARCHITECTURE}" )
    if( ${ARCHITECTURE} STREQUAL  "arm64" )
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PND_DIR}/pnd/lib/darwin_arm64)
        link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    elseif( ${ARCHITECTURE} STREQUAL  "x86_64" )
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PND_DIR}/pnd/lib/darwin_x86_64)
        link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    else()
        message( "ERROR: Platform architecture mismatch in macos" )
    endif()
ELSEIF (UNIX)
    EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
    message( STATUS "Architecture: ${ARCHITECTURE}" )
    if( ${ARCHITECTURE} STREQUAL  "armv7l" )
        link_directories(
            ${PROJECT_SOURCE_DIR}/${PND_DIR}/pnd-cpp-sdk/pnd/lib/linux_armv7l
        )
    elseif( ${ARCHITECTURE} STREQUAL  "x86_64" )
        link_directories(
            ${PROJECT_SOURCE_DIR}/${PND_DIR}/pnd-cpp-sdk/pnd/lib/linux_x86_64
        )
    else()
        message( "ERROR: Platform architecture mismatch" )
    endif()
ENDIF ()

target_link_libraries(${PROJECT_NAME}
    PRIVATE pnd pndc++
)
```

## [wiki](http://wiki.pndrobotics.com//getting-started_cn/)
