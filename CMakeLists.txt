CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
cmake_policy(SET CMP0072 NEW)
PROJECT(pnd_adam_deploy_public)
enable_language(C)
enable_language(CXX)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "-O2 -Wall")
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_PREFIX_PATH
    ${PROJECT_SOURCE_DIR}/libtorch/share/cmake/Torch
    ${PROJECT_SOURCE_DIR}/third_party/lcm-1.5.0-lib/lib/lcm/cmake
)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(Torch REQUIRED)
FIND_PACKAGE(lcm REQUIRED)

set(adam_type CACHE STRING "adam type: adam_lite or adam_standard or adam_inspire")
set(adam_env_type CACHE STRING "adam env type: real or mujoco or webots")
set(middleware_type CACHE STRING "middleware_type: ros2 or null")

MESSAGE("-- adam_type: ${adam_type}")
MESSAGE("-- adam_env_type: ${adam_env_type}")
MESSAGE("-- middleware_type: ${middleware_type}")

if (${adam_type} STREQUAL "adam_lite")
    ADD_DEFINITIONS(-DADAM_LITE)
elseif(${adam_type} STREQUAL "adam_standard")
    ADD_DEFINITIONS(-DADAM_STANDARD)
elseif(${adam_type} STREQUAL "adam_inspire")
    ADD_DEFINITIONS(-DADAM_INSPIRE)
else()
    MESSAGE(FATAL_ERROR "adam_type must be adam_lite or adam_standard or adam_inspire")
endif()

if(${adam_env_type} STREQUAL "real")
    # nothing to do
    message("-- real robot")
elseif(${adam_env_type} STREQUAL "webots")
    message("-- webots")
    ADD_DEFINITIONS(-DWEBOTS)
elseif(${adam_env_type} STREQUAL "mujoco")
    message("-- mujoco")
    FIND_PACKAGE(mujoco REQUIRED)
    ADD_DEFINITIONS(-DMUJOCO)
else()
    MESSAGE(FATAL_ERROR "adam_env_type must be real or mujoco or webots")
endif()

if("${middleware_type}" STREQUAL "ros2")
    ADD_DEFINITIONS(-Dros2)
    find_package(rclcpp REQUIRED)
    find_package(robotstatepub REQUIRED) 
    message("-- ros2 True")
else()    # nothing to do
    message("-- ros2 False")
endif()

include(FetchContent)
FetchContent_Declare(
    libserial
    GIT_REPOSITORY https://gitee.com/clvhao/libserial.git
    GIT_TAG        master
)
option(LIBSERIAL_ENABLE_TESTING OFF)
option(LIBSERIAL_BUILD_EXAMPLES OFF)
option(LIBSERIAL_PYTHON_ENABLE OFF)
option(LIBSERIAL_BUILD_DOCS OFF)
set(CMAKE_INSTALL_LIBDIR bin)
set(CMAKE_INSTALL_INCLUDEDIR ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_INCLUDEDIR})
set(CMAKE_INSTALL_DATAROOTDIR ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_DATAROOTDIR})
FetchContent_MakeAvailable(libserial)
include_directories(${LibSerial_SOURCE_DIR}/src)

SET(PND_DIR src/pnd-cpp-sdk)
SET(pnd_cpp_sdk_build_dir pnd-cpp-sdk-${CMAKE_BUILD_TYPE})
STRING(TOLOWER ${pnd_cpp_sdk_build_dir} pnd_cpp_sdk_build_dir)
GET_FILENAME_COMPONENT(pnd_cpp_sdk_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${pnd_cpp_sdk_build_dir}" REALPATH)
ADD_SUBDIRECTORY(${PND_DIR} ${pnd_cpp_sdk_build_dir})

LINK_DIRECTORIES(src/pnd-cpp-sdk/pnd/lib/linux_x86_64)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/robot_interface/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party/libs)
# ros
if("${middleware_type}" STREQUAL "ros2")
    include_directories(${rclcpp_INCLUDE_DIRS})
    include_directories(${PROJECT_SOURCE_DIR}/robotPublisher/include)
endif()

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/pnd-cpp-sdk/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/pnd-cpp-sdk/pnd/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party/vnIMU/include)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/robot_interface/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party/yaml-cpp-yaml-cpp-0.7.0/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party/lcm-1.5.0-lib/include)

if("${middleware_type}" STREQUAL "ros2")
    set(main_DIR ${PROJECT_SOURCE_DIR}/example/src/main_ros.cpp)
elseif("${middleware_type}" STREQUAL "lcm")
    set(main_DIR ${PROJECT_SOURCE_DIR}/example/src/main_lcm.cpp)
else()
    set(main_DIR ${PROJECT_SOURCE_DIR}/example/src/main.cpp)
endif()
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/fsm fsm_DIR)

if (${adam_env_type} STREQUAL "webots")
    SET(WEBOTS_HOME "/usr/local/webots")
    LINK_DIRECTORIES(${WEBOTS_HOME}/lib/controller)
    SET(SIM_DEPENDENCIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/WebotsInterface/include)
    INCLUDE_DIRECTORIES(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
    AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/WebotsInterface/src AgentSrc)
elseif(${adam_env_type} STREQUAL "mujoco")
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/mujoco/include)
    AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/mujoco AgentSrc)
    AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/mujoco/src AgentSrc)
    set(SIM_DEPENDENCIES 
    pthread
    mujoco::mujoco
    glfw)
else()
    set(AgentSrc
        ${PROJECT_SOURCE_DIR}/src/real_robot.cpp 
        third_party/vnIMU/src/vnIMU.cpp
        src/imu.cpp
        src/imu_check.cpp
    )
    AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/third_party/vnIMU/src/vn AgentSrc)
    AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/third_party/hi14r5 AgentSrc)
endif()

# Setup the target executable
ADD_EXECUTABLE(${PROJECT_NAME}
    ${main_DIR}
    ${fsm_DIR}
    ${AgentSrc} 
    src/basic_function.cpp
    src/joystick.cpp
    src/fsm_state_impl.cpp
    src/robot_handler.cpp
    src/joint_interface.cpp
    src/putil.cpp
)

# And link the library against the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC 
    Threads::Threads 
    "${TORCH_LIBRARIES}" 
    RobotInterface
    yaml-cpp
    lcm
    libserial_shared
    pnd pndc++
    ${SIM_DEPENDENCIES}
)

if("${middleware_type}" STREQUAL "ros2")
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC 
    rclcpp::rclcpp  
)
endif()

if("${middleware_type}" STREQUAL "ros2")
    ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    robotstatepub
    )
endif()

if (${adam_env_type} STREQUAL "webots")
    set(webots_ctrl_path ${adam_type})
    MESSAGE("-- webots_ctrl_path: ${webots_ctrl_path}")
    ADD_CUSTOM_COMMAND(TARGET  ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E 
        copy_directory ${PROJECT_SOURCE_DIR}/python_scripts/source ${CMAKE_SOURCE_DIR}/webots/${webots_ctrl_path}/controllers/${PROJECT_NAME}/python_scripts/source)
    ADD_CUSTOM_COMMAND(TARGET  ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E 
        copy_directory ${PROJECT_SOURCE_DIR}/robot_interface/config ${CMAKE_SOURCE_DIR}/webots/${webots_ctrl_path}/controllers/${PROJECT_NAME}/robot_interface/config)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
        copy ${CMAKE_BINARY_DIR}/${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/webots/${webots_ctrl_path}/controllers/${PROJECT_NAME}/)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
        copy ${CMAKE_SOURCE_DIR}/config.yaml ${CMAKE_SOURCE_DIR}/webots/${webots_ctrl_path}/controllers/${PROJECT_NAME}/config.yaml)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
        copy ${CMAKE_SOURCE_DIR}/src/pnd-cpp-sdk/pnd/lib/linux_x86_64/libpnd.so.1.5.1 ${CMAKE_SOURCE_DIR}/webots/${webots_ctrl_path}/controllers/${PROJECT_NAME}/libpnd.so.1.5.1)
elseif(${adam_env_type} STREQUAL "mujoco")
    ADD_CUSTOM_COMMAND(TARGET  ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E 
    copy_directory ${PROJECT_SOURCE_DIR}/python_scripts/source ${CMAKE_SOURCE_DIR}/build_adam_lite_mujoco/python_scripts/source)
    ADD_CUSTOM_COMMAND(TARGET  ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E 
    copy_directory ${PROJECT_SOURCE_DIR}/robot_interface/config ${CMAKE_SOURCE_DIR}/build_adam_lite_mujoco/robot_interface/config)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_SOURCE_DIR}/config.yaml ${CMAKE_SOURCE_DIR}/build_adam_lite_mujoco/config.yaml)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_SOURCE_DIR}/config.yaml ${CMAKE_SOURCE_DIR}/build_adam_lite_mujoco/config.yaml)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_SOURCE_DIR}/src/pnd-cpp-sdk/pnd/lib/linux_x86_64/libpnd.so.1.5.1 ${CMAKE_SOURCE_DIR}/build_adam_lite_mujoco/libpnd.so.1.5.1)
else()
    INSTALL(TARGETS ${PROJECT_NAME}
            RUNTIME DESTINATION bin)
    INSTALL(DIRECTORY robot_interface/config/ DESTINATION bin/robot_interface/config
            PATTERN "robot_interface/config/*"
            PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
            GROUP_EXECUTE GROUP_READ)
    INSTALL(DIRECTORY python_scripts/ DESTINATION bin/python_scripts
            PATTERN "python_scripts/*"
            PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
            GROUP_EXECUTE GROUP_READ)

    INSTALL(DIRECTORY src/pnd-cpp-sdk/pnd/lib/linux_x86_64/ DESTINATION bin
            PATTERN "src/pnd-cpp-sdk/pnd/lib/linux_x86_64/*")
    INSTALL(FILES ${CMAKE_BINARY_DIR}/pnd-cpp-sdk-release/libpndc++.so DESTINATION bin)
    INSTALL(FILES ${CMAKE_BINARY_DIR}/pnd-cpp-sdk-release/libpndc++.so.1 DESTINATION bin)
    INSTALL(FILES ${CMAKE_BINARY_DIR}/pnd-cpp-sdk-release/libpndc++.so.1.5.1 DESTINATION bin)
    INSTALL(FILES robot_interface/lib/libRobotInterface.so DESTINATION bin)
    INSTALL(DIRECTORY libtorch/lib/ DESTINATION bin
            PATTERN "libtorch/lib/*.a" EXCLUDE
            PATTERN "libtorch/lib/*.so")
    INSTALL(FILES ${PROJECT_NAME}.sh DESTINATION bin)
    INSTALL(DIRECTORY third_party/libs/ DESTINATION bin
            PATTERN "third_party/libs/*")
endif()
