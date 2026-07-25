cmake_minimum_required(VERSION 3.10)
project(g1_comp_servo_service)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|x86_64")
    message(STATUS "Architecture not supported")
endif()


find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

set(UNITREE_SDK2_ROOT "/opt/unitree_sdk2_latest" CACHE PATH
    "Root directory of the Unitree SDK2 installation")
find_path(UNITREE_SDK2_INCLUDE_DIR
    NAMES unitree/idl/go2/MotorCmds_.hpp
    PATHS "${UNITREE_SDK2_ROOT}/include"
    NO_DEFAULT_PATH
    REQUIRED)
find_library(UNITREE_SDK2_LIBRARY
    NAMES unitree_sdk2
    PATHS "${UNITREE_SDK2_ROOT}/lib"
    NO_DEFAULT_PATH
    REQUIRED)

file(GLOB DYNAMIXEL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++/src/dynamixel_sdk/*.cpp")
list(REMOVE_ITEM DYNAMIXEL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++/src/dynamixel_sdk/port_handler_arduino.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++/src/dynamixel_sdk/port_handler_mac.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++/src/dynamixel_sdk/port_handler_windows.cpp")
add_library(dynamixel_sdk STATIC ${DYNAMIXEL_SOURCES})
target_include_directories(dynamixel_sdk PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DynamixelSDK/c++/include/dynamixel_sdk")

include_directories(
  include
  "${UNITREE_SDK2_INCLUDE_DIR}"
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

link_libraries(
  "${UNITREE_SDK2_LIBRARY}"
  dynamixel_sdk
  ddsc
  ddscxx
  rt
  pthread
  yaml-cpp
  fmt::fmt)

# test
add_executable(main main.cpp)
add_executable(test_read_yaml test/test_read_yaml.cpp)
add_executable(test_read_encoder test/test_read_encoder.cpp)
add_executable(test_dds_state test/test_dds_state.cpp)
add_executable(g1_motion_mode g1_motion_mode.cpp)
add_executable(test_lowcmd_monitor test/test_lowcmd_monitor.cpp)
add_executable(test_lowstate_monitor test/test_lowstate_monitor.cpp)
add_executable(test_calibration test/test_calibration.cpp)
add_executable(test_joint0_control test/test_joint0_control.cpp)
add_executable(test_joint1_control test/test_joint1_control.cpp)
add_executable(test_servo_control test/test_servo_control.cpp)
add_executable(test_read_angle test/test_read_angle.cpp)
add_executable(test_servo_homePostion test/test_servo_homePostion.cpp)
