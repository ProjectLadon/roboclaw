cmake_minimum_required(VERSION 3.5)
project(roboclaw_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(roboclaw REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MotorVelocity.msg"
        "msg/MotorVelocitySingle.msg"
        "msg/MotorDutySingle.msg"
        "msg/EncoderSteps.msg"
        "msg/MotorPosition.msg"
        "msg/MotorPositionSingle.msg"
        "msg/MotorVoltsAmps.msg"
        "msg/EncoderVelocity.msg"
        "msg/Status.msg"
        "msg/MotorVelocityStamped.msg"
        "msg/MotorVelocitySingleStamped.msg"
        "msg/MotorDutySingleStamped.msg"
        "msg/EncoderStepsStamped.msg"
        "msg/MotorPositionStamped.msg"
        "msg/MotorPositionSingleStamped.msg"
        "msg/MotorVoltsAmpsStamped.msg"
        "msg/EncoderVelocityStamped.msg"
        "msg/StatusStamped.msg"
        "msg/MotorPwmStamped.msg"
        "srv/GetVelocityPid.srv"
        "srv/SetVelocityPid.srv"
        "srv/GetPositionPid.srv"
        "srv/SetPositionPid.srv"
        "srv/ResetEncoder.srv"
        "srv/ResetMotor.srv"
        "srv/WriteEeprom.srv"
        "srv/ReadEeprom.srv"
        "srv/SetCurrentLimit.srv"
        "srv/GetCurrentLimit.srv"
        DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

###########
## Build ##
###########

#############
## Install ##
#############

ament_export_dependencies(rosidl_default_runtime)

if(rcl_interfaces_VERSION VERSION_LESS "1.2.0")
  install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
  )
else()
  # NOTE(vooon): Humble
  install(
    DIRECTORY include/
    DESTINATION include/roboclaw_interfaces
    FILES_MATCHING PATTERN "*.hpp"
  )
endif()

ament_package()
