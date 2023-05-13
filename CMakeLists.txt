cmake_minimum_required(VERSION 3.5)
project(roboclaw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial)
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

add_executable(roboclaw_node
        src/roboclaw_driver.cpp
        src/roboclaw_node.cpp)

target_include_directories(roboclaw_node PRIVATE ${SERIAL_INCLUDE_DIRS})

ament_target_dependencies(roboclaw_node
        "rclcpp"
        "rclcpp_action"
        "geometry_msgs"
        "std_msgs"
        "nav_msgs")

add_executable(diffdrive_node
        src/roboclaw_driver.cpp
        src/diffdrive_node.cpp)

ament_target_dependencies(diffdrive_node
        "rclcpp"
        "rclcpp_action"
        "geometry_msgs"
        "std_msgs"
        "nav_msgs"
        "tf2_ros")

target_link_libraries(roboclaw_node "${cpp_typesupport_target}" ${SERIAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(diffdrive_node "${cpp_typesupport_target}" ${SERIAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#############
## Install ##
#############

install(TARGETS
        roboclaw_node
        diffdrive_node
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
