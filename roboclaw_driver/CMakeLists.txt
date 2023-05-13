cmake_minimum_required(VERSION 3.8)
project(roboclaw_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial)
find_package(Boost REQUIRED COMPONENTS system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
        include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

###########
## Build ##
###########

add_library(roboclaw_driver src/roboclaw_driver.cpp)
ament_target_dependencies(roboclaw_driver "rclcpp")
target_link_libraries(roboclaw_driver ${SERIAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
ament_export_targets(roboclaw_driver HAS_LIBRARY_TARGET)

#############
## Install ##
#############

install(
    DIRECTORY include/roboclaw_driver 
    DESTINATION include 
)

install(
    TARGETS roboclaw_driver
    EXPORT roboclaw_driver
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib 
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include 
)

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
