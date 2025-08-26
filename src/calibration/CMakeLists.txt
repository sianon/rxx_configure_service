cmake_minimum_required(VERSION 3.8)
project(calibration_collect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dobot_msgs_v3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LIB_DIR})

add_executable(turtle_circle  src/turtle_circle.cpp)
ament_target_dependencies(turtle_circle rclcpp geometry_msgs sensor_msgs)

add_executable(calibration_collect src/calibration_collect.cpp
        src/csv_write.cpp
        src/csv_write.h)
ament_target_dependencies(calibration_collect rclcpp geometry_msgs sensor_msgs dobot_msgs_v3 OpenCV cv_bridge)

install(TARGETS
        calibration_collect
        turtle_circle
        DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
