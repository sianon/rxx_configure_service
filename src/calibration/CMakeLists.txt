cmake_minimum_required(VERSION 3.8)
project(calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(YAML_CPP_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(PkgConfig REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dobot_msgs_v3 REQUIRED)
find_package(calibration_srv REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_definitions(-DCPPHTTPLIB_NO_SSL)

#pkg_check_modules(LIBNL REQUIRED libnl-3.0 libnl-genl-3.0)

find_package(yaml-cpp CONFIG REQUIRED)
message(STATUS "Found yaml-cpp version: ${yaml-cpp_VERSION}")
message(STATUS "yaml-cpp library path: ${YAML_CPP_LIBRARY}")
# 包含 libnl 头文件
#include_directories(${LIBNL_INCLUDE_DIRS})
include_directories(include)
# 链接 libnl 库
#link_directories(${LIBNL_LIBRARY_DIRS})

include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LIB_DIR})

add_executable(network_test src/network_test.cpp
        src/network_config.cpp
        src/network_config.h
        src/network_control.cpp
        src/network_control.h
        src/csv_write.h
        src/csv_write.cpp
        src/device_launch_conf.cpp
        src/device_launch_conf.h
        include/base64.h
        include/base64.cpp
        include/Kutil.cpp
        include/Kutil.h)
target_link_libraries(network_test yaml-cpp)
ament_target_dependencies(network_test rclcpp geometry_msgs sensor_msgs dobot_msgs_v3 cv_bridge tf2_geometry_msgs calibration_srv)

install(TARGETS
        network_test
#        calibration_test
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
