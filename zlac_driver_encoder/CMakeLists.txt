cmake_minimum_required(VERSION 3.8)
project(zlac_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Thu vien core (khong phu thuoc ROS): dieu khien banh + doc encoder
add_library(zlac_driver_lib src/zlac_driver.cpp src/zlac_encoder.cpp)
target_include_directories(zlac_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Node ROS 2: 1 tien trinh gom 2 node zlac_driver + zlac_encoder
add_executable(zlac_node src/zlac_node.cpp)
target_link_libraries(zlac_node zlac_driver_lib)
ament_target_dependencies(zlac_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS zlac_driver_lib
  EXPORT export_zlac_driver_lib
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS zlac_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_zlac_driver_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs)
ament_package()
