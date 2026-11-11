cmake_minimum_required(VERSION 3.8)
project(clothoid_image_hub)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(hub_ops src/hub_ops.cpp)
target_include_directories(hub_ops PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(hub_ops ${OpenCV_LIBS})
set_target_properties(hub_ops PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(image_hub SHARED src/image_hub.cpp)
target_link_libraries(image_hub hub_ops)
ament_target_dependencies(image_hub rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(image_hub
  PLUGIN "clothoid_image_hub::ImageHub"
  EXECUTABLE image_hub_node)

install(TARGETS hub_ops image_hub
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hub_ops test/test_hub_ops.cpp)
  target_link_libraries(test_hub_ops hub_ops)
endif()

ament_package()
