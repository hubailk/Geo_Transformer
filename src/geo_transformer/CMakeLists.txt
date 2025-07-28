cmake_minimum_required(VERSION 3.8)
project(geo_transformer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
)

# Generate interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocalCoordinateSet.srv"
  "srv/LocalCoordinateGet.srv"
  "srv/FromLL.srv"
  "srv/ToLL.srv"
)

# Get the type support target
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Create and configure the node executable
add_executable(geo_transformer_node src/geo_transformer_node.cpp)
ament_target_dependencies(geo_transformer_node
  "rclcpp" "std_msgs" "geographic_msgs")
target_link_libraries(geo_transformer_node
  ${GeographicLib_LIBRARIES}
  "${cpp_typesupport_target}")

# Create and configure the test client executable
add_executable(test_client src/test_client.cpp)
ament_target_dependencies(test_client
  "rclcpp")
target_link_libraries(test_client
  "${cpp_typesupport_target}")

# Install the executables
install(TARGETS 
  geo_transformer_node
  test_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
