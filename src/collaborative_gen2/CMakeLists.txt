cmake_minimum_required(VERSION 3.8)
project(collaborative_gen2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 1. Servidor
add_executable(server_node src/central_server.cpp)
ament_target_dependencies(server_node rclcpp moveit_ros_planning_interface moveit_core geometry_msgs std_msgs tf2_geometry_msgs)

# 2. Cliente A
add_executable(client_a src/client_pos.cpp)
ament_target_dependencies(client_a rclcpp geometry_msgs)

# 3. Cliente B
add_executable(client_b src/client_orient.cpp)
ament_target_dependencies(client_b rclcpp geometry_msgs std_msgs)

install(TARGETS server_node client_a client_b
  DESTINATION lib/${PROJECT_NAME}
)

# --- AGREGA ESTO PARA ARREGLAR EL ERROR ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# ------------------------------------------

ament_package()