cmake_minimum_required(VERSION 3.8)
project(rover_scp)

# ============================================================
# This package is now a mixed Python + C++ ament_cmake package.
#   Python node: map_publisher_node.py  (unchanged)
#   C++ nodes:   rover_pp_node          (replaces rover_pp_node.py)
# ============================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ---- ROS2 dependencies ----
find_package(ament_cmake         REQUIRED)
find_package(ament_cmake_python  REQUIRED)
find_package(rclcpp              REQUIRED)
find_package(rclpy               REQUIRED)
find_package(std_msgs            REQUIRED)
find_package(geometry_msgs       REQUIRED)
find_package(nav_msgs            REQUIRED)

# ---- Third-party C++ dependencies ----
find_package(Eigen3 3.3     REQUIRED NO_MODULE)
find_package(OsqpEigen      REQUIRED)
find_package(nlohmann_json  REQUIRED)

# ============================================================
# C++ library: rover_scp_lib
# Bundles rover_model + rover + scp so the node links cleanly.
# ============================================================
add_library(rover_scp_lib
  src/rover_model.cpp
  src/rover.cpp
  src/scp.cpp
)
target_include_directories(rover_scp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rover_scp_lib
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)
ament_target_dependencies(rover_scp_lib rclcpp)

# ============================================================
# C++ executable: rover_pp_node
# ============================================================
add_executable(rover_pp_node src/rover_pp_node.cpp)
target_link_libraries(rover_pp_node
  rover_scp_lib
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(rover_pp_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS rover_pp_node rover_scp_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ============================================================
# Python node: map_publisher_node (stays Python, no changes)
# ============================================================
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  rover_scp/map_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_publisher_node
)

ament_package()
