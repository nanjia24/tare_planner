cmake_minimum_required(VERSION 3.5)
project(tare_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Select an architecture-compatible OR-Tools distribution explicitly.
# Configuration must not download files or modify another workspace.
set(ORTOOLS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/or-tools" CACHE PATH "OR-Tools C++ distribution root")
find_library(ORTOOLS_LIBRARY NAMES ortools PATHS "${ORTOOLS_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT ORTOOLS_LIBRARY)
  message(FATAL_ERROR "OR-Tools library missing; set ORTOOLS_ROOT to its C++ distribution")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
  ${ORTOOLS_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
  /opt/ros/humble/include
  # /usr/local/include
  # /usr/include
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

# ## Specify additional locations for library files
# link_directories(
#   /usr/local/lib # Location when using 'make system_install'
#   /usr/lib       # More usual location (e.g. when installing using a package)
#   or-tools/lib
# )

add_library(lidar_model src/lidar_model/lidar_model.cpp)
ament_target_dependencies(lidar_model rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 visualization_msgs pcl_conversions)
# target_link_libraries(lidar_model ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(graph src/graph/graph.cpp)
ament_target_dependencies(graph rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
# target_link_libraries(graph ${catkin_LIBRARIES})

add_library(misc_utils src/utils/misc_utils.cpp)
ament_target_dependencies(misc_utils rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
# target_link_libraries(misc_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(pointcloud_utils src/utils/pointcloud_utils.cpp)
ament_target_dependencies(pointcloud_utils rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
# target_link_libraries(pointcloud_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tsp_solver src/tsp_solver/tsp_solver.cpp)
ament_target_dependencies(tsp_solver rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(tsp_solver ${catkin_LIBRARIES} ${ORTOOLS_LIBRARY})

add_library(viewpoint src/viewpoint/viewpoint.cpp)
ament_target_dependencies(viewpoint rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(viewpoint ${catkin_LIBRARIES} lidar_model)

add_library(rolling_grid src/rolling_grid/rolling_grid.cpp)
ament_target_dependencies(rolling_grid rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
target_link_libraries(rolling_grid ${catkin_LIBRARIES} misc_utils pointcloud_utils)

add_library(viewpoint_manager src/viewpoint_manager/viewpoint_manager.cpp)
ament_target_dependencies(viewpoint_manager rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(viewpoint_manager ${catkin_LIBRARIES} rolling_grid viewpoint grid_world)

add_library(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
ament_target_dependencies(local_coverage_planner rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(local_coverage_planner ${catkin_LIBRARIES} viewpoint_manager)

add_library(grid_world src/grid_world/grid_world.cpp)
ament_target_dependencies(grid_world rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(grid_world ${catkin_LIBRARIES} viewpoint_manager tsp_solver keypose_graph exploration_path)

add_library(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
ament_target_dependencies(pointcloud_manager rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
target_link_libraries(pointcloud_manager ${catkin_LIBRARIES})

add_library(keypose_graph src/keypose_graph/keypose_graph.cpp)
ament_target_dependencies(keypose_graph rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(keypose_graph ${catkin_LIBRARIES} ${PCL_LIBRARIES} viewpoint_manager)

add_library(rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
ament_target_dependencies(rolling_occupancy_grid rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
target_link_libraries(rolling_occupancy_grid ${catkin_LIBRARIES})

add_library(planning_env src/planning_env/planning_env.cpp)
ament_target_dependencies(planning_env rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(planning_env ${catkin_LIBRARIES} ${PCL_LIBRARIES} rolling_occupancy_grid)

add_library(exploration_path src/exploration_path/exploration_path.cpp)
ament_target_dependencies(exploration_path rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
target_link_libraries(exploration_path ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
ament_target_dependencies(tare_visualizer rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions visualization_msgs)
target_link_libraries(tare_visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
ament_target_dependencies(sensor_coverage_planner_ground rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(sensor_coverage_planner_ground ${catkin_LIBRARIES} graph planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer)

add_executable(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
ament_target_dependencies(navigationBoundary rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions)
target_link_libraries(navigationBoundary ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
ament_target_dependencies(tare_planner_node rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(tare_planner_node ${catkin_LIBRARIES} sensor_coverage_planner_ground)

# All point-cloud code uses PCL directly; no pcl_ros API is used.
foreach(target lidar_model graph misc_utils pointcloud_utils tsp_solver viewpoint
    rolling_grid viewpoint_manager local_coverage_planner grid_world pointcloud_manager
    keypose_graph rolling_occupancy_grid planning_env exploration_path tare_visualizer
    sensor_coverage_planner_ground navigationBoundary tare_planner_node)
  target_link_libraries(${target} ${PCL_LIBRARIES})
endforeach()
set_target_properties(tare_planner_node PROPERTIES INSTALL_RPATH "$ORIGIN/or-tools")
install(DIRECTORY "${ORTOOLS_ROOT}/lib/" DESTINATION lib/${PROJECT_NAME}/or-tools
        FILES_MATCHING PATTERN "*.so*")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_collision_geometry test/test_collision_geometry.cpp)
  ament_add_gtest(
    test_execution_reference_shortcut
    test/test_execution_reference_shortcut.cpp)
endif()

#############
## Install ##
#############

install(TARGETS
  navigationBoundary
  tare_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY rviz/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}
      )
install(DIRECTORY data/
        DESTINATION share/${PROJECT_NAME}
      )

ament_package()
