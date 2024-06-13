cmake_minimum_required(VERSION 3.8)
project(very_simple_robot_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include Cpp "include" directory
include_directories(
                     include/${PROJECT_NAME}
                     ${rclcpp_INCLUDE_DIRS}
                   )

# Create Cpp executable
add_executable( kinect_simulator src/kinect_simulator_node.cpp
                                 src/kinect_simulator.cpp
                                 src/vsrs_utils.cpp )

ament_target_dependencies( kinect_simulator rclcpp
                                            geometry_msgs
                                            std_msgs
                                            nav_msgs
                                            tf2
                                            tf2_geometry_msgs
                                            image_transport
                                            cv_bridge
                                            OpenCV )

add_executable( lidar_simulator src/lidar_simulator_node.cpp
                                src/lidar_simulator.cpp
                                src/vsrs_utils.cpp )

ament_target_dependencies( lidar_simulator rclcpp
                                           geometry_msgs
                                           std_msgs
                                           nav_msgs
                                           tf2
                                           tf2_geometry_msgs
                                           image_transport
                                           cv_bridge
                                           OpenCV )

#add_executable( kobuki_simulator src/kobuki_simulator.cpp
#                                 src/kobuki_simulator_node.cpp)

#ament_target_dependencies( kobuki_simulator rclcpp
#                                            geometry_msgs
#                                            std_msgs
#                                            nav_msgs
#                                            tf2
#                                            tf2_geometry_msgs )

install(TARGETS kinect_simulator DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_simulator DESTINATION lib/${PROJECT_NAME})
#install(TARGETS kobuki_simulator DESTINATION lib/${PROJECT_NAME})

#add_executable(minimal_opencv_ros2_node src/minimal_opencv_ros2_node.cpp)
#ament_target_dependencies(minimal_opencv_ros2_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

#install(TARGETS minimal_opencv_ros2_node DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS scripts/kinect_simulator.py
                 scripts/kobuki_simulator.py
                 scripts/lidar_simulator.py
                 scripts/rangefinder.py
                 scripts/utils.py
                 scripts/world_state_gui.py
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

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
