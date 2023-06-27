cmake_minimum_required(VERSION 3.5)
project(scara_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)

# Add Python scripts
ament_python_install_package(src)


# Install Python scripts
install(PROGRAMS
  src/scara.py
  src/scara_robot.py
  src/scara_robot_controller.py
  DESTINATION lib/${PROJECT_NAME})
  
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

# Install URDF file
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install resource directory
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME})

# Install test directory
install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME})

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY worlds 
  DESTINATION share/${PROJECT_NAME})

  
 ament_export_dependencies(ament_cmake xacro gazebo_ros)

# Setup CMake package
ament_package()

