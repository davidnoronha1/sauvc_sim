cmake_minimum_required(VERSION 3.8)
project(sauvc_sim)

find_package(
  ament_cmake REQUIRED
  ament_cmake_python REQUIRED
  rclcpp REQUIRED
  rclpy REQUIRED
	gazebo_ros REQUIRED
  ros_gz_interfaces REQUIRED
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/teleop.py
  scripts/teleop25.py
  scripts/entity_spawner.py
  scripts/sim_testing_node.py
  scripts/capture_view.py
  scripts/generate_dataset.py
  scripts/make_pool_textures.py
  scripts/underwater_camera_node.py
  scripts/altimeter_to_pressure_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  worlds
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
