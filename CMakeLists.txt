cmake_minimum_required(VERSION 3.5)
project(gazebo_simulation_ros2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf world models
  DESTINATION share/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  find_package(ament_cmake_pytest REQUIRED)

#  ament_add_pytest_test(description test/test_description.py)
#endif()

ament_package()