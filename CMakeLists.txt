cmake_minimum_required(VERSION 3.1.3)
project(my_util_scripts)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
    nodes/current_time.py
    nodes/pub_mesh.py
    nodes/tf_static_from_bag.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
