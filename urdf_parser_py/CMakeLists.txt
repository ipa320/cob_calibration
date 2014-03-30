cmake_minimum_required(VERSION 2.4.6)
project(urdf_parser_py)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()


###install tags
install(PROGRAMS examples/display_urdf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples
)
