cmake_minimum_required(VERSION 2.4.6)
project(cob_image_capture)

find_package(catkin REQUIRED COMPONENTS)

#catkin_python_setup()

catkin_package()


###install tags
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/image_capture.py
                 src/simple_capture_srv_caller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

