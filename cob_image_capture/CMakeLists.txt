cmake_minimum_required(VERSION 2.4.6)
project(cob_image_calibration)

find_package(catkin REQUIRED COMPONENTS)

#catkin_python_setup()

catkin_package()


###install tags
install(DIRECTORY launch
  DESTINATIN ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
