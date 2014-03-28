cmake_minimum_required(VERSION 2.4.6)
project(cob_calibration_executive)

find_package(catkin REQUIRED COMPONENTS)

#catkin_python_setup()

catkin_package()


###install tags
install(DIRECTORY config
  DESTINATIN ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATIN ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY common
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
