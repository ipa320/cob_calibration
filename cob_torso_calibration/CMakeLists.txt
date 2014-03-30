cmake_minimum_required(VERSION 2.4.6)
project(cob_torso_calibration)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()


###install tags
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/calibration_lower_upper_neck.py
                 src/update_cob_torso_calibration_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

