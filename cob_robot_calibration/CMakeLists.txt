cmake_minimum_required(VERSION 2.4.6)
project(cob_robot_calibration)

find_package(catkin REQUIRED COMPONENTS)

#catkin_python_setup()

catkin_package()


###install tags
install(DIRECTORY launch
  DESTINATIN ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(PROGRAMS src/collect_data.py
                 src/generate_config.py
                 src/update_cob_calibration_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(PROGRAMS test/test_calibration_urdf_updater.py
                 test/test_camera_yaml_updater.py
                 test/test_with_cb.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)

install(DIRECTORY test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATiON}/test
)
