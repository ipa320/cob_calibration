cmake_minimum_required(VERSION 2.4.6)
project(cob_laser_calibration)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()


###install tags
install(DIRECTORY launch
  DESTINATIN ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY objects
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/determine_cal_obj_pose.py
                 src/write_comment_lines_to_file.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

