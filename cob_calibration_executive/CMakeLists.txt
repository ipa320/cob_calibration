cmake_minimum_required(VERSION 2.4.6)
project(cob_calibration_executive)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()


###install tags
#install(DIRECTORY config
#  DESTINATIN ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(DIRECTORY launch
  DESTINATIN ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/chessboard_positions_broadcast.py
                 src/collect_robot_calibration_data.py
                 src/collect_stereo_data.py
                 src/generate_positions.py
                 src/move_arm_to_calibration_position.py
                 src/test_dh_chain_parameter.py
                 src/test_positions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(PROGRAMS src/test/pan_tilt_angle_test.py
                 src/torso_ik_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src/test
)

install(PROGRAMS src/utils/generate_calibration_positions.py
                 src/utils/joint_state_listener_arm.py
                 src/utils/joint_state_listener_torso.py
                 src/kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src/utils
)

install(FILES verify_calibration.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
