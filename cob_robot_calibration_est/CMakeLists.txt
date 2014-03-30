cmake_minimum_required(VERSION 2.4.6)
project(cob_robot_calibration_est)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()


###install tags
install(PROGRAMS src/multi_step_cov_estimator.py
                 src/multi_step_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(PROGRAMS test/camera_chain_sensor_unittest.py
                 test/camera_unittest.py
                 test/chain_sensor_unittest.py
                 test/checkerboard_unittest.py
                 test/dh_chain_unittest.py
                 test/full_chain_dh_vs_fk_arm_unittest.py
                 test/full_chain_dh_vs_fk_unittest.py
                 test/full_chain_unittest.py
                 test/robot_params_unittest.py
                 test/single_transform_unittest.py
                 test/torso_chain_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)

install(DIRECTORY test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

