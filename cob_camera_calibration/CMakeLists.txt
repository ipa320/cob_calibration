cmake_minimum_required(VERSION 2.4.6)
project(cob_camera_calibration)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

###install tags
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/mono_calibration.py
                 src/publish_detections.py
                 src/stereo_calibration.py
                 src/test_bagfile.py
                 src/visibility_checker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(PROGRAMS test/test_calibration_data.py
                 test/test_calibration_object_detector.py
                 test/test_calibration_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)

install(DIRECTORY test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
