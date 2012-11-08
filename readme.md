cob_calibration
===============

Calibrates cameras, arm and torso of Care-O-bot

Execution:
----------

bring up robot
	roslaunch cob_bringup robot.launch

start arm navigation
	roslaunch cob_arm_navigation start_planning_environment.launch

Calibrate Torso(WIP)

run data collection
	roslaunch cob_calibration_executive collect_robot_calibration_data.launch

calibrate_cameras(WIP)

restart robot(kill bringup then restart)

run robot_calibration
	roslaunch cob_robot_calibration run_robot_calibration.launch

???
