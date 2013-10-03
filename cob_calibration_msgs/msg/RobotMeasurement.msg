string sample_id    # Tag to figure out which yaml file this was generated from

string target_id    # Defines the target that we were sensing.
string chain_id     # Defines where this target was attached

CameraMeasurement[] M_cam
pr2_controllers_msgs/JointTrajectoryControllerState[] M_chain 
