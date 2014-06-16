cmake_minimum_required(VERSION 2.4.6)
project(cob_calibration_config)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#Add new robots to the list
set(robotlist
cob3-3
)


foreach(robot ${robotlist})
    install(DIRECTORY ${robot}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endforeach(robot)


install(DIRECTORY common
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
