cmake_minimum_required(VERSION 2.8.3)
project(handeyecalibration)

find_package(catkin REQUIRED COMPONENTS
    visp_hand2eye_calibration
    std_msgs
    geometry_msgs
    std_srvs
    message_generation
)

catkin_python_setup()

########################
## Message generation ##
########################

add_message_files(
    FILES
    HandeyeCalibration.msg
    SampleList.msg
)

add_service_files(
    FILES
    TakeSample.srv
    ComputeCalibration.srv
    RemoveSample.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    visp_hand2eye_calibration
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs visp_hand2eye_calibration std_msgs geometry_msgs
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
    scripts/calibrate.py
    scripts/handeye_calibration_commander.py
    scripts/publish.py
    scripts/mkdir_calibration.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS aruco_hand_eye aruco_hand_eye_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    launch/calibrate.launch
    launch/load.launch
    launch/publish.launch
    launch/save.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
