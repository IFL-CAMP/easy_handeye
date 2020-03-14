cmake_minimum_required(VERSION 3.0.2)
project(easy_handeye)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
    std_srvs
    visp_hand2eye_calibration
)

catkin_python_setup()

add_message_files(
    FILES
    HandeyeCalibration.msg
    SampleList.msg
)

add_service_files(
    FILES
    ComputeCalibration.srv
    RemoveSample.srv
    TakeSample.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    visp_hand2eye_calibration
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime std_msgs visp_hand2eye_calibration
)

install(PROGRAMS
    scripts/calibrate.py
    scripts/handeye_calibration_commander.py
    scripts/publish.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/calibrate.launch
    launch/publish.launch
    launch/rqt_${PROJECT_NAME}.perspective
    launch/rviz_${PROJECT_NAME}.config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
