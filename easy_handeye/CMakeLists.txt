cmake_minimum_required(VERSION 2.8.3)
project(easy_handeye)

find_package(catkin REQUIRED COMPONENTS
    visp_hand2eye_calibration
    std_msgs
    geometry_msgs
    std_srvs
    message_generation
)

catkin_python_setup()

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

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs visp_hand2eye_calibration std_msgs geometry_msgs
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
    launch/rqt_easy_handeye.perspective
    launch/rviz_easy_handeye.config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
