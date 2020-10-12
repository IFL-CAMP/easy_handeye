cmake_minimum_required(VERSION 3.0.2)
project(easy_handeye)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
    scripts/calibrate.py
    scripts/robot.py
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
