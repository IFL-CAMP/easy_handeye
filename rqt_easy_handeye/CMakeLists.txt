cmake_minimum_required(VERSION 3.0.2)
project(rqt_easy_handeye)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
    scripts/rqt_calibration_evaluator
    scripts/rqt_calibrationmovements
    scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
    resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
