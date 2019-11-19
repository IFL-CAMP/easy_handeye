cmake_minimum_required(VERSION 2.8.3)
project(rqt_easy_handeye)

find_package(catkin REQUIRED )

catkin_python_setup()

catkin_package( )

install(PROGRAMS
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
