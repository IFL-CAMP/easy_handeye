cmake_minimum_required(VERSION 2.8.3)
project(rqt_handeyecalibration)

find_package(catkin REQUIRED )

catkin_python_setup()

catkin_package( )

install(PROGRAMS
    scripts/rqt_handeyecalibration
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
