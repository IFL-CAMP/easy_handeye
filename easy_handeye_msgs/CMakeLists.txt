cmake_minimum_required(VERSION 2.8.3)
project(easy_handeye_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
        DIRECTORY msg
        FILES
        HandeyeCalibration.msg
        SampleList.msg
        TargetPoseList.msg
)

add_service_files(
        DIRECTORY srv/calibration
        FILES
        ComputeCalibration.srv
        RemoveSample.srv
        TakeSample.srv
)

add_service_files(
        DIRECTORY srv/robot_movements
        FILES
        CheckStartingPose.srv
        EnumerateTargetPoses.srv
        SelectTargetPose.srv
        PlanToSelectedTargetPose.srv
        ExecutePlan.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)