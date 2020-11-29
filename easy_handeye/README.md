
easy_handeye: TF / OpenCV / MoveIt! Hand-Eye Calibration
============================================

This package wraps the hand-eye calibration routine from the OpenCV library to provide a simple
camera pose estimation solution. Input is expected as transformations published in `tf`.

Two scripts are provided:
- calibrator: takes care of sampling the specified transformations, computing the calibration and storing the result in a .yaml file.
- publisher: loads the calibration from the .yaml file and publishes it in `tf`; by default uses the frames specified during calibration, but they can be overridden individually via ROS params.

The included launch files allow to override all the script params, and to start the GUI contained in `rqt_easy_handeye`.

## ROS API

All following topics and parameters live in the namespace specified by the top-level launch file
(e.g. `robot_device_eye_on_{hand,base}`). This allows multiple calibrations to coexist.

### Topics/Services

- `take_sample`: a sample of each transformation is added to the list
- `remove_sample(int)`: removes the specified sample from the list
- `compute_calibration`: the calibration transformation is computed from the lists of transformations
- `save_calibration`: the calibration is saved in the parameters and to file
- `sample_list`: after adding or removing a sample, the list is published here
- `calibration_result`: after computation the transform is published on this topic

### Parameters

- `namespace`: the calibrator script will save the result in `~/.ros/easy_handeye/$namespace`; the publisher will load the data from the same path.
- `eye_on_hand`: if true, this is an eye-on-hand calibration, else eye-on-base.
- `tracking_base_frame`: contains the tf id of the tracking system coordinate origin frame.
- `tracking_marker_frame`: contains the tf id of the tracking system target.
- `robot_base_frame`: contains the tf id of the robot's base link.
- `robot_effector_frame`: contains the tf id of the robot's end effector link.

## References

[1] *Tsai, Roger Y., and Reimar K. Lenz. "A new technique for fully autonomous
and efficient 3D robotics hand/eye calibration." Robotics and Automation, IEEE
Transactions on 5.3 (1989): 345-358.*
