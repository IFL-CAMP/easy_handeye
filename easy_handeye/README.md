
easy_handeye: TF / VISP Hand-Eye Calibration
============================================

This package wraps the hand-eye calibration routine from the ViSP library
(contained in the `visp_hand2eye_calibration` package) to provide a simple
camera pose estimation solution. Input is expected as transformations published in TF.

Additionally, a solution for saving and publishing the resulting calibration is provided.
Computing and using a calibration can be achieved by including/starting a single launch file respectively.

A (optional) GUI for the position sampling and automatic robot movement is provided in the `rqt_easy_handeye` package.

## Use Cases

If you are unfamiliar with Tsai's hand-eye calibration [1], it can be used in two ways:

- **eye-in-hand** -- To compute the static transform between the reference frames of
  a robot's hand effector and that of a tracking system, e.g. the optical frame
  of an RGB camera used to track AR markers. In this case, the camera is
  mounted on the end-effector, and you place the visual target so that it is
  fixed relative to the base of the robot; for example, you can place an AR marker on a table.
- **eye-on-base** -- To compute the static transform from a robot's base to a tracking system, e.g. the
  optical frame of a camera standing on a tripod next to the robot. In this case you can attach a marker,
  e.g. an AR marker, to the end-effector of the robot.

The (arguably) best part is, that you do not have to care about the placement of the auxiliary marker
(the one on the table in the eye-in-hand case, or on the robot in the eye-on-base case). The algorithm
will erase that transformation out, and only return the transformation you are interested in.

## Usage

Two launch files, one for computing and one for publishing the calibration respectively,
are provided to be included in your own. The default arguments should be
overridden to specify the correct tf reference frames, and to avoid conflicts when using
multiple calibrations at once.

### Calibration

For both use cases, you can either launch the `calibrate.launch`
launch file, or you can include it in another launchfile as shown below. Either
way, the launch file will bring up the `visp_hand2eye_calibration` solver, along with an
integration script. By default, the integration script will interactively ask you
to accept or discard each sample. At the end, the parameters will be saved in a yaml file.

#### eye-in-hand

```xml
<launch>
  <include file="$(find easy_handeye)/launch/calibrate.launch">
    <arg name="eye_on_hand" value="true"/>

    <arg name="robot_base_frame" value="/base_link"/>
    <arg name="robot_effector_frame" value="/ee_link"/>

    <arg name="tracking_base_frame" value="/optical_origin"/>
    <arg name="tracking_marker_frame" value="/optical_target"/>
  </include>
</launch>
```

#### eye-on-base

```xml
<launch>
  <include file="$(find easy_handeye)/launch/calibrate.launch">
    <arg name="eye_on_hand" value="false"/>

    <arg name="robot_base_frame" value="/base_link"/>
    <arg name="robot_effector_frame" value="/ee_link"/>

    <arg name="tracking_base_frame" value="/optical_origin"/>
    <arg name="tracking_marker_frame" value="/optical_target"/>
  </include>
</launch>
```

#### Expected input

The Tsai-Lenz method computes the transformation between the end effector of the robot and the optical frame of the camera,
given a set of corresponding transformations between the robot base and the robot base effector, and between the camera
and a tracked target.
This package expects these transformations to be published in real time in TF, in order to save them simultaneously as pairs.
The names of the TF reference frames can be passed as input; the defaults conform to the ROS Industrial standard [2].

#### Tips for accuracy

The following tips are given in [1], paragraph 1.3.2.

- Maximize rotation between poses.
- Minimize the distance from the target to the camera of the tracking system.
- Minimize the translation between poses.
- Use redundant poses.
- Calibrate the camera intrinsics if necessary / applicable.
- Calibrate the robot if necessary / applicable.

### Publishing
The `publish.launch` starts a node that publishes the transformation found during calibration in tf.
The parameters are automatically loaded from the yaml file.

## ROS API

All following topics and parameters live in the namespace specified by the top-level launch file
(typically robot_device_eye_on_{hand,base}). This allows multiple calibrations to coexist.

### Topics/Services

- `take_sample`: a sample of each transformation is added to the list
- `remove_sample(int)`: removes the specified sample from the list
- `compute_calibration`: the calibration transformation is computed from the lists of transformations
- `save_calibration`: the calibration is saved in the parameters and to file


- `sample_list`: after adding or removing a sample, the list is published here
- `calibration_result`: after computation the transform is published on this topic

### Parameters

- `eye_on_hand`: if true, this is an eye-on-hand calibration, else eye-on-base.
- `tracking_base_frame`: contains the tf id of the tracking system coordinate origin frame
- `tracking_marker_frame`: contains the tf id of the tracking system target
- `robot_base_frame`: contains the tf id of the robot's base link
- `robot_effector_frame`: contains the tf id of the robot's end effector link

## References

[1] *Tsai, Roger Y., and Reimar K. Lenz. "A new technique for fully autonomous
and efficient 3D robotics hand/eye calibration." Robotics and Automation, IEEE
Transactions on 5.3 (1989): 345-358.*

[2] ROS Industrial: <http://wiki.ros.org/Industrial>