# Troubleshooting

### Calibration frames

- the **robot frames** should be present in tf (at least the base and the end effector). You can check this by enabling the tf module of Rviz
- the **tracking** system should also publish the transform between the tracking base (e.g. camera or field generator) and marker
- in order to be able to visualize the aforementioned transforms at the same time, we advise to make use of an additional static transform publisher between the robot and the tracking system. If the calibration is eye-on-base, this should be between the robot base and the camera; in the eye-on-hand case, between the end effector and the camera

### Sampling
- the **number of samples** must be sufficient and the robot end effector must have moved into **all its degrees of freedom** (e.g. about all axes)
- when acquiring the samples, check that the **tracking is stable** (the marker tf frame is not shaking)
- when acquiring the samples, check that the **tf frames have stopped moving** (some component of your system might have some delay, so you might have to wait even after the robot has stopped moving)
- when acquiring the samples, check that the **marker is within the field of view** of the camera (the tf frame might "stop moving" at the edge of the field of view and remain stuck there, because of tf buffering)

### Marker tracking

- the tracking should be **stable**; when the system is not moving (i.e. in a steady state), no frames should be moving or shaking
- the tracking should be **accurate**; this can be verified by moving the robot with its control panel by a known quantity, and compare this movement to how the tracking system output changes (e.g. by moving the robot 5 cm in one direction, the marker coordinates should change accordingly). Of course, before calibration it is only possible to check the magnitude of the motion, but this can already identify problems with e.g. the AR marker size
- the **marker should be solidly fixed** to a surface or to the robot hand, so that it does not move accidentally throughout the calibration procedure

### AR markers

AR marker tracking accuracy can degrade:
- if the **marker size is wrong**. It can happen that the printer automatically scales your PDF to fit it into the page, so ALWAYS measure the marker size after printing it (it feels stupid, but losing hours because of it feels much worse)
- if the camera is **not intrinsically calibrated**. You can use the relative camera_calibration ROS package for this purpose
- if the marker is too small within the image; in this case, stretch the robot in the direction of the camera, or print a bigger marker
- if the camera is **directly in front of the marker**; because of its symmetry, pose estimation may be ambiguous and the output may become unstable, constantly changing between two possible solutions. In this case, move the camera so that the marker does not appear like a square, but rather as asymmetric as possible

### Automatic robot motion
- Check that the marker is visible in all positions before acquiring samples
- If the program crashes with a "Got crazy plan!" exception, the **starting position** might be bad for the robot; if the robot is redundant, you can try moving its elbow


