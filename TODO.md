Roadmap

- rviz UI
  - make "check starting pose" an action and update the UI at each pose
- ros2
- adjusting motion speed from UI
- online feedback for samples
- plan and save joint trajectories for whole calibration at startup for repeatability
- automatic discovery of motion range
- sampling of multiple calibrations at once
- automatic robot movement 
- other calibration algorithms

Issues 
- namespace 
    - iiwa_stack has a namespace with the robot name (e.g. /iiwa/robot_description); test with robots that have no namespace (panda, ur5?)
- planning to next pose
    - plan to home then target, and visualize

Ideas

- review documentation
    - rviz plugin
    - move_group argument
    - make video

- UIs
    - remove commander? or update it?
    - capability to switch calibrations? (also requires launch file review)
    - feedback about missing directions

- robot movement    
    - UI to adjust motion speed interactively
    - file with list of movements (e.g. as CSV: direction, rot/transl, degrees/cm)
    - save trajectory to file, replay it
        - compute plan to each position and back, save it
        - execute it without calibrating, ask user if good and should be saved
        - when replaying: if too far, plan movement to home position, ask user to confirm manually
        - automatic unattended calibration
    - preview of next movement
    - automatically finding maximum motion range 
        - em, etc: go to max angle that doesn't cause crazy motion
        - optical: find max angle that doesn't cause crazy motion, and where we still get an updated tf transform
        - validate by checking that there are no outliers, and motion range is sufficient
        - automatically finding lag: move back and forth, wait for tf to settle

- calibration
    - move evaluation to backend
    - leave-1-out outlier detection
    - add other calibration algorithms (e.g. optimal hand-eye calibration from strobl/hirzinger)
    - meaningful accuracy measure
    
- simulator
    - more noise models
    - visibility limitation
    - lag
    
- testing
    - end-2-end automatic calibration
        - with different noise
        - with limited visibility
        - with lag