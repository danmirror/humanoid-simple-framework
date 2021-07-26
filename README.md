# rscuad-control
> simple control


#### running simple joint

- [x] Gazebo running
> $ roslaunch rscuad_gazebo rscuad_world.launch

- [x] move joint 
- change parameter in rscuad_move_joint_node.cpp

> $ rosrun rscuad_move_joint rscuad_move_joint_node


#### running walking

- [x] Gazebo running
> $ roslaunch rscuad_gazebo rscuad_world.launch

- [x]  walking
> $ roslaunch rscuad_demo demo.launch

## ROS API

All topics are provided in the /robotis_op namespace.

Actuators (radians for position control, arbitrary normalized speed for cmd_vel):

      /rscuad/l_hip_yaw_position/command
      /rscuad/l_hip_roll_position/command
      /rscuad/l_hip_pitch_position/command
      /rscuad/l_knee_position/command
      /rscuad/l_ank_roll_position/command
      /rscuad/l_ank_pitch_position/command
      /rscuad/r_hip_yaw_position/command
      /rscuad/r_hip_roll_position/command
      /rscuad/r_hip_pitch_position/command
      /rscuad/r_knee_position/command
      /rscuad/r_ank_roll_position/command
      /rscuad/r_ank_pitch_position/command
      /rscuad/l_sho_pitch_position/command
      /rscuad/l_sho_roll_position/command
      /rscuad/l_el_position/command
      /rscuad/r_sho_pitch_position/command
      /rscuad/r_sho_roll_position/command
      /rscuad/r_el_position/command
      /rscuad/head_pan_position/command
      /rscuad/head_tilt_position/command


#### checker flow urdf
> $ rosrun xacro xacro.py src/rscuad/urdf/rscuad.xacro 

#### Author
> <a href="https://me-danuandrean.github.io/">Danu andrean </a>
