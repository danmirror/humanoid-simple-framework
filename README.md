# rscuad-control
> simple control

#### running simple joint
- [x] Gazebo running
> roslaunch rscuad_gazebo rscuad_world.launch
- [x] manager flow
> rosrun rscuad_manager rscuad_manager
- [x] send data
> rosrun rscuad_gazebo rscuad_gazebo_talker


#### running walking
- [x] Gazebo running
> roslaunch rscuad_gazebo rscuad_world.launch

- [] walking
> rosrun robotis_op_simulation_walking robotis_op_simulation_walking_node

##### optional
- [] demo velocity [under construction]
> rosrun robotis_op_simulation_walking walker_demo.py

- [] see value walking
> rosrun robotis_op_simulation_walking walker.py


#### checker flow urdf
> rosrun xacro xacro.py src/rscuad/urdf/rscuad.xacro 

#### Author
> <a href="https://me-danuandrean.github.io/">Danu andrean </a>
