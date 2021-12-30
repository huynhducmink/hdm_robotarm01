# hdm_robotarm01
This package include a simple model of a 4DOF robot arm with gripper created in URDF format, controlled using effort_controllers package.

# Instruction
Run the following command to open a gazebo world instance and spawn the robot model
```
roslaunch hdm_robotarm01 gazebo_world.launch
```
Next run the following command to use the controller
```
rosrun hdm_robotarm01 model1_controller
```
Currently the robot can only be controlled by entering the rotation angle of each joint independently.

If you want to spawn the model in an opened gazebo world, use the following command
```
roslaunch hdm_robotarm01 gazebo_spawn.launch
```
