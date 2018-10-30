# Roboy Xylophone model
To run roboy xylophpne model, use:
```bash
roslaunch kindyn robot.launch robot_name:=roboy_xylophone start_controllers:='hip_joint sphere_axis0 sphere_axis1 sphere_axis2 elbow_left wrist_left' gazebo:=true xylophone:=true
```

To run roboy_upper_body model with hand and palm, use:
```bash
roslaunch kindyn robot.launch robot_name:=roboy_upper_body start_controllers:='hip_joint sphere_head_axis0 sphere_head_axis1 sphere_head_axis2 sphere_left_axis0 sphere_left_axis1 sphere_left_axis2 elbow_left_rot0 elbow_left_rot1 sphere_right_axis0 sphere_right_axis1 sphere_right_axis2 elbow_right_rot0 elbow_right_rot1 wrist_left_0 wrist_left_1' gazebo:=true xylophone:=true
```