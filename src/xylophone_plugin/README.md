# Roboy Xylophone model

To run roboy xylophpne model, use:
```bash
roslaunch xylophone_plugin robot.launch robot_name:=roboy_xylophone start_controllers:='sphere_head_axis0 sphere_head_axis1 sphere_head_axis2 sphere_left_axis0 sphere_left_axis1 sphere_left_axis2 elbow_left_rot0 elbow_left_rot1 sphere_right_axis0 sphere_right_axis1 sphere_right_axis2 elbow_right_rot0 elbow_right_rot1 left_wrist_0 left_wrist_1 right_wrist_0 right_wrist_1 hip_joint left_stick_tip_joint right_stick_tip_joint' gazebo:=true xylophone:=true
```