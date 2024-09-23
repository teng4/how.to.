# How To convert `ROBOT.urdf.xacro` to `ROBOT.urdf`?

Terminal command to convert the `ROBOT.urdf.xacro` to `ROBOT.urdf` (open a terminal inside its folder).

```
rosrun xacro xacro --inorder -o ./niryo_one.urdf ./niryo_one.urdf.xacro
```

## How to generate urdf form xacro 
(from dvrk code, dvrk_model/Readme.md, not tested.)

```
rosrun xacro xacro mtm_right_only.urdf.xacro > mtm_right_only.urdf
```

## Note

An example of robot URDF source files can be found here [niryo_one_ros](https://github.com/NiryoRobotics/niryo_one_ros).


------
Created on 2024-09-23.
