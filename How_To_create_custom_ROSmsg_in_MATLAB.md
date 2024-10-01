# 123
# How To create a customized ROS msg in MATLAB?

## step-1. Place your custom messages in a location and note the folder path. 

It is recommended to put all your custom message definitions in a single packages folder.

```
folderpath = 'c:\MATLAB\custom_msgs\packages';
folderpath = '/home/teng/MATLAB/custom_msgs/packages';
```

For example, my custom msg is located in `/home/teng/catkin_ws2/src/phantom_omni/teng4pkg_msgs`, then I copy the package folder `teng4pkg_msgs` into the MATLAB path `/home/teng/MATLAB/custom_msgs/`.

In MATLAB, run the command to create custom msg for MATLAB, 

```
rosgenmsg('c:\MATLAB\custom_msgs')
rosgenmsg('/home/teng/MATLAB/custom_msgs/')
```
Matlab will run and show the following info if successful,

% rosgenmsg('/home/teng/MATLAB/custom_msgs/')
% Identifying message files in folder '/home/teng/MATLAB/custom_msgs'..Done.
% Validating message files in folder '/home/teng/MATLAB/custom_msgs'..Done.
% [1/1] Generating MATLAB interfaces for custom message packages... Done.
% Running catkin build in folder '/home/teng/MATLAB/custom_msgs/matlab_msg_gen_ros1/glnxa64'.
% Build in progress. This may take several minutes...
% Build succeeded.build_log. 

To use the custom messages in Matlab, follow these steps: 
1. Add the custom message folder to the MATLAB path by executing:

```
addpath('/home/teng/MATLAB/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
savepath
```
 
2. Refresh all message class definitions, which requires clearing the workspace, by executing:

```
clear classes
rehash toolboxcache
```
 
3. Verify that you can use the custom messages. 

Enter "rosmsg list" and ensure that the output contains the generated custom message types.

```
rosmsg list
```

It will show a long list including your custom msg types. For example, mine is,

> teng4pkg_msgs/Arm_State                                        
> teng4pkg_msgs/PoseCartesian6DOF                                
> teng4pkg_msgs/Vector7DOF

Check the details of a specific msg type, by

```
custommsg = rosmessage('B/Standalone')
custommsg = rosmessage('teng4pkg_msgs/PoseCartesian6DOF')
```

> custommsg = 
>   ROS PoseCartesian6DOF message with properties:
>     MessageType: 'teng4pkg_msgs/PoseCartesian6DOF'
>               X: 0
>               Y: 0
>               Z: 0
>               A: 0
>               B: 0
>               G: 0
>   Use showdetails to show the contents of the message

Bingo. All done. Now you can use the custom msg in Matlab.


--------
Created on 2024-10-01. 
