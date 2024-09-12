# How To create a customized ROS msg in a catkin_ws?

In ROS, custom message types are defined using `.msg` files, which are stored in a package's msg folder. These message types allow you to define custom data structures that can be used for inter-node communication. Below is a guide on how to create and use custom message types in a `catkin_ws` workspace.

Steps to Customize a msg Folder and Files in a `catkin_ws`.

## 1. Create a ROS Package

First, create a package in your `catkin_ws` workspace. This package will hold your custom messages.

```
cd ~/catkin_ws/src
catkin_create_pkg my_custom_msgs std_msgs rospy roscpp
```

Here, `my_custom_msgs` is the package name, and it depends on `std_msgs`, `rospy`, and `roscpp`.

## 2. Create a `msg` Directory

Inside your package, create a `msg` folder where your custom message types will be defined.

```
cd ~/catkin_ws/src/my_custom_msgs
mkdir msg
```

## 3. Create a '.msg' File

Inside the `msg` folder, create a file with a `.msg` extension. This file defines your custom message type.

Example: `Position.ms`

```
gedit msg/Position.msg
```

Inside `Position.msg`, define the structure of the message. For example, for a 3D position:

```
float64 x
float64 y
float64 z
```

Each line defines a field in the message, with the type followed by the field name.

## 4. Modify `CMakeLists.txt`

You need to modify your package's `CMakeLists.txt` file to include the message generation.

- Find and uncomment (or add) `message_generation` as a required component in `find_package`:

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

- Add the custom message files under `add_message_files`:

```
add_message_files(
  FILES
  Position.msg
)
```

- Add the following after `add_message_files()` to actually generate the messages. Add `generate_messages()` and ensure `std_msgs` and other dependencies are listed:

```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

- Ensure the `catkin_package` includes `message_runtime`:

```
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
```


## 5. Modify `package.xml`

In your package's `package.xml`, ensure you have the following dependencies for message generation:

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

## 6. Build the Package

After modifying the files, go back to the root of your `catkin_ws` workspace and build the workspace.

```
cd ~/catkin_ws
catkin_make
```

If everything is set up correctly, the custom message types will be generated and ready for use.

## 7. Using the Custom Message

Now, you can use your custom message type in your ROS nodes.

**C++ Example:**

```
// teng4note, the Position.h may not be easy to find or even not exist. But it does not matter.
#include "my_custom_msgs/Position.h"

// Publishing a custom message
my_custom_msgs::Position position_msg;
position_msg.x = 1.0;
position_msg.y = 2.0;
position_msg.z = 3.0;
```

**Python Example:**

```
from my_custom_msgs.msg import Position

# Publishing a custom message
position_msg = Position()
position_msg.x = 1.0
position_msg.y = 2.0
position_msg.z = 3.0
```

## 8. Verify the Custom Message

Now, we can verify the custom message in a terminal. First you need to run your file.

```
echo $ROS_PACKAGE_PATH

// May need to source the Workspace Again
source devel/setup.bash

// note, the Terminal needs `cd` to the correct catkin_ws, in order to be able to echo the customized msg.
cd ~/catkin_ws
rostopic list
rostopic list -v

// Check Message Availability, suppose msg route is /teng4pkg_msgs/PoseCartesian6DOF
rosmsg show teng4pkg_msgs/PoseCartesian6DOF

// echo the rostopic, suppose the publisher is /teng4/test1_pose6dof
rostopic echo /teng4/test1_pose6dof

// some other cmd for retrieving the ROS param.
rosparam list
rosparam get /PSM1/rate
cd catkin_ws
echo $ROS_PACKAGE_PATH
source devel/setup.bash
```

## Summary

- Create a ROS package.
- Create a `msg` directory and define your custom `.msg` file.
- Update the `CMakeLists.txt` and `package.xml` to include message generation.
- Build your workspace with `catkin_make`.
- Use the custom message in your ROS nodes for inter-node communication.

This setup allows you to define custom data structures for your ROS applications.

--------
Created on 2024-09-11. 
