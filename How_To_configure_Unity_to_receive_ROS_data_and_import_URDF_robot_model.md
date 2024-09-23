# How to configure Unity to receive ROS data and import URDF robot model

Suppose, you have already had a well-configured `catkin_ws` for publishing data to a ROS node (which will be subscribed by Unity later).
- Require a `ROS-TCP-Endpoint` package in your `catkin_ws`.
- Require a `msg` folder where you defined your own message type, it should be in a specific package in your `catkin_ws`.

## Step-1: Unity add packages (ROS-TCP; Visualizations)
([Source](https://github.com/Unity-Technologies/ROS-TCP-Connector ))

Using Unity 2020.2 or later, open the Package Manager from `Window` --> `Package Manager`.

In the `Package Manager` window, find and click the `+` button in the upper lefthand corner of the window. Select `Add package from git URL....`
Enter the git URL for the desired package. Note: you can append a version tag to the end of the git url, like `#v0.4.0` or `#v0.5.0`, to declare a specific package version, or exclude the tag to get the latest from the package's main branch.

- For the ROS-TCP-Connector, enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`.

- For Visualizations, enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations`. Note that visualization is for visualizing the ROS connection etc (e.g., an icon displays the ROS connection status).

- Click `Add`.


## Step-2: Unity add packages (URDF-Importer)
([Source](https://github.com/Unity-Technologies/URDF-Importer ))

Enter the git URL for the URDF Importer with the latest version tag (currently v0.5.2) `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2` in the text box and press `Enter`.

## Step-3: Generate corresponding msg in Unity.

In Unity top menu, find `Robotics` --> `Generate ROS Message`.

- For `ROS message path`, click `Browse` to find the msg definitions folder in your specific `catkin_ws`, where the `ROS-TCP-Endpoint` package is also installed. Either use the `msg` folder or its package folder in `catkin_ws` both OK, it does not matter, will get the same results. 

- For `Built message path`, click `Browse` to find the folder where you want to contain the build msg results. By default, it will be using 'unityROSproject_test1/Assets/RosMessages/mynewpkg/msg/Arm_StateMsg.cs'

## Step-4: Unity add a script to receive ROS data.

- Add a `Scripts` folder inside folder `Assets`.
- Inside `Scripts` folder, create a new cs script (say, `ROSDataReceiver.cs`) to receive ROS data msg.
- Create an empty object in the scene, rename it as `ROSDataReceiver`, and attach the `ROSDataReceiver.cs` to it.

Copy-paste the following code into `ROSDataReceiver.cs` for receiving data from ROS.

```
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Mynewpkg; //note, use your customized msg-type if necessary.
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;

public class ROSDataReceiver : MonoBehaviour
{
    //Unity.Robotics.UrdfImporter.Control.Controller controller; //note, this "controller" is the "Controller.cs" attached to the robot object.
    
    // Required for ROS communication
    [SerializeField]
    string m_SubscriberName = "/arm_target1"; //note, /arm_target1 is command values in Arm_State.msg type. [joint_1_angle,.., joint_2_angle].
    GameObject myRobot; //note, this will be defined by selecting the robot object as its 'value'.
    public float data_received_a1 = 0.0f; //to display the received ROS data.
    public float data_received_a2 = 0.0f; //to display the received ROS data.
    public float data_received_a3 = 0.0f; //to display the received ROS data.
    public float data_received_a4 = 0.0f; //to display the received ROS data.
    public float data_received_a5 = 0.0f; //to display the received ROS data.
    public float data_received_a6 = 0.0f; //to display the received ROS data.
   
    // ROS Connector
    ROSConnection m_Ros;        
    
    // Start is called before the first frame update
    void Start()
    {
        //Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        
        //Get the Controller.cs component from robot
        //controller = myRobot.GetComponent<Controller>(); //this retrieves the Controller.cs attached to the robot object.        
        //print("test, controller.name=: " + controller.name);
    }

    // Update is called once per frame
    void Update()
    {
        //Receiving data from ROS node.
        m_Ros.Subscribe<Arm_StateMsg>(m_SubscriberName, MsgDataProcessing);
    }    
    
    void MsgDataProcessing(Arm_StateMsg dataMsg){ 
        data_received_a1 = (float)dataMsg.joint_1_angle; //to display the received ROS data.
        data_received_a2 = (float)dataMsg.joint_2_angle; //to display the received ROS data.
        data_received_a3 = (float)dataMsg.joint_3_angle; //to display the received ROS data.
        data_received_a4 = (float)dataMsg.joint_4_angle; //to display the received ROS data.
        data_received_a5 = (float)dataMsg.joint_5_angle; //to display the received ROS data.
        data_received_a6 = (float)dataMsg.joint_6_angle; //to display the received ROS data.              
    }
}
```

## Notes.

- Originally, `com.unity.robotics.urdf-importer@90f353e435` is located in path `youUnityProj/Library/PackageCache/`. You can copy-paste it from `Library/PackageCache` to folder `Packages` for convenience. (Note that, in it, a `Controller` folder with several controller scripts need to be modified for your own use.)
- Automatically, the imported tool is already attached with a “Controller.cs”.

## Test Unity receives ROS data.

In Terminal(s), do the following:

```
$ roscore

$ echo $ROS_PACKAGE_PATH
$ cd catkin_ws2
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
$ roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000 //this will launch the bridge between ROS and Unity.

$ echo $ROS_PACKAGE_PATH
$ cd catkin_ws2
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
$ roslaunch omni_common omni_state4.launch //this will publish data to ROS. And now you should be able to receive data in Unity “ROSDataReceiver” object.
```

------
Created on 2024-09-23.
