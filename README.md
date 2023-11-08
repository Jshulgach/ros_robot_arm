 
 # ROS2 Robot Arm Driver
 
 This package provides a node which serves as a ROS2 wrapper for interfacing with the Desktop or Mini arm robots. 
 
 The package was designed with [ROS2 Iron](https://docs.ros.org/en/iron/index.html) on Ubuntu 22.04, but should work for Windows and potentially older versions of ROS2 as well.
 
 ## Getting Started
 
 Please see the documentation on the [Desktop Arm](https://github.com/Jshulgach/Desktop-Arm) robot or [Mini Arm](https://github.com/Jshulgach/Mini-Arm) robot for instructions on setting up hardware. 
 
 You may need to install some python dependencies:
 ```bash
 python3 -m pip install pyserial
 ```
 
 Navitage to your ROS2 workspace, clone the repository, build and source:
 ```bash
 cd /path/to/workspace 
 git clone https://github.com/Jshulgach/ros_robot_arm.git src/ros_robot_arm
 colcon build 
 source install/setup.bash
 ```
 
To run the node, it can be started with the following command:
```
ros2 run ros_robot_arm node
```

To interface your own client node into a larger system, the `example.launch.yaml` file is a good start. You can run the example file with the following command:
```
ros2 launch ros_robot_arm example.launch.yaml
```

## License

Copyright 2023-2024

Created by: Jonathan Shulgach (jshulgac@andrew.cmu.edu)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
