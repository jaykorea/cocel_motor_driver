# T-motor(AK series) Driver - ROS2(foxy) Interface 

## Install Driver
```
i).   git clone to "your_ws/src"
ii).  colcon build --symlink-install
iii). source install/local_setup.bash
```
## Launch
```
ros2 launch cocel_driver cocel_driver.launch.py
```

## Notice
```
1).   Motor driver parameter configuration file is on 'config' folder
ii).  Basic can frame interface api is from here(Added getMotorState function on MotorDrive.cpp) - https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can
iii). Motor status topic : "~ns/motor_status", tpye: std_msgs::msg::Float32MultiArray
iv).  Run command topic : "~ns/cmd_topic", type : cocel_driver::msg::CocelDriverCmd
v).   Stop command topic : "~ns/stop_topic", type : std_msgs::msg::Empty
``` 
## Todo
```
i.)   Set Run command limitation 
```
