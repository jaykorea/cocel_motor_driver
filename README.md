# T-motor(AK series) Driver - ROS2(foxy) Interface 

## Install Driver
```
1). git clone to "your_ws/src"
2). colcon build --symlink-install
3). source install/local_setup.bash
```
## Launch
```
ros2 launch cocel_driver cocel_driver.launch.py
```

## Notice
```
1).  Motor driver parameter configuration file is on 'config' folder
2).  Basic can frame interface api is from here(Added getMotorState function on MotorDrive.cpp) - https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can
3). Motor status topic : "~ns/motor_status", tpye: std_msgs::msg::Float32MultiArray
4). Run command topic : "~ns/cmd_topic", type : cocel_driver::msg::CocelDriverCmd
5). Stop command topic : "~ns/stop_topic", type : std_msgs::msg::Empty
``` 
## Todo
```
1). Set Run command limitation - Done 
```
