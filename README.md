# ros2_vicon

`ros2_vicon` is a ROS2 Vicon that retrieves data from Vicon software and
publishes it on ROS2 topics


## Usage

1. Find the IP address running Vicon Tracker software via `ipconfig` in the
   command-prompt in windows.

2. Clone `ros2_vicon`, download VICON DataStream SDK and run

```
cd <colcon workspace>/src
git clone https://github.com/chutsu/ros2_vicon
cd ros2_vicon && make deps

cd <colcon workspace>/src
colcon build
source install/setup.bash
ros2 run ros2_vicon --ros-args -p hostname:=<ip address of vicon tracker>
```

## LICENCE

MIT
