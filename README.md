# gazebo_simulation_ros2


## How to setup
```
$ git clone https://github.com/Ipl-Abe/mycobot_moveit_confg.git
$ git clone https://github.com/Ipl-Abe/gazebo_simulation_ros2.git
$ cd ~/ros2_humble/
$ colcon build --symlink-install
$ . install/setup.bash
$ . /usr/share/gazebo/setup.bash
```

## Command for running gazebo and moveit
```
$ ros2 launch gazebo_simulation_ros2 mycobot_gazebo_moveit.launch.py
```
