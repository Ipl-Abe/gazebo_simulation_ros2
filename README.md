# gazebo_simulation_ros2


## How to setup gazebo and moveit (Virtual)
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



## How to setup gazebo and moveit with mycobot hardware (WSL2)
See this: https://qiita.com/motoJinC25/items/e332d731111c2a29ca25
```
$ usbipd list
$ usbipd attach --wsl --busid <busid for CP2104 USB to UART Bridge Controller>
```



## Command for running gazebo and moveit with mycobot hardware
```
$ ros2 launch gazebo_simulation_ros2 mycobot_gazebo_moveit.launch.py use_hardware:=true
```
