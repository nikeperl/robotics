# Пересобрать пакеты
```
cd ~/ros2_ws
rm -rf build install log
colcon build
colcon build --packages-select <pkg>
```

## ex01
```
source .ros_bashrc
ros2 run service_full_name service_name
```
```
source .ros_bashrc
ros2 run service_full_name client_name Иванов Иван Иванович
```
## ex02
```
nano ros2_ws/src/ros_tutorials/turtlesim/src/turtle_frame.cpp 
```
строки с размерами: setFixedSize(500, 500);
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
ros2 bag record -o turtle_cmd_vel --storage mcap /turtle1/cmd_vel
ros2 bag play turtle_cmd_vel --storage mcap --rate 1.0
```
## ex03
```
ros2 doctor --report > robotics/ex03/doctor.txt
```
## ex04
```
ros2 run turtlesim turtlesim_node
ros2 run move_to_goal move_to_goal 10 10 -2.25
ros2 run move_to_goal move_to_goal 1 10 -3.14
ros2 run move_to_goal move_to_goal 1 1 -1.57
ros2 run move_to_goal move_to_goal 10 1 0
ros2 run move_to_goal move_to_goal 10 10 1.57
```
## ex05
```
source ~/.ros_bashrc 
ros2 run turtlesim turtlesim_node
ros2 run action_cleaning_robot cleaning_action_server
ros2 action list
ros2 run action_cleaning_robot cleaning_action_client

```
