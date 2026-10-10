# cartpole_mujooco_ros2
This project aims to provide a getting started with mujoco and ROS2. It uses the well-known case of a cartpole.


# setup

```
cd ~/workspace/ROS2/cartpole_mujooco_ros2
source env.sh
cbuild --packages-select cartpole_mujoco cartpole_mujoco_panel
source install/setup.bash
```

then

```
# Terminal 1
ros2 launch cartpole_mujoco cartpole_mujoco.launch.py model:=mesh   # oder ohne model:= für das Primitiv-Modell

# Terminal 2 (neues Terminal: erst source env.sh bzw. source install/setup.bash)
source ~/workspace/ROS2/cartpole_mujooco_ros2/install/setup.bash
ros2 run cartpole_mujoco_panel control_panel
```
