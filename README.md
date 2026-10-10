# cartpole_mujooco_ros2
This project aims to provide a getting started with mujoco and ROS2. It uses the well-known case of a cartpole.



# prerequisits

uv
uv init
uv venv --python /usr/bin/python3 --system-site-packages --no-python-downloads .venv
uv pip install mujoco



# execution

```
cd ~/workspace/ROS2/cartpole_mujooco_ros2
source env.sh
cbuild --packages-select cartpole_mujoco cartpole_mujoco_panel
source env.sh  # source install/setup.bash included in env.sh
```

then

```
# Terminal 1
source env.sh
ros2 launch cartpole_mujoco cartpole_mujoco.launch.py model:=mesh   # oder ohne model:= für das Primitiv-Modell

# Terminal 2 (neues Terminal: erst source env.sh bzw. source install/setup.bash)
source env.sh
ros2 run cartpole_mujoco_panel control_panel
```

# Troubleshooting

## test virtual env and mujoco

```
which python3        # should be .../cartpole_mujooco_ros2/.venv/bin/python3
python3 -c "import mujoco, rclpy; print(mujoco.__version__)"
```