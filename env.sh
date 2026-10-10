#!/usr/bin/env bash
# Umgebung fuer dieses Repo. Mit "source env.sh" laden.
# ROS-Distribution bei Bedarf anpassen (hier: Lyrical Luth auf Ubuntu 26.04).
source /opt/ros/lyrical/setup.bash

# Der System-Python muss von CMake/colcon verwendet werden, nicht ein
# Python-Shim aus uv/pyenv.
export Python3_EXECUTABLE=/usr/bin/python3

alias cbuild='colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'

# Nach dem Bauen: source install/setup.bash
