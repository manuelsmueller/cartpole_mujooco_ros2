source /opt/ros/lyrical/setup.bash
export Python3_EXECUTABLE=/usr/bin/python3
_here="$(dirname "${BASH_SOURCE[0]}")"
[ -f "$_here/.venv/bin/activate" ] && source "$_here/.venv/bin/activate"
[ -f "$_here/install/setup.bash" ] && source "$_here/install/setup.bash"
alias cbuild='python3 -m colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
