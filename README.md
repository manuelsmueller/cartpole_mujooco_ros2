# cartpole_mujooco_ros2

A getting-started project for **MuJoCo with ROS 2**, built around the classic cart-pole (inverted pendulum on a cart).

The physics runs in MuJoCo, while ROS 2 stays in the loop as the middleware. The simulation, the PID controller and a small control panel are separate ROS 2 nodes that talk over plain topics. There is no `ros2_control` and no Gazebo.

Two models are included:

| Model | Description | Masses |
|-------|-------------|--------|
| `primitive` | Box and cylinder primitives, rail travel ±1.8 m | cart 1.0 kg, pole 0.1 kg |
| `mesh` | STL parts from a CAD design (tongue-and-groove cart, ball weight on the pole), scaled 10×, rail travel ±0.85 m | rail 7.04 kg, cart 1.87 kg, pole 1.34 kg |

## Contents

- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Build](#build)
- [Run](#run)
- [Architecture](#architecture)
- [Controller and parameters](#controller-and-parameters)
- [Models](#models)
- [Troubleshooting](#troubleshooting)
- [Lessons learned](#lessons-learned)
- [Known limitations](#known-limitations)
- [Repository structure](#repository-structure)
- [Background](#background)
- [License](#license)

## Prerequisites

- Ubuntu 26.04
- ROS 2 Lyrical Luth (`ros-lyrical-desktop`)
- `colcon` (`python3-colcon-common-extensions`)
- `python3-tk` for the control panel
- [`uv`](https://docs.astral.sh/uv/) for the Python environment
- System Python 3.14 (`/usr/bin/python3`)

MuJoCo comes from the PyPI wheel, which bundles `libmujoco`. You do not need a separate MuJoCo installation or `MUJOCO_PATH`. A `cp314` wheel exists (tested with MuJoCo 3.12).

## Setup

### 1. Python environment

Create the virtual environment **from the system Python**, with access to the system site packages. This is what makes `rclpy`, `colcon` and `setuptools` from `/opt/ros/lyrical` visible inside the venv.

```bash
cd ~/workspace/ROS2/cartpole_mujooco_ros2
source /opt/ros/lyrical/setup.bash
uv venv --python /usr/bin/python3 --system-site-packages --no-python-downloads .venv
source .venv/bin/activate
uv pip install mujoco
```

- `--python /usr/bin/python3` and `--no-python-downloads` keep uv from using its own Python shim.
- Packages installed with `pip install --user` (`~/.local`) are **not** visible inside a venv, so install `mujoco` here.

### 2. `env.sh`

`env.sh` prepares every terminal: ROS, the venv, the workspace overlay and the `cbuild` alias.

```bash
source /opt/ros/lyrical/setup.bash
export Python3_EXECUTABLE=/usr/bin/python3
_here="$(dirname "${BASH_SOURCE[0]}")"
[ -f "$_here/.venv/bin/activate" ] && source "$_here/.venv/bin/activate"
[ -f "$_here/install/setup.bash" ] && source "$_here/install/setup.bash"
alias cbuild='python3 -m colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
```

- **Source** it, never execute it: `source env.sh`. A subshell would discard the environment.
- The order matters: ROS, then the venv, then `install/`.
- `install/setup.bash` only exists after the first build. Source `env.sh` again after each build so it gets picked up.

## Build

```bash
cd ~/workspace/ROS2/cartpole_mujooco_ros2
source env.sh
cbuild --packages-select cartpole_mujoco cartpole_mujoco_panel
source env.sh        # re-source: picks up install/setup.bash
```

`cbuild` runs `python3 -m colcon build`. That matters: the entry-point scripts get the shebang of the interpreter that ran the build. Running plain `colcon build` bakes `/usr/bin/python3` into the scripts, which does not see the venv's `mujoco`. Check it after building:

```bash
head -1 install/cartpole_mujoco/lib/cartpole_mujoco/simulation_node
# expected: #!/home/<user>/.../cartpole_mujooco_ros2/.venv/bin/python3
```

## Run

Terminal 1, simulation and controller:

```bash
cd ~/workspace/ROS2/cartpole_mujooco_ros2
source env.sh
ros2 launch cartpole_mujoco cartpole_mujoco.launch.py                 # primitive model
# ros2 launch cartpole_mujoco cartpole_mujoco.launch.py model:=mesh     # mesh model
# ros2 launch cartpole_mujoco cartpole_mujoco.launch.py viewer:=false   # headless
```

Terminal 2, control panel:

```bash
cd ~/workspace/ROS2/cartpole_mujooco_ros2
source env.sh
ros2 run cartpole_mujoco_panel control_panel
```

In the panel:

- **Start** enables the PID controller.
- **Reset** resets the simulation to an initial pole deflection of 0.1 rad and re-enables the controller.
- The gains are loaded from the controller shortly after the panel opens. Edit them and press **Apply parameters** to change them live. **Reload values from controller** reads them again.

Without the panel:

```bash
ros2 topic pub --once /cartpole/pid_enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /cartpole/reset std_msgs/msg/Empty "{}"
ros2 param set /cartpole_pid_controller kp 80.0
```

The pole starts deflected and falls over until you press **Start**. This is intended, so you can see the controller catch it.

## Architecture

```
  control_panel ──pid_enable / reset──▶ pid_controller_node ──effort_cmd──▶ simulation_node
                                               ▲                                   │
                                               └────────── /joint_states ◀─────────┘
```

| Node | Role |
|------|------|
| `simulation_node` | Holds `MjModel`/`MjData`, steps the physics from a timer, opens the passive MuJoCo viewer, publishes `/joint_states` |
| `pid_controller_node` | Node name `cartpole_pid_controller`. Reads `/joint_states`, publishes `/cartpole/effort_cmd` |
| `control_panel` | Tkinter GUI for start, reset and gains |

| Topic | Type | Direction |
|-------|------|-----------|
| `/joint_states` | `sensor_msgs/JointState` | simulation → controller, panel (stamp = simulation time) |
| `/cartpole/effort_cmd` | `std_msgs/Float64` | controller → simulation (force on the cart, N) |
| `/cartpole/pid_enable` | `std_msgs/Bool` | panel → controller |
| `/cartpole/reset` | `std_msgs/Empty` | panel → simulation and controller (the controller clears its integrators) |

Gains are changed through the standard ROS 2 parameter service (`set_parameters`).

Joint names are `slider_to_cart` (cart position) and `cart_to_pole` (pole angle). Both models use the same names and interfaces, so the controller and panel work with either model unchanged.

## Controller and parameters

The controller is a MISO PID: pole angle and rate plus cart position and rate in, one force out.

```
effort = -(kp·e_φ + ki·∫e_φ + kd·(-φ̇)) + (kp_cart·e_x + ki_cart·∫e_x + kd_cart·(-ẋ))
```

The output is clamped to `effort_limit`. The integrator time step comes from the `/joint_states` stamp (simulation time), so the controller is independent of wall-clock jitter. The controller always publishes a real number, `0.0` when disabled, never `NaN`. See [Lessons learned](#lessons-learned) for why.

Gains live in `src/cartpole_mujoco/config/pid_<model>.yaml`:

| Parameter | primitive | mesh |
|-----------|-----------|------|
| `kp` | 77.3 | -100.0 |
| `ki` | 0.0 | 0.0 |
| `kd` | 20.6 | -3.0 |
| `kp_cart` | -6.3 | -10.0 |
| `ki_cart` | -2.5 | 0.0 |
| `kd_cart` | -10.3 | -3.0 |
| `effort_limit` | 60 N | 50 N |

(The remaining parameters `i_clamp` and `i_clamp_cart` limit the integral terms.)

**PID signs are model-specific.** The mesh model uses a different axis convention (rail along Y, pole tips about X), so the signs are the opposite of the primitive model. Never carry gains from one model to another without verifying them.

The primitive gains also worked in the Gazebo version of this project (same masses). The mesh values started from the Gazebo set and were tuned in the control panel. Treat them as starting points for your own tuning.

## Models

Both models are native MJCF files in `src/cartpole_mujoco/cartpole_mujoco/model/`.

### Primitive

Rail (box, fixed at z = 1.0 m), cart (box, slide joint along X, ±1.8 m), pole (1 m cylinder on a hinge about Y). Hanging the rail at 1 m keeps the pole clear of the ground even at 180°.

### Mesh

Three STL files (`CartPole-Rail.stl`, `CartPole-cart.stl`, `CartPole-pole.stl`) in `model/meshes/`, referenced with `meshdir="meshes"` and `scale="0.01 0.01 0.01"`. Mass, centre of mass and inertia come from a `trimesh` analysis of the scaled meshes and are written into explicit `<inertial>` blocks. That analysis is independent of the simulator and was reused 1:1 from the earlier Gazebo version.

The CAD design is 10× enlarged on purpose. At the original size (rail 206 mm, a few grams) the dynamics run too fast to observe or debug, and the engine's tolerances are tuned for metre/kilogram scales.

MuJoCo uses the **convex hull** of a mesh for collisions, so the groove in the cart cannot act as a contact surface. The motion is guided by the slide joint instead, and rail/cart contact is excluded explicitly.

## Troubleshooting

Check the environment:

```bash
which python3        # should be .../cartpole_mujooco_ros2/.venv/bin/python3
python3 -c "import mujoco, rclpy; print(mujoco.__version__)"
```

| Symptom | Likely cause and fix |
|---------|----------------------|
| `ModuleNotFoundError: mujoco` on `ros2 launch` | The entry-point shebang points to `/usr/bin/python3`. Build with `cbuild` (`python3 -m colcon build`) with the venv active, then check `head -1 …/simulation_node` |
| `ignoring unknown package 'cartpole_mujoco'` | `colcon` found no packages. Both package directories must sit directly under `src/`. Check with `colcon list` |
| `Error opening file 'meshes/CartPole-cart.stl'` | The meshes were not installed. `setup.py` needs its own `glob` entry in `data_files` for `model/meshes/*.stl` |
| CMake picks a wrong Python | A uv/pyenv shim shadows the system Python. Make sure `Python3_EXECUTABLE=/usr/bin/python3` is set (done by `env.sh` / `cbuild`) |
| `rclpy` not found inside the venv | The venv was created without `--system-site-packages`, or ROS was not sourced before activating it |
| No MuJoCo window | Start without `viewer:=false`. The viewer needs a display |
| Build problems with `setuptools` | Try `uv pip install "setuptools<80"` in the venv and rebuild (untested on Lyrical) |
| `cbuild` not found | The alias only exists in shells that sourced `env.sh` |
| Cart does not move under force | Diagnose with `data.ncon` and the contact list (see Lessons learned) |

A clean rebuild: `rm -rf build install log`, then `cbuild …` again.

## Lessons learned

**MuJoCo**

1. **MuJoCo checks every body pair for collisions, including jointed ones.** Overlapping cart and rail geometry produced a permanent contact force and the cart looked "stuck", although the commanded force arrived correctly (verified with an isolated `data.ctrl[0]` test without ROS). Excluding jointed pairs with `<contact><exclude>` fixed it, and `data.ncon` at rest revealed it immediately.
2. **An `<exclude>` covers only the named pair, not transitive neighbours.** After excluding rail↔cart and cart↔pole, rail↔pole remained and touched at large deflections.
3. **The ground is a collision body, too.** A long hanging pole can reach below the floor at 180° if the rail is mounted low.
4. **Test the whole range of motion, not just the rest pose.** Sweeping several pole angles and slider positions found contacts that a rest-pose test hid.
5. **MuJoCo is headless by default.** A window needs `mujoco.viewer.launch_passive(model, data)` and a regular `viewer.sync()` in the stepping loop.
6. **Loading a URDF with `MjModel.from_xml_path` is a quick sanity check** of structure, joint names and masses. `ros2_control` blocks and Gazebo plugins are simply ignored. It is not a replacement for a native MJCF once you need actuators or MuJoCo-specific features.
7. **Gains transfer between simulators for equal masses, not between different mass distributions.** The mass distribution mattered more than the simulator.

**ROS 2 and Python tooling**

8. **Check wheel availability with `--dry-run` before an architecture decision.** For the very new Python 3.14 a `cp314` MuJoCo wheel already existed, so `mujoco` and `rclpy` run in one interpreter and no process bridge is needed.
9. **`setup.py` does not copy subfolders automatically.** Every `data_files` subfolder needs an explicit glob, otherwise the files are missing under `install/share`.
10. **Entry-point shebangs follow the interpreter that ran the build.** With a venv, build with `python3 -m colcon build`.
11. **Keep uv's Python shim away from CMake and colcon** by pinning `Python3_EXECUTABLE=/usr/bin/python3`.
12. **A permanent `NaN` command can freeze a joint** (observed in the Gazebo predecessor). The controller therefore always publishes a real number.

**Modelling from CAD**

13. **Bounding boxes lie for real CAD geometry.** The shaft of the pole is 4 mm, the 10 mm width came from the ball at its tip, and the cart does not rest flat on the rail because a groove surrounds a narrower rib. Real cross sections (`trimesh.section`) or a rendered view show this, bounding-box numbers do not.
14. **Compute mass and inertia from the watertight mesh** (`mesh.density`, `mesh.mass`, `mesh.moment_inertia`) instead of using hand formulas.

## Known limitations

- **Mesh model, rail ↔ pole:** the rail sits only 5 cm above the ground and rail↔pole collisions are not excluded. If the pole falls without the controller running, it lands on the rail or the ground. With the controller active this does not occur. Both cases are accepted deliberately.
- **Convex-hull collisions:** the groove and tongue of the cart/rail connection do not act as contacts (see [Models](#models)).
- Verified setup: Ubuntu 26.04, ROS 2 Lyrical Luth, Python 3.14, MuJoCo 3.12. Other combinations are untested.

## Repository structure

```
cartpole_mujooco_ros2/
├── env.sh                  # environment: ROS, venv, install overlay, cbuild alias
├── pyproject.toml          # uv project file
├── LICENSE
├── README.md
└── src/
    ├── cartpole_mujoco/              # simulation, PID controller, models, launch, config
    │   ├── package.xml
    │   ├── setup.py / setup.cfg
    │   ├── resource/cartpole_mujoco
    │   ├── launch/cartpole_mujoco.launch.py
    │   ├── config/
    │   │   ├── pid_primitive.yaml
    │   │   └── pid_mesh.yaml
    │   └── cartpole_mujoco/
    │       ├── __init__.py
    │       ├── simulation_node.py
    │       ├── pid_controller_node.py
    │       └── model/
    │           ├── cartpole_primitive.xml
    │           ├── cartpole_mesh.xml
    │           └── meshes/     # CartPole-Rail.stl, CartPole-cart.stl, CartPole-pole.stl
    └── cartpole_mujoco_panel/        # Tkinter control panel
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── resource/cartpole_mujoco_panel
        └── cartpole_mujoco_panel/
            ├── __init__.py
            └── control_panel.py
```

`build/`, `install/`, `log/` and `.venv/` are generated and listed in `.gitignore`.

## Background

This repository is the MuJoCo port of an earlier ROS 2 Lyrical + Gazebo Jetty project with `ros2_control` (primitive model first, then the mesh model). The interfaces (joint names, effort command, enable and reset topics) were kept so the controller and panel carried over. Only the simulation backend changed.

### Dynamics

Cart mass $M$, pole mass $m$, pole length $L$ with $l = L/2$, pole angle $\theta$ from the upright position, force $u$ on the cart:

```
(M + m)·ẍ + m·l·θ̈·cosθ − m·l·θ̇²·sinθ = u
J·θ̈ + m·l·ẍ·cosθ − m·g·l·sinθ = 0,          J = m·L²/12 + m·l²
```

The pole joint is unactuated, so the force `u` reaches the pole only through the coupling term `m·l·ẍ·cosθ`. The term `m·g·l·sinθ` is destabilising and is why the upright pose is unstable.

Linearised around the upright pose (x, ẋ, θ, θ̇), the primitive model (M = 1.0 kg, m = 0.1 kg, L = 1.0 m) gives approximately `a32 ≈ 15.8`, `b2 ≈ −1.46`, `a12 ≈ −0.72`, `b1 ≈ 0.98`.

### Gains

The primitive gains started from an LQR design with an extra integral state on the cart position (solved numerically with `scipy.linalg.solve_continuous_are`). The values in use are more conservative than the LQR optimum and were confirmed empirically on the nonlinear system.

## License

Apache-2.0, see [LICENSE](LICENSE).