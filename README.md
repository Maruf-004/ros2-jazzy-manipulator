# ROS 2 Jazzy Manipulator

A complete ROS 2 **Jazzy** workspace for a 3‑DOF “Arduinobot” manipulator with: URDF/Xacro description, Gazebo simulation, MoveIt 2 planning, ros2\_control controllers, and C++/Python example nodes.

> **Tested on:** Ubuntu 24.04.2 LTS + ROS 2 *Jazzy*
> **Author:** Md. Maruf
> **GitHub:** [https://github.com/Maruf-004](https://github.com/Maruf-004)

---

## Table of Contents

* [Repository Structure](#repository-structure)
* [Packages Overview](#packages-overview)
* [Prerequisites](#prerequisites)
* [Quick Start](#quick-start)
* [Launch Files](#launch-files)
* [Controllers & Topics](#controllers--topics)
* [Build Tips](#build-tips)
* [Troubleshooting](#troubleshooting)
* [Contributing](#contributing)
* [License](#license)

---

## Repository Structure

```
arduinobot_ws/                      # Workspace root (this repo)
├── src/
│   ├── arduinobot_description/     # URDF/Xacro, meshes, RViz, Gazebo launch
│   │   ├── meshes/                 # STL assets (tracked, consider Git LFS)
│   │   ├── rviz/                   # RViz configs (e.g., display.rviz)
│   │   ├── urdf/                   # *.xacro files
│   │   └── launch/                 # display.launch.py, gazebo.launch.py
│   ├── arduinobot_moveit/          # MoveIt 2 config & launch
│   │   ├── config/                 # srdf, kinematics, ompl_planning.yaml
│   │   └── launch/                 # moveit.launch.py
│   ├── arduinobot_controller/      # ros2_control controllers & launch
│   │   ├── config/                 # arduinobot_controllers.yaml
│   │   └── launch/                 # controller.launch.py
│   ├── arduinobot_cpp_examples/    # C++ example nodes
│   └── arduinobot_py_examples/     # Python example nodes
├── media/                          # Screenshots/GIFs for README (add yours)
├── .gitignore
├── .gitattributes                  # Only if using Git LFS
├── LICENSE
└── README.md
```

> You can keep your workspace directory named `arduinobot_ws` locally; the GitHub repo name can be `ros2-jazzy-manipulator`.

---

## Packages Overview

* **arduinobot\_description** – URDF/Xacro robot, STL meshes, RViz setup, Gazebo integration.
* **arduinobot\_moveit** – SRDF, joint limits, kinematics, OMPL planning configuration, MoveIt launch.
* **arduinobot\_controller** – `ros2_control` controller YAML and a launch file to bring up controllers.
* **arduinobot\_cpp\_examples / arduinobot\_py\_examples** – Minimal publisher/subscriber & parameter demos.

---

## Prerequisites

Install ROS 2 *Jazzy* and common tools:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs \
                 ros-jazzy-moveit ros-jazzy-moveit-planners-ompl \
                 python3-colcon-common-extensions
```

Optional – **Git LFS** for large meshes (>50 MB):

```bash
sudo apt install git-lfs
git lfs install
```

---

## Quick Start

Clone, build, and launch RViz visualization:

```bash
# 1) Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2) Build the workspace
colcon build --symlink-install

# 3) Source the overlay
source install/setup.bash

# 4) Show the robot in RViz
ros2 launch arduinobot_description display.launch.py
```

Gazebo simulation:

```bash
ros2 launch arduinobot_description gazebo.launch.py
```

MoveIt 2 planning (OMPL):

```bash
ros2 launch arduinobot_moveit moveit.launch.py
```

Controllers bring-up:

```bash
ros2 launch arduinobot_controller controller.launch.py
```

---

## Launch Files

* `arduinobot_description/launch/display.launch.py` – Loads URDF to the parameter server and opens RViz.
* `arduinobot_description/launch/gazebo.launch.py` – Spawns robot into Gazebo and loads controllers if configured.
* `arduinobot_moveit/launch/moveit.launch.py` – Starts MoveIt 2 with OMPL (ensure `config/ompl_planning.yaml` is included).
* `arduinobot_controller/launch/controller.launch.py` – Starts `controller_manager` with `arduinobot_controllers.yaml`.

---

## Controllers & Topics

Controller YAML example lives in `arduinobot_controller/config/arduinobot_controllers.yaml`.

Check controllers/hardware interfaces:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Example **gripper** command (Float64MultiArray):

```bash
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray \
'{layout: {dim: [], data_offset: 0}, data: [-1.0]}' --once
```

---

## Build Tips

* Always source ROS 2 before building: `source /opt/ros/jazzy/setup.bash`.
* After a successful build: `source install/setup.bash` (consider adding to your `~/.bashrc`).
* Ensure your `CMakeLists.txt` for description package installs assets:

```cmake
install(
  DIRECTORY meshes urdf launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
```

---

## Troubleshooting

* **MoveIt: "No planning library loaded"**
  Install OMPL planners and ensure your `ompl_planning.yaml` is passed to `move_group` in the MoveIt launch file.

* **Meshes not found in RViz/Gazebo**
  Confirm the `install(...)` rule above, rebuild with `colcon build`, and re‑source your overlay.

* **RViz has trouble saving config**
  Try saving to a simple path (e.g., `~/arduinobot_ws/src/arduinobot_description/rviz/display.rviz`) and ensure you have write permissions.

* **Controllers not available**
  Wait for `controller_manager` services; verify your hardware/sim interfaces are loaded and the controller YAML is referenced in the launch.

---

## Contributing

PRs and issues are welcome! Please propose improvements to URDF, MoveIt config, control tuning, or examples.

---

## License

This project is released under the **MIT License** (see `LICENSE`).

---

## Screenshots / Demo

Add images to `media/` and reference them like:

```md
![RViz](media/rviz_display.png)
![Gazebo](media/gazebo_spawn.png)
```

---

## `.gitignore` (copy into the repo root)

```gitignore
# ROS 2 / colcon
build/
install/
log/
.colcon*

# Python
__pycache__/
*.pyc
*.pyo
*.pyd

# CMake/Cpp
CMakeFiles/
CMakeCache.txt
cmake_install.cmake
Makefile
*.o
*.so

# Editors/IDE
.vscode/
*.code-workspace
.idea/

# Misc
*.swp
*.swo
.DS_Store
Thumbs.db
```

> Note: RViz configs, meshes, and YAML files **are intentionally not ignored** so they remain versioned.

---

## `.gitattributes` (only if using Git LFS)

```gitattributes
# Track large 3D assets with Git LFS
*.stl filter=lfs diff=lfs merge=lfs -text
*.STL filter=lfs diff=lfs merge=lfs -text
*.dae filter=lfs diff=lfs merge=lfs -text
*.ply filter=lfs diff=lfs merge=lfs -text
```

**Initialize LFS** before the first commit:

```bash
git lfs install
git lfs track "*.stl" "*.STL" "*.dae" "*.ply"
```
