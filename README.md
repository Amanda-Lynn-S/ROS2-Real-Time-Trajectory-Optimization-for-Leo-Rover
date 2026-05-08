# Rover Trajectory Optimization via Sequential Convex Programming (SCP)

This project implements a **2D rover trajectory optimization framework** using **Sequential Convex Programming (SCP)** with **collision avoidance via Signed Distance Fields (SDFs)**.  
It follows the methodology of "GuSTO: Guaranteed Sequential Trajectory Optimization via Sequential Convex Programming" (Bonalli et al., ICRA 2019) for safe, smooth, and dynamically consistent path planning. 

The system is implemented as a **ROS2 Jazzy** application split across three packages: **`zed-ros2-wrapper package`** (publishing real-time point cloud from camera), **`rover_perception`** (point cloud processing and occupancy grid generation), and **`rover_scp`** (map bridging and trajectory optimization). Each major component runs as a ROS2 node communicating via published and subscribed topics. The perception stack produces a live occupancy grid, the map node bridges it into SDF+metadata, the planning node consumes those to run SCP, and the resulting trajectory is streamed to the rover via `/cmd_vel`.

The codebase is the real-time ROS2 application extension of the following project: https://github.com/Amanda-Lynn-S/ROS2-Trajectory-Optimization-for-Leo-Rover
The perception stack used can be found here: https://github.com/SujayCh07/rover_perception/tree/main

---

# Project Overview

## zed-ros2-wrapper package

| Module | ROS2 Role | Description |
|--------|-----------|-------------|
| **`zed_camera.launch.py`** | **Publisher** → `/zed/zed_node/point_cloud/cloud_registered` | Launches the ZED ROS 2 wrapper, publishing real-time point cloud data from the camera. This is remapped to `/point_cloud/cloud_registered` for compatibility with the perception pipeline. |

## rover_perception package

| Module | ROS2 Role | Description |
|--------|-----------|-------------|
| **`mock_cloud_pub.py`** | **Publisher** → `/point_cloud/cloud_registered` | Publishes a synthetic PointCloud2 in the ZED optical frame for testing, simulating a flat floor with a raised bump obstacle. |
| **`cloud_to_target_frame.py`** | **Subscriber** ← `/point_cloud/cloud_registered` · **Publisher** → `/cloud_in_target_frame` | Transforms the incoming point cloud into the target frame using TF2. |
| **`height_costmap.py`** | **Subscriber** ← `/cloud_in_target_frame` · **Publisher** → `/height_costmap` | Discretizes the point cloud into a 2D grid, estimates floor height via a low percentile of Z values, and marks cells as occupied (cost=100) if their max relative height exceeds a threshold. Publishes a `nav_msgs/OccupancyGrid`. |

**Note:** **`mock_cloud_pub.py`** is only used for testing the framework; otherwise, the ZED ROS 2 wrapper needs to be launched to get a livestream from the camera and perform the correct frame transformation to get the occupancy grid. 

## rover_scp package

`rover_scp` is now a **mixed Python + C++ ROS 2 package** as an extension to the reference codebase that relies on Python + ROS 2 only. The map bridge remains Python, but the real-time planning stack was converted to C++ and is built with `CMakeLists.txt`.

| Module | ROS2 Role | Description |
|--------|-----------|-------------|
| **`map_publisher_node.py`** | **Subscriber** ← `/height_costmap` · **Publisher** → `/map_meta`, `/sdf_grid` | Python node. Subscribes to the live occupancy grid from `rover_perception`. On first message received, binarizes the grid (occupied=1, free/unknown=0), computes the SDF, and publishes map metadata and the SDF array. Republishes every 2 seconds so late-starting subscribers can receive the map. Start/End positions are set via ROS2 parameters `start_x`, `start_y`, `end_x`, `end_y` (grid-cell coordinates). |
| **`rover_model.cpp` / `rover_model.hpp`** | *(no ROS2 role — C++ library)* | C++ replacement for `rover_model.py`. Defines the rover continuous-time dynamics, analytical Jacobians, Taylor-series linearization, and ZOH discretization. Used internally by the C++ `Rover` class. |
| **`rover.cpp` / `rover.hpp`** | *(no ROS2 role — C++ library)* | C++ replacement for `Rover.py`. Defines the `Rover` class, including model parameters, state/control constraints, SDF conversion/interpolation, SDF gradients, and dynamics interfaces. Constructed directly by the C++ planning node once map data has been received. |
| **`scp.cpp` / `scp.hpp`** | *(no ROS2 role — C++ library)* | C++ replacement for `SCP.py`. Implements the Sequential Convex Programming planner. The convex subproblem is formulated directly as a sparse QP and solved with **OsqpEigen**, replacing the previous Python/CVXPY workflow. |
| **`rover_pp_node.cpp`** | **Subscriber** ← `/map_meta`, `/sdf_grid` · **Publisher** → `/scp_trajectory_states`, `/scp_trajectory_controls`, `/cmd_vel` | C++ replacement for `rover_pp_node.py`. Waits to receive both map topics, instantiates `Rover` and `SCP`, runs the optimizer in a background thread, publishes the full optimized state/control trajectories once, and streams `/cmd_vel` at the SCP timestep rate (`dt`). |
| **`CMakeLists.txt`** | *(build configuration)* | Builds `rover_scp` as a mixed `ament_cmake` + `ament_cmake_python` package. Installs the Python map publisher, builds the C++ `rover_scp_lib`, and builds the `rover_pp_node` executable. |

**Note:** The old Python files `rover_pp_node.py`, `Rover.py`, `SCP.py`, and `rover_model.py` are no longer the active real-time planner implementation. They have been replaced by the C++ files above.

---

# ROS2 Topic Summary

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | `zed_camera.launch.py` → remap | Raw point cloud in ZED optical frame. |
| `/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | remap → `cloud_to_target_frame` | Remapped input topic expected by `cloud_to_target_frame`. |
| `/cloud_in_target_frame` | `sensor_msgs/PointCloud2` | `cloud_to_target_frame` → `height_costmap` | Point cloud transformed into target frame. |
| `/height_costmap` | `nav_msgs/OccupancyGrid` | `height_costmap` → `map_publisher_node` | Live occupancy grid: -1=unknown, 0=free, 100=occupied. |
| `/map_meta` | `std_msgs/String` | `map_publisher_node` → `rover_pp_node` | JSON string containing map width, height, cell size, and start/end grid coordinates. |
| `/sdf_grid` | `std_msgs/Float32MultiArray` | `map_publisher_node` → `rover_pp_node` | Row-major SDF array (shape H×W) with grid dimensions encoded in the message layout. Values in grid-cell units. |
| `/scp_trajectory_states` | `std_msgs/Float32MultiArray` | `rover_pp_node` → *(logger / downstream)* | Full optimized state trajectory, shape N×5 `[px, py, θ, v, ω]`. Published once after SCP convergence. |
| `/scp_trajectory_controls` | `std_msgs/Float32MultiArray` | `rover_pp_node` → *(logger / downstream)* | Full optimized control trajectory, shape (N−1)×2 `[T_R, T_L]`. Published once after SCP convergence. |
| `/cmd_vel` | `geometry_msgs/Twist` | `rover_pp_node` → Leo Rover | Velocity commands streamed at rate `dt`. `linear.x = v`, `angular.z = ω`. |

---

# Data Flow

```
[ZED Camera / mock_cloud_pub]
  └──▶ /point_cloud/cloud_registered
          │
          ▼
  cloud_to_target_frame
  └──▶ /cloud_in_target_frame
          │
          ▼
  height_costmap
  └──▶ /height_costmap  (nav_msgs/OccupancyGrid)
          │
          ▼
  map_publisher_node
  (binarize → compute SDF → publish)
  ├──▶ /map_meta   (JSON metadata)  ─┐
  └──▶ /sdf_grid   (SDF array)     	      ──┤
                                      				     ▼
                              			         rover_pp_node.cpp
                              		   (builds C++ Rover + SCP, runs optimizer)
                                      				     │
                  ┌──────────────┼───────────────────┐
                  ▼                   				     ▼ 								 ▼
     /scp_trajectory_states  		    /scp_trajectory_controls  					    /cmd_vel
       (full state traj)           	                  (full control traj)    						  (Leo Rover)

```

---

# Requirements 

- Ubuntu 22.04
- ROS 2 Jazzy
- Requires the Stereolabs ZED SDK installed on the target Ubuntu machine [Install from: https://www.stereolabs.com/developers/]
- C++ build dependencies for `rover_scp`: `Eigen3`, `OsqpEigen`, and `nlohmann_json`
- Python dependency for `map_publisher_node.py`: `scipy`

---

# Setting Up the Workspace

Source the ROS2 installation:
```bash
source /opt/ros/jazzy/setup.bash
```

Create a directory:
```bash
mkdir -p ~/ros2_ws
```

Upload the project:
```bash
cd ~/ros2_ws
git clone https://github.com/Amanda-Lynn-S/ROS2-Real-Time-Trajectory-Optimization-for-Leo-Rover.git
```

Upload the ZED ROS 2 wrapper:
```bash
cd ~/ros2_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
```

Install missing dependencies for packages in `src`:
```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

Build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Source the workspace:
```bash
source install/local_setup.bash
```

**Build note:** Because `rover_scp` now contains C++ code, the package is built through `CMakeLists.txt` rather than only through the old Python `setup.py` flow.

---

# Running the System

Launch each node in a separate terminal (source the workspace first in each terminal):

```bash
ros2 launch zed_wrapper zed_camera.launch.py
```
```bash
ros2 run rover_perception cloud_to_target_frame \
--ros-args \
-r /point_cloud/cloud_registered:=/zed/zed_node/point_cloud/cloud_registered
```
```bash
ros2 run rover_perception height_costmap
```
`map_publisher_node` publishes immediately on receiving the first `/height_costmap` message and republishes every 2 seconds (ros arguments - start and end points - can be chosen as desired):
```bash
ros2 run rover_scp map_publisher_node --ros-args -p start_x:=10 -p start_y:=10 -p end_x:=90 -p end_y:=90
```
`rover_pp_node` is the C++ planning executable. It idles until it receives both `/map_meta` and `/sdf_grid`, then runs SCP automatically:
```bash
ros2 run rover_scp rover_pp_node
```
