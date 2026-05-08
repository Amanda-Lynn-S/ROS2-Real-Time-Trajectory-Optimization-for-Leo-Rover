# 🚀 Lunar Rover Perception – Height-Based Costmap (ROS 2 Jazzy)

End-to-end perception MVP for lunar rover navigation.

```text
ZED Camera
   ↓
PointCloud2
   ↓
TF2 Transform → odom
   ↓
Height Grid
   ↓
OccupancyGrid
   ↓
Nav2 / RViz2
```

---

## 🖥 Requirements

* Ubuntu 22.04
* ROS 2 Jazzy
* ZED ROS2 Wrapper
* RViz2
* NVIDIA Jetson (Orin / Thor recommended)

Install ROS Jazzy desktop:

```bash
sudo apt install ros-jazzy-desktop
```

Install RViz if missing:

```bash
sudo apt install ros-jazzy-rviz2
```

---

## 🔧 Setup

### 1️⃣ Create Workspace

```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
git clone https://github.com/YOUR_USERNAME/rover_perception.git
```

### 2️⃣ Build

```bash
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Optional convenience:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/rover_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🎥 Running the Pipeline

Open **3 terminals**.

In every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rover_ws/install/setup.bash
```

---

### 🟢 Terminal 1 – Launch ZED

```bash
ros2 launch zed_wrapper zed_camera.launch.py
```

Verify cloud:

```bash
ros2 topic list | grep cloud
```

You should see:

```text
/zed/zed_node/point_cloud/cloud_registered
```

---

### 🔵 Terminal 2 – Transform Cloud to Odom

```bash
ros2 run rover_perception cloud_to_target_frame \
--ros-args \
-r /point_cloud/cloud_registered:=/zed/zed_node/point_cloud/cloud_registered
```

This transforms:

```text
zed_left_camera_frame → odom
```

Verify:

```bash
ros2 topic hz /cloud_in_target_frame
```

---

### 🟣 Terminal 3 – Height Costmap

```bash
ros2 run rover_perception height_costmap
```

Verify:

```bash
ros2 topic hz /height_costmap
```

---

## 👁 Visualizing in RViz2

Launch:

```bash
rviz2
```

Set:

* Fixed Frame → `odom`

Add displays:

* PointCloud2 → `/cloud_in_target_frame`
* Map → `/height_costmap`
* TF
* Grid (optional)

If cloud does not appear:

Change Reliability to **Best Effort** in the PointCloud2 display settings.

---

## 🧠 How It Works

### cloud_to_target_frame.py

* Subscribes to ZED `PointCloud2`
* Uses TF2 to transform into `odom`
* Publishes `/cloud_in_target_frame`

### height_costmap.py

* Subscribes to `/cloud_in_target_frame`
* Discretizes XY into grid
* Stores max Z per cell
* Applies height threshold
* Publishes `nav_msgs/OccupancyGrid`

---

## 📐 Frame Conventions

ZED optical frame:

```text
X right
Y down
Z forward
```

Target frame:

```text
odom
```

All costmap calculations are done in the target frame.

---

## 🧪 Debugging

Check cloud rate:

```bash
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom zed_left_camera_frame
```

Check topic info:

```bash
ros2 topic info /cloud_in_target_frame -v
ros2 topic info /height_costmap -v
```

---

## ✅ Current Status

* Real ZED cloud integrated
* TF transform validated
* RViz visualization working
* Height-based OccupancyGrid publishing

---

## 🔮 Next Steps

* Add slope filtering
* Add semantic layer
* Integrate with Nav2 costmap server
* Convert critical nodes to C++ if needed

---

**Author:** Lunar Rover Perception Lab
**ROS Version:** Jazzy
**Status:** MVP Working
