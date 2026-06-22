# zed_sensor_viewer

# ZED Sensor & Positional Tracking RViz Visualizer Guide

This guide describes how to run and customize the new ZED SDK positional tracking and sensor visualization package in RViz.

> [!NOTE]
> All visual and spatial calculations are executed natively in the ROS **REP-103** coordinate system (Right-Handed, Z-up, X-forward) by specifying `sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD` in the ZED initialization parameters. This completely eliminates manual frame transformations and orientation mismatch.

---

## 3D Spatial & Motion Mappings in RViz

The following visualization components are rendered inside RViz based on real-time data from the ZED camera:

| Component | RViz Display Type | Visual Mapping / Design | Color | Description / Logic |
| :--- | :--- | :--- | :--- | :--- |
| **Ground Grid** | `MarkerArray` (`LINE_LIST`) | Crosses at $1\text{m}$ spacing | **Light Green/Yellow** | Replicates the background reference grid shown in the ZED Sensor Viewer screenshot. |
| **Camera Chassis** | `Marker` (`CUBE`) | $3.3 \times 17.5 \times 3.0\text{ cm}$ box | **Dark Matte Grey** | Renders the actual dimensions of the ZED camera chassis. |
| **Lenses** | `Marker` (`SPHERE`) | Two $1.8\text{ cm}$ spheres | **Cyan Blue** | Placed at the front-left and front-right camera coordinates to indicate camera face direction. |
| **Facing Vector** | `Marker` (`ARROW`) | $35\text{ cm}$ vector forward from camera | **Purple** | Clearly defines the camera's sightline vector. |
| **Trajectory Path** | `Path` (`geometry_msgs/PoseStamped`) | Cumulative trail of motion | **Vibrant Orange** | Plots the historical path of camera movements (cached up to 2000 points). |
| **Raw Accelerometer** | `Marker` (`ARROW`) | Measured IMU acceleration | **Red** | Vector representing the raw accelerometer force (includes gravity component). |
| **Gravity Vector** | `Marker` (`ARROW`) | Gravity reference | **Green** | Arrow pointing straight down ($-\text{Z}$ axis) with length matching $9.81\text{ m/s}^2$. |
| **Dynamic Acceleration** | `Marker` (`ARROW`) | Dynamic motion acceleration | **Cyan** | True physical acceleration vector with gravity subtracted ($\vec{a}_{world} - \begin{bmatrix}0 \\ 0 \\ 9.81\end{bmatrix}$). |

---

## Build and Launch Instructions

### 1. Build the Node
Open a terminal in your ROS 2 workspace (`/home/uday/ros2_ws`) and rebuild the package:
```bash
colcon build --symlink-install --packages-select zed_rviz
```

### 2. Run the ZED Sensor RViz Node
Source the workspace setup files and run the compiled C++ executable:
```bash
source install/setup.bash
ros2 run zed_rviz zed_sensor_rviz
```

### 3. Launch RViz with the Pre-configured Theme
In a separate terminal, source your workspace and launch RViz using the preset layout:
```bash
source install/setup.bash
rviz2 -d src/zed_rviz/zed_sensor_viewer.rviz
```

---

## Troubleshooting & Tips

* **Stationary Noise Filter:** A noise filter threshold of $0.15\text{ m/s}^2$ is applied to the **Dynamic Acceleration** vector. This keeps the vector hidden when the camera is static to prevent sensor jitter from displaying random arrow bursts.
* **Camera Model Compatibility:** The node programmatically checks if the connected ZED model supports an IMU sensor. If you use a camera without an IMU (such as the original ZED v1), pose tracking and paths will still function perfectly, but acceleration arrows will remain hidden.
* **Fixed Frame:** Ensure your RViz **Fixed Frame** is set to `odom` (this is configured automatically if you launch using the provided `.rviz` file).
