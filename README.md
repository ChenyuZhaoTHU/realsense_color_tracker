
# 🎯 RealSense Color Tracker (ROS 2)

A robust ROS 2 package for 3D object tracking using an Intel RealSense D435i camera. It detects objects based on HSV color thresholds and calculates their real-world 3D coordinates (X, Y, Z) using depth alignment.

![License](https://img.shields.io/badge/license-Apache%202.0-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-green)
![Python](https://img.shields.io/badge/Python-3.10-yellow)

## ✨ Features

* **3D Localization**: Calculates the precise X, Y, Z coordinates relative to the camera frame.
* **Depth Alignment**: Uses hardware-aligned depth-to-color streams for pixel-perfect accuracy.
* **Robust Filtering**:
    * Hardware spatial & temporal filters (via launch configuration).
    * Contour area and aspect ratio filtering to reject noise.
    * Depth hole filling and neighborhood search for zero-depth pixels.
* **Smoothing**: Implements a Moving Average filter to reduce coordinate jitter.
* **Visualization**: Real-time OpenCV window showing the bounding box, center point, and distance.

---

## 🛠️ Prerequisites

### Hardware
* Intel RealSense D435i (USB 3.0 connection required for High-Res mode).

### Software
* **OS**: Ubuntu 22.04 (Jammy Jellyfish).
* **ROS 2**: Humble Hawksbill.
* **Python Dependencies**:
    ```bash
    pip install opencv-python numpy imutils
    ```
    *(Note: Ensure `numpy<2.0` is installed to avoid conflict with ROS 2 cv_bridge)*

* **ROS Packages**:
    ```bash
    sudo apt install ros-humble-realsense2-camera
    ```

---

## 🚀 Installation

1.  **Clone into your workspace:**
    ```bash
    cd ~/ros2_ws/src
    # Copy the package folder here
    ```

2.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select realsense_color_tracker
    ```

3.  **Source the environment:**
    ```bash
    source install/setup.bash
    ```

---

## 🎨 Calibration (Crucial Step)

Before running the tracker, you **must** calibrate the HSV thresholds for your specific lighting environment and object color.

1.  Run the provided tuning script (save the code below as `hsv_tuner.py` in your workspace root if not present):
    ```bash
    python3 hsv_tuner.py
    ```
    *(See `scripts/hsv_tuner.py` or use the code provided in the project documentation).*

2.  **Tuning Steps**:
    * **Initialize**: Set L-H, L-S, L-V to 0 and U-H, U-S, U-V to max.
    * **Isolate Hue**: Adjust `H` sliders until only your object's color is visible.
    * **Refine**: Adjust `S` (Saturation) to remove white/glare, and `V` (Value) to remove shadows.
    * **Goal**: Make the object pure white and background pure black in the `mask` window.

3.  **Update Code**:
    Open `realsense_color_tracker/tracker_node.py` and update these lines with your values:
    ```python
    self.targetLower = (102, 168, 85) # Replace with your L-H, L-S, L-V
    self.targetUpper = (118, 255, 255) # Replace with your U-H, U-S, U-V
    ```

---

## ▶️ Usage

### Option 1: One-Line Launch (Recommended)
This launches both the RealSense driver (with filters enabled) and the tracker node.

```bash
ros2 launch realsense_color_tracker tracking.launch.py
````

### Option 2: Manual Launch

If you want to debug or run nodes separately.

**Terminal 1 (Driver):**

```bash
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    filters:=spatial,temporal,hole_filling \
    rgb_camera.profile:=640x480x30 \
    depth_module.profile:=640x480x30
```

**Terminal 2 (Tracker):**

```bash
ros2 run realsense_color_tracker tracker
```

-----

## 📡 ROS 2 Topics

### Subscribed

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Raw RGB stream. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth stream aligned to RGB viewpoint. |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Intrinsic parameters (fx, fy, ppx, ppy). |

### Published

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/target_pose` | `geometry_msgs/PoseStamped` | 3D position of the object in meters (Camera Frame). |

**Message Example:**

```yaml
header:
  frame_id: camera_color_optical_frame
pose:
  position:
    x: 0.15  # Right (meters)
    y: 0.39 # Forward (meters)
    z: -0.10  # Up (meters)
```

-----

## ❓ Troubleshooting

### 1\. `AttributeError: _ARRAY_API not found` (NumPy Error)

**Cause:** Incompatibility between NumPy 2.x and ROS 2 `cv_bridge`.
**Fix:** Downgrade NumPy to 1.x:

```bash
pip install "numpy<2.0"
```

### 2\. `Hardware Notification: Depth stream start failure`

**Cause:** USB 2.0 connection cannot handle high resolution (1280x720) bandwidth.
**Fix:**

  * Ensure you are using a USB 3.0 cable and port (Blue/Red port).
  * Flip the USB-C connector (sometimes helps with pin detection).
  * Or, lower the resolution in `run_all.launch.py` to `640x480`.

### 3\. Object lost at distance (\> 2m)

  * Ensure the **Aspect Ratio** check in `tracker_node.py` matches your object shape.
  * The code filters areas smaller than `100` pixels. If tracking small objects, ensure the resolution is set to 1280x720.

-----

## 📝 Coordinate System

  * **X**: Horizontal (Right is positive)
  * **Y**: Depth (Forward is positive)
  * **Z**: Vertical (Down is positive)
  * The origin `(0,0,0)` is the center of the RGB Camera lens.

<!-- end list -->

