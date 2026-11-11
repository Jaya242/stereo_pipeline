```markdown
# Stereo Pipeline – ROS 2 Autonomous Vision & Obstacle Avoidance

A ROS 2 (Jazzy) package built in Python and OpenCV for real-time mobile camera video ingestion over RTSP, multi-color/foreground object detection, 3D spatial coordinate estimation, and autonomous obstacle avoidance control for an RC car platform.

---

## Project Structure

```text
stereo_pipeline/
├── launch/
│   └── rtsp_stream.launch.py
├── resource/
│   └── stereo_pipeline
├── stereo_pipeline/
│   ├── __init__.py
│   ├── camera_publisher.py
│   ├── car_listener.py
│   ├── obstacle_controller.py
│   ├── rtsp_node.py
│   ├── rtsp_stream.py
│   ├── sensor_talker.py
│   ├── stereo_node.py
│   └── stereo_processor.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md

```

---

## System Architecture

The package consists of three primary vision-to-control pipeline nodes and two utility nodes for testing inter-node communication:

* **`rtsp_camera_node` (`rtsp_stream.py`)**: Connects to an external mobile camera RTSP stream (such as an iPhone camera feed) and publishes raw BGR video frames to `/camera/image_raw`.
* **`stereo_processor` (`stereo_processor.py`)**: Subscribes to `/camera/image_raw`, filters and detects target objects using OpenCV, estimates 3D spatial coordinates $(X, Y, Z)$, renders bounding boxes with coordinate overlays on `/camera/object_debug`, and publishes target poses to `/box_pose`.
* **`obstacle_controller` (`obstacle_controller.py`)**: Subscribes to `/box_pose` and translates target depth and lateral offset into real-time velocity and steering commands on `/cmd_vel` alongside proximity alerts on `/obstacle_status`.
* **`sensor_talker` (`sensor_talker.py`) & `car_listener` (`car_listener.py`)**: Standalone publisher/subscriber utility nodes for testing real-time ROS 2 topic communication across `/sensor_distance`.

---

## Prerequisites

* **Operating System**: Ubuntu 24.04 LTS
* **ROS 2 Distribution**: ROS 2 Jazzy Jalisco
* **Python & System Dependencies**:
* `rclpy`
* `sensor_msgs`
* `geometry_msgs`
* `std_msgs`
* `cv_bridge`
* `opencv-python` (`cv2`)
* `numpy`



---

## Installation & Build

1. **Navigate to your ROS 2 workspace `src` directory and clone the repository:**
```bash
cd ~/ros2_ws/src
git clone [https://github.com/Jaya242/stereo_pipeline.git](https://github.com/Jaya242/stereo_pipeline.git)

```


2. **Build the package using `colcon`:**
```bash
cd ~/ros2_ws
colcon build --packages-select stereo_pipeline --symlink-install

```


3. **Source the workspace overlay:**
```bash
source install/setup.bash

```



---

## Running the Pipeline

Open separate terminal windows/tabs for each node (make sure to run `source ~/ros2_ws/install/setup.bash` in each terminal):

1. **Launch the RTSP Camera Stream Node:**
```bash
ros2 launch stereo_pipeline rtsp_stream.launch.py

```


2. **Run the Stereo Object Detection & Tracking Node:**
```bash
ros2 run stereo_pipeline stereo_processor

```


3. **Run the Obstacle Avoidance Controller Node:**
```bash
ros2 run stereo_pipeline obstacle_controller

```


4. **Inspect Generated Velocity Commands (Optional):**
```bash
ros2 topic echo /cmd_vel

```


5. **Run Publisher/Subscriber Test Nodes (Optional):**
```bash
# Terminal A
ros2 run stereo_pipeline sensor_talker

# Terminal B
ros2 run stereo_pipeline car_listener

```



---

## ROS 2 Topics

| Topic | Message Type | Publisher | Description |
| --- | --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | `rtsp_camera_node` | Raw BGR video frames ingested from the RTSP camera feed |
| `/camera/object_debug` | `sensor_msgs/Image` | `stereo_processor` | Annotated video stream showing bounding boxes and $(X, Y, Z)$ overlays |
| `/box_pose` | `geometry_msgs/PoseStamped` | `stereo_processor` | Estimated 3D spatial position of the detected target in `camera_link` |
| `/cmd_vel` | `geometry_msgs/Twist` | `obstacle_controller` | Linear and angular velocity commands for RC car motor control |
| `/obstacle_status` | `std_msgs/String` | `obstacle_controller` | Real-time proximity alerts and navigation status messages |
| `/sensor_distance` | `std_msgs/Float32` | `sensor_talker` | Simulated distance readings for pub/sub latency testing |

---

## Distributed Testing Across Machines

To run publisher and subscriber nodes across separate laptops or onboard hardware over the same Wi-Fi network, set an identical `ROS_DOMAIN_ID` on every machine before launching:

```bash
export ROS_DOMAIN_ID=42

```

---

## Future Improvements

* Replace fixed disparity estimation with calibrated dual-camera stereo block matching (`cv2.StereoSGBM`).
* Perform checkerboard intrinsic calibration (`cv2.calibrateCamera`) to eliminate lens distortion.
* Integrate Kalman filtering or SORT for multi-frame object ID persistence and trajectory smoothing.

```

### Add & Push to Your Repository

```bash
cd ~/ros2_ws/src/stereo_pipeline
cat << 'EOF' > README.md
# Paste the markdown content above here, or edit directly with: nano README.md
EOF
git add README.md
git commit -m "Add comprehensive README.md"
git push

```
