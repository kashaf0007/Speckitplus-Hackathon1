---
sidebar_position: 4
---

# Isaac ROS

**Isaac ROS** is a collection of GPU-accelerated ROS 2 packages optimized for NVIDIA Jetson and discrete GPUs. This section covers setting up Isaac ROS for perception tasks.

## Isaac ROS Packages Overview

Isaac ROS provides drop-in replacements for common ROS 2 perception packages:

| Standard ROS 2 | Isaac ROS | Speedup |
|---------------|-----------|---------|
| image_proc | isaac_ros_image_proc | ~10x |
| stereo_image_proc | isaac_ros_stereo_image_proc | ~15x |
| ORB-SLAM | cuVSLAM | ~20x |
| YOLO (CPU) | isaac_ros_yolov8 | ~50x |
| AprilTag | isaac_ros_apriltag | ~10x |

## Isaac ROS Perception Stack

```mermaid
graph TB
    subgraph "Sensors"
        C1[RGB Camera]
        C2[Stereo Camera]
        D[Depth Sensor]
    end

    subgraph "Isaac ROS Packages"
        IP[isaac_ros_image_proc]
        VS[cuVSLAM]
        DNN[isaac_ros_dnn_inference]
        AT[isaac_ros_apriltag]
    end

    subgraph "Outputs"
        TF[/tf]
        OD[/detections]
        MAP[/map]
    end

    C1 --> IP
    C2 --> VS
    D --> VS
    IP --> DNN
    IP --> AT
    VS --> TF
    VS --> MAP
    DNN --> OD
    AT --> TF
```

## Isaac ROS Docker Setup

The recommended way to run Isaac ROS is via Docker:

```bash
# Clone Isaac ROS workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Build the container
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh ~/isaac_ros_ws

# Inside the container, build packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

Expected output:
```
Summary: X packages finished [Xm Xs]
```

### Verify Isaac ROS Installation

```bash
# Check for CUDA availability
nvidia-smi

# Run Isaac ROS demo
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

## cuVSLAM for Visual SLAM

**cuVSLAM** is NVIDIA's GPU-accelerated Visual SLAM implementation.

### cuVSLAM Launch Configuration

```python
# launch/visual_slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            parameters=[{
                'enable_image_denoising': True,
                'rectified_images': True,
                'enable_imu_fusion': True,
                'gyro_noise_density': 0.000244,
                'gyro_random_walk': 0.000019,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,
                'calibration_frequency': 200.0,
                'image_jitter_threshold_ms': 34.0,
            }],
            remappings=[
                ('visual_slam/image_0', '/camera/left/image_rect'),
                ('visual_slam/camera_info_0', '/camera/left/camera_info'),
                ('visual_slam/image_1', '/camera/right/image_rect'),
                ('visual_slam/camera_info_1', '/camera/right/camera_info'),
                ('visual_slam/imu', '/imu/data'),
            ]
        ),
    ])
```

### cuVSLAM Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/visual_slam/tracking/odometry` | Odometry | Robot pose estimate |
| `/visual_slam/vis/observations_cloud` | PointCloud2 | Tracked features |
| `/visual_slam/status` | DiagnosticStatus | SLAM status |

:::tip Gazebo/ORB-SLAM3 Fallback
If you don't have Isaac ROS:
```bash
# Use ORB-SLAM3 instead
sudo apt install ros-humble-orb-slam3

ros2 launch orb_slam3_ros2 stereo-inertial.launch.py \
  vocabulary:=/path/to/ORBvoc.txt \
  settings:=/path/to/stereo_config.yaml
```
:::

## DNN Inference Nodes

Isaac ROS provides TensorRT-accelerated inference nodes:

```python
# launch/object_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Image preprocessing
        Node(
            package='isaac_ros_dnn_image_encoder',
            executable='dnn_image_encoder',
            parameters=[{
                'network_image_width': 640,
                'network_image_height': 480,
                'image_mean': [0.485, 0.456, 0.406],
                'image_stddev': [0.229, 0.224, 0.225],
            }],
            remappings=[
                ('image', '/camera/image'),
                ('encoded_tensor', '/tensor')
            ]
        ),

        # TensorRT inference
        Node(
            package='isaac_ros_tensor_rt',
            executable='tensor_rt_node',
            parameters=[{
                'model_file_path': '/models/yolov8n.onnx',
                'engine_file_path': '/models/yolov8n.engine',
                'input_tensor_names': ['images'],
                'input_binding_names': ['images'],
                'output_tensor_names': ['output0'],
                'output_binding_names': ['output0'],
                'verbose': False,
            }],
            remappings=[
                ('tensor_pub', '/tensor'),
                ('tensor_sub', '/inference_output')
            ]
        ),

        # Detection decoder
        Node(
            package='isaac_ros_yolov8',
            executable='yolov8_decoder_node',
            parameters=[{
                'confidence_threshold': 0.5,
                'nms_threshold': 0.45,
            }],
            remappings=[
                ('tensor', '/inference_output'),
                ('detections', '/detections')
            ]
        ),
    ])
```

## DNN Inference Node Example

Reading detection results:

```python
#!/usr/bin/env python3
"""Process Isaac ROS detections."""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10)

    def detection_callback(self, msg):
        for detection in msg.detections:
            # Get bounding box
            bbox = detection.bbox
            cx, cy = bbox.center.position.x, bbox.center.position.y
            w, h = bbox.size_x, bbox.size_y

            # Get class and confidence
            if detection.results:
                result = detection.results[0]
                class_id = result.hypothesis.class_id
                score = result.hypothesis.score

                self.get_logger().info(
                    f'Detected {class_id} ({score:.2f}) at '
                    f'({cx:.0f}, {cy:.0f}) size {w:.0f}x{h:.0f}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Expected output:
```
[INFO] [detection_subscriber]: Detected person (0.87) at (320, 240) size 150x300
[INFO] [detection_subscriber]: Detected cup (0.72) at (450, 380) size 50x60
```

## Perception Pipeline Launch File

Complete launch file combining visual SLAM and object detection:

```python
# launch/perception_pipeline.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Visual SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('my_humanoid'), '/launch/visual_slam.launch.py'
            ])
        ),

        # Object detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('my_humanoid'), '/launch/object_detection.launch.py'
            ])
        ),

        # TF publisher for detected objects
        Node(
            package='my_humanoid',
            executable='detection_to_tf',
            name='detection_to_tf',
        ),
    ])
```

## Isaac ROS vs Standard ROS 2 Packages

| Feature | Standard ROS 2 | Isaac ROS |
|---------|---------------|-----------|
| Execution | CPU | GPU (CUDA) |
| Inference | OpenCV DNN | TensorRT |
| SLAM | ORB-SLAM, RTAB-Map | cuVSLAM |
| Image processing | cv_bridge | CUDA kernels |
| Hardware | Any | NVIDIA GPU |
| Latency | 50-200ms | 5-20ms |

## Summary

You've learned:
- **Isaac ROS** provides GPU-accelerated ROS 2 packages
- **cuVSLAM** enables real-time visual SLAM
- **TensorRT inference** accelerates object detection
- **Docker workflow** simplifies deployment

In the next section, we'll configure **Nav2 for bipedal humanoids**.
