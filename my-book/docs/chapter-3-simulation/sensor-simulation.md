---
sidebar_position: 4
---

# Sensor Simulation

Robots perceive their environment through sensors. This section covers simulating LiDAR, depth cameras, and IMUs in Gazebo, and visualizing their data in RViz2.

## Sensor Types in Robotics

### Proprioceptive Sensors (Internal State)

| Sensor | Measures | Example |
|--------|----------|---------|
| **Joint encoders** | Joint angles | Motor shaft position |
| **IMU** | Orientation, acceleration | Balance sensing |
| **Force/torque** | Contact forces | Foot pressure |

### Exteroceptive Sensors (External Environment)

| Sensor | Measures | Example |
|--------|----------|---------|
| **LiDAR** | Distance to surfaces | 3D point clouds |
| **Camera** | Light intensity | RGB images |
| **Depth camera** | Depth per pixel | RGB-D images |
| **Ultrasonic** | Distance | Obstacle detection |

## Sensor Data Flow

```mermaid
graph LR
    subgraph "Gazebo Simulation"
        W[World/Environment]
        L[LiDAR Plugin]
        C[Camera Plugin]
        I[IMU Plugin]
    end

    subgraph "ros_gz Bridge"
        B[gz_bridge]
    end

    subgraph "ROS 2"
        TL[/scan Topic]
        TC[/camera/image Topic]
        TI[/imu/data Topic]
        RV[RViz2]
    end

    W --> L
    W --> C
    W --> I
    L --> B
    C --> B
    I --> B
    B --> TL
    B --> TC
    B --> TI
    TL --> RV
    TC --> RV
    TI --> RV
```

## LiDAR Simulation in Gazebo

### LiDAR Sensor Plugin Configuration (SDF)

Add this to your robot model's SDF:

```xml
<!-- LiDAR sensor on robot head -->
<link name="lidar_link">
  <pose relative_to="head">0 0 0.15 0 0 0</pose>
  <sensor name="lidar" type="gpu_lidar">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>true</visualize>

    <lidar>
      <scan>
        <horizontal>
          <samples>640</samples>
          <resolution>1</resolution>
          <min_angle>-2.35619</min_angle>  <!-- -135 degrees -->
          <max_angle>2.35619</max_angle>   <!-- +135 degrees -->
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.26</min_angle>     <!-- -15 degrees -->
          <max_angle>0.26</max_angle>      <!-- +15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </lidar>

    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </sensor>
</link>
```

### LiDAR Scan Pattern

```
        ^ Forward
        |
   +----|----+
   |    |    |
   |  LIDAR  |    Horizontal FOV: 270°
   |    |    |    Vertical FOV: 30°
   +----+----+
       Robot

  Top View:
         /         \
        /   Scan    \
       /   Area      \
      /               \
     +-------X-------+
       Robot Head
```

### Bridge LiDAR Data to ROS 2

```bash
# Add to your launch file
ros2 run ros_gz_bridge parameter_bridge \
  /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
```

## Depth Camera (RGB-D) Simulation

### Depth Camera Plugin Configuration

```xml
<!-- RGB-D camera on robot head -->
<link name="camera_link">
  <pose relative_to="head">0.1 0 0.05 0 0 0</pose>
  <sensor name="rgbd_camera" type="rgbd_camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>

    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <depth_camera>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </depth_camera>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </sensor>
</link>
```

### Bridge Camera Data to ROS 2

```bash
# RGB image
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image[gz.msgs.Image

# Depth image
ros2 run ros_gz_bridge parameter_bridge \
  /camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image

# Camera info
ros2 run ros_gz_bridge parameter_bridge \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

## IMU Simulation

### IMU Plugin Configuration

```xml
<!-- IMU in robot torso -->
<link name="imu_link">
  <pose relative_to="torso">0 0 0 0 0 0</pose>
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>
  </sensor>
</link>
```

### Bridge IMU Data to ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU
```

## Sensor Noise Models

Realistic simulation requires sensor noise:

| Sensor | Noise Type | Typical Values |
|--------|-----------|----------------|
| LiDAR | Gaussian | σ = 0.01-0.03m |
| Camera | Gaussian | σ = 0.007 (depth) |
| IMU (gyro) | Gaussian + bias | σ = 0.0002 rad/s |
| IMU (accel) | Gaussian + bias | σ = 0.017 m/s² |

### Configuring Noise

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>       <!-- Bias -->
  <stddev>0.01</stddev>  <!-- Standard deviation -->
</noise>
```

## RViz2 Sensor Visualization Setup

Create an RViz2 config file to visualize all sensors:

```yaml
# config/sensors.rviz
Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Global Options:
    Fixed Frame: base_link
  Displays:
    # Robot model
    - Class: rviz_default_plugins/RobotModel
      Name: Robot
      Robot Description: robot_description

    # LiDAR point cloud
    - Class: rviz_default_plugins/PointCloud2
      Name: LiDAR
      Topic: /lidar/points
      Size (m): 0.02
      Color Transformer: Intensity
      Enabled: true

    # RGB Image
    - Class: rviz_default_plugins/Image
      Name: RGB Camera
      Topic: /camera/image
      Enabled: true

    # Depth Image
    - Class: rviz_default_plugins/Image
      Name: Depth Camera
      Topic: /camera/depth_image
      Enabled: true

    # TF frames
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
```

Launch RViz2 with config:
```bash
rviz2 -d config/sensors.rviz
```

## Reading Sensor Topics in Python

```python
#!/usr/bin/env python3
"""Read sensor data from ROS 2 topics."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu
import numpy as np


class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')

        # LiDAR subscriber
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points',
            self.lidar_callback, 10)

        # Camera subscriber
        self.camera_sub = self.create_subscription(
            Image, '/camera/image',
            self.camera_callback, 10)

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/imu',
            self.imu_callback, 10)

    def lidar_callback(self, msg):
        # PointCloud2 contains point data
        self.get_logger().info(
            f'LiDAR: {msg.width * msg.height} points, '
            f'frame: {msg.header.frame_id}')

    def camera_callback(self, msg):
        self.get_logger().info(
            f'Camera: {msg.width}x{msg.height}, '
            f'encoding: {msg.encoding}')

    def imu_callback(self, msg):
        # Extract orientation (quaternion)
        quat = msg.orientation
        # Extract angular velocity
        ang_vel = msg.angular_velocity
        # Extract linear acceleration
        lin_acc = msg.linear_acceleration

        self.get_logger().info(
            f'IMU: accel=({lin_acc.x:.2f}, {lin_acc.y:.2f}, {lin_acc.z:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Expected output:
```
[INFO] [sensor_reader]: LiDAR: 10240 points, frame: lidar_link
[INFO] [sensor_reader]: Camera: 640x480, encoding: rgb8
[INFO] [sensor_reader]: IMU: accel=(0.01, -0.02, 9.81)
```

## Summary

You've learned to simulate:
- **LiDAR**: 3D point clouds for mapping and navigation
- **Depth cameras**: RGB-D data for perception
- **IMU**: Orientation and motion sensing
- **Sensor noise**: Realistic uncertainty modeling

These simulated sensors provide the perceptual inputs your humanoid needs. In [Chapter 4: NVIDIA Isaac](/docs/chapter-4-isaac), you'll learn about GPU-accelerated simulation for AI training at scale.
