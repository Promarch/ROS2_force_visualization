# Force Visualization ROS2 Package

## Overview
This ROS2 package provides a node for visualizing force measurements from a sensor in a specified target frame using visualization markers.

## Features
- Subscribes to force (wrench) messages from a sensor
- Transforms force vectors between different coordinate frames
- Publishes visualization markers representing force magnitude and direction
- Configurable source and target frames
- Color-coded force visualization based on magnitude

## Dependencies
- ROS2 (Humble or later recommended)
- Packages:
  - rclcpp
  - geometry_msgs
  - tf2_ros
  - tf2_msgs
  - tf2_geometry_msgs
  - visualization_msgs

## Building the Package
```bash
# Assuming you're in your ROS2 workspace
colcon build --packages-select force_visualization
```

## Launch
```bash
ros2 launch force_visualization force_visualizer.launch.py
```

### Launch Parameters
- `force_frame`: Frame in which forces are recorded (default: "sensor_body")
- `target_frame`: Frame in which forces are displayed (default: "tibia_body")
- `debug_mode`: Enable debug information output (default: false)

## Node Details
### Subscribed Topics
- `/ft_sensor/wrench_stamped` (geometry_msgs/WrenchStamped)

### Published Topics
- `/force_marker` (visualization_msgs/Marker)

## Visualization
The node creates an arrow marker where:
- Arrow length represents force magnitude
- Arrow color transitions from blue (low force) to red (high force)
- Arrow orientation indicates force direction in the target frame

## Troubleshooting
- Ensure TF2 transforms are correctly configured between source and target frames
- Check that force sensor messages are being published
- Verify coordinate frame names match your robot configuration

## License
[Insert appropriate license information]

## Author
[Insert author/maintainer information]
