# Isaac ROS AprilTag RealSense Bringup

Standalone bringup for:

- `realsense2_camera`
- `isaac_ros_image_proc::RectifyNode`
- `isaac_ros_apriltag::AprilTagNode`
- `foxglove_bridge`

## Default AprilTag Parameters

- `size:=0.08`
- `max_tags:=24`
- `tag_family:=tag36h11`
- `backends:=CUDA`

## Performance Defaults

- RealSense color profile defaults to `1280,720,15` (reduced FPS for lower processing load)

## Run

```bash
ros2 launch isaac_ros_apriltag_realsense_bringup apriltag_realsense_foxglove.launch.py
```

## Key Topics

- Input image: `/camera/camera/color/image_raw`
- Input camera info: `/camera/camera/color/camera_info`
- Detections: `/tag_detections`
- TF output from detections: `/tf`

## Foxglove

Foxglove bridge starts by default at `ws://<robot-ip>:8765`.

Use this layout file for the AprilTag bringup:

- `/ssd/ros2_ws/foxglove/apriltag_realsense_layout.json`
