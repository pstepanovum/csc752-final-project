# HSR Plane Detection Package

This ROS package implements real-time plane detection for the Toyota HSR robot using depth map analysis and RANSAC algorithm. Detected planes are visualized in RViz with color-coded markers, surface normals, and area measurements.

## Features

- **Real-time plane detection** from RGBD sensor point clouds
- **RANSAC-based algorithm** for robust plane fitting
- **Multiple plane detection** (configurable, default: 5 planes)
- **Rich RViz visualization**:
  - Color-coded plane surfaces
  - Surface normal vectors
  - Plane area labels
  - Point cloud overlay
- **Configurable parameters** for different environments
- **Automatic filtering** of small/invalid planes

## Dependencies

- ROS Noetic
- Python 3.8+
- sensor_msgs
- geometry_msgs
- visualization_msgs
- tf2_ros
- cv_bridge
- pcl_ros
- numpy
- scikit-learn

## Installation

1. **Clone into your catkin workspace:**
   ```bash
   cd ~/catkin_ws/src
   # Already cloned if you're reading this
   ```

2. **Install Python dependencies:**
   ```bash
   pip3 install numpy scikit-learn opencv-python
   ```

3. **Build the package:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Quick Start

1. **Start Isaac Sim with HSR robot** (ensure RGBD sensor is publishing)

2. **Launch plane detection:**
   ```bash
   roslaunch hsr_plane_detection plane_detection.launch
   ```

   This will start:
   - Plane detection node
   - RViz with pre-configured visualization

### Launch File Parameters

You can customize the behavior using launch arguments:

```bash
roslaunch hsr_plane_detection plane_detection.launch \
  min_plane_points:=800 \
  ransac_threshold:=0.03 \
  max_planes:=3 \
  min_plane_area:=0.2 \
  downsample_factor:=2
```

**Parameters:**
- `min_plane_points` (default: 500): Minimum points required for a valid plane
- `ransac_threshold` (default: 0.02): RANSAC distance threshold in meters
- `max_planes` (default: 5): Maximum number of planes to detect
- `min_plane_area` (default: 0.1): Minimum plane area in m²
- `downsample_factor` (default: 4): Point cloud downsampling factor (higher = faster)
- `use_rviz` (default: true): Whether to launch RViz automatically

### Without RViz

To run only the detection node:

```bash
roslaunch hsr_plane_detection plane_detection.launch use_rviz:=false
```

### Manual RViz Launch

```bash
rosrun rviz rviz -d $(rospack find hsr_plane_detection)/rviz/plane_detection.rviz
```

## Topics

### Subscribed Topics

- `/hsrb/head_rgbd_sensor/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
  - Input point cloud from HSR's RGBD sensor

### Published Topics

- `/plane_markers` ([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html))
  - Visualization markers for RViz showing:
    - Plane surfaces (colored triangular meshes)
    - Plane labels with area measurements
    - Surface normal vectors (yellow arrows)

- `/detected_plane_cloud` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
  - Point cloud of detected plane inliers (future use)

## Algorithm Overview

The plane detection system uses a multi-stage RANSAC approach:

1. **Point Cloud Acquisition**: Subscribe to RGBD sensor point cloud
2. **Preprocessing**:
   - Remove NaN and infinite values
   - Filter points beyond 10m
   - Downsample for performance
3. **Iterative RANSAC Plane Fitting**:
   - Fit plane using RANSAC regression
   - Try multiple orientations (horizontal and vertical planes)
   - Extract inliers above threshold
4. **Plane Validation**:
   - Check minimum point count
   - Estimate and validate plane area
5. **Visualization**:
   - Create triangulated mesh for plane surface
   - Compute and display surface normals
   - Label with area measurements

## Visualization Guide

When you open RViz, you should see:

- **White/Gray Points**: Raw point cloud from RGBD sensor
- **Colored Surfaces**: Detected planes (red, green, blue, yellow, magenta, cyan)
- **Yellow Arrows**: Surface normal vectors (pointing outward from plane)
- **White Text Labels**: Plane number and area in m²
- **TF Frames**: Coordinate frames showing robot pose

### RViz Display Configuration

The pre-configured RViz setup includes:
- **Grid**: Reference ground plane
- **PointCloud2**: Raw sensor data
- **MarkerArray**: Detected planes and annotations
- **TF**: Coordinate frames
- **RobotModel**: HSR robot visualization

## Tuning Parameters

### For Large Open Spaces
```bash
roslaunch hsr_plane_detection plane_detection.launch \
  max_planes:=3 \
  min_plane_area:=0.5 \
  ransac_threshold:=0.03
```

### For Cluttered Environments
```bash
roslaunch hsr_plane_detection plane_detection.launch \
  max_planes:=8 \
  min_plane_area:=0.05 \
  ransac_threshold:=0.015 \
  min_plane_points:=300
```

### For Better Performance
```bash
roslaunch hsr_plane_detection plane_detection.launch \
  downsample_factor:=8 \
  max_planes:=3
```

### For Higher Accuracy
```bash
roslaunch hsr_plane_detection plane_detection.launch \
  downsample_factor:=2 \
  ransac_threshold:=0.01 \
  min_plane_points:=1000
```

## Troubleshooting

### No planes detected
- **Check point cloud**: Verify `/hsrb/head_rgbd_sensor/points` is publishing
  ```bash
  rostopic echo /hsrb/head_rgbd_sensor/points --noarr
  ```
- **Reduce thresholds**: Try lower `min_plane_points` or larger `ransac_threshold`
- **Check environment**: Ensure robot is viewing surfaces (walls, floor, tables)

### Poor performance / lag
- **Increase downsampling**: Set `downsample_factor:=8` or higher
- **Reduce max planes**: Set `max_planes:=2` or `max_planes:=3`
- **Check point cloud density**: Very dense clouds may need more downsampling

### Planes flickering
- **Increase RANSAC threshold**: Set `ransac_threshold:=0.03`
- **Increase minimum points**: Set `min_plane_points:=800`
- This usually happens with noisy sensor data

### Wrong planes detected
- **Increase min area**: Set `min_plane_area:=0.2` to filter small surfaces
- **Adjust threshold**: Fine-tune `ransac_threshold` for your environment

## Development

### Node Structure

```
plane_detection_node.py
├── PlaneDetectionNode
│   ├── __init__(): Initialize parameters and ROS communication
│   ├── point_cloud_callback(): Main processing pipeline
│   ├── pointcloud2_to_array(): Convert ROS message to numpy
│   ├── detect_planes(): Multi-plane RANSAC detection
│   ├── fit_plane_ransac(): Single plane fitting
│   ├── estimate_plane_area(): Compute plane dimensions
│   └── publish_plane_markers(): Create visualization markers
```

### Extending the Package

To add custom plane analysis:

1. Modify `detect_planes()` to extract additional features
2. Add new topics in `__init__()` for publishing results
3. Create custom marker types in `publish_plane_markers()`

### Alternative: Using Depth Images Directly

The package currently uses point clouds. To process depth images:

1. Subscribe to `/hsrb/head_rgbd_sensor/depth/image_rect_raw`
2. Convert depth image to point cloud using camera intrinsics
3. Apply the same RANSAC pipeline

## Performance Characteristics

- **Processing Rate**: ~10-30 Hz (depends on point cloud size and parameters)
- **Latency**: <100ms typical
- **Memory**: ~200MB for full resolution point clouds
- **CPU Usage**: 20-50% single core (with downsampling)

## References

- [RANSAC Algorithm](https://en.wikipedia.org/wiki/Random_sample_consensus)
- [ROS Visualization Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker)
- [Point Cloud Library](https://pointclouds.org/)

## License

MIT License

## Authors

CSC 752 Final Project Team

## Support

For issues or questions:
1. Check this README and troubleshooting section
2. Review ROS logs: `rosnode info plane_detection_node`
3. Verify sensor topics: `rostopic list | grep hsrb`
