# Plane Detection System - Quick Start Guide

This guide will help you set up and run the plane detection system for the HSR robot.

## Overview

The plane detection system analyzes depth map data from the HSR's RGBD sensor and detects planar surfaces in the environment using RANSAC algorithm. Detected planes are visualized in RViz with:
- Color-coded surfaces
- Surface normal vectors
- Area measurements

## Installation

### 1. Copy ROS Package to Your Catkin Workspace

```bash
# From the project directory
cp -r hsr_plane_detection ~/catkin_ws/src/
```

### 2. Install Dependencies

```bash
pip3 install numpy scikit-learn opencv-python
```

### 3. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running the System

### Option 1: Using Point Clouds (Recommended)

This is the faster and recommended approach:

```bash
# Start Isaac Sim with HSR robot first

# Launch plane detection
roslaunch hsr_plane_detection plane_detection.launch
```

### Option 2: Using Depth Images

This processes raw depth images and converts them to point clouds:

```bash
roslaunch hsr_plane_detection plane_detection_from_depth.launch
```

## What You'll See in RViz

- **White Points**: Raw point cloud from RGBD sensor
- **Colored Surfaces**: Detected planes (different colors for each plane)
- **Yellow Arrows**: Surface normal vectors
- **White Labels**: Plane ID and area in square meters

## Customizing Parameters

### For Different Environments

**Large open spaces** (detect only major surfaces):
```bash
roslaunch hsr_plane_detection plane_detection.launch \
  max_planes:=3 \
  min_plane_area:=0.5
```

**Cluttered environments** (detect more surfaces):
```bash
roslaunch hsr_plane_detection plane_detection.launch \
  max_planes:=8 \
  min_plane_area:=0.05 \
  min_plane_points:=300
```

**Performance mode** (faster processing):
```bash
roslaunch hsr_plane_detection plane_detection.launch \
  downsample_factor:=8 \
  max_planes:=3
```

### Key Parameters

- `max_planes`: Maximum number of planes to detect (default: 5)
- `min_plane_area`: Minimum surface area in m² (default: 0.1)
- `ransac_threshold`: RANSAC distance threshold in meters (default: 0.02)
- `min_plane_points`: Minimum points for valid plane (default: 500)
- `downsample_factor`: Point cloud downsampling (default: 4, higher = faster)

## Troubleshooting

### No planes detected
1. Check if point cloud is publishing:
   ```bash
   rostopic echo /hsrb/head_rgbd_sensor/points --noarr
   ```
2. Try lower thresholds:
   ```bash
   roslaunch hsr_plane_detection plane_detection.launch \
     min_plane_points:=200 \
     ransac_threshold:=0.03
   ```

### System is slow
1. Increase downsampling:
   ```bash
   roslaunch hsr_plane_detection plane_detection.launch \
     downsample_factor:=8
   ```

### Planes are flickering
1. Increase stability thresholds:
   ```bash
   roslaunch hsr_plane_detection plane_detection.launch \
     ransac_threshold:=0.03 \
     min_plane_points:=800
   ```

## ROS Topics

### Subscribed
- `/hsrb/head_rgbd_sensor/points` - Input point cloud
- `/hsrb/head_rgbd_sensor/depth/image_rect_raw` - Depth images (alternative)
- `/hsrb/head_rgbd_sensor/rgb/image_rect_color` - RGB images (for coloring)

### Published
- `/plane_markers` - Visualization markers for RViz
- `/detected_plane_cloud` - Point cloud of detected planes

## Detailed Documentation

For complete documentation, see:
- **Package README**: `~/catkin_ws/src/hsr_plane_detection/README.md`
- **Project Exploration**: `./CODEBASE_EXPLORATION.md`
- **ROS Architecture**: `./ROS_ARCHITECTURE.txt`

## Testing the System

### 1. Verify ROS is running
```bash
roscore &
```

### 2. Check available topics
```bash
rostopic list | grep hsrb
```

### 3. View a single point cloud message
```bash
rostopic echo /hsrb/head_rgbd_sensor/points -n 1
```

### 4. Monitor plane detection output
```bash
rostopic echo /plane_markers
```

### 5. Check node status
```bash
rosnode info /plane_detection_node
```

## Next Steps

1. **Start Isaac Sim** with the HSR robot
2. **Launch plane detection**: `roslaunch hsr_plane_detection plane_detection.launch`
3. **Move the robot** to view different surfaces
4. **Tune parameters** based on your environment
5. **Extend the system** by modifying the detection algorithms

## Support

For issues or questions, check:
1. Package README for detailed documentation
2. ROS logs: `rosnode info plane_detection_node`
3. Verify sensor topics are publishing

## Architecture

```
Isaac Sim (HSR Robot)
    ├── RGBD Sensor
    │   ├── Point Cloud → /hsrb/head_rgbd_sensor/points
    │   ├── Depth Image → /hsrb/head_rgbd_sensor/depth/image_rect_raw
    │   └── RGB Image   → /hsrb/head_rgbd_sensor/rgb/image_rect_color
    │
    ↓
Plane Detection System
    ├── depth_processor_node.py (optional)
    │   └── Converts depth images → point clouds
    │
    ├── plane_detection_node.py
    │   ├── RANSAC plane fitting
    │   ├── Multi-plane detection
    │   └── Validation & filtering
    │
    └── Visualization
        ├── RViz markers
        ├── Surface meshes
        └── Normal vectors
```

## Algorithm Flow

1. **Input**: Point cloud from RGBD sensor
2. **Preprocessing**: Remove invalid points, downsample
3. **RANSAC Loop**: For each plane (up to max_planes):
   - Fit plane to remaining points
   - Extract inliers
   - Validate size and point count
   - Remove inliers from pool
4. **Output**: Visualization markers for RViz

---

**Ready to start?** Run:
```bash
roslaunch hsr_plane_detection plane_detection.launch
```
