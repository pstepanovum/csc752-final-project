# CSC 752 Final Project - Quick Reference Guide

## Project Summary
- **Name:** Plane Detection & Depth Map Processing for HSR Robot
- **Simulator:** NVIDIA Isaac Sim
- **Robot:** Toyota HSR (Human Support Robot)
- **Framework:** ROS Noetic + Python
- **Focus:** Depth camera integration, plane detection, visualization

## Repository Status
- **Current Files:** 2 (startup script + instructions)
- **Missing:** ROS package structure, nodes, launch files, configurations
- **Status:** Template phase - needs implementation

## Key Resources in This Directory
1. **CODEBASE_EXPLORATION.md** - Complete project analysis (main document)
2. **PROJECT_ARCHITECTURE.md** - ROS package structure and organization
3. **ROS_ARCHITECTURE.txt** - Detailed communication architecture
4. **INSTRUCTIONS.md** - Comprehensive ROS command reference
5. **QUICK_REFERENCE.md** - This file

## What Needs to Be Created

### Critical Files
```
~/catkin_ws/src/hsr_isaac_localization/
├── CMakeLists.txt                          ← Build config
├── package.xml                             ← Package metadata
├── launch/
│   └── robocanes_hsr_correction_sim.launch ← Launches ROS nodes
├── rviz/
│   └── hsr_pf_localization_sim.rviz       ← Visualization config
├── scripts/
│   ├── plane_detection_node.py            ← Main algorithm
│   ├── depth_processor.py                 ← Depth processing
│   └── localization_node.py               ← Localization (optional)
├── config/
│   └── params.yaml                        ← Node parameters
└── hsr_localization_world.py              ← Isaac Sim world
```

## Expected Sensor Topics

### Input (From Isaac Sim)
```
/hsrb/head_rgbd_sensor/depth/image_rect_raw    ← Depth image (uint16)
/hsrb/head_rgbd_sensor/depth/camera_info       ← Camera calibration
/hsrb/head_rgbd_sensor/rgb/image_rect_color    ← RGB image
/hsrb/head_rgbd_sensor/points                  ← Point cloud
/hsrb/odom                                     ← Odometry
/hsrb/base_scan                                ← Laser scan
/tf, /tf_static                                ← Transforms
```

### Output (From Your Nodes)
```
/detected_planes      ← Detected plane positions/orientations
/plane_markers        ← Visualization markers for RViz
/depth_map_processed  ← Processed depth image
/odometry/filtered    ← Filtered odometry (if localization)
```

## Core Node Structure

```
depth_processor_node
├── Subscribe to:
│   ├── /hsrb/head_rgbd_sensor/depth/image_rect_raw
│   ├── /hsrb/head_rgbd_sensor/depth/camera_info
│   └── /tf
├── Process:
│   ├── Decode depth values (uint16 → float)
│   ├── Convert to point cloud
│   ├── Filter outliers
│   ├── Detect planes (RANSAC)
│   └── Generate visualization markers
└── Publish:
    ├── /detected_planes
    └── /plane_markers
```

## RViz Display Configuration
Expected displays in RViz config:
- RobotModel (shows HSR URDF)
- TF (shows coordinate frames)
- PointCloud2 (depth camera points)
- Marker/MarkerArray (detected planes)
- LaserScan (base laser)
- Odometry (trajectories)
- Image (RGB/depth feeds)

Fixed frame: `/map` or `/odom`

## Key ROS Packages Needed
```
rospy/roscpp              # ROS core
sensor_msgs              # Image, PointCloud2, CameraInfo
geometry_msgs            # Transform, Pose, Twist
tf2/tf2_ros              # Coordinate transformations
pcl_ros                  # Point Cloud Library integration
visualization_msgs       # Markers for RViz
robot_localization       # EKF/UKF (if localization needed)
std_msgs                 # Standard message types
```

## Development Workflow
1. Create ROS package in `~/catkin_ws/src/`
2. Write CMakeLists.txt and package.xml
3. Implement depth_processor_node
4. Create launch file
5. Design RViz configuration
6. Define Isaac Sim world
7. Test with Isaac Sim running
8. Debug with ROS tools (rostopic, rqt_graph, etc.)
9. Commit and push to GitHub

## Important Coordinate Frames
```
/map                 (Global reference)
  └── /odom          (Odometry frame)
      └── /base_link (Robot body)
          └── /head_tilt_link
              ├── /head_rgbd_sensor_rgb_optical_frame
              └── /head_rgbd_sensor_depth_optical_frame
```

## Depth Image Processing Steps
1. **Decode:** Convert uint16 raw values to float depth (meters)
2. **Validate:** Remove NaN, inf, and out-of-range values
3. **Project:** Convert 2D image to 3D point cloud (use camera_info)
4. **Filter:** Remove outliers, downsample for performance
5. **Detect:** RANSAC plane segmentation
6. **Output:** Create visualization markers and topic messages

## RANSAC Plane Detection Parameters
```yaml
distance_threshold: 0.02     # 2 cm - distance to plane
max_iterations: 1000         # iterations to try
probability: 0.99            # confidence level
model_type: SACMODEL_PLANE   # plane fitting model
method_type: SAC_RANSAC      # algorithm type
```

## Typical ROS Commands for Debugging
```bash
# Monitor topics
rostopic list
rostopic echo /hsrb/odom -n 1
rostopic hz /hsrb/head_rgbd_sensor/depth/image_rect_raw

# Check nodes
rosnode list
rosnode info /depth_processor_node

# View computation graph
rqt_graph

# Watch point clouds
rosrun image_view image_view image:=/hsrb/head_rgbd_sensor/rgb/image_rect_color

# Check transforms
tf_echo /base_link /head_rgbd_sensor_depth_optical_frame

# View logs
rqt_console
```

## Key Libraries & APIs
- **PCL (Point Cloud Library):** `pcl_ros`, plane detection, clustering
- **OpenCV:** Image processing, depth handling
- **NumPy:** Mathematical operations
- **tf2:** Transform operations
- **rospy:** ROS Python client library

## Testing Checklist
- [ ] Isaac Sim world loads successfully
- [ ] ROS topics publishing from simulator
- [ ] Launch file starts nodes without errors
- [ ] Depth processor subscribes to correct topics
- [ ] Point cloud generation working
- [ ] Plane detection finds planes
- [ ] RViz displays all expected visualizations
- [ ] Markers show detected planes correctly
- [ ] No frame_id or timestamp errors in logs

## Common Issues & Solutions

**"Package not found"**
- Ensure you've run: `cd ~/catkin_ws && catkin_make`
- Source setup file: `source ~/catkin_ws/devel/setup.bash`

**"No messages on topic"**
- Check Isaac Sim is running and publishing
- Verify topic name is correct: `rostopic list`
- Check frame_id matches in code

**"Transform error"**
- Isaac Sim must publish /tf
- Ensure cameras are properly attached in URDF
- Check frames exist: `tf_echo /parent_frame /child_frame`

**"RViz displays nothing"**
- Check Fixed Frame is set correctly
- Verify topics exist: `rostopic list`
- Check topic message types match display types

## File Locations Reference
```
Repository:     /home/user/csc752-final-project/
ROS Package:    ~/catkin_ws/src/hsr_isaac_localization/
Launch Files:   ~/catkin_ws/src/hsr_isaac_localization/launch/
RViz Config:    ~/catkin_ws/src/hsr_isaac_localization/rviz/
Scripts:        ~/catkin_ws/src/hsr_isaac_localization/scripts/
Config:         ~/catkin_ws/src/hsr_isaac_localization/config/
Isaac World:    ~/catkin_ws/src/hsr_isaac_localization/
```

## Learning Resources
- **ROS Wiki:** http://wiki.ros.org/
- **PCL Tutorials:** http://pointclouds.org/documentation/tutorials/
- **OpenCV Docs:** https://docs.opencv.org/
- **RANSAC Plane Detection:** Search "PCL RANSAC plane segmentation tutorial"

## Next Steps
1. Read **CODEBASE_EXPLORATION.md** for complete details
2. Refer to **PROJECT_ARCHITECTURE.md** for file structure
3. Check **ROS_ARCHITECTURE.txt** for communication patterns
4. Start with creating `package.xml` and `CMakeLists.txt`
5. Implement depth processor step by step
6. Test with simple point cloud visualization first
7. Add plane detection once basic flow works
8. Fine-tune visualization in RViz

## Quick Start After Setup
```bash
# 1. Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# 2. Create package
cd src
catkin_create_pkg hsr_isaac_localization rospy sensor_msgs geometry_msgs tf2

# 3. Build
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 4. Create directories
mkdir -p ~/catkin_ws/src/hsr_isaac_localization/{launch,rviz,scripts,config}

# 5. Start implementing!
```

