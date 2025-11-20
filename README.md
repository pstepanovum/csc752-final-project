# CSC752 Final Project - Plane Detection with Depth Maps

Real-time plane detection for the Toyota HSR robot using depth map analysis and RANSAC algorithm. Detected planes are visualized in RViz with color-coded markers, surface normals, and area measurements.

## Features

- **Real-time plane detection** from RGBD sensor point clouds
- **RANSAC-based algorithm** for robust plane fitting
- **Multiple plane detection** (configurable, default: 5 planes)
- **Rich RViz visualization**:
  - Color-coded plane surfaces
  - Surface normal vectors
  - Plane area labels
  - Camera overlay with plane markers
- **Configurable parameters** for different environments
- **Isaac Sim integration** with proper time synchronization

## Installation

1. **Add to catkin workspace:**
   ```bash
   cd ~/catkin_ws/src
   ln -s ~/hsr_robocanes_omniverse/src/csc752-final-project csc752-final-project
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

### Running with Isaac Sim (Recommended)

**Step 1:** Start the simulation infrastructure and RViz
```bash
csc752-final-project-start.py
```

This launches:
- HSR Isaac Localization system
- Isaac Sim world
- RViz with plane detection visualization

**Step 2:** In a new terminal, launch the plane detection node
```bash
roslaunch csc752-final-project plane_detection_with_isaac.launch
```

This starts the plane detection processing and publishes markers to RViz.

### Standalone Mode (Without Startup Script)

If you want to run without the startup script:

```bash
roslaunch csc752-final-project plane_detection.launch
```

This launches both the plane detection node and RViz.

## Launch Files

- **`plane_detection_with_isaac.launch`** - Use with Isaac Sim (after running startup script)
  - Sets `use_sim_time=true` for simulation
  - Launches only the plane detection node
  - No RViz (already launched by startup script)

- **`plane_detection.launch`** - Standalone mode
  - Launches plane detection node and RViz
  - Use when NOT using the startup script

## Parameters

Customize plane detection behavior:

```bash
roslaunch csc752-final-project plane_detection_with_isaac.launch \
  min_plane_points:=800 \
  ransac_threshold:=0.03 \
  max_planes:=3 \
  min_plane_area:=0.2 \
  downsample_factor:=2
```

**Available Parameters:**
- `min_plane_points` (default: 500) - Minimum points for valid plane
- `ransac_threshold` (default: 0.02) - RANSAC distance threshold in meters
- `max_planes` (default: 5) - Maximum number of planes to detect
- `min_plane_area` (default: 0.1) - Minimum plane area in m²
- `downsample_factor` (default: 4) - Point cloud downsampling (higher = faster)

## ROS Topics

### Subscribed
- `/hsrb/head_rgbd_sensor/depth_registered/rectified_points` - Point cloud input

### Published
- `/plane_markers` - Visualization markers for detected planes

## Visualization

In RViz you'll see:
- **Robot model** (semi-transparent HSR)
- **Map** (if available)
- **LaserScan** (base laser)
- **PointCloud** (RGBD depth data)
- **PlaneMarkers** (detected planes with colors, normals, labels)
- **Camera view** (RGB feed with plane overlay)

## Tuning Tips

**For large open spaces:**
```bash
max_planes:=3 min_plane_area:=0.5 ransac_threshold:=0.03
```

**For cluttered environments:**
```bash
max_planes:=8 min_plane_area:=0.05 min_plane_points:=300
```

**For better performance:**
```bash
downsample_factor:=8 max_planes:=3
```

## Troubleshooting

**No planes detected:**
- Check point cloud topic: `rostopic echo /hsrb/head_rgbd_sensor/depth_registered/rectified_points --noarr`
- Lower thresholds: `min_plane_points:=200 ransac_threshold:=0.03`

**Performance issues:**
- Increase downsampling: `downsample_factor:=8`
- Reduce max planes: `max_planes:=2`

**Planes flickering:**
- Increase stability: `ransac_threshold:=0.03 min_plane_points:=800`

## Project Structure

```
csc752-final-project/
├── scripts/
│   ├── plane_detection_node.py      # Main plane detection
│   └── depth_processor_node.py      # Depth image processor
├── launch/
│   ├── plane_detection_with_isaac.launch  # Use with Isaac Sim
│   └── plane_detection.launch             # Standalone mode
├── doc/                              # Documentation
├── plane_detection.rviz              # RViz configuration
├── csc752-final-project-start.py     # Startup script
└── README.md
```

## License

MIT License
