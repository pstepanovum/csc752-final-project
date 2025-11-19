#!/usr/bin/env python3
"""
Depth Image Processor Node for Toyota HSR Robot
Converts depth images to point clouds and detects planes
Alternative to using pre-computed point clouds
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs import Header


class DepthProcessorNode:
    def __init__(self):
        rospy.init_node('depth_processor_node', anonymous=False)

        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_rgb = None

        # Camera intrinsics (will be updated from camera_info)
        self.fx = 525.0  # Focal length x (default)
        self.fy = 525.0  # Focal length y (default)
        self.cx = 320.0  # Principal point x (default)
        self.cy = 240.0  # Principal point y (default)

        # Parameters
        self.depth_scale = rospy.get_param('~depth_scale', 0.001)  # Convert to meters
        self.max_depth = rospy.get_param('~max_depth', 10.0)  # Maximum depth in meters
        self.min_depth = rospy.get_param('~min_depth', 0.1)   # Minimum depth in meters
        self.downsample = rospy.get_param('~downsample', 2)   # Downsample factor

        rospy.loginfo("Depth Processor Node Initialized")
        rospy.loginfo(f"Parameters: depth_scale={self.depth_scale}, "
                     f"max_depth={self.max_depth}, downsample={self.downsample}")

        # Publishers
        self.cloud_pub = rospy.Publisher('/depth_point_cloud', PointCloud2, queue_size=10)

        # Subscribers
        self.depth_sub = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/depth/image_rect_raw',
            Image,
            self.depth_callback,
            queue_size=1
        )

        self.rgb_sub = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/rgb/image_rect_color',
            Image,
            self.rgb_callback,
            queue_size=1
        )

        self.camera_info_sub = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/depth/camera_info',
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        rospy.loginfo("Waiting for depth images and camera info...")

    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera_info topic"""
        if self.camera_info is None:
            self.camera_info = msg
            # Extract intrinsic parameters
            # K matrix: [fx  0 cx]
            #           [ 0 fy cy]
            #           [ 0  0  1]
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]

            rospy.loginfo(f"Camera intrinsics updated: fx={self.fx}, fy={self.fy}, "
                         f"cx={self.cx}, cy={self.cy}")

            # Unsubscribe after getting info once
            self.camera_info_sub.unregister()

    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")

    def depth_callback(self, depth_msg):
        """Process depth image and convert to point cloud"""
        try:
            # Convert depth image
            if depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            elif depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            else:
                rospy.logwarn(f"Unexpected depth encoding: {depth_msg.encoding}")
                return

            # Convert to meters
            if depth_msg.encoding == '16UC1':
                depth_image = depth_image.astype(np.float32) * self.depth_scale

            # Create point cloud
            point_cloud = self.depth_to_pointcloud(depth_image, depth_msg.header)

            if point_cloud is not None:
                self.cloud_pub.publish(point_cloud)
                rospy.loginfo_throttle(2.0, f"Published point cloud from depth image")

        except Exception as e:
            rospy.logerr(f"Error in depth callback: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def depth_to_pointcloud(self, depth_image, header):
        """
        Convert depth image to point cloud using camera intrinsics

        Args:
            depth_image: numpy array of depth values in meters
            header: ROS header for the point cloud

        Returns:
            PointCloud2 message
        """
        height, width = depth_image.shape

        # Downsample if requested
        if self.downsample > 1:
            depth_image = depth_image[::self.downsample, ::self.downsample]
            height, width = depth_image.shape

        # Create coordinate grids
        # For each pixel (u, v), compute 3D point (x, y, z)
        u_grid, v_grid = np.meshgrid(
            np.arange(width) * self.downsample,
            np.arange(height) * self.downsample
        )

        # Get depth values
        z = depth_image

        # Filter valid depth values
        valid_mask = (z > self.min_depth) & (z < self.max_depth) & np.isfinite(z)

        # Compute 3D coordinates using pinhole camera model
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        x = (u_grid - self.cx) * z / self.fx
        y = (v_grid - self.cy) * z / self.fy

        # Extract valid points
        x_valid = x[valid_mask]
        y_valid = y[valid_mask]
        z_valid = z[valid_mask]

        if len(x_valid) == 0:
            rospy.logwarn_throttle(5.0, "No valid points in depth image")
            return None

        # Create point cloud
        points = np.column_stack((x_valid, y_valid, z_valid))

        # Add RGB color if available
        if self.latest_rgb is not None:
            try:
                # Downsample RGB to match depth
                rgb_downsampled = self.latest_rgb[::self.downsample, ::self.downsample]

                # Extract RGB values for valid points
                rgb_valid = rgb_downsampled[valid_mask]

                # Pack RGB into single float
                rgb_float = np.zeros(len(rgb_valid), dtype=np.float32)
                for i in range(len(rgb_valid)):
                    r, g, b = rgb_valid[i]
                    # Pack RGB into 32-bit float
                    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                    rgb_float[i] = np.frombuffer(np.uint32(rgb_int).tobytes(), dtype=np.float32)[0]

                # Create structured array with XYZRGB
                points_rgb = np.zeros((len(points), 4), dtype=np.float32)
                points_rgb[:, :3] = points
                points_rgb[:, 3] = rgb_float

                # Create PointCloud2 message with color
                cloud_msg = pc2.create_cloud_xyz32(header, points_rgb[:, :3].tolist())

                # For RGB, we need to create a custom cloud
                # (Simplified version without RGB for now)
                cloud_msg = pc2.create_cloud_xyz32(header, points.tolist())

            except Exception as e:
                rospy.logwarn(f"Error adding RGB to point cloud: {e}")
                cloud_msg = pc2.create_cloud_xyz32(header, points.tolist())
        else:
            # Create point cloud without color
            cloud_msg = pc2.create_cloud_xyz32(header, points.tolist())

        rospy.loginfo_throttle(5.0, f"Created point cloud with {len(points)} points")

        return cloud_msg

    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DepthProcessorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
