#!/usr/bin/env python3
"""
LiDAR Activator Node - OPTIMIZED FOR FULL ROOM COVERAGE
========================================================
Subscribes to HSR robot's LiDAR sensor and converts to point cloud.
Configured for MAXIMUM range - no artificial distance limits.

Topics:
    Subscribed:
        /hsrb/base_scan (sensor_msgs/LaserScan): Raw LiDAR data from HSR

    Published:
        /lidar/scan_processed (sensor_msgs/LaserScan): Processed LiDAR data
        /lidar/point_cloud (sensor_msgs/PointCloud2): Converted point cloud
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import laser_geometry.laser_geometry as lg


class LidarActivator:
    """Activates and processes LiDAR data from the HSR robot."""

    def __init__(self):
        """Initialize the LiDAR activator node."""
        rospy.init_node('lidar_activator', anonymous=False)

        # ========================================
        # Subscription topic
        # ========================================
        self.scan_topic = rospy.get_param('~scan_topic', '/hsrb/base_scan')

        # ========================================
        # Range filtering (MAXIMIZED for full room coverage)
        # ========================================
        # Min range: Very close (5cm) - removes sensor noise
        self.min_range = rospy.get_param('~min_range', 0.05)
        
        # Max range: Essentially unlimited (100m = entire building)
        # HSR LiDAR physical limit is ~30m, but we set higher to never limit
        self.max_range = rospy.get_param('~max_range', 100.0)

        # Laser geometry projector for converting LaserScan to PointCloud2
        self.laser_projector = lg.LaserProjection()

        # Publishers
        self.scan_pub = rospy.Publisher('/lidar/scan_processed', LaserScan, queue_size=10)
        self.cloud_pub = rospy.Publisher('/lidar/point_cloud', PointCloud2, queue_size=10)

        # Subscriber
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        # Performance tracking
        self.scan_count = 0
        self.last_log_time = rospy.Time.now()

        rospy.loginfo("=" * 70)
        rospy.loginfo("LiDAR Activator INITIALIZED - Full Room Coverage Mode")
        rospy.loginfo("=" * 70)
        rospy.loginfo(f"Subscribing to: {self.scan_topic}")
        rospy.loginfo(f"Range filter: {self.min_range}m to {self.max_range}m")
        rospy.loginfo(f"Mode: UNLIMITED range (entire room)")
        rospy.loginfo("=" * 70)

    def scan_callback(self, scan_msg):
        """
        Process incoming laser scan data.

        Args:
            scan_msg (sensor_msgs/LaserScan): Raw laser scan message
        """
        try:
            self.scan_count += 1

            # Filter the scan data (removes out-of-range readings)
            filtered_scan = self.filter_scan(scan_msg)

            # Publish filtered scan
            self.scan_pub.publish(filtered_scan)

            # Convert to point cloud
            point_cloud = self.laser_projector.projectLaser(filtered_scan)

            # Publish point cloud
            self.cloud_pub.publish(point_cloud)

            # Performance logging (every 5 seconds)
            current_time = rospy.Time.now()
            if (current_time - self.last_log_time).to_sec() >= 5.0:
                valid_count = sum(1 for r in filtered_scan.ranges if r < self.max_range)
                total_count = len(filtered_scan.ranges)
                valid_percent = (valid_count / total_count * 100) if total_count > 0 else 0
                
                rospy.loginfo(
                    f"LiDAR: {self.scan_count} scans processed | "
                    f"Valid points: {valid_count}/{total_count} ({valid_percent:.1f}%)"
                )
                self.last_log_time = current_time

        except Exception as e:
            rospy.logerr(f"Error processing scan: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def filter_scan(self, scan):
        """
        Filter laser scan to remove invalid readings.
        
        Keeps only readings within [min_range, max_range].
        Out-of-range readings are set to infinity (invalid).

        Args:
            scan (sensor_msgs/LaserScan): Input scan message

        Returns:
            sensor_msgs/LaserScan: Filtered scan message
        """
        # Create filtered scan (copy metadata)
        filtered_scan = LaserScan()
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.angle_increment = scan.angle_increment
        filtered_scan.time_increment = scan.time_increment
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = self.min_range
        filtered_scan.range_max = self.max_range

        # ========================================
        # VECTORIZED filtering (faster than loop)
        # ========================================
        ranges_array = np.array(scan.ranges)
        
        # Create mask: True for valid ranges
        valid_mask = (ranges_array >= self.min_range) & (ranges_array <= self.max_range)
        
        # Apply mask: invalid readings → infinity
        filtered_ranges = np.where(valid_mask, ranges_array, float('inf'))
        
        filtered_scan.ranges = filtered_ranges.tolist()
        filtered_scan.intensities = scan.intensities

        return filtered_scan

    def run(self):
        """Run the node."""
        rospy.loginfo("LiDAR Activator running - processing scans...")
        rospy.spin()


if __name__ == '__main__':
    try:
        activator = LidarActivator()
        activator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("LiDAR Activator shut down")
        pass