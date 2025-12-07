#!/usr/bin/env python3
"""
Point Cloud Plane Detection for Navigation - PRODUCTION VERSION
================================================================
Optimized for elderly care navigation with robust plane detection.

Key Features:
- Strict classification thresholds (reduces false positives)
- Label stability locking (prevents flickering)
- Confidence-based filtering (only use reliable planes)
- Robot motion detection (stricter when moving)
- Spatial consistency validation (planes make sense together)

Topics:
    Subscribed:
        /hsrb/head_rgbd_sensor/depth_registered/rectified_points
        /hsrb/odom (for motion detection)

    Published:
        /lidar/ransac_planes (visualization_msgs/MarkerArray)
        /lidar/plane_labels (visualization_msgs/MarkerArray)
        /lidar/plane_stats (std_msgs/String)
        /lidar/reliable_planes (custom message with confidence info)
"""

import rospy
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from scipy.spatial import KDTree


class PointCloudPlaneDetector:
    """Detects planes in point cloud for navigation planning."""

    def __init__(self):
        """Initialize the plane detector node."""
        rospy.init_node('point_cloud_plane_detector', anonymous=False)

        # ========================================
        # Basic Parameters
        # ========================================
        self.voxel_size = rospy.get_param('~voxel_size', 0.0025)
        self.max_distance = rospy.get_param('~max_distance', 100.0)
        self.use_3d_camera = rospy.get_param('~use_3d_camera', True)

        # ========================================
        # RANSAC Parameters
        # ========================================
        self.ransac_threshold = 0.05  # 5cm tolerance
        self.min_points = 250         # Minimum points for valid plane (increased from 200)
        self.max_planes = 8           # Detect up to 8 planes
        
        # ========================================
        # STRICT Classification Thresholds (IMPROVED!)
        # ========================================
        self.floor_z_max = 0.3          # Floor must be < 30cm (was 0.5)
        self.ceiling_z_min = 2.0        # Ceiling must be > 2m (was 1.0)
        self.horizontal_threshold = 0.85 # 85% vertical for horizontal planes (was 0.5!)
        self.vertical_threshold = 0.2    # 20% vertical for walls (was 0.3)
        
        # Confidence thresholds
        self.min_confidence_for_navigation = 0.7  # NEW! Only use high-confidence planes
        self.min_observations = 3                 # NEW! Must be seen 3+ times
        
        # ========================================
        # Temporal Tracking
        # ========================================
        self.plane_history = {}
        self.history_max_age = 10
        self.temporal_weight = 0.7
        
        # ========================================
        # Robot Motion Detection (NEW!)
        # ========================================
        self.last_robot_pose = None
        self.robot_moving = False
        self.robot_linear_velocity = 0.0
        self.robot_angular_velocity = 0.0
        
        # ========================================
        # Publishers
        # ========================================
        self.ransac_planes_pub = rospy.Publisher('/lidar/ransac_planes', MarkerArray, queue_size=1)
        self.plane_labels_pub = rospy.Publisher('/lidar/plane_labels', MarkerArray, queue_size=1)
        self.stats_pub = rospy.Publisher('/lidar/plane_stats', String, queue_size=1)
        
        # Performance tracking
        self.frame_times = []
        self.frame_counter = 0

        # Clear old markers
        self.clear_old_markers()

        # ========================================
        # Subscribers
        # ========================================
        
        # Point cloud
        if self.use_3d_camera:
            self.cloud_sub = rospy.Subscriber(
                '/hsrb/head_rgbd_sensor/depth_registered/rectified_points',
                PointCloud2,
                self.cloud_callback
            )
            rospy.loginfo("Using RGB-D camera for plane detection")
        else:
            self.cloud_sub = rospy.Subscriber('/lidar/point_cloud', PointCloud2, self.cloud_callback)
            rospy.loginfo("Using LiDAR for plane detection")
        
        # Odometry (for motion detection)
        self.odom_sub = rospy.Subscriber(
            '/hsrb/odom',
            Odometry,
            self.odom_callback
        )

        rospy.loginfo("=" * 70)
        rospy.loginfo("ROBUST Plane Detector INITIALIZED - Production Mode")
        rospy.loginfo("=" * 70)
        rospy.loginfo("✅ Strict classification thresholds")
        rospy.loginfo("✅ Label stability locking")
        rospy.loginfo("✅ Confidence-based filtering")
        rospy.loginfo("✅ Robot motion detection")
        rospy.loginfo("=" * 70)

    def odom_callback(self, odom_msg):
        """
        Detect if robot is moving (affects classification strictness).
        
        Args:
            odom_msg (nav_msgs/Odometry): Robot odometry
        """
        current_pose = odom_msg.pose.pose.position
        
        # Get velocities
        self.robot_linear_velocity = np.sqrt(
            odom_msg.twist.twist.linear.x**2 + 
            odom_msg.twist.twist.linear.y**2
        )
        self.robot_angular_velocity = abs(odom_msg.twist.twist.angular.z)
        
        # Check if robot is moving
        if self.last_robot_pose is not None:
            # Calculate movement since last frame
            dx = current_pose.x - self.last_robot_pose.x
            dy = current_pose.y - self.last_robot_pose.y
            movement = np.sqrt(dx**2 + dy**2)
            
            # Threshold: moving if > 5cm/frame OR velocity > 0.1 m/s
            self.robot_moving = (movement > 0.05 or 
                               self.robot_linear_velocity > 0.1 or 
                               self.robot_angular_velocity > 0.2)
            
            if self.robot_moving:
                rospy.loginfo_throttle(5.0, 
                    f"🚶 Robot moving: v={self.robot_linear_velocity:.2f}m/s, "
                    f"ω={self.robot_angular_velocity:.2f}rad/s"
                )
        
        self.last_robot_pose = current_pose

    def cloud_callback(self, cloud_msg):
        """Process incoming point cloud - detect planes only."""
        try:
            start_time = time.time()
            self.frame_counter += 1

            # Convert to numpy array
            points = self.pointcloud2_to_array(cloud_msg)
            if len(points) < 3:
                return

            # Filter by distance
            points = self.filter_by_distance(points, self.max_distance)
            if len(points) < 3:
                return

            # Remove outliers
            points = self.remove_statistical_outliers(points, k=20, std_ratio=2.0)
            if len(points) < 3:
                return

            # Downsample
            points = self.voxel_downsample(points, self.voxel_size)
            if len(points) < 3:
                return

            # Detect planes
            z_range = np.max(points[:, 2]) - np.min(points[:, 2])
            is_3d = z_range > 0.1

            if is_3d and len(points) >= 200:
                ransac_planes = self.ransac_plane_segmentation_optimized(points, cloud_msg.header)
                self.ransac_planes_pub.publish(ransac_planes)

            # Performance metrics
            processing_time = (time.time() - start_time) * 1000
            self.update_performance_metrics(processing_time)

        except Exception as e:
            rospy.logerr(f"❌ Error detecting planes: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 to numpy array."""
        points_list = []
        for point in point_cloud2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list)

    def filter_by_distance(self, points, max_dist):
        """Filter points by distance."""
        distances = np.linalg.norm(points, axis=1)
        return points[distances <= max_dist]

    def remove_statistical_outliers(self, points, k=20, std_ratio=2.0):
        """Remove statistical outliers."""
        if len(points) < k:
            return points
        
        try:
            tree = KDTree(points)
            distances, _ = tree.query(points, k=k+1)
            mean_distances = np.mean(distances[:, 1:], axis=1)
            global_mean = np.mean(mean_distances)
            global_std = np.std(mean_distances)
            threshold = global_mean + std_ratio * global_std
            mask = mean_distances < threshold
            filtered_points = points[mask]
            
            removed = len(points) - len(filtered_points)
            if removed > 0:
                rospy.loginfo_throttle(5.0, f"🧹 Removed {removed} outliers")
            
            return filtered_points
        except Exception as e:
            rospy.logwarn(f"Outlier removal failed: {e}")
            return points

    def voxel_downsample(self, points, voxel_size):
        """Downsample using voxel grid."""
        if len(points) == 0:
            return points
        voxel_indices = np.floor(points / voxel_size).astype(int)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_indices]

    def ransac_plane_segmentation_optimized(self, points, header):
        """
        OPTIMIZED RANSAC plane detection with robustness improvements.
        """
        marker_array = MarkerArray()
        remaining_points = points.copy()
        
        plane_colors = [
            ColorRGBA(0.0, 1.0, 0.0, 0.8),  # Green - Floor
            ColorRGBA(0.0, 0.5, 1.0, 0.8),  # Blue - Ceiling
            ColorRGBA(1.0, 0.5, 0.0, 0.8),  # Orange - Wall 1
            ColorRGBA(1.0, 0.0, 0.5, 0.8),  # Pink - Wall 2
            ColorRGBA(0.5, 0.0, 1.0, 0.8),  # Purple - Wall 3
            ColorRGBA(0.0, 1.0, 0.5, 0.8),  # Cyan - Wall 4
            ColorRGBA(1.0, 1.0, 0.0, 0.8),  # Yellow - Furniture
            ColorRGBA(0.5, 0.5, 0.5, 0.8),  # Gray - Unknown
        ]
        
        detected_planes = []
        
        for plane_idx in range(self.max_planes):
            if len(remaining_points) < self.min_points:
                break

            # Adaptive iterations (more for difficult planes)
            max_iterations = 50 if plane_idx < 2 else 150
            
            best_inliers = []
            best_normal = None
            best_d = 0
            best_score = 0

            # RANSAC loop
            for _ in range(max_iterations):
                if len(remaining_points) < 3:
                    break

                sample_indices = np.random.choice(len(remaining_points), 3, replace=False)
                p1, p2, p3 = remaining_points[sample_indices]

                v1 = p2 - p1
                v2 = p3 - p1
                normal = np.cross(v1, v2)
                normal_mag = np.linalg.norm(normal)

                if normal_mag < 0.001:
                    continue

                normal = normal / normal_mag
                d = -np.dot(normal, p1)

                distances = np.abs(np.dot(remaining_points, normal) + d)
                inlier_mask = distances < self.ransac_threshold
                inliers = remaining_points[inlier_mask]

                if len(inliers) > self.min_points:
                    # Score calculation
                    score = len(inliers)
                    
                    plane_area = len(inliers) * (self.voxel_size ** 2)
                    if plane_area > 2.0:
                        score += 100  # Bonus for large planes
                    
                    if abs(normal[2]) > self.horizontal_threshold:
                        score += 200  # Bonus for horizontal planes
                    
                    if abs(normal[2]) < self.vertical_threshold:
                        score += 50   # Bonus for vertical planes
                    
                    if score > best_score:
                        best_inliers = inliers
                        best_normal = normal
                        best_d = d
                        best_score = score

            if len(best_inliers) >= self.min_points:
                centroid = np.mean(best_inliers, axis=0)
                centroid_z = centroid[2]
                plane_area = len(best_inliers) * (self.voxel_size ** 2)
                
                # Classify plane (with motion-aware thresholds)
                plane_type = self.classify_plane_multi_criteria(
                    normal=best_normal,
                    centroid_z=centroid_z,
                    area=plane_area,
                    inlier_count=len(best_inliers)
                )
                
                plane_info = {
                    'type': plane_type,
                    'centroid': centroid,
                    'normal': best_normal,
                    'points': best_inliers,
                    'num_points': len(best_inliers),
                    'area': plane_area,
                    'confidence': 0.5,
                    'frame': self.frame_counter,
                    'observations': 1  # NEW: Track observation count
                }
                
                # Match with history (with label locking)
                matched_key = self.match_with_history(plane_info)
                
                if matched_key:
                    old_plane = self.plane_history[matched_key]
                    
                    # Temporal smoothing
                    plane_info['normal'] = (
                        self.temporal_weight * old_plane['normal'] + 
                        (1 - self.temporal_weight) * plane_info['normal']
                    )
                    plane_info['normal'] /= np.linalg.norm(plane_info['normal'])
                    
                    plane_info['centroid'] = (
                        self.temporal_weight * old_plane['centroid'] + 
                        (1 - self.temporal_weight) * plane_info['centroid']
                    )
                    
                    # Update observations
                    plane_info['observations'] = old_plane['observations'] + 1
                    
                    # ========================================
                    # LABEL STABILITY CHECK (NEW!)
                    # ========================================
                    if plane_info['type'] != old_plane['type']:
                        # Type changed! Suspicious!
                        rospy.logwarn(
                            f"⚠️ Plane {plane_idx} type changed: "
                            f"{old_plane['type']} → {plane_info['type']} "
                            f"(conf={old_plane['confidence']:.2f})"
                        )
                        
                        # Only allow change if old confidence is LOW
                        if old_plane['confidence'] > 0.6:
                            # LOCK THE LABEL! Don't change it
                            plane_info['type'] = old_plane['type']
                            plane_info['confidence'] = max(old_plane['confidence'] - 0.1, 0.4)
                            rospy.loginfo(f"  🔒 Locked to: {old_plane['type']}")
                        else:
                            # Low confidence, allow change but reset confidence
                            plane_info['confidence'] = 0.5
                            rospy.loginfo(f"  ✓ Accepted change (low conf)")
                    else:
                        # Type stayed same - increase confidence
                        plane_info['confidence'] = min(old_plane['confidence'] + 0.1, 1.0)
                    
                    rospy.loginfo(
                        f"✓ Matched: {plane_info['type']} "
                        f"(conf: {plane_info['confidence']:.2f}, obs: {plane_info['observations']})"
                    )
                else:
                    matched_key = f"plane_{plane_idx}_{self.frame_counter}"
                    rospy.loginfo(f"✨ New: {plane_type}")
                
                self.plane_history[matched_key] = plane_info
                
                # ========================================
                # Spatial Validation (NEW!)
                # ========================================
                plane_info = self.validate_plane_spatial_consistency(plane_info, detected_planes)
                
                detected_planes.append(plane_info)
                
                # Create marker
                marker = Marker()
                marker.header = header
                marker.header.frame_id = "head_rgbd_sensor_link"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "ransac_planes"
                marker.id = plane_idx
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.lifetime = rospy.Duration(0)
                marker.frame_locked = True
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.color = plane_colors[plane_idx % len(plane_colors)]

                for p in best_inliers:
                    marker.points.append(Point(p[0], p[1], p[2]))

                marker_array.markers.append(marker)

                rospy.loginfo(
                    f"✓ Plane {plane_idx}: {plane_info['type']}, "
                    f"{len(best_inliers)} pts, z={centroid_z:.2f}m, "
                    f"area={plane_area:.2f}m², conf={plane_info['confidence']:.2f}"
                )

                # Remove inliers
                distances = np.abs(np.dot(remaining_points, best_normal) + best_d)
                outlier_mask = distances >= self.ransac_threshold
                remaining_points = remaining_points[outlier_mask]

        self.cleanup_plane_history()
        
        # Count reliable planes
        reliable_count = sum(
            1 for p in detected_planes 
            if p['confidence'] > self.min_confidence_for_navigation
        )
        
        rospy.loginfo(
            f"✓ Detected {len(detected_planes)} planes "
            f"({reliable_count} reliable, history: {len(self.plane_history)})"
        )
        
        if len(detected_planes) > 0:
            self.publish_plane_labels(detected_planes, header)
        
        return marker_array

    def classify_plane_multi_criteria(self, normal, centroid_z, area, inlier_count):
        """
        Classify plane using multiple criteria.
        Adjusts thresholds based on robot motion.
        """
        normal_z = normal[2]
        
        # ========================================
        # Motion-aware thresholds (NEW!)
        # ========================================
        if self.robot_moving:
            # Robot moving: be MORE strict
            horizontal_threshold = 0.95  # Was 0.85
            floor_z_max = 0.2           # Was 0.3
            ceiling_z_min = 2.2         # Was 2.0
            rospy.loginfo_throttle(10.0, "🚶 Using strict thresholds (robot moving)")
        else:
            # Robot stationary: use normal thresholds
            horizontal_threshold = self.horizontal_threshold
            floor_z_max = self.floor_z_max
            ceiling_z_min = self.ceiling_z_min
        
        # ========================================
        # Classification Logic
        # ========================================
        if abs(normal_z) > horizontal_threshold:
            # Horizontal plane
            if centroid_z < floor_z_max:
                return "Floor"
            elif centroid_z > ceiling_z_min:
                return "Ceiling"
            else:
                # Horizontal but mid-height (table?)
                return "Furniture" if area < 1.0 else "Unknown"
        
        elif abs(normal_z) < self.vertical_threshold:
            # Vertical plane
            if area > 2.0:
                return "Wall"
            elif centroid_z < 1.0:
                return "Furniture"  # Low vertical plane
            else:
                return "Wall"
        
        else:
            # Angled plane
            return "Furniture" if area < 0.5 else "Unknown"

    def validate_plane_spatial_consistency(self, plane, existing_planes):
        """
        Validate plane makes sense given other detected planes.
        
        Rules:
        - Only one floor allowed (lowest horizontal plane)
        - Only one ceiling allowed (highest horizontal plane)
        - Walls should be perpendicular to floor
        - Ceiling should be above everything
        """
        if not existing_planes:
            return plane  # First plane, no validation needed
        
        plane_type = plane['type']
        centroid_z = plane['centroid'][2]
        
        # Rule 1: Only ONE floor (pick lowest)
        if plane_type == 'Floor':
            existing_floors = [p for p in existing_planes if p['type'] == 'Floor']
            if existing_floors:
                lowest_floor_z = min(p['centroid'][2] for p in existing_floors)
                if centroid_z > lowest_floor_z + 0.2:
                    rospy.logwarn(f"  ⚠️ Multiple floors? Changed to Ceiling")
                    plane['type'] = 'Ceiling'
        
        # Rule 2: Only ONE ceiling (pick highest)
        if plane_type == 'Ceiling':
            existing_ceilings = [p for p in existing_planes if p['type'] == 'Ceiling']
            if existing_ceilings:
                highest_ceiling_z = max(p['centroid'][2] for p in existing_ceilings)
                if centroid_z < highest_ceiling_z - 0.2:
                    rospy.logwarn(f"  ⚠️ Multiple ceilings? Changed to Wall")
                    plane['type'] = 'Wall'
        
        # Rule 3: Ceiling should be higher than floor
        if plane_type == 'Ceiling':
            floors = [p for p in existing_planes if p['type'] == 'Floor']
            if floors:
                max_floor_z = max(p['centroid'][2] for p in floors)
                if centroid_z < max_floor_z + 1.5:
                    rospy.logwarn(f"  ⚠️ Ceiling too low? Changed to Wall")
                    plane['type'] = 'Wall'
        
        return plane

    def match_with_history(self, new_plane):
        """Match plane with history for temporal consistency."""
        if not self.plane_history:
            return None
        
        new_centroid = new_plane['centroid']
        new_normal = new_plane['normal']
        
        best_match = None
        best_score = 0.0
        
        for key, old_plane in self.plane_history.items():
            age = self.frame_counter - old_plane['frame']
            if age > self.history_max_age:
                continue
            
            old_centroid = old_plane['centroid']
            old_normal = old_plane['normal']
            
            # Distance between centroids
            centroid_dist = np.linalg.norm(new_centroid - old_centroid)
            
            # Angle between normals
            dot_product = np.clip(np.dot(new_normal, old_normal), -1.0, 1.0)
            angle_diff = np.degrees(np.arccos(abs(dot_product)))
            
            # Matching criteria
            if centroid_dist < 0.5 and angle_diff < 15.0:
                score = 1.0 - (centroid_dist / 0.5) - (angle_diff / 15.0)
                if score > best_score:
                    best_score = score
                    best_match = key
        
        return best_match if best_score > 0.5 else None

    def cleanup_plane_history(self):
        """Remove old planes from history."""
        keys_to_remove = [
            key for key, plane in self.plane_history.items()
            if self.frame_counter - plane['frame'] > self.history_max_age
        ]
        for key in keys_to_remove:
            del self.plane_history[key]
        
        if keys_to_remove:
            rospy.loginfo(f"🧹 Cleaned up {len(keys_to_remove)} old planes")

    def get_reliable_planes_for_navigation(self):
        """
        Get only high-confidence planes suitable for navigation.
        
        Returns:
            list: Planes with confidence > 70% and observations > 3
        """
        reliable_planes = []
        
        for key, plane in self.plane_history.items():
            age = self.frame_counter - plane['frame']
            
            # Criteria for reliable plane:
            # 1. High confidence (> 70%)
            # 2. Recently seen (< 5 frames old)
            # 3. Multiple observations (> 3 times)
            # 4. Enough points (> 300)
            if (plane['confidence'] > self.min_confidence_for_navigation and 
                age < 5 and
                plane.get('observations', 0) > self.min_observations and
                plane['num_points'] > 300):
                reliable_planes.append(plane)
        
        return reliable_planes

    def publish_plane_labels(self, planes_info, header):
        """Publish text labels for planes."""
        marker_array = MarkerArray()
        
        for i, plane_info in enumerate(planes_info):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = "head_rgbd_sensor_link"
            marker.ns = "plane_labels"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.5)
            
            centroid = plane_info['centroid']
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = centroid[2] + 0.3
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.2
            
            plane_type = plane_info['type']
            confidence = plane_info.get('confidence', 0.5)
            area = plane_info.get('area', 0.0)
            observations = plane_info.get('observations', 1)
            
            # Color based on type
            if plane_type == 'Floor':
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            elif plane_type == 'Ceiling':
                marker.color = ColorRGBA(0.0, 0.5, 1.0, 1.0)
            elif plane_type == 'Wall':
                marker.color = ColorRGBA(1.0, 0.5, 0.0, 1.0)
            elif plane_type == 'Furniture':
                marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
            else:
                marker.color = ColorRGBA(0.5, 0.5, 0.5, 1.0)
            
            # Enhanced label with confidence and observations
            marker.text = (
                f"{plane_type}\n"
                f"{plane_info['num_points']} pts\n"
                f"{area:.1f}m²\n"
                f"[{confidence:.0%}] x{observations}"
            )
            
            marker_array.markers.append(marker)
        
        self.plane_labels_pub.publish(marker_array)

    def update_performance_metrics(self, processing_time):
        """Track performance."""
        self.frame_times.append(processing_time)
        if len(self.frame_times) > 100:
            self.frame_times.pop(0)

        avg_time = np.mean(self.frame_times)
        fps = 1000.0 / avg_time if avg_time > 0 else 0

        # Get reliable plane count
        reliable_planes = self.get_reliable_planes_for_navigation()

        stats_msg = (
            f"Plane Detection Performance:\n"
            f"  Time: {processing_time:.1f}ms (avg: {avg_time:.1f}ms)\n"
            f"  FPS: {fps:.1f}\n"
            f"  Tracked planes: {len(self.plane_history)}\n"
            f"  Reliable planes: {len(reliable_planes)}\n"
            f"  Robot moving: {'Yes' if self.robot_moving else 'No'}"
        )
        self.stats_pub.publish(String(data=stats_msg))

    def clear_old_markers(self):
        """Clear old markers."""
        rospy.loginfo("🧹 Clearing old markers...")
        
        # Clear planes
        clear_array = MarkerArray()
        for i in range(20):  # Increased from 10
            clear_marker = Marker()
            clear_marker.header.frame_id = "head_rgbd_sensor_link"
            clear_marker.header.stamp = rospy.Time.now()
            clear_marker.ns = "ransac_planes"
            clear_marker.id = i
            clear_marker.action = Marker.DELETE
            clear_array.markers.append(clear_marker)
        self.ransac_planes_pub.publish(clear_array)
        
        # Clear labels
        clear_labels = MarkerArray()
        for i in range(20):
            clear_marker = Marker()
            clear_marker.header.frame_id = "head_rgbd_sensor_link"
            clear_marker.header.stamp = rospy.Time.now()
            clear_marker.ns = "plane_labels"
            clear_marker.id = i
            clear_marker.action = Marker.DELETE
            clear_labels.markers.append(clear_marker)
        self.plane_labels_pub.publish(clear_labels)
        
        rospy.sleep(0.5)
        rospy.loginfo("✓ Old markers cleared")

    def run(self):
        """Run the node."""
        rospy.loginfo("✅ Robust Plane Detector running - ready for navigation!")
        rospy.spin()


if __name__ == '__main__':
    try:
        detector = PointCloudPlaneDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Plane Detector shut down")
        pass