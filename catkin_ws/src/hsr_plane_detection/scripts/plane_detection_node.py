#!/usr/bin/env python3
"""
Plane Detection Node for Toyota HSR Robot
Uses RANSAC algorithm to detect planes from point cloud data
Publishes visualization markers for RViz
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
import sensor_msgs.point_cloud2 as pc2
from sklearn.linear_model import RANSACRegressor
import struct


class PlaneDetectionNode:
    def __init__(self):
        rospy.init_node('plane_detection_node', anonymous=False)

        # Parameters
        self.min_plane_points = rospy.get_param('~min_plane_points', 500)
        self.ransac_threshold = rospy.get_param('~ransac_threshold', 0.02)
        self.max_planes = rospy.get_param('~max_planes', 5)
        self.min_plane_area = rospy.get_param('~min_plane_area', 0.1)
        self.downsample_factor = rospy.get_param('~downsample_factor', 4)

        rospy.loginfo("Plane Detection Node Initialized")
        rospy.loginfo(f"Parameters: min_points={self.min_plane_points}, "
                     f"threshold={self.ransac_threshold}, max_planes={self.max_planes}")

        # Publishers
        self.marker_pub = rospy.Publisher('/plane_markers', MarkerArray, queue_size=10)
        self.plane_cloud_pub = rospy.Publisher('/detected_plane_cloud', PointCloud2, queue_size=10)

        # Subscriber
        self.cloud_sub = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/points',
            PointCloud2,
            self.point_cloud_callback,
            queue_size=1
        )

        # Color palette for different planes
        self.colors = [
            ColorRGBA(1.0, 0.0, 0.0, 0.6),  # Red
            ColorRGBA(0.0, 1.0, 0.0, 0.6),  # Green
            ColorRGBA(0.0, 0.0, 1.0, 0.6),  # Blue
            ColorRGBA(1.0, 1.0, 0.0, 0.6),  # Yellow
            ColorRGBA(1.0, 0.0, 1.0, 0.6),  # Magenta
            ColorRGBA(0.0, 1.0, 1.0, 0.6),  # Cyan
        ]

        rospy.loginfo("Waiting for point cloud data...")

    def point_cloud_callback(self, cloud_msg):
        """Process incoming point cloud and detect planes"""
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(cloud_msg)

            if points is None or len(points) < self.min_plane_points:
                rospy.logwarn_throttle(5.0, "Insufficient points in cloud")
                return

            # Downsample for performance
            if self.downsample_factor > 1:
                points = points[::self.downsample_factor]

            rospy.loginfo_throttle(2.0, f"Processing {len(points)} points")

            # Detect planes
            planes = self.detect_planes(points)

            if len(planes) > 0:
                rospy.loginfo_throttle(1.0, f"Detected {len(planes)} planes")
                # Visualize planes
                self.publish_plane_markers(planes, cloud_msg.header)
            else:
                rospy.logwarn_throttle(5.0, "No planes detected")

        except Exception as e:
            rospy.logerr(f"Error in point cloud callback: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        try:
            # Read points from point cloud
            points_list = []

            for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([point[0], point[1], point[2]])

            if len(points_list) == 0:
                return None

            points = np.array(points_list, dtype=np.float32)

            # Filter out invalid points (inf, nan, too far)
            valid_mask = np.all(np.isfinite(points), axis=1)
            valid_mask &= np.linalg.norm(points, axis=1) < 10.0  # Max 10m distance

            return points[valid_mask]

        except Exception as e:
            rospy.logerr(f"Error converting point cloud: {e}")
            return None

    def detect_planes(self, points):
        """Detect multiple planes using RANSAC"""
        planes = []
        remaining_points = points.copy()

        for plane_idx in range(self.max_planes):
            if len(remaining_points) < self.min_plane_points:
                break

            # Fit plane using RANSAC
            plane_info = self.fit_plane_ransac(remaining_points)

            if plane_info is None:
                break

            plane_points, plane_coeffs, inlier_mask = plane_info

            # Check if plane is large enough
            plane_area = self.estimate_plane_area(plane_points)
            if plane_area < self.min_plane_area:
                rospy.loginfo(f"Plane {plane_idx} too small ({plane_area:.3f} m²), skipping")
                break

            planes.append({
                'points': plane_points,
                'coefficients': plane_coeffs,
                'area': plane_area,
                'normal': plane_coeffs[:3],
                'centroid': np.mean(plane_points, axis=0)
            })

            # Remove inliers from remaining points
            remaining_points = remaining_points[~inlier_mask]

            rospy.loginfo(f"Plane {plane_idx}: {len(plane_points)} points, "
                         f"area={plane_area:.3f} m², normal={plane_coeffs[:3]}")

        return planes

    def fit_plane_ransac(self, points):
        """Fit a plane to points using RANSAC"""
        if len(points) < self.min_plane_points:
            return None

        try:
            # Use X, Y to predict Z (assumes planes are not too vertical)
            # We'll try both orientations and pick the best

            best_fit = None
            best_inliers = 0

            # Try fitting Z from X,Y
            X = points[:, :2]  # X, Y
            y = points[:, 2]   # Z

            ransac = RANSACRegressor(
                min_samples=3,
                residual_threshold=self.ransac_threshold,
                max_trials=100,
                random_state=42
            )

            try:
                ransac.fit(X, y)
                inlier_mask = ransac.inlier_mask_
                n_inliers = np.sum(inlier_mask)

                if n_inliers >= self.min_plane_points:
                    # Convert to plane equation: ax + by + cz + d = 0
                    # From z = mx + ny + c  =>  mx + ny - z + c = 0
                    a, b = ransac.estimator_.coef_
                    c = -1.0
                    d = ransac.estimator_.intercept_

                    # Normalize
                    norm = np.sqrt(a**2 + b**2 + c**2)
                    coeffs = np.array([a/norm, b/norm, c/norm, d/norm])

                    best_fit = (points[inlier_mask], coeffs, inlier_mask)
                    best_inliers = n_inliers
            except Exception as e:
                rospy.logwarn(f"RANSAC fitting failed: {e}")

            # Try fitting Y from X,Z (for vertical planes)
            X2 = points[:, [0, 2]]  # X, Z
            y2 = points[:, 1]       # Y

            ransac2 = RANSACRegressor(
                min_samples=3,
                residual_threshold=self.ransac_threshold,
                max_trials=100,
                random_state=42
            )

            try:
                ransac2.fit(X2, y2)
                inlier_mask2 = ransac2.inlier_mask_
                n_inliers2 = np.sum(inlier_mask2)

                if n_inliers2 > best_inliers and n_inliers2 >= self.min_plane_points:
                    # Convert to plane equation: ax + by + cz + d = 0
                    # From y = mx + nz + c  =>  mx - y + nz + c = 0
                    a2 = ransac2.estimator_.coef_[0]
                    b2 = -1.0
                    c2 = ransac2.estimator_.coef_[1]
                    d2 = ransac2.estimator_.intercept_

                    # Normalize
                    norm = np.sqrt(a2**2 + b2**2 + c2**2)
                    coeffs = np.array([a2/norm, b2/norm, c2/norm, d2/norm])

                    best_fit = (points[inlier_mask2], coeffs, inlier_mask2)
                    best_inliers = n_inliers2
            except Exception as e:
                rospy.logwarn(f"RANSAC fitting (vertical) failed: {e}")

            return best_fit

        except Exception as e:
            rospy.logerr(f"Error in RANSAC plane fitting: {e}")
            return None

    def estimate_plane_area(self, points):
        """Estimate the area of a plane using its bounding box"""
        if len(points) < 3:
            return 0.0

        # Project points onto plane's 2D space
        # Simple estimation using min/max in each dimension
        ranges = np.ptp(points, axis=0)  # Range in each dimension

        # Area is approximately the product of two largest ranges
        sorted_ranges = np.sort(ranges)[::-1]
        area = sorted_ranges[0] * sorted_ranges[1]

        return area

    def publish_plane_markers(self, planes, header):
        """Publish visualization markers for detected planes"""
        marker_array = MarkerArray()

        for idx, plane in enumerate(planes):
            # Create a marker for the plane
            marker = Marker()
            marker.header = header
            marker.ns = "planes"
            marker.id = idx
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            # Set color
            marker.color = self.colors[idx % len(self.colors)]

            # Create mesh from plane points
            points = plane['points']

            # Downsample points for visualization
            if len(points) > 1000:
                indices = np.random.choice(len(points), 1000, replace=False)
                points = points[indices]

            # Create triangles using convex hull approximation
            # For simplicity, we'll create a mesh by connecting nearby points
            centroid = plane['centroid']

            # Create a local coordinate system for the plane
            normal = plane['normal']

            # Get points relative to centroid
            rel_points = points - centroid

            # Create basis vectors for the plane
            if abs(normal[2]) < 0.9:
                u = np.cross(normal, [0, 0, 1])
            else:
                u = np.cross(normal, [1, 0, 0])
            u = u / np.linalg.norm(u)
            v = np.cross(normal, u)

            # Project points to 2D
            u_coords = np.dot(rel_points, u)
            v_coords = np.dot(rel_points, v)

            # Create simple triangulation (fan from centroid)
            # Sort points by angle
            angles = np.arctan2(v_coords, u_coords)
            sorted_indices = np.argsort(angles)

            # Create triangles
            n_triangles = min(len(sorted_indices) - 1, 100)  # Limit triangles
            step = max(1, len(sorted_indices) // n_triangles)

            for i in range(0, len(sorted_indices) - step, step):
                idx1 = sorted_indices[i]
                idx2 = sorted_indices[(i + step) % len(sorted_indices)]

                # Triangle: centroid, point1, point2
                p1 = Point(centroid[0], centroid[1], centroid[2])
                p2 = Point(points[idx1][0], points[idx1][1], points[idx1][2])
                p3 = Point(points[idx2][0], points[idx2][1], points[idx2][2])

                marker.points.extend([p1, p2, p3])

            marker_array.markers.append(marker)

            # Add text label
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "plane_labels"
            text_marker.id = idx + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = centroid[0]
            text_marker.pose.position.y = centroid[1]
            text_marker.pose.position.z = centroid[2]
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.1
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text_marker.text = f"Plane {idx}\n{plane['area']:.2f} m²"

            marker_array.markers.append(text_marker)

            # Add normal vector arrow
            arrow_marker = Marker()
            arrow_marker.header = header
            arrow_marker.ns = "plane_normals"
            arrow_marker.id = idx + 2000
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD

            # Arrow from centroid in direction of normal
            start = Point(centroid[0], centroid[1], centroid[2])
            end_point = centroid + normal * 0.3  # 30cm arrow
            end = Point(end_point[0], end_point[1], end_point[2])

            arrow_marker.points = [start, end]
            arrow_marker.scale.x = 0.02  # Shaft diameter
            arrow_marker.scale.y = 0.04  # Head diameter
            arrow_marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow

            marker_array.markers.append(arrow_marker)

        # Delete old markers
        for i in range(len(planes), 20):
            delete_marker = Marker()
            delete_marker.header = header
            delete_marker.ns = "planes"
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

            delete_marker2 = Marker()
            delete_marker2.header = header
            delete_marker2.ns = "plane_labels"
            delete_marker2.id = i + 1000
            delete_marker2.action = Marker.DELETE
            marker_array.markers.append(delete_marker2)

            delete_marker3 = Marker()
            delete_marker3.header = header
            delete_marker3.ns = "plane_normals"
            delete_marker3.id = i + 2000
            delete_marker3.action = Marker.DELETE
            marker_array.markers.append(delete_marker3)

        self.marker_pub.publish(marker_array)

    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PlaneDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
