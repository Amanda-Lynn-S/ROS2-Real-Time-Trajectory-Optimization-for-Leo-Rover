import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs_py import point_cloud2


class HeightCostmap(Node):
    """
    MVP height-based costmap:
      - Subscribes to PointCloud2 (optionally transforms to target frame)
      - Discretizes into XY grid
      - Stores max Z per cell
      - Marks cell as occupied if maxZ > height_threshold AND enough points in cell
      - Publishes nav_msgs/OccupancyGrid

    Designed for 'camera stationary on loner floor' stability testing:
      - crop range
      - crop Z band
      - min points per cell
    """

    def __init__(self):
        super().__init__('height_costmap')

        # Topics/frames
        self.declare_parameter('input_topic', '/cloud_in_target_frame')
        self.declare_parameter('output_topic', '/height_costmap')
        self.declare_parameter('target_frame', 'zed_optical_frame') #switched from odom to map frame (WIP)

        # Grid geometry
        self.declare_parameter('resolution', 0.10)     # m/cell
        self.declare_parameter('width', 100)           # cells
        self.declare_parameter('height', 100)          # cells
        self.declare_parameter('origin_x', -5.0)       # meters (lower-left)
        self.declare_parameter('origin_y', -5.0)       # meters

        # Binary safe / not-safe rule
        self.declare_parameter('height_threshold', 0.10)     # meters (unsafe if > threshold)

        # Stability knobs (lab-ready)
        self.declare_parameter('min_points_per_cell', 5)     # ignore cells with fewer points
        self.declare_parameter('range_max', 6.0)             # meters (radial crop in XY)
        self.declare_parameter('z_min', -0.30)               # meters
        self.declare_parameter('z_max', 0.50)                # meters

        # Unknown handling: if a cell has no data, publish -1 (unknown) or 0 (free)
        self.declare_parameter('unknown_is_free', False)

        # Logging throttle (seconds)
        self.declare_parameter('log_period_s', 2.0)

        # Read params
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.declare_parameter('floor_percentile', 10.0)
        self.floor_percentile = float(self.get_parameter('floor_percentile').value)

        self.resolution = float(self.get_parameter('resolution').value)
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)

        self.height_threshold = float(self.get_parameter('height_threshold').value)
        self.min_points_per_cell = int(self.get_parameter('min_points_per_cell').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        self.unknown_is_free = bool(self.get_parameter('unknown_is_free').value)
        self.log_period_s = float(self.get_parameter('log_period_s').value)
        self._last_log_time = self.get_clock().now()

        # Sub/Pub (Sensor QoS for point clouds)
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cb,
            qos_profile_sensor_data
        )
        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'HeightCostmap: {self.input_topic} -> {self.output_topic} | '
            f'grid={self.w}x{self.h} res={self.resolution} origin=({self.origin_x},{self.origin_y}) | '
            f'target_frame={self.target_frame} thr={self.height_threshold}m '
            f'minPts={self.min_points_per_cell} rangeMax={self.range_max} z=[{self.z_min},{self.z_max}]'
        )

    def _throttled_log(self, msg: str):
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds * 1e-9 >= self.log_period_s:
            self.get_logger().info(msg)
            self._last_log_time = now

    def cb(self, msg: PointCloud2):
        cloud = msg

        # Transform to target frame if needed
        if msg.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()  # latest available
                )
                cloud = do_transform_cloud(msg, tf)
            except Exception as e:
                self.get_logger().warn(f'TF to target frame failed: {e}')
                return

        # Per-cell max height and point counts
        max_z = np.full((self.h, self.w), -np.inf, dtype=np.float32)
        counts = np.zeros((self.h, self.w), dtype=np.uint16)

        # Iterate points (MVP: simple loop, no optimization)
        # Apply range + z-band filtering before binning
        # First pass: collect filtered points so we can estimate floor height
        pts = []
        rmax2 = self.range_max * self.range_max

        for (x, y, z) in point_cloud2.read_points(
            cloud,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            if (x * x + y * y) > rmax2:
                continue
            if z < self.z_min or z > self.z_max:
                continue
            pts.append((x, y, z))

        if len(pts) == 0:
            return

        # Estimate floor height from low percentile of filtered z values
        z_values = np.array([p[2] for p in pts], dtype=np.float32)
        floor_z = float(np.percentile(z_values, self.floor_percentile))

        # Per-cell max relative height and counts
        max_rel_z = np.full((self.h, self.w), -np.inf, dtype=np.float32)
        counts = np.zeros((self.h, self.w), dtype=np.uint16)

        for (x, y, z) in pts:
            z_rel = z - floor_z

            gx = int((x - self.origin_x) / self.resolution)
            gy = int((y - self.origin_y) / self.resolution)

            if 0 <= gx < self.w and 0 <= gy < self.h:
                counts[gy, gx] += 1
                if z_rel > max_rel_z[gy, gx]:
                    max_rel_z[gy, gx] = z_rel

        # OccupancyGrid data: -1 unknown, 0 free, 100 occupied
        if self.unknown_is_free:
            data = np.zeros((self.h, self.w), dtype=np.int8)   # default free
        else:
            data = np.full((self.h, self.w), -1, dtype=np.int8)  # default unknown

        # Cells with enough points are "known"
        known = counts >= self.min_points_per_cell
        data[known] = 0

        # Unsafe if max height > threshold
        unsafe = known & (max_z > self.height_threshold)
        data[unsafe] = 100

        # Build message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.target_frame

        info = MapMetaData()
        info.resolution = self.resolution
        info.width = self.w
        info.height = self.h

        origin = Pose()
        origin.position.x = self.origin_x
        origin.position.y = self.origin_y
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        info.origin = origin

        grid.info = info
        grid.data = data.flatten(order='C').tolist()

        self.pub.publish(grid)

        # Throttled status
        known_cells = int(np.count_nonzero(known))
        unsafe_cells = int(np.count_nonzero(unsafe))
        self._throttled_log(f'Published costmap: known={known_cells} unsafe={unsafe_cells}')


def main(args=None):
    rclpy.init(args=args)
    node = HeightCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
