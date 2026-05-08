#!/usr/bin/env python3
"""
map_publisher_node.py (perception-driven version)
This node subscribes to the live /height_costmap from rover_perception, computes the SDF, and publishes the map metadata and SDF grid for use by the planner.

Instead of manually generating map with obstacles, this node:
1. Subscribes to /height_costmap (nav_msgs/OccupancyGrid) from the rover_perception package
2. On first message received, computes the SDF and publishes:
   /map_meta  -> std_msgs/String  (JSON: width, height, cell_size, start, end)
   /sdf_grid  -> std_msgs/Float32MultiArray  (H×W SDF in grid-cell units)
3. Republishes every 2 seconds so late-starting subscribers (e.g. rover_pp_node) can still receive the map.

Start/End are set as ROS2 parameters (grid-cell coordinates):
    start_x, start_y  (default 10, 10)
    end_x,   end_y    (default 90, 90)
Run with: ros2 run rover_scp map_publisher_node --ros-args -p start_x:=X -p start_y:=Y -p end_x:=X -p end_y:=Y

Key Notes:
1)  OccupancyGrid.info.resolution [m/cell]
    Rover.py uses: scale = cell_size * pix_to_m | pix_to_m = 0.01 m/px
    => cell_size = resolution / pix_to_m [pixels/cell]
2)  SDF is published in grid-cell units (same as the original draw-based version).
    Rover.py converts to meters: self.sdf = sdf_array * scale
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid
import json
import numpy as np
from scipy import ndimage


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class MapPublisherNode(Node):
    """
    Subscribes to /height_costmap and publishes /map_meta + /sdf_grid.
    """

    PIX_TO_M = 0.01 #must match Rover.py's pix_to_m constant

    def __init__(self):
        super().__init__('map_publisher_node')
        # Default Start/End positions in grid-cell coordinates (not pixels):
        self.declare_parameter('start_x', 10) 
        self.declare_parameter('start_y', 10)
        self.declare_parameter('end_x', 90)
        self.declare_parameter('end_y', 90)
        # Publishers:
        self.meta_pub = self.create_publisher(String, '/map_meta', 10)
        self.sdf_pub = self.create_publisher(Float32MultiArray, '/sdf_grid', 10)
        # Subscriber for perception costmap:
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/height_costmap', self._costmap_callback, 10)
        self._cmd_timer = None 
        self._occ_array = None #H×W uint8: 0=free, 1=obstacle (if unknown -> represent as free)
        self.get_logger().info("map_publisher_node ready — waiting for /height_costmap")

    def _costmap_callback(self, msg: OccupancyGrid):
        """
        Convert OccupancyGrid to a binary numpy array:
        -1 (unknown) -> 0 (considered free)
        0 (free) -> 0
        100 (occupied) -> 1
        """
        if self._occ_array is not None: #firing once
            return
        H = msg.info.height #units in grid cells, not pixels
        W = msg.info.width
        raw = np.array(msg.data, dtype=np.int8).reshape(H, W)
        self._occ_array = np.where(raw == 100, 1, 0).astype(np.uint8) #this means: 100 -> 1, everything else (0 and -1) -> 0
        self._grid_info = msg.info
        self.get_logger().info(f"Received /height_costmap {W}×{H} | res={msg.info.resolution:.3f} m/cell")
        sx = int(self.get_parameter('start_x').value)
        sy = int(self.get_parameter('start_y').value)
        ex = int(self.get_parameter('end_x').value)
        ey = int(self.get_parameter('end_y').value)
        self.publish_map((sx, sy), (ex, ey))
        self._cmd_timer = self.create_timer(2.0, self._republish) #republish every 2 seconds in case the planner needs to re-query

    def _republish(self):
        if self._occ_array is not None:
            sx = int(self.get_parameter('start_x').value)
            sy = int(self.get_parameter('start_y').value)
            ex = int(self.get_parameter('end_x').value)
            ey = int(self.get_parameter('end_y').value)
            self.publish_map((sx, sy), (ex, ey))
    
    @staticmethod
    def _compute_sdf(occ: np.ndarray) -> np.ndarray:
        """
        Signed Distance Field in grid-cell units:
        > Positive outside obstacles (free space)
        > Negative inside obstacles
        """
        # Distance to nearest obstacle:
        dist_to_obstacle = ndimage.distance_transform_edt(np.logical_not(occ)) #EDT takes 1-occ as input array -> free space=1 & obstacles=0 since EDT computes distance from non-zero/non-background point to nearest zero/background point
        # Distance to nearest free space:
        dist_to_free = ndimage.distance_transform_edt(occ)
        return dist_to_obstacle - dist_to_free #SDF -> distances across grid space to nearest obstacle
    
    def publish_map(self, start_grid: tuple, end_grid: tuple) -> bool:
        """
        Input Parameters:
        > start_grid : (gx, gy)  grid-cell coordinates of the start
        > end_grid   : (gx, gy)  grid-cell coordinates of the goal
        """
        occ  = self._occ_array.copy()
        info = self._grid_info
        cell_size = int(round(info.resolution / self.PIX_TO_M)) #derive cell_size so that cell_size * PIX_TO_M == resolution (m/cell)
        # /map_meta -> JSON string with map metadata:
        meta = {
            "width":     info.width,
            "height":    info.height,
            "cell_size": cell_size,
            "start":     [int(start_grid[0]), int(start_grid[1])],
            "end":       [int(end_grid[0]),   int(end_grid[1])],
        }
        meta_msg = String()
        meta_msg.data = json.dumps(meta)
        self.meta_pub.publish(meta_msg)
        self.get_logger().info(f"Published /map_meta: {meta}")
        # /sdf_grid -> Float32MultiArray with the SDF values in grid-cell units:
        sdf = self._compute_sdf(occ)
        rows, cols = sdf.shape
        sdf_msg = Float32MultiArray()
        sdf_msg.layout.dim = [
            MultiArrayDimension(label="rows", size=rows,  stride=rows * cols),
            MultiArrayDimension(label="cols", size=cols,  stride=cols),
        ]
        sdf_msg.layout.data_offset = 0
        sdf_msg.data = sdf.astype(np.float32).flatten().tolist()
        self.sdf_pub.publish(sdf_msg)
        self.get_logger().info(f"Published /sdf_grid ({rows}×{cols})")
        return True


# ─────────────────────────────────────────────────────────────────────────────
# Main entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
