import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class MockCloudPub(Node):
    def __init__(self):
        super().__init__('mock_cloud_pub')
        self.pub = self.create_publisher(PointCloud2, 'point_cloud/cloud_registered', 10)
        self.timer = self.create_timer(0.5, self.publish_cloud)
        self.frame_id = 'zed_optical_frame'  # pretend ZED optical frame

    def publish_cloud(self):
        # x=right, y=down, z=forward
        pts = []
        for i in range(-20, 21):
            for j in range(1, 41):
                x = i * 0.05
                z = j * 0.05
                y = 0.0

                # "bump": in optical frame, up is -y
                if -0.3 < x < 0.3 and 1.0 < z < 1.4:
                    y = -0.2

                pts.append((x, y, z))

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(pts)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        buf = bytearray()
        for (x, y, z) in pts:
            buf.extend(struct.pack('fff', x, y, z))
        msg.data = bytes(buf)

        self.pub.publish(msg)
        self.get_logger().info(f'Published mock cloud: {msg.width} points in {self.frame_id}')

def main():
    rclpy.init()
    node = MockCloudPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
