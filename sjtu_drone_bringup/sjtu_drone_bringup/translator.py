import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from std_msgs.msg import Header
import struct


class PointCloudConverter(Node):
    def __init__(self):
        super().__init__('pointcloud_converter')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/simple_drone/mid360/out',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        custom_msg = self.convert_pointcloud2_to_livox(msg)
        self.publisher_.publish(custom_msg)

    def convert_pointcloud2_to_livox(self, cloud_msg):
        fmt = '>' if cloud_msg.is_bigendian else '<'
        point_step = cloud_msg.point_step
        offset_time = 0  # Example offset time, update as needed
        livox_points = []

        for i in range(cloud_msg.width * cloud_msg.height):
            point_data = cloud_msg.data[i * point_step:(i + 1) * point_step]
            x, y, z = struct.unpack(fmt + 'fff', point_data[:12])
            reflectivity = 100  # Example value, update as needed
            tag = 1  # Example value, update as needed
            line = 0  # Example value, update as needed

            livox_point = CustomPoint(
                offset_time=offset_time,
                x=x,
                y=y,
                z=z,
                reflectivity=reflectivity,
                tag=tag,
                line=line
            )

            livox_points.append(livox_point)
            offset_time += 1  # Example increment, update as needed

        custom_msg = CustomMsg(
            header=Header(),  # Provide a ROS standard message header
            timebase=0,  # Example value, update as needed
            point_num=len(livox_points),  # Total number of points
            lidar_id=0,  # Example value, update as needed
            rsvd=[0, 0, 0],  # Example value, update as needed
            points=livox_points
        )

        return custom_msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
