#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.path_pub = self.create_publisher(Path, '/executed_path', 10)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.odom_sub = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, qos)
        self.path = Path()
        self.path.header.frame_id = 'map'

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
