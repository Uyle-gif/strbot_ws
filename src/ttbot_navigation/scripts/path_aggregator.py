#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathAggregator(Node):
    def __init__(self):
        super().__init__('path_aggregator')
        self.subscription = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.publisher = self.create_publisher(Path, '/mpc_path', 10)
        self.full_path = Path()
        self.get_logger().info('Initialized Path Aggregator Node')

    def path_callback(self, msg):
        if len(self.full_path.poses) > 500: 
            self.full_path.poses = []

        self.full_path.header = msg.header
        self.full_path.poses.extend(msg.poses)
        
        self.publisher.publish(self.full_path)

def main(args=None):
    rclpy.init(args=args)
    node = PathAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()