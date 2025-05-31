#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class GTTransformPublisher(Node):
    def __init__(self):
        super().__init__('gt_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/gt_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # 🟢 Use timestamp from bag file
        t.header.frame_id = 'map'
        t.child_frame_id = 'rslidar'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = GTTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

