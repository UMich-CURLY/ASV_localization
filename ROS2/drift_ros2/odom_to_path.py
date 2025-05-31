import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf_transformations as tf

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/wamv/sensors/position/ground_truth_odometry',  # Change this topic as needed
            self.odom_callback,
            10)
        
        self.path_publisher = self.create_publisher(Path, '/ground_truth/path', 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        self.initialized = False
        self.initial_pose = None
        
    def odom_callback(self, msg):
        if not self.initialized:
            self.initial_pose = msg.pose.pose
            self.initialized = True
            self.ref_orientation = tf.quaternion_matrix([
                self.initial_pose.orientation.x,
                self.initial_pose.orientation.y,
                self.initial_pose.orientation.z,
                self.initial_pose.orientation.w
            ])
            self.ref_orientation[:3, 3] = [
                self.initial_pose.position.x,
                self.initial_pose.position.y,
                self.initial_pose.position.z
            ]
            
        # Transform current position to reference frame
        current_pose = msg.pose.pose
        cur_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z, 1])
        inv_ref_orientation = np.linalg.inv(self.ref_orientation)
        transformed_pos = inv_ref_orientation @ cur_pos
        
        # Create PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = transformed_pos[0]
        pose_stamped.pose.position.y = transformed_pos[1]
        pose_stamped.pose.position.z = transformed_pos[2]
        pose_stamped.pose.orientation = current_pose.orientation  # Keeping raw orientation
        
        self.path_msg.poses.append(pose_stamped)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

