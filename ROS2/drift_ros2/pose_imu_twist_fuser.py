import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np

class PoseTwistIMUFuser(Node):

    def __init__(self):
        super().__init__('pose_twist_imu_fuser')

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/wamv/inekf_odom_correction/pose', self.pose_callback, 1000)
        self.twist_sub = self.create_subscription(TwistStamped, '/wamv/inekf_odom_correction/twist', self.twist_callback, 1000)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 1000)

        self.pose_pub = self.create_publisher(PoseStamped, '/gt_pose', 100)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 100)

        self.latest_pose = None
        self.latest_twist = None
        self.last_imu_orientation = None
        self.last_imu_time = None
        self.initial_orientation_inv = None  # To align IMU to +x direction

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.try_publish()

    def twist_callback(self, msg):
        self.latest_twist = msg.twist  # Extract Twist from TwistStamped
        self.try_publish()

    def imu_callback(self, msg):
        imu_q = msg.orientation
        q = [imu_q.x, imu_q.y, imu_q.z, imu_q.w]

        if self.initial_orientation_inv is None:
            self.initial_orientation_inv = tf_transformations.quaternion_inverse(q)
            self.last_imu_orientation = q
            self.last_imu_time = msg.header.stamp
            return

        corrected_orientation = tf_transformations.quaternion_multiply(
            self.initial_orientation_inv,
            q
        )

        self.current_orientation = corrected_orientation

        if self.last_imu_orientation is not None and self.last_imu_time is not None:
            now = msg.header.stamp
            dt = (now.sec + now.nanosec * 1e-9) - (self.last_imu_time.sec + self.last_imu_time.nanosec * 1e-9)

            if dt > 0:
                dq = tf_transformations.quaternion_multiply(
                    tf_transformations.quaternion_inverse(self.last_imu_orientation),
                    corrected_orientation
                )
                angle = 2 * np.arccos(np.clip(dq[3], -1.0, 1.0))
                axis = np.array(dq[0:3])
                if np.linalg.norm(axis) > 1e-6:
                    axis = axis / np.linalg.norm(axis)
                    angular_velocity = axis * angle / dt
                else:
                    angular_velocity = np.zeros(3)

                self.latest_angular_velocity = angular_velocity

        self.last_imu_orientation = corrected_orientation
        self.last_imu_time = msg.header.stamp

    def try_publish(self):
        if self.latest_pose is None or self.latest_twist is None or not hasattr(self, 'current_orientation'):
            return

        new_pose = PoseStamped()
        new_pose.header = self.latest_pose.header
        new_pose.pose = self.latest_pose.pose.pose
        new_pose.pose.orientation.x = self.current_orientation[0]
        new_pose.pose.orientation.y = self.current_orientation[1]
        new_pose.pose.orientation.z = self.current_orientation[2]
        new_pose.pose.orientation.w = self.current_orientation[3]

        self.pose_pub.publish(new_pose)

        odom = Odometry()
        odom.header = self.latest_pose.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose = new_pose.pose
        # odom.pose.covariance = new_pose.pose.covariance
        odom.twist.twist = self.latest_twist

        if hasattr(self, 'latest_angular_velocity'):
            odom.twist.twist.angular.x = self.latest_angular_velocity[0]
            odom.twist.twist.angular.y = self.latest_angular_velocity[1]
            odom.twist.twist.angular.z = self.latest_angular_velocity[2]

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = PoseTwistIMUFuser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
