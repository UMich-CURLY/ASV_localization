import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PoseTwistIMUFuser(Node):

    def __init__(self):
        super().__init__('pose_twist_imu_fuser')

        # Subscribers (message_filters)
        self.pose_sub = Subscriber(self, PoseWithCovarianceStamped, '/wamv/inekf_odom_correction/pose')
        self.twist_sub = Subscriber(self, TwistStamped, '/wamv/inekf_odom_correction/twist')

        self.ats = ApproximateTimeSynchronizer([self.pose_sub, self.twist_sub], queue_size=10, slop=0.05)
        self.ats.registerCallback(self.sync_callback)

        # Regular IMU subscriber
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 1000)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/gt_pose', 100)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 100)

        # State
        self.current_orientation = None
        self.latest_angular_velocity = None
        self.last_imu_orientation = None
        self.last_imu_time = None
        self.initial_orientation_inv = None  # Align IMU to forward +x

    def imu_callback(self, msg):
        imu_q = msg.orientation
        q = [imu_q.x, imu_q.y, imu_q.z, imu_q.w]

        if self.initial_orientation_inv is None:
            self.initial_orientation_inv = tf_transformations.quaternion_inverse(q)
            self.last_imu_orientation = q
            self.last_imu_time = msg.header.stamp
            return

        corrected_orientation = tf_transformations.quaternion_multiply(
            self.initial_orientation_inv, q
        )
        self.current_orientation = corrected_orientation

        # Compute angular velocity from quaternion change
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

    def sync_callback(self, pose_msg: PoseWithCovarianceStamped, twist_msg: TwistStamped):
        # Don't proceed if IMU hasn't initialized yet
        if self.current_orientation is None:
            self.get_logger().warn("Waiting for IMU orientation initialization.")
            return

        # Publish PoseStamped
        pose_out = PoseStamped()
        pose_out.header = pose_msg.header
        pose_out.pose = pose_msg.pose.pose
        pose_out.pose.orientation.x = self.current_orientation[0]
        pose_out.pose.orientation.y = self.current_orientation[1]
        pose_out.pose.orientation.z = self.current_orientation[2]
        pose_out.pose.orientation.w = self.current_orientation[3]
        self.pose_pub.publish(pose_out)

        # Publish Odometry
        odom = Odometry()
        odom.header = pose_msg.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose_out.pose
        odom.twist.twist = twist_msg.twist

        if self.latest_angular_velocity is not None:
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

