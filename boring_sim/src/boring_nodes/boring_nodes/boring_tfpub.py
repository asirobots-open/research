import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
import tf_transformations
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class BoringTfPub(Node):
    def __init__(self):
        super().__init__('boring_tfpub')
        self.declare_parameter('odometry_topic', 'odom_topic')
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        qos_profile = 10 #QoSProfile(depth=1)
        # qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT      # .RELIABLE
        # qos_profile.history = QoSHistoryPolicy.KEEP_LAST                # .KEEP_ALL
        # qos_profile.durability = QoSDurabilityPolicy.VOLATILE           # .TRANSIENT_LOCAL

        self.odom_sub = self.create_subscription(Odometry, odometry_topic, self.odometryCb, qos_profile)
        self.tfbroadcaster_ = TransformBroadcaster(self)

    def odometryCb(self, msg):
        ns = self.get_namespace()
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = "map"
        odom_trans.child_frame_id = "{}/base_link".format(ns if len(ns) > 1 else "")
        odom_trans.transform.translation.x = msg.pose.pose.position.x
        odom_trans.transform.translation.y = msg.pose.pose.position.y
        odom_trans.transform.translation.z = msg.pose.pose.position.z
        odom_trans.transform.rotation = msg.pose.pose.orientation
        # o_q = msg.pose.pose.orientation
        # euler = tf_transformations.euler_from_quaternion([o_q.x, o_q.y, o_q.z, o_q.w])
        # q = tf_transformations.quaternion_from_euler(0, 0, math.pi/2-euler[2])
        # odom_trans.transform.rotation.x = q[0]
        # odom_trans.transform.rotation.y = q[1]
        # odom_trans.transform.rotation.z = q[2]
        # odom_trans.transform.rotation.w = q[3]

        self.tfbroadcaster_.sendTransform(odom_trans)


def main(args=None):
    rclpy.init(args=args)
    boringnode = BoringTfPub()
    rclpy.spin(boringnode)
    boringnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

