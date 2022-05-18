import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

has_broadcaster = True
try:
    from tf2_ros import TransformBroadcaster
except Exception as exc:
    from tf2_msgs.msg import TFMessage
    has_broadcaster = False

class BoringTfPub(Node):
    def __init__(self):
        super().__init__('boring_tfpub')
        self.declare_parameter('odometry_topic', 'odom_topic')
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        pqos_profile = QoSProfile(depth=10)
        sqos_profile = QoSProfile(depth=10)
        sqos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT      # .RELIABLE
        sqos_profile.history = QoSHistoryPolicy.KEEP_LAST                # .KEEP_ALL
        sqos_profile.durability = QoSDurabilityPolicy.VOLATILE           # .TRANSIENT_LOCAL

        self.odom_sub = self.create_subscription(Odometry, odometry_topic, self.odometryCb, sqos_profile)
        if has_broadcaster:
            self.tfbroadcaster_ = TransformBroadcaster(self)
        else:
            self.tfpublisher_ = self.create_publisher(msg_type=TFMessage, topic="/tf")

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
        if has_broadcaster:
            self.tfbroadcaster_.sendTransform(odom_trans)
        else:
            tf_msg = TFMessage()
            tf_msg.transforms = [odom_trans]
            self.tfpublisher_.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    boringnode = BoringTfPub()
    rclpy.spin(boringnode)
    boringnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

