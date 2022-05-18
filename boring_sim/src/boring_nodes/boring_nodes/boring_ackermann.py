import math
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from rclpy.node import Node
from boring_nodes.ackermann import Ackermann
from nav_msgs.msg import Odometry
from asi_msgs.msg import AsiTBSG
from geometry_msgs.msg import TransformStamped
import transforms3d

class BoringAckermann(Node):
    def __init__(self):
        super().__init__('boring_ackermann')
        self.declare_parameter('command_topic', 'vcu_wrench')
        self.declare_parameter('odometry_topic', 'odom_topic')
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('heading', 0.0)
        self.ack = Ackermann()
        self.ack.state.x = self.get_parameter('x').get_parameter_value().double_value
        self.ack.state.y = self.get_parameter('y').get_parameter_value().double_value
        self.ack.state.yaw = self.get_parameter('heading').get_parameter_value().double_value*math.pi/180
        
        self.timer_period = 0.05  # seconds
        pqos_profile = QoSProfile(depth=10)
        sqos_profile = QoSProfile(depth=10)
        sqos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT      # .RELIABLE
        sqos_profile.history = QoSHistoryPolicy.KEEP_LAST                # .KEEP_ALL
        sqos_profile.durability = QoSDurabilityPolicy.VOLATILE           # .TRANSIENT_LOCAL

        self.simtimer = self.create_timer(self.timer_period, self.simulation)
        self.publisher_ = self.create_publisher(Odometry, odometry_topic, pqos_profile)
        self.subscriber_ = self.create_subscription(AsiTBSG, command_topic, self.cmdCb, sqos_profile)

    def simulation(self):
        self.ack.step(self.timer_period)
        odom = Odometry()
        odom.header.frame_id = "map";
        ns = self.get_namespace()
        odom.child_frame_id = "{}/base_link".format(ns if len(ns) > 1 else "")
        odom.pose.pose.position.x = self.ack.state.x
        odom.pose.pose.position.y = self.ack.state.y
        q = transforms3d.euler.euler2quat(0, 0, self.ack.state.yaw, 'sxyz')

        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.twist.twist.linear.x = self.ack.state.longitudinal_velocity
        self.publisher_.publish(odom)

    def cmdCb(self, msg):
        self.ack.command.propulsive_force = msg.throttle_cmd * self.ack.model.mass * 1 # msg.gear_cmd = -1,0,1?
        self.ack.command.resistive_force = msg.brake_cmd * self.ack.model.mass * 10
        self.ack.command.target_steer_angle = -msg.steer_cmd * self.ack.model.max_steer_angle

def main(args=None):
    rclpy.init(args=args)
    boringnode = BoringAckermann()
    rclpy.spin(boringnode)
    boringnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

