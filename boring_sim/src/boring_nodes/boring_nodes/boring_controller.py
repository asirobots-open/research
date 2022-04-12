import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from boring_nodes import clothoid, arc
from nav_msgs.msg import Odometry
from asi_msgs.msg import AsiTBSG
from nav_msgs.msg import OccupancyGrid
from asi_msgs.msg import AsiClothoidPath
import tf_transformations

def unwrap(dq):
  return ((dq+math.pi)%(2*math.pi))-math.pi

class BoringController(Node):
    def __init__(self):
        super().__init__('boring_controller')
        self.TERRAIN = None
        self.PATH = None
        self.count = 0

        self.declare_parameter('command_topic', 'vcu_wrench')
        self.declare_parameter('plan_topic', 'planned_path')
        self.declare_parameter('odometry_topic', 'odom_topic')
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value

        qos_profile = 10 #QoSProfile(depth=10)
        # qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT      # .RELIABLE
        # qos_profile.history = QoSHistoryPolicy.KEEP_LAST                # .KEEP_ALL
        # qos_profile.durability = QoSDurabilityPolicy.VOLATILE           # .TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(AsiTBSG, command_topic, qos_profile)
        self.plan_sub = self.create_subscription(AsiClothoidPath, plan_topic, self.planCb, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, odometry_topic, self.odometryCb, qos_profile)

    def planCb(self, msg):
        self.PATH = []
        for seg in msg.segments:
          arcs = clothoid.toArcs(seg.start_x_m, seg.start_y_m, seg.start_heading_rad, 
            seg.start_curvature_inv_m, seg.delta_curvature_per_length_inv_m2, seg.length_m, seg.speed_mps)
          self.PATH += arcs
        self.get_logger().info('RECEIVED path with {} segments'.format(len(self.PATH)))

    def terrainCb(self, msg):
        self.TERRAIN = msg

    def odometryCb(self, msg):
        self.count += 1
        steer_pgain = 1
        steer_dgain = 4
        pitch_gain = 1
        velocity_gain = 0.005
        lookahead_time_sec = 0.5
        wheel_base = 4
        max_steer_angle = 0.785

        if self.TERRAIN:
          pass # DO something with the terrain
        if self.PATH is None: # No plan to drive to
          return

        o_q = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        euler = tf_transformations.euler_from_quaternion([o_q.x, o_q.y, o_q.z, o_q.w])
        roll = euler[0]
        pitch = euler[1]
        heading = euler[2]
        velocity = msg.twist.twist.linear.x
        vabs = max(2,min(200,abs(velocity)))

        pathl,finished = arc.getNearestPointAndPop(self.PATH,pos.x,pos.y)
        lookahead = pathl + lookahead_time_sec*vabs
        nearest = arc.state(self.PATH,pathl)
        off_path_error =  (pos.x-nearest['x'])*math.sin(nearest['heading']) - \
                          (pos.y-nearest['y'])*math.cos(nearest['heading'])
        heading_error = unwrap(nearest['heading'] - heading)
        goal = arc.state(self.PATH, lookahead)
        kcmd = goal['curvature'] + steer_pgain/vabs/vabs*off_path_error + steer_dgain/vabs*heading_error;

        prop = velocity_gain * (goal['speed'] - velocity) + pitch_gain*pitch 

        cmd = AsiTBSG()
        if finished:
          cmd.throttle_cmd = 0.0
          cmd.brake_cmd = 1.0
        else:
          cmd.throttle_cmd = prop if prop > 0 else 0.0
          cmd.brake_cmd = -10*prop if prop < 0 else 0.0
        
        cmd.steer_cmd = -math.atan(kcmd*wheel_base)/max_steer_angle
        # cmd.steer_cmd = -steer_gain * (((desired_heading - heading + math.pi) % (2*math.pi)) - math.pi)
        cmd.gear_cmd = 1
        self.publisher_.publish(cmd)
        if (self.count % 20) == 0:
          self.get_logger().info('off_path_error = '+str(round(off_path_error,2))+'; heading_error = '+str(round(heading_error,2))+
            '; pos=['+str(round(pos.x,1))+', '+str(round(pos.y,1))+', '+str(round(pos.z,1))+
            '];  bodyvelocity = '+str(round(velocity,2))+
            '; throttle = '+str(round(cmd.throttle_cmd,3))+
            '; brake = '+str(round(cmd.brake_cmd,3))+
            '; steer = '+str(round(cmd.steer_cmd,2))+
            '; roll = '+str(round(roll*180/3.14,1))+
            '; pitch = '+str(round(pitch*180/3.14,1))+
            '; yaw = '+str(round(heading*180/3.14,1)))

def main(args=None):
    rclpy.init(args=args)
    boringnode = BoringController()
    rclpy.spin(boringnode)
    boringnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


