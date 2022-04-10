import rclpy
from rclpy.node import Node
import math, json
from asi_msgs.msg import AsiClothoidPath, AsiClothoid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from boring_nodes import clothoid, arc

class BoringPlanner(Node):
    def __init__(self):
        super().__init__('boring_planner')
        latching_qos = rclpy.qos.QoSProfile(depth=1,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)        

        self.declare_parameter('plan_topic', 'planned_path')
        self.declare_parameter('viz_topic', 'vizplan')
        self.declare_parameter('calibration_file', '')
        plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value
        viz_topic = self.get_parameter('viz_topic').get_parameter_value().string_value
        planpth = self.get_parameter('calibration_file').get_parameter_value().string_value

        self.radius = 10
        self.MYCLOTHOIDS = None
        self.default(planpth)
        self.cmd_pub = self.create_publisher(AsiClothoidPath, plan_topic, qos_profile=latching_qos)
        self.vis_pub = self.create_publisher(Path, viz_topic, qos_profile=latching_qos)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def default(self,planpth):
        # Create a default path,
        try: # Try to read from calibration file specified in arguments
          with open(planpth,'r') as r:
            x = json.loads(r.read())
          if 'path' in x and len(x['path']['segments']) > 0:
            self.MYCLOTHOIDS = x['path']['segments']
          elif 'viapoints' in x and len(x['viapoints']) > 0:
            viapoints = [[float(vv) for vv in v.split(',')] for v in x['viapoints'].split(';')] #[x0,y0,v0;x1,y1,v1]
            for ii in range(1,len(viapoints)):
              vp0 = viapoints[ii-1]
              dx = viapoints[ii][0]-vp0[0]
              dy = viapoints[ii][1]-vp0[1]
              self.MYCLOTHOIDS.append({'start_x_m':vp0[0],'start_y_m':vp0[1],'start_heading_rad':math.atan2(dy,dx),
                                  'start_curvature_inv_m':0,'delta_curvature_per_length_inv_m2':0,
                                  'length_m':math.hypot(dy,dx),'speed_mps':vp0[2]})
        except Exception as exc:
          self.get_logger().info('Failed to parse calibration file {}'.format(exc))

    def pathplan(self, clothoids):
      cmd = AsiClothoidPath()
      for seg in clothoids:
        c = AsiClothoid()
        c.start_x_m = float(seg['start_x_m'])
        c.start_y_m = float(seg['start_y_m'])
        c.start_heading_rad = float(seg['start_heading_rad'])
        c.start_curvature_inv_m = float(seg['start_curvature_inv_m'])
        c.delta_curvature_per_length_inv_m2 = float(seg['delta_curvature_per_length_inv_m2'])
        c.length_m = float(seg['length_m'])
        c.speed_mps = float(seg['speed_mps'])
        cmd.segments.append(c)
      return cmd

    def plotarcs(self, arcs):
      vis = Path()
      vis.header.frame_id = 'map'
      for s in arc.linspace(arcs,200):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(s['x'])
        ps.pose.position.y = float(s['y'])
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = math.sin(s['heading']/2)
        ps.pose.orientation.w = math.cos(s['heading']/2)
        vis.poses.append(ps)
      return vis

    def timer_callback(self):
        if self.MYCLOTHOIDS:
            arcs = clothoid.ASIClothoidToArcs(self.MYCLOTHOIDS)
            self.vis_pub.publish(self.plotarcs(arcs))
            self.get_logger().info('Publishing clothoid path containing {} clothoids!'.format(len(self.MYCLOTHOIDS)))
            self.cmd_pub.publish(self.pathplan(self.MYCLOTHOIDS))
        self.MYCLOTHOIDS = None

def main(args=None):
    rclpy.init(args=args)
    boringnode = BoringPlanner()
    rclpy.spin(boringnode)
    boringnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

