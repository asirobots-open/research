import rospy, math, sys, json, logging
from asi_msgs.msg import AsiClothoidPath, AsiClothoid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import clothoid, arc

# MAKE SURE THE "ius0" matches the namespace
PLAN_TOPIC = '/ius0/planned_path'
VIS_TOPIC = '/ius0/vizplan'

# Create a default path,
radius = 10
MYCLOTHOIDS = []

try: # Try to read from calibration file specified in arguments
  with open(sys.argv[1],'r') as r:
    x = json.loads(r.read())
  if 'path' in x and len(x['path']['segments']) > 0:
    MYCLOTHOIDS = x['path']['segments']
  elif 'viapoints' in x and len(x['viapoints']) > 0:
    viapoints = [[float(vv) for vv in v.split(',')] for v in x['viapoints'].split(';')] #[x0,y0,v0;x1,y1,v1]
    for ii in range(1,len(viapoints)):
      vp0 = viapoints[ii-1]
      dx = viapoints[ii][0]-vp0[0]
      dy = viapoints[ii][1]-vp0[1]
      MYCLOTHOIDS.append({'start_x_m':vp0[0],'start_y_m':vp0[1],'start_heading_rad':math.atan2(dy,dx),
                          'start_curvature_inv_m':0,'delta_curvature_per_length_inv_m2':0,
                          'length_m':math.hypot(dy,dx),'speed_mps':vp0[2]})

except Exception as exc:
  logging.error('Failed to parse calibration file {}'.format(exc))
  pass

def plotarcs(arcs):
  vis = Path()
  vis.header.frame_id = 'map'
  ii = 0
  for s in arc.linspace(arcs,200):
    ps = PoseStamped()
    ps.header.seq = ii
    ps.header.frame_id = 'map'
    ps.pose.position.x = s['x']
    ps.pose.position.y = s['y']
    ps.pose.position.z = 0
    ps.pose.orientation.x = 0
    ps.pose.orientation.y = 0
    ps.pose.orientation.z = math.sin(s['heading']/2)
    ps.pose.orientation.w = math.cos(s['heading']/2)
    vis.poses.append(ps)
    ii += 1
  return vis

def pathplan(clothoids):
  cmd = AsiClothoidPath()
  for seg in clothoids:
    c = AsiClothoid()
    c.start_x_m = seg['start_x_m']
    c.start_y_m = seg['start_y_m']
    c.start_heading_rad = seg['start_heading_rad']
    c.start_curvature_inv_m = seg['start_curvature_inv_m']
    c.delta_curvature_per_length_inv_m2 = seg['delta_curvature_per_length_inv_m2']
    c.length_m = seg['length_m']
    c.speed_mps = seg['speed_mps']
    cmd.segments.append(c)
  return cmd

cmd_pub = rospy.Publisher(PLAN_TOPIC, AsiClothoidPath, queue_size=10, latch=True)
vis_pub = rospy.Publisher(VIS_TOPIC, Path, queue_size=10, latch=True)
rospy.init_node('boring_planner', anonymous=True)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
  if MYCLOTHOIDS:
    arcs = clothoid.ASIClothoidToArcs(MYCLOTHOIDS)
    vis_pub.publish(plotarcs(arcs))
    logging.error('Publishing clothoid path containing {} clothoids!'.format(len(MYCLOTHOIDS)))
    cmd_pub.publish(pathplan(MYCLOTHOIDS))

  MYCLOTHOIDS = None
  rate.sleep()


