import rospy, math, sys, json, logging
from asi_msgs.msg import AsiClothoidPath, AsiClothoid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import arc

# MAKE SURE THE "ius0" matches the namespace
PLAN_TOPIC = '/ius0/planned_path'
VIS_TOPIC = '/ius0/vizplan'

# Create a default path,
radius = 10
MYPATH = arc.makeArcPath(0,0,0,[{'distance':20,'curvature':0,'speed':10},
                              {'distance':radius*math.pi/2,'curvature':1.0/radius,'speed':2},
                              {'distance':20,'curvature':0,'speed':10},
                              {'distance':radius*math.pi/2,'curvature':1.0/radius,'speed':2},
                              {'distance':20,'curvature':0,'speed':10},
                              {'distance':radius*math.pi/2,'curvature':1.0/radius,'speed':2},
                              {'distance':20,'curvature':0,'speed':10}])

try: # Try to read from calibration file specified in arguments
  with open(sys.argv[1],'r') as r:
    x = json.loads(r.read())
  if 'segments' in x and len(x['segments']) > 0:
    MYPATH = [arc.Arc(seg['start_x_m'],seg['start_y_m'],seg['start_heading_rad'],
              seg['start_curvature_inv_m'],seg['length_m'],seg['speed_mps']) for seg in x['segments']]
  elif 'viapoints' in x and len(x['viapoints']) > 0:
    viapoints = [[float(vv) for vv in v.split(',')] for v in x['viapoints'].split(';')] #[x0,y0,v0;x1,y1,v1]
    MYPATH = []
    for ii in range(1,len(viapoints)):
      vp0 = viapoints[ii-1]
      dx = viapoints[ii][0]-vp0[0]
      dy = viapoints[ii][1]-vp0[1]
      MYPATH.append(arc.Arc(vp0[0],vp0[1],math.atan2(dy,dx),0,math.hypot(dy,dx),vp0[2]))

except Exception as exc:
  logging.error('Failed to parse calibration file {}'.format(exc))
  pass

def pathplan(arcs):
  cmd = AsiClothoidPath()
  for segment in arcs:
    seg = AsiClothoid()
    seg.start_x_m = segment.start_x
    seg.start_y_m = segment.start_y
    seg.start_heading_rad = segment.start_heading
    seg.start_curvature_inv_m = segment.curvature
    seg.delta_curvature_per_length_inv_m2 = 0
    seg.length_m = segment.distance
    seg.speed_mps = segment.speed
    cmd.segments.append(seg)
  vis = Path()
  vis.header.frame_id = 'map'
  lpath = sum([a.length() for a in arcs])
  for d in range(100):
    s = arc.state(arcs,float(d)/99.0*lpath)
    ps = PoseStamped()
    ps.header.seq = d
    ps.header.frame_id = 'map'
    ps.pose.position.x = s['x']
    ps.pose.position.y = s['y']
    ps.pose.position.z = 0
    ps.pose.orientation.x = 0
    ps.pose.orientation.y = 0
    ps.pose.orientation.z = math.sin(s['heading']/2)
    ps.pose.orientation.w = math.cos(s['heading']/2)
    vis.poses.append(ps)
  return cmd,vis

cmd_pub = rospy.Publisher(PLAN_TOPIC, AsiClothoidPath, queue_size=10, latch=True)
vis_pub = rospy.Publisher(VIS_TOPIC, Path, queue_size=10, latch=True)
rospy.init_node('boring_planner', anonymous=True)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
  if MYPATH is not None:
    cmd,vis = pathplan(MYPATH)
    cmd_pub.publish(cmd)
    vis_pub.publish(vis)
    MYPATH = None

  rate.sleep()


