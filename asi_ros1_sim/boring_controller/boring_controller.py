import rospy, math, logging, sys
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from asi_msgs.msg import AsiTBSG
from nav_msgs.msg import OccupancyGrid
from asi_msgs.msg import AsiClothoidPath, AsiClothoid

import arc
global TERRAIN, PATH
TERRAIN = None
PATH = None

# MAKE SURE THE "ius0" matches the namespace
COMMAND_TOPIC = '/ius0/vcu_wrench'
TERRAIN_TOPIC = '/ius0/terrain_cost'
ODOMETRY_TOPIC = '/ius0/odom_topic'
PLAN_TOPIC = '/ius0/planned_path'

rospy.init_node('boring_controller', anonymous=True)
cmd_pub = rospy.Publisher(COMMAND_TOPIC, AsiTBSG, queue_size=10)

def planCb(msg):
  global PATH
  pth = []
  for seg in msg.segments:
    pth.append(arc.Arc(seg.start_x_m, seg.start_y_m, seg.start_heading_rad, 
      seg.start_curvature_inv_m, seg.length_m, seg.speed_mps))
  PATH = pth
  logging.warning('RECEIVED path {}'.format(PATH))

def terrainCb(msg):
  global TERRAIN
  TERRAIN = msg

global count
count = 0
def odometryCb(msg):
    global count
    count += 1
    steer_gain = 1
    pitch_gain = 1
    velocity_gain = 0.05
    if TERRAIN:
      pass # DO something with the terrain
    if PATH is None: # No plan to drive to
      return

    o_q = msg.pose.pose.orientation
    pos = msg.pose.pose.position
    euler = euler_from_quaternion([o_q.x, o_q.y, o_q.z, o_q.w])
    roll = euler[0]
    pitch = euler[1]
    heading = euler[2]

    pathl,finished = arc.getNearestPointAndPop(PATH,pos.x,pos.y)
    lookahead = pathl + 3*abs(msg.twist.twist.linear.x)
    nearest = arc.state(PATH,pathl)
    off_path_error =  (pos.x-nearest['x'])*math.sin(nearest['heading']) - \
                      (pos.y-nearest['y'])*math.cos(nearest['heading'])

    goal = arc.state(PATH, lookahead)
    desired_heading = math.atan2(goal['y'] - pos.y,goal['x'] - pos.x)

    cmd = AsiTBSG()
    prop = velocity_gain * (goal['speed'] - msg.twist.twist.linear.x) + pitch_gain*pitch 
    if finished:
      cmd.throttle_cmd = 0
      cmd.brake_cmd = 1
    else:
      cmd.throttle_cmd = prop if prop > 0 else 0
      cmd.brake_cmd = -10*prop if prop < 0 else 0
    
    cmd.steer_cmd = -steer_gain * (((desired_heading - heading + math.pi) % (2*math.pi)) - math.pi)
    cmd.gear_cmd = 1
    cmd_pub.publish(cmd)
    if (count % 20) == 0:
      print('off_path_error = '+str(round(off_path_error,2))+
        '; x = '+str(round(pos.x,1))+'; y = '+str(round(pos.y,1))+'; z = '+str(round(pos.z,1))+
        '; roll = '+str(round(roll*180/3.14,1))+
        '; pitch = '+str(round(pitch*180/3.14,1))+
        '; yaw = '+str(round(heading*180/3.14,1)))
      sys.stdout.flush()
rospy.Subscriber(PLAN_TOPIC,AsiClothoidPath,planCb)
rospy.Subscriber(TERRAIN_TOPIC,OccupancyGrid,terrainCb)
rospy.Subscriber(ODOMETRY_TOPIC,Odometry,odometryCb)
rospy.spin()

