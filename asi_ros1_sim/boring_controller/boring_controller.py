import rospy, math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from asi_msgs.msg import AsiTBSG
from nav_msgs.msg import OccupancyGrid

global TERRAIN
TERRAIN = None
GOAL = {'x':100,'y':0}

# MAKE SURE THE "ius0" matches the namespace
COMMAND_TOPIC = '/ius0/vcu_wrench'
TERRAIN_TOPIC = '/ius0/terrain_cost'
ODOMETRY_TOPIC = '/ius0/odom_topic'

rospy.init_node('boring_controller', anonymous=True)
cmd_pub = rospy.Publisher(COMMAND_TOPIC, AsiTBSG, queue_size=10)

def terrainCb(msg):
  global TERRAIN
  TERRAIN = msg

def odometryCb(msg):
    if TERRAIN:
      pass # DO something with the terrain

    o_q = msg.pose.pose.orientation
    pos = msg.pose.pose.position
    euler = euler_from_quaternion([o_q.x, o_q.y, o_q.z, o_q.w])
    desired_heading = math.atan2(GOAL['y'] - pos.y,GOAL['x'] - pos.x)
    steer_gain = 0.1

    msg = AsiTBSG()
    msg.throttle_cmd = 0.3
    msg.brake_cmd = 0
    msg.steer_cmd = steer_gain * (((desired_heading - euler[2]) % (2*math.pi)) - math.pi)
    msg.gear_cmd = 1
    cmd_pub.publish(msg)


rospy.Subscriber(TERRAIN_TOPIC,OccupancyGrid,terrainCb)
rospy.Subscriber(ODOMETRY_TOPIC,Odometry,odometryCb)
rospy.spin()

