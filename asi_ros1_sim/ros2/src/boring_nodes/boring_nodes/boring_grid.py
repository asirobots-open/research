import math, json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import tf_transformations

class BoringGrid(Node):
    def __init__(self):
        super().__init__('boring_grid')
        self.circles = []
        self.shapes = []
        self.empty_value = 127 # ? chr(255)
        self.occupied_value = 0 # ? chr(0)
        self.declare_parameter('terrain_topic', 'terrain_cost')
        self.declare_parameter('odometry_topic', 'odom_topic')
        terrain_topic = self.get_parameter('terrain_topic').get_parameter_value().string_value
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(OccupancyGrid, terrain_topic, 10)        
        self.odom_sub = self.create_subscription(Odometry, odometry_topic, self.odometryCb, 10)

        self.declare_parameter('calibration_file', '')
        self.read_calibration( self.get_parameter('calibration_file').get_parameter_value().string_value )

        self.declare_parameter('republish_period_sec', 0.1)
        timer_period = self.get_parameter('republish_period_sec').get_parameter_value().double_value
        self.simtimer = self.create_timer(timer_period, self.publish)
        self.new_odometry = False

        self.declare_parameter('inertial_frame', False)
        self.declare_parameter('front_cells', 100)
        self.declare_parameter('width_cells', 100)
        self.declare_parameter('back_cells', 10)
        self.declare_parameter('grid_resolution', 0.25)
        self.xpos = 0
        self.ypos = 0
        self.heading = 0
        self.inertial_frame = self.get_parameter('inertial_frame').get_parameter_value().bool_value
        front_cells = max(10,self.get_parameter('front_cells').get_parameter_value().integer_value)
        width_cells = max(10,self.get_parameter('width_cells').get_parameter_value().integer_value)
        back_cells = max(0,self.get_parameter('back_cells').get_parameter_value().integer_value)
        ns = self.get_namespace()
        self.occ_grid = OccupancyGrid()
        self.occ_grid.header.frame_id = "/map" if self.inertial_frame else "{}/base_link".format(ns if len(ns) > 1 else "")
        self.occ_grid.info.resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.occ_grid.info.width = front_cells + back_cells
        self.occ_grid.info.height = width_cells
        self.grid_origin_x = -self.occ_grid.info.resolution*back_cells
        self.grid_origin_y = -self.occ_grid.info.resolution*width_cells/2
        self.update_position(0,0)
        self.occ_grid.data =  [self.empty_value] * self.occ_grid.info.width * self.occ_grid.info.height

    def update_position(self,x,y):
        self.occ_grid.info.origin.position.x = x + self.grid_origin_x
        self.occ_grid.info.origin.position.y = y + self.grid_origin_y

    def read_calibration(self,calibration_file):
        # Create a default path,
        try: # Try to read from calibration file specified in arguments
          with open(calibration_file,'r') as r:
            x = json.loads(r.read())
            try:
                for obst in x.get('obstacles','').strip().split(';'):
                    prms = [float(p) for p in obst.strip().split(',')]
                    if len(prms) == 3:
                        self.circles.append({'x':prms[0],'y':prms[1],'r':prms[2]})
            except Exception as exc:
                print(exc)
            self.shapes = x.get('map',{}).get('shapes',[])
        except Exception as exc:
          self.get_logger().info('Failed to parse calibration file {}'.format(exc))

    def publish(self):
        if self.new_odometry:
            self.occ_grid.data =  [self.empty_value] * self.occ_grid.info.width * self.occ_grid.info.height
            cq = math.cos(-self.heading)
            sq = math.sin(-self.heading)
            for circle in self.circles:
                xx = circle['x'] - self.xpos
                yy = circle['y'] - self.ypos
                x = cq*xx - sq*yy - self.grid_origin_x
                y = sq*xx + cq*yy - self.grid_origin_y
                self.add_with_radius(x, y, circle['r'])

            for shp in self.shapes:
                if len(shp['vertices']) < 2: continue
                lastv = self.grid_coordinates(shp['vertices'][0]['x'], shp['vertices'][0]['y'], cq, sq) 
                for ii in range(1,len(shp['vertices'])):
                    thisv = self.grid_coordinates(shp['vertices'][ii]['x'], shp['vertices'][ii]['y'], cq, sq); 
                    self.add_segment_to_grid(thisv, lastv, self.occ_grid.info.resolution)
                    lastv = thisv

            self.publisher_.publish(self.occ_grid)
            self.new_odometry = False

    def odometryCb(self, msg):
        o_q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([o_q.x, o_q.y, o_q.z, o_q.w])
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        if self.inertial_frame:
            self.update_position(self.xpos, self.ypos)
            self.heading = 0
        else:
            self.heading = euler[2]
        self.new_odometry = True

    def add_with_radius(self, x, y, r):
        c_center = int(x/self.occ_grid.info.resolution)
        r_center = int(y/self.occ_grid.info.resolution)
        ri = int(r/self.occ_grid.info.resolution)
        cmin = max(0,                           c_center - ri)
        cmax = min(self.occ_grid.info.width-1,  c_center + ri)
        rmin = max(0,                           r_center - ri)
        rmax = min(self.occ_grid.info.height-1, r_center + ri)
        for c in range(cmin, cmax+1):
            for r in range(rmin, rmax+1):
                d = math.hypot(c-c_center,r-r_center)
                if d <= ri:
                    self.occ_grid.data[r*self.occ_grid.info.width+c] = self.occupied_value

    def getIntersection(self, x1, y1, x2, y2, x3, y3, x4, y4):
        #double &x1, double &y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        # Check if the segment from point (x1,y1) to (x2,y2) intersects with the segment from point (x3,y3) to (x4,y4)
        # returns true if it does and also moves x1 y1 to the intersection point
        # x1 + u*(x2-x1) = x3 + v*(x4-x3)
        # y1 + u*(y2-y1) = y3 + v*(y4-y3)
        dx43 = x4 - x3
        dy43 = y4 - y3
        dx12 = x1 - x2
        dy12 = y1 - y2
        dx13 = x1 - x3
        dy13 = y1 - y3
        # dx13 = u*dx12 + v*dx43
        # dy13 = u*dy12 + v*dy43
        det = dx12*dy43 - dy12*dx43
        if det != 0:
            u = ( dy43*dx13 - dx43*dy13)/det
            v = (-dy12*dx13 + dx12*dy13)/det
            if ((u <= 1 and u >= 0) and (v <= 1 and v >= 0)):
                x1 += u*(x2 - x1)
                y1 += u*(y2 - y1)
                return (True,x1,y1)
        return False,x1,y1

    def checkRectangleIntersection(self, x1, y1, x2, y2, xmax, ymax):
        (s,x,y) = self.getIntersection(x1, y1, x2, y2, 0,       0, xmax,    0); 
        if (s): return (True,x,y)
        (s,x,y) = self.getIntersection(x1, y1, x2, y2, xmax,    0, xmax, ymax); 
        if (s): return (True,x,y)
        (s,x,y) = self.getIntersection(x1, y1, x2, y2, xmax, ymax,    0, ymax); 
        if (s): return (True,x,y)
        (s,x,y) = self.getIntersection(x1, y1, x2, y2, 0,    ymax,    0,    0); 
        if (s): return (True,x,y)
        return (False,x1,y1)

    def checkDoubleRectangleIntersection(self, pts, xmax, ymax):
        xlst = [0,0,xmax,xmax]
        ylst = [0,ymax,ymax,0]
        x1temp=0
        y1temp=0
        found_one = False
        for ii in range(4):
            (s,x1_,y1_) = self.getIntersection(pts['x1'], pts['y1'], pts['x2'], pts['y2'], xlst[ii],  ylst[ii], xlst[(ii+1) % 4], ylst[(ii+1) % 4])
            if s:
                if not found_one:
                    x1temp = x1_
                    y1temp = y1_
                    found_one = True
                else:
                    pts['x1'] = x1temp
                    pts['y1'] = y1temp
                    pts['x2'] = x1_
                    pts['y2'] = y1_
                    return True
        return False

    def in_grid(self,pts):
        xmax = self.occ_grid.info.width*self.occ_grid.info.resolution
        ymax = self.occ_grid.info.height*self.occ_grid.info.resolution
        x1_is_inside = (pts['x1'] >= 0 and pts['x1'] <= xmax) and (pts['y1'] >= 0 and pts['y1'] <= ymax)
        x2_is_inside = (pts['x2'] >= 0 and pts['x2'] <= xmax) and (pts['y2'] >= 0 and pts['y2'] <= ymax)
        pts['nsteps'] = 0
        segment_is_in_grid = False
        if (x1_is_inside and x2_is_inside):
            segment_is_in_grid = True
        elif (x1_is_inside):
            segment_is_in_grid,pts['x2'],pts['y2'] = self.checkRectangleIntersection(pts['x2'], pts['y2'], pts['x1'], pts['y1'], xmax, ymax)
        elif (x2_is_inside):
            segment_is_in_grid,pts['x1'],pts['y1'] = self.checkRectangleIntersection(pts['x1'], pts['y1'], pts['x2'], pts['y2'], xmax, ymax)
        else:
            segment_is_in_grid = self.checkDoubleRectangleIntersection(pts, xmax, ymax)

        if segment_is_in_grid:
            pts['dx'] = pts['x2']-pts['x1']
            pts['dy'] = pts['y2']-pts['y1']
            det = math.hypot(pts['dx'],pts['dy'])
            if det == 0: return
            delta = self.occ_grid.info.resolution/det
            pts['dx'] *= delta
            pts['dy'] *= delta
            pts['nsteps'] = 0 if (delta == 0) else 1.0/delta

    def add_segment_to_grid(self, p1, p2, radius):
        pts = {'x1':p1['x'],'y1':p1['y'],'x2':p2['x'],'y2':p2['y'],'dx':0,'dy':0,'nsteps':0}
        self.in_grid(pts)
        for jj in range(int(pts['nsteps'])):
            self.add_with_radius(pts['x1'] + pts['dx']*jj, pts['y1'] + pts['dy']*jj, radius)

    def grid_coordinates(self, x, y, cq, sq):
        dx = x-self.xpos
        dy = y-self.ypos
        return {
            'x':cq*dx - sq*dy - self.grid_origin_x,
            'y':sq*dx + cq*dy - self.grid_origin_y
        }


def main(args=None):
    rclpy.init(args=args)
    boringnode = BoringGrid()
    rclpy.spin(boringnode)
    boringnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

