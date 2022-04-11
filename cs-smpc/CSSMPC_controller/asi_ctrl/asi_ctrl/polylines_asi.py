#! /usr/bin/env python3
import os
import numpy as np
#import matplotlib.pyplot as plt
import time
import sys
import copy
import rclpy
import rclpy.node
import scipy.interpolate
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
# from autorally_msgs.msg import wheelSpeeds
from scipy.spatial.transform import Rotation
from asi_msgs.msg import MapCA
from asi_msgs.msg import AsiClothoidPath
from asi_msgs.msg import AsiClothoid
from asi_msgs.msg import MapBounds
import rospkg


class Map_CA(rclpy.node.Node):

    def __init__(self):
        super().__init__('map_ca')
        latching_qos = rclpy.qos.QoSProfile(depth=1,
                                            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(AsiClothoidPath, "/ius0/planned_path", self.path_cb, latching_qos)
        self.path_pub = self.create_publisher(Path, "/smoothed_path", latching_qos)
        self.declare_parameter('num_smoothed_path_points', 500)
        self.num_smoothed_path_points =  self.get_parameter('num_smoothed_path_points').get_parameter_value().integer_value
        self.declare_parameter('width_cells', 120)
        self.width_cells = self.get_parameter('width_cells').get_parameter_value().integer_value
        self.declare_parameter('front_cells', 60)
        self.front_cells = self.get_parameter('front_cells').get_parameter_value().integer_value
        self.declare_parameter('back_cells', 120)
        self.back_cells = self.get_parameter('back_cells').get_parameter_value().integer_value
        self.declare_parameter('grid_resolution', 0.25)
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.declare_parameter('inertial_frame', False)
        self.inertial_frame = self.get_parameter('inertial_frame').get_parameter_value().bool_value
        self.declare_parameter('bound_celing', 20.0)
        self.bound_celing = self.get_parameter('bound_celing').get_parameter_value().double_value
        self.declare_parameter('bound_length', 25.0)
        self.bound_length = self.get_parameter('bound_length').get_parameter_value().double_value
        self.declare_parameter('bound_stride', 1.0)
        self.bound_stride = self.get_parameter('bound_stride').get_parameter_value().double_value

        # rospack = rospkg.RosPack()
        # # package_path = rospack.get_path('autorally_private_control')
        # package_path = '/home/user/catkin_ws/src/autorally_private/autorally_private_control'
        # file_name = 'CCRF_2021-01-10.npz'
        # track_dict = np.load(package_path + '/src/maps/CCRF/' + file_name)
        # p_x = track_dict['X_cen_smooth']
        # p_y = track_dict['Y_cen_smooth']
        # file_name = 'ccrf_track_optimal_1000s_15m.npz'
        # track_dict = np.load(package_path + '/src/maps/CCRF/' + file_name)
        # try:
        #     p_x = track_dict['X_cen_smooth']
        #     p_y = track_dict['Y_cen_smooth']
        # except KeyError:
        #     p_x = track_dict['pts'][:, 0]
        #     p_y = track_dict['pts'][:, 1]
        #     self.rho = track_dict['curvature']
        # self.p = np.array([p_x, p_y])
        # dif_vecs = self.p[:, 1:] - self.p[:, :-1]
        # self.dif_vecs = dif_vecs
        # self.slopes = dif_vecs[1, :] / dif_vecs[0, :]
        # self.midpoints = self.p[:, :-1] + dif_vecs/2
        # self.s = np.cumsum(np.linalg.norm(dif_vecs, axis=0))
        self.p = np.empty(())
        self.dif_vecs = np.empty(())
        self.slopes = np.empty(())
        self.midpoints = np.empty(())
        self.s = np.empty(())
        self.speeds = np.empty(())
        self.curvatures = np.empty(())
        self.mapca = MapCA()
        self.start = False

        self.wf = 0.1
        self.wr = 0.1

        # plt.plot(p_x, p_y, '.-')
        # plt.plot(self.midpoints[0], self.midpoints[1], 'x')
        # plt.show()

        self.create_subscription(Odometry, "/ius0/odom_topic", self.odom_cb, 1)
        # rospy.Subscriber("/wheelSpeeds", wheelSpeeds, self.wheel_cb)
        self.mapca_pub = self.create_publisher(MapCA, '/MAP_CA/mapCA', 10)
        self.create_subscription(OccupancyGrid, "/ius0/terrain_cost", self.obstacle_callback, 1)
        self.boundary_pub = self.create_publisher(Path, "/boundaries", 10)
        self.bounds_array_pub = self.create_publisher(MapBounds, "/bounds_array", 2)

    def localize(self, M, psi):
        # dists = np.linalg.norm(np.subtract(M.reshape((-1,1)), self.p), axis=0)
        # mini = np.argmin(dists)
        # p0 = self.p[:, mini]
        # p1 = p0 + self.p_length[0, mini] * np.cos(self.p_theta[0, mini]) + self.p_length[0, mini] * np.sin(self.p_theta[0 ,mini])
        # # plt.plot(M[0], M[1], 'x')
        # # plt.plot(p0[0], p0[1], 'o')
        # # plt.plot(p1[0], p1[1], 'o')
        # # ortho = -1/(self.slopes[mini] + 1e-8)
        # # a = M[1] - ortho * M[0]
        # # a_0 = p0[1] - ortho*p0[0]
        # # a_1 = p1[1] - ortho*p1[0]
        # printi=0
        # if 1:#a_0 < a < a_1 or a_1 < a < a_0:
        #     norm_dist = np.sign(np.cross(p1 - p0, M - p0)) * np.linalg.norm(np.cross(p1 - p0, M - p0)) / np.linalg.norm(p1 - p0)
        #     s_dist = np.linalg.norm(np.dot(M-p0, p1-p0))
        # else:
        #     printi=1
        #     norm_dist = np.sign(np.cross(p1 - p0, M - p0)) * np.linalg.norm(M - p0)
        #     s_dist = 0
        # # if norm_dist > 0.5:
        # #     print('here')
        # # s_dist += self.s[mini]
        # # head_dist = psi - np.arctan2(self.dif_vecs[1, mini], self.dif_vecs[0, mini])
        # head_dist = psi - self.p_theta[0, mini]
        # if head_dist > np.pi:
        #     # print(psi, np.arctan2(self.dif_vecs[1, mini], self.dif_vecs[0, mini]))
        #     head_dist -= 2*np.pi
        #     print(norm_dist, s_dist, head_dist * 180 / np.pi)
        # elif head_dist < -np.pi:
        #     head_dist += 2*np.pi
        #     print(norm_dist, s_dist, head_dist * 180 / np.pi)
        # # if printi:
        # #     print(norm_dist, s_dist, head_dist*180/np.pi)
        # #     printi=0
        # # plt.show()
        # return head_dist, norm_dist, s_dist
        dists = np.linalg.norm(np.subtract(M.reshape((-1, 1)), self.midpoints), axis=0)
        mini = np.argmin(dists)
        p0 = self.p[:, mini]
        p1 = self.p[:, mini + 1]
        # plt.plot(M[0], M[1], 'x')
        # plt.plot(p0[0], p0[1], 'o')
        # plt.plot(p1[0], p1[1], 'o')
        ortho = -1 / self.slopes[mini]
        a = M[1] - ortho * M[0]
        a_0 = p0[1] - ortho * p0[0]
        a_1 = p1[1] - ortho * p1[0]
        printi = 0
        if 1:  # a_0 < a < a_1 or a_1 < a < a_0:
            norm_dist = np.sign(np.cross(p1 - p0, M - p0)) * np.linalg.norm(np.cross(p1 - p0, M - p0)) / np.linalg.norm(
                p1 - p0)
            s_dist = np.dot(M-p0, p1-p0) / np.linalg.norm(p1-p0)
        else:
            printi = 1
            norm_dist = np.sign(np.cross(p1 - p0, M - p0)) * np.linalg.norm(M - p0)
            s_dist = 0
        # if norm_dist > 0.5:
        #     print('here')
        s_dist += self.s[mini]
        head_dist = psi - np.arctan2(self.dif_vecs[1, mini], self.dif_vecs[0, mini])
        if head_dist > np.pi:
            # print(psi, np.arctan2(self.dif_vecs[1, mini], self.dif_vecs[0, mini]))
            head_dist -= 2 * np.pi
            print(norm_dist, s_dist, head_dist * 180 / np.pi)
        elif head_dist < -np.pi:
            head_dist += 2 * np.pi
            print(norm_dist, s_dist, head_dist * 180 / np.pi)
        # if printi:
        #     print(norm_dist, s_dist, head_dist*180/np.pi)
        #     printi=0
        # plt.show()
        speed = self.speeds[mini]
        return head_dist, norm_dist, s_dist, speed

    def odom_cb(self, odom):
        if not self.start:
            return
        # self.get_logger().info('odom received')
        # start = time.time()
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        r = Rotation.from_quat([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        yaw, pitch, roll = r.as_euler('zyx', degrees=False)
        # vx = odom.twist.twist.linear.x * np.cos(yaw) + odom.twist.twist.linear.y * np.sin(yaw)
        # vy = odom.twist.twist.linear.x * -np.sin(yaw) + odom.twist.twist.linear.y * np.cos(yaw)
        # for whatever reason map_msg y velocity seems inverted so apply -
        vx = odom.twist.twist.linear.x #* np.cos(yaw) - odom.twist.twist.linear.y * np.sin(yaw)
        # self.get_logger().info('vxs: ' + str(odom.twist.twist.linear.x) + ' ' + str(vx))
        vy = odom.twist.twist.linear.y # odom.twist.twist.linear.x * -np.sin(yaw) - odom.twist.twist.linear.y * np.cos(yaw)
        wz = odom.twist.twist.angular.z

        d_psi, n, s, speed = self.localize(np.asarray([x, y]), yaw)
        self.mapca.header.stamp = self.get_clock().now().to_msg()
        self.mapca.vx = vx
        self.mapca.vy = vy
        self.mapca.wz = wz
        self.mapca.wf = self.wf
        self.mapca.wr = self.wr
        self.mapca.s = s
        self.mapca.ey = n
        self.mapca.epsi = d_psi
        self.mapca.x = x
        self.mapca.y = y
        self.mapca.yaw = yaw
        self.mapca.path_s = s
        self.mapca.speed = speed
        self.mapca_pub.publish(self.mapca)
        # self.get_logger().info('published map odom')

        # print(time.time() - start)

    # def wheel_cb(self, speeds):
    #     self.wf = (speeds.lfSpeed + speeds.rfSpeed) / 2.0
    #     self.wr = (speeds.lbSpeed + speeds.rbSpeed) / 2.0

    def path_cb(self, path):
        self.get_logger().info("received path")
        num_segs = len(path.segments) - 2
        p_x = np.zeros((1, num_segs))
        p_y = np.zeros((1, num_segs))
        p_theta = np.zeros((1, num_segs))
        p_curvature = np.zeros((1, num_segs))
        p_delta_curvature = np.zeros((1, num_segs))
        p_length = np.zeros((1, num_segs))
        p_speed = np.zeros((1, num_segs))

        for ii, clothoid in enumerate(path.segments):
            if ii == num_segs:
                break
            p_x[0, ii] = clothoid.start_x_m
            p_y[0, ii] = clothoid.start_y_m
            p_theta[0, ii] = clothoid.start_heading_rad
            p_curvature[0, ii] = clothoid.start_curvature_inv_m
            p_delta_curvature[0, ii] = clothoid.delta_curvature_per_length_inv_m2
            p_length[0, ii] = clothoid.length_m
            p_speed[0, ii] = clothoid.speed_mps
        self.p = np.vstack([p_x, p_y])
        self.p = np.append(self.p, (self.p[:, -1].reshape((-1, 1)) + np.vstack((p_length[0, -1]*np.cos(p_theta[0, -1]), p_length[0, -1]*np.sin(p_theta[0, -1])))).reshape((-1, 1)), axis=1)
        self.p_theta = np.append(p_theta, np.arctan2(self.p[1, 0] - self.p[1, -1], self.p[0, 0] - self.p[0, -1]).reshape((1, 1)), axis=1)
        self.p_length = np.append(p_length, np.linalg.norm(self.p[:, 0] - self.p[:, -1]).reshape((1, 1)), axis=1)
        self.p_speeds = np.append(p_speed, p_speed[:, -1].reshape((1, 1)), axis=1)
        # print(self.p, self.p_theta)
        # print(self.p_speeds.shape)
        dif_vecs = self.p[:, 1:] - self.p[:, :-1]
        self.dif_vecs = dif_vecs.copy()
        self.slopes = dif_vecs[1, :] / (dif_vecs[0, :] + 1e-6)
        self.midpoints = self.p[:, :-1] + dif_vecs / 2
        self.s = np.cumsum(np.linalg.norm(dif_vecs, axis=0))
        # print(self.p.shape)
        closed_ps = np.append(self.p, self.p[:, 0:1], axis=1)
        closed_speeds = np.append(self.p_speeds, self.p_speeds[:, 0:1], axis=1)
        tck, u = scipy.interpolate.splprep(np.vstack((closed_ps, closed_speeds)), u=None, s=0.0, per=1)
        num_steps = self.num_smoothed_path_points
        u_new = np.linspace(u.min(), u.max(), num_steps)
        x_new, y_new, speed_new = scipy.interpolate.splev(u_new, tck, der=0)
        dx, dy, _ = scipy.interpolate.splev(u_new, tck, der=1)
        ddxx, ddyy, _ = scipy.interpolate.splev(u_new, tck, der=2)
        curvatures = (dx * ddyy - dy * ddxx) / (dx ** 2 + dy ** 2) ** (3 / 2)
        self.rho = curvatures
        self.p = np.array([x_new, y_new])
        self.speeds = speed_new
        dif_vecs = self.p[:, 1:] - self.p[:, :-1]
        self.dif_vecs = dif_vecs
        self.slopes = dif_vecs[1, :] / dif_vecs[0, :]
        self.midpoints = self.p[:, :-1] + dif_vecs/2
        self.s = np.cumsum(np.linalg.norm(dif_vecs, axis=0))
        print('s set to:', self.s)
        # plt.plot(self.p[0, :], self.p[1, :])
        # plt.plot(x_new, y_new)
        # plt.show()
        trajectory = Path()
        path_stride = 2
        path_time = self.get_clock().now().to_msg()
        for ii in range(int(num_steps / path_stride)):
            pose_stamped = PoseStamped()
            # pose_stamped.header.seq = ii
            pose_stamped.header.stamp = path_time
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = self.p[0, ii * path_stride]
            pose_stamped.pose.position.y = self.p[1, ii*path_stride]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.z = 1.0
            trajectory.poses.append(pose_stamped)
        # trajectory.header.seq = 0
        trajectory.header.stamp = path_time
        trajectory.header.frame_id = 'map'
        self.path_pub.publish(trajectory)
        self.trajectory = trajectory
        np.savez('planned_path.npz', s=self.s, rho=self.rho, p=self.p, dif_vecs=self.dif_vecs)
        self.get_logger().info('saving to ' + str(os.getcwd()))
        self.start = True
        return p_curvature, p_speed, self.s

    def convert_obs_to_constraints(self, obs_locs):
        ceiling = self.bound_celing
        bound_length = self.bound_length
        bound_stride = self.bound_stride
        boundary_dists = ceiling * np.ones((2, int(bound_length / bound_stride)))
        # boundary_dists = ceiling * np.ones((2, self.N))
        try:
            obs_locs_map = np.ones_like(obs_locs)
            for ii in range(obs_locs.shape[1]):
                _, norm_dist, s_dist, speed = self.localize(obs_locs[:, ii], 0.0)
                obs_locs_map[:, ii] = np.array([norm_dist, s_dist])
            # print(obs_locs_map[:, :])
            # print(self.state[-2:])
            try:
                left_bound = np.min(obs_locs_map[0, obs_locs_map[0, :] > 0.0])
            except ValueError:
                left_bound = ceiling
            try:
                right_bound = np.max(obs_locs_map[0, obs_locs_map[0, :] < 0.0])
            except ValueError:
                right_bound = -ceiling
            # if np.abs(right_bound) < np.abs(left_bound) or len(left_bound) == 0:
            #     left_bound = ceiling
            # elif len(right_bound) == 0:
            #     right_bound = -ceiling
            s_start = np.min(obs_locs_map[1, :])
            s_end = np.max(obs_locs_map[1, :])
            # self.get_logger().info(str((s_start, s_end, left_bound, right_bound)))
            # k_start = np.argmin(np.abs(xs[-1, :] - s_start))
            # k_end = np.argmin(np.abs(xs[-1, :] - s_end))
            # dist_start = np.min(np.abs(xs[-1, :] - s_start))
            # dist_end = np.min(np.abs(xs[-1, :] - s_end))
            # # if dist_start and dist_end < 10.0:
            # boundary_dists[:, k_start:k_end+1] = np.vstack((left_bound, -1*right_bound))
        except (IndexError, ValueError) as e:
            self.get_logger().info('no obs')
            return boundary_dists

        k_start = np.argmin(np.abs(np.arange(self.mapca.s, self.mapca.s + bound_length, bound_stride) - s_start))
        k_end = np.argmin(np.abs(np.arange(self.mapca.s, self.mapca.s + bound_length, bound_stride) - s_end))
        boundary_dists[:, k_start:k_end + 1] = np.vstack((left_bound, -1 * right_bound))
        left_bound = Path()
        right_bound = Path()
        bound_time = self.get_clock().now().to_msg()
        for ii in range(int(bound_length / bound_stride)):
            pose_stamped = PoseStamped()
            dists = (self.mapca.s + ii * bound_stride) - self.s
            mini = np.argmin(np.abs(dists))
            p = self.p[:, mini]
            theta = np.arctan2(self.dif_vecs[1, mini], self.dif_vecs[0, mini])
            pose_stamped.pose.position.x = p[0] + dists[mini] * np.cos(theta) - boundary_dists[0, ii] * np.sin(theta)
            pose_stamped.pose.position.y = p[1] + dists[mini] * np.sin(theta) + boundary_dists[0, ii] * np.cos(theta)
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.z = 1.0
            # pose_stamped.header.seq = ii
            pose_stamped.header.stamp = bound_time
            pose_stamped.header.frame_id = 'map'
            left_bound.poses.append(copy.deepcopy(pose_stamped))
            pose_stamped.pose.position.x = p[0] + dists[mini] * np.cos(theta) - -1*boundary_dists[1, ii] * np.sin(theta)
            pose_stamped.pose.position.y = p[1] + dists[mini] * np.sin(theta) + -1*boundary_dists[1, ii] * np.cos(theta)
            right_bound.poses.append(pose_stamped)
        left_bound.header.frame_id = 'map'
        # left_bound.header.seq = 0
        left_bound.header.stamp = bound_time
        self.boundary_pub.publish(left_bound)
        right_bound.header.frame_id = 'map'
        # right_bound.header.seq = 0
        right_bound.header.stamp = bound_time
        self.boundary_pub.publish(right_bound)

        # boundary_dists = boundary_dists[:, ::-1]
        # print('boundary dists:', boundary_dists)
        return boundary_dists

    def obstacle_callback(self, occupancy_msg=None):
        np.set_printoptions(threshold=sys.maxsize)
        occ = np.asarray(occupancy_msg.data).reshape((self.width_cells, self.front_cells + self.back_cells))
        # print(occ)
        # self.get_logger().info(str(occ))
        obs_locs_y_map, obs_locs_x_map = np.where(occ < 1.0)
        obs_locs_x_car = obs_locs_x_map * self.grid_resolution - self.back_cells * self.grid_resolution
        obs_locs_y_car = obs_locs_y_map * self.grid_resolution - self.width_cells / 2.0 * self.grid_resolution
        # self.get_logger().info(str(obs_locs_y_car))
        # obs_locs_x_car = np.array([10.0, 12.0])
        # obs_locs_y_car = np.array([14.0, 15.0])
        # lateral_distance_grid = 14 * np.ones((60, 60))
        # mask = np.vstack((obs_locs_y_map, obs_locs_x_map))
        # lateral_distance_grid[mask[0, :], mask[1, :]] = obs_locs_y_car
        # right_half = lateral_distance_grid[:30, 14:]
        # left_half = lateral_distance_grid[30:, 14:]
        # left_lateral_dists = np.min(np.abs(left_half), axis=0)
        # right_lateral_dists = np.min(np.abs(right_half), axis=0)
        # self.obs_locs = np.vstack((left_lateral_dists, right_lateral_dists))
        if not self.inertial_frame:
            obs_locs_x_cartesian = self.mapca.x + obs_locs_x_car * np.cos(self.mapca.yaw) - obs_locs_y_car * np.sin(self.mapca.yaw)
            obs_locs_y_cartesian = self.mapca.y + obs_locs_y_car * np.cos(self.mapca.yaw) + obs_locs_x_car * np.sin(self.mapca.yaw)
        else:
            obs_locs_x_cartesian = self.mapca.x + obs_locs_x_car.copy()
            obs_locs_y_cartesian = self.mapca.y + obs_locs_y_car.copy()
        # self.get_logger().info(str(obs_locs_y_cartesian))
        obs_locs = np.vstack((obs_locs_x_cartesian, obs_locs_y_cartesian))
        # try:
            # print(self.x, self.y)
            # print(obs_locs_x_car[0], obs_locs_y_car[0])
            # print(obs_locs_x_cartesian[0], obs_locs_y_cartesian[0])
        # except IndexError:
        #     pass
        # point_cloud = PointCloud()
        # point_cloud.header.frame_id = 'map'
        # for ii in range(len(obs_locs_x_cartesian)):
        #     point = Point32()
        #     point.x = obs_locs_x_cartesian[ii]
        #     point.y = obs_locs_y_cartesian[ii]
        #     point.z = 0.0
        #     point_cloud.points.append(point)
        # # channel = ChannelFloat32()
        # # channel.name = 'intensity'
        # # channel.values = np.ones
        # # point_cloud.channels.append(channel)
        # self.goal_pub.publish(point_cloud)
        boundary_dists = self.convert_obs_to_constraints(obs_locs)
        bounds_msg = MapBounds()
        data0 = boundary_dists.astype('float64').tolist()[0]
        data1 = boundary_dists.astype('float64').tolist()[1]
        data = data0 + data1
        # self.get_logger().info(data)
        bounds_msg.array.data = data
        self.bounds_array_pub.publish(bounds_msg)
        return


# def map_publish():
#     map_pub = rospy.Publisher('/path', AsiClothoidPath, queue_size=2)
#     path = AsiClothoidPath()
#     package_path = '/autorally_private_control'
#     file_name = 'CCRF_2021-01-10.npz'
#     track_dict = np.load(package_path + '/src/maps/CCRF/' + file_name)
#     p_x = track_dict['X_cen_smooth']
#     p_y = track_dict['Y_cen_smooth']
#     file_name = 'ccrf_track_optimal_1000s_15m.npz'
#     track_dict = np.load(package_path + '/src/maps/CCRF/' + file_name)
#     try:
#         p_x = track_dict['X_cen_smooth']
#         p_y = track_dict['Y_cen_smooth']
#     except KeyError:
#         p_x = track_dict['pts'][:, 0]
#         p_y = track_dict['pts'][:, 1]
#         rho = track_dict['curvature']
#     num_segments = len(rho)
#     for ii in range(num_segments):
#         clothoid = AsiClothoid()
#         clothoid.start_x_m = p_x[ii]
#         clothoid.start_y_m = p_y[ii]
#         clothoid.start_curvature_inv_m = rho[ii]
#         path.segments.append(clothoid)
#     path.header.stamp = rospy.Time.now()
#     time.sleep(1)
#     map_pub.publish(path)
#     time.sleep(1)
#     print('pubbed')


def main():
    rclpy.init()
    map_ca = Map_CA()
    rclpy.spin(map_ca)
    map_ca.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
