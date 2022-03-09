#! /usr/bin/env python3
import numpy as np
import time
#import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import copy
import cs_model_asi as cs_model
import cs_solver_obstacle as cs_solver
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point32
from sensor_msgs.msg import PointCloud, ChannelFloat32
from asi_msgs.msg import AsiTBSG
import rospkg
import roslib
# roslib.load_manifest('autorally_private_control')
#from dynamic_reconfigure.server import Server
#from autorally_private_control.cfg import CSSMPC_paramsConfig
#from autorally_msgs.msg import runstop
from asi_msgs.msg import mapCA
import polylines_asi


class CS_SMPC:
    def __init__(self):
        rospy.init_node('CSSMPC', anonymous=True)
        self.command_pub = rospy.Publisher("/ius0/vcu_wrench", AsiTBSG, queue_size=1)
        # self.runstop_pub = rospy.Publisher("/LTVMPC/runstop", runstop, queue_size=10)
        self.path_pub = rospy.Publisher("/trajectory", Path, queue_size=1)
        self.goal_pub = rospy.Publisher("/goal", PointCloud, queue_size=1)
        self.boundary_pub = rospy.Publisher("/boundaries", Path, queue_size=2)
        self.chassis_command = AsiTBSG()
        self.begin = 0

        self.n = 6
        self.m = 2
        self.l = 6
        self.N = 40
        self.dt_linearization = 0.10
        self.dt_solve = 0.10

        self.target_speed = 7
        self.x_target = np.tile(np.array([self.target_speed, 0, 0, 0, 0, 0]).reshape((-1, 1)), (self.N - 1, 1))
        self.x_target = np.vstack((self.x_target, np.array([2, 0, 0, 0, 0, 0]).reshape((-1, 1))))
        self.mu_N = 10000 * np.array([5., 2., 4., 4., 4., 300.]).reshape((-1, 1))
        self.v_range = np.array([[-1.0, 1.0], [-0.2, 1.0]])
        self.slew_rate = np.array([[-0.3, 0.3], [-0.20, 0.05]])
        self.prob_lvl = 0.6
        self.load_k = 0
        self.track_w = 9999.0
        self.x = 0
        self.y = 0
        self.yaw = 0

        # self.goal = PointStamped()
        # self.goal.header.frame_id = "map"
        # self.goal.point.x = self.x_target[4, 0]
        # self.goal.point.y = self.x_target[5, 0]
        # self.goal.point.z = 0.0
        # self.goal_pub.publish(self.goal)
        # self.server = Server(CSSMPC_paramsConfig, self.dynamic_reconfigure)

        self.ar = cs_model.Model(self.N, vehicle_centric=False, map_coords=True)

        if self.load_k:
            self.solver = cs_solver.CSSolver(self.n, self.m, self.l, self.N, self.u_min, self.u_max, mean_only=True, lti_k=False)
        else:
            self.solver = cs_solver.CSSolver(self.n, self.m, self.l, self.N, self.v_range, self.slew_rate, (False, 4*self.N), mean_only=True, k_form=1, prob_lvl=self.prob_lvl, chance_const_N=self.N, boundary_dim=-2, delta_slew_cost=1.0)
            # self.solver.M.setSolverParam("mioTolAbsGap", 100)
            # self.solver.M.setSolverParam("mioMaxTime", 0.1)
            # self.solver.M.setSolverParam("mioTolFeas", 1.0e-3)

        Q = np.zeros((self.n, self.n))
        Q[0, 0] = 10
        Q[1, 1] = 0
        Q[2, 2] = 0
        Q[3, 3] = 300
        Q[4, 4] = 30
        self.Q_bar = np.kron(np.eye(self.N, dtype=int), Q)
        # self.Q_bar[-8, -8] = 30
        # self.Q_bar[-7, -7] = 1000
        # Q_bar[-6, -6] = 100
        self.Q_bar[-3, -3] = 300*40*2
        self.Q_bar[-2, -2] = 30*40*10
        self.Q_bar[-1, -1] = 0
        R = np.zeros((self.m, self.m))
        R[0, 0] = 2.0  # 2
        R[1, 1] = 2  # 1
        self.R_bar = np.kron(np.eye(self.N, dtype=int), R)

        self.state = np.zeros((self.n, 1))  # np.array([5, 0, 0, 50, 50, 0, 0, 0])
        self.control = np.zeros((self.m, 1))
        self.obs_locs = np.empty(())

        rospy.Subscriber("/MAP_CA/mapCA", mapCA, self.odom_callback)
        rospy.Subscriber("/ius0/terrain_cost", OccupancyGrid, self.obstacle_callback)

    # def dynamic_reconfigure(self, config, level):
    #     self.N = config["N"]
    #     self.target_speed = config["speed"]
    #     self.x_target = np.tile(np.array([self.target_speed, 0, 0, 0, 0, 0, 0, 0]).reshape((-1, 1)), (self.N - 1, 1))
    #     self.x_target = np.vstack((self.x_target, np.array([2, 0, 0, 0, 0, 0, 0, 0]).reshape((-1, 1))))
    #     self.v_range = np.array([[config["steer_min"], config["steer_max"]], [config["throttle_min"], config["throttle_max"]]])
    #     self.slew_rate = np.array([[config["steer_rate_min"], config["steer_rate_max"]], [config["throttle_rate_min"], config["throttle_rate_max"]]])
    #     self.prob_lvl = config["prob_lvl"]
    #     self.load_k = config["load_k"]
    #     self.track_w = config["track_w"]
    #
    #     Q = np.zeros((self.n, self.n))
    #     Q[0, 0] = config["Q_vx"]
    #     Q[1, 1] = config["Q_vy"]
    #     Q[2, 2] = config["Q_psidot"]
    #     Q[5, 5] = config["Q_psi"]
    #     Q[6, 6] = config["Q_ey"]
    #     Q[7, 7] = config["Q_s"]
    #     self.Q_bar = np.kron(np.eye(self.N, dtype=int), Q)
    #     self.Q_bar[-8, -8] = config["QN_vx"]
    #     self.Q_bar[-7, -7] = config["QN_vy"]
    #     self.Q_bar[-6, -6] = config["QN_psidot"]
    #     self.Q_bar[-3, -3] = config["QN_epsi"]
    #     self.Q_bar[-2, -2] = config["QN_ey"]
    #     self.Q_bar[-1, -1] = config["QN_s"]
    #     R = np.zeros((self.m, self.m))
    #     R[0, 0] = config["R_delta"]
    #     R[1, 1] = config["R_T"]
    #     self.R_bar = np.kron(np.eye(self.N, dtype=int), R)
    #     return config

    def odom_callback(self, map_msg):
        # X = map_msg.pose.pose.position.x
        # Y = map_msg.pose.pose.position.y
        # r = Rotation.from_quat(
        #     [map_msg.pose.pose.orientation.x, map_msg.pose.pose.orientation.y, map_msg.pose.pose.orientation.z,
        #      map_msg.pose.pose.orientation.w])
        # yaw, pitch, roll = r.as_euler('zyx', degrees=False)
        # # for whatever reason map_msg y velocity seems inverted so apply -
        # vx = map_msg.twist.twist.linear.x #* np.cos(yaw) - map_msg.twist.twist.linear.y * np.sin(yaw)
        # vy = map_msg.twist.twist.linear.y #<-change back to x* -np.sin(yaw) - map_msg.twist.twist.linear.y * np.cos(yaw)
        # wz = map_msg.twist.twist.angular.z
        # self.state = np.array([vx, vy, wz, yaw, X, Y]).reshape((-1, 1))
        # self.begin = 1
        vehicle_rF = 0.095
        vehicle_rR = 0.090
        vx = map_msg.vx
        vy = map_msg.vy
        wz = map_msg.wz
        wF = map_msg.wf / vehicle_rF
        if wF > 100:
            wF = 100
        wR = map_msg.wr / vehicle_rR
        if wR > 100:
            wR = 100
        epsi = map_msg.epsi
        ey = map_msg.ey
        s = map_msg.s
        self.x = map_msg.x
        self.y = map_msg.y
        self.yaw = map_msg.yaw
        self.target_speed = map_msg.speed
        self.x_target = np.tile(np.array([self.target_speed - 10.0, 0, 0, 0, 0, 0]).reshape((-1, 1)), (self.N, 1))
        self.state = np.array([vx, vy, wz, epsi, ey, s]).reshape((-1, 1))
        self.begin = 1

    def obstacle_callback(self, occupancy_msg=None):
        # np.set_printoptions(threshold=sys.maxsize)
        occ = np.asarray(occupancy_msg.data).reshape((60, 60))
        obs_locs_y_map, obs_locs_x_map = np.where(occ > -1)
        obs_locs_x_car = obs_locs_x_map / 2.0 - 7.0
        obs_locs_y_car = obs_locs_y_map / 2.0 - 14.5
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
        obs_locs_x_cartesian = self.x + obs_locs_x_car * np.cos(self.yaw) - obs_locs_y_car * np.sin(self.yaw)
        obs_locs_y_cartesian = self.y + obs_locs_y_car * np.cos(self.yaw) + obs_locs_x_car * np.sin(self.yaw)
        self.obs_locs = np.vstack((obs_locs_x_cartesian, obs_locs_y_cartesian))
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
        return

    def roll_out_trajectory(self, x_0, us, N, lin=None):
        x_0 = x_0.copy()
        # x_0[3:, 0] = 0
        if lin:
            A, B, d = lin
            xs = np.dot(A, x_0) + np.dot(B, us.flatten().reshape((-1, 1))) + d
            xs = xs.reshape((self.n, self.N), order='F')
            xs = np.hstack((x_0, xs[:, :(N-1)]))
        else:
            xs = np.zeros((self.n, N))
            xs[:, 0] = x_0.flatten()
            for ii in range(N - 1):
                # dists = np.abs(xs[-1, ii] - self.map_ca.s)
                # mini = np.argmin(dists)
                # curvature = self.map_ca.rho[mini]
                # if ii == 0:
                #     print('curvature', mini, curvature)
                xs[:, ii + 1] = self.ar.update_dynamics(xs[:, ii:ii+1], us[:, ii:ii+1], self.dt_linearization).flatten()
            # print(x_0, us, xs)
        return xs

    def update_target_speed(self, xs):
        rhos = np.abs(self.ar.get_curvature(xs[7, 1:]))
        target_speeds = self.target_speed - rhos / 0.6 * (self.target_speed - 2)
        self.x_target[0:-self.n:self.n, 0] = target_speeds

    def convert_obs_to_constraints(self, xs):
        ceiling = 20.0
        boundary_dists = ceiling * np.ones((2, self.N))
        try:
            obs_locs_map = np.ones_like(self.obs_locs)
            for ii in range(self.obs_locs.shape[1]):
                _, norm_dist, s_dist, speed = self.ar.map_ca.localize(self.obs_locs[:, ii], 0.0)
                obs_locs_map[:, ii] = np.array([norm_dist, s_dist])
            # print(obs_locs_map[:, :])
            # print(self.state[-2:])
            left_bound = np.min(obs_locs_map[0, :])
            right_bound = np.max(obs_locs_map[0, :])
            if np.abs(right_bound) < np.abs(left_bound):
                left_bound = ceiling
            else:
                right_bound = -ceiling
            s_start = np.min(obs_locs_map[1, :])
            s_end = np.max(obs_locs_map[1, :])
            k_start = np.argmin(np.abs(xs[-1, :] - s_start))
            k_end = np.argmin(np.abs(xs[-1, :] - s_end))
            dist_start = np.min(np.abs(xs[-1, :] - s_start))
            dist_end = np.min(np.abs(xs[-1, :] - s_end))
            # if dist_start and dist_end < 10.0:
            boundary_dists[:, k_start:k_end+1] = np.vstack((left_bound, -1*right_bound))
        except (IndexError, ValueError) as e:
            print('no obs')

        left_bound = Path()
        right_bound = Path()
        bound_stride = 1
        bound_time = rospy.Time.now()
        for ii in range(int(self.N / bound_stride)):
            pose_stamped = PoseStamped()
            dists = xs[-1, ii * bound_stride] - self.ar.map_ca.s
            mini = np.argmin(np.abs(dists))
            p = self.ar.map_ca.p[:, mini]
            theta = np.arctan2(self.ar.map_ca.dif_vecs[1, mini], self.ar.map_ca.dif_vecs[0, mini])
            pose_stamped.pose.position.x = p[0] + dists[mini] * np.cos(theta) - boundary_dists[0, ii * bound_stride] * np.sin(theta)
            pose_stamped.pose.position.y = p[1] + dists[mini] * np.sin(theta) + boundary_dists[0, ii * bound_stride] * np.cos(theta)
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.z = 1.0
            pose_stamped.header.seq = ii
            pose_stamped.header.stamp = bound_time
            pose_stamped.header.frame_id = 'map'
            left_bound.poses.append(copy.deepcopy(pose_stamped))
            pose_stamped.pose.position.x = p[0] + dists[mini] * np.cos(theta) - -1*boundary_dists[1, ii * bound_stride] * np.sin(theta)
            pose_stamped.pose.position.y = p[1] + dists[mini] * np.sin(theta) + -1*boundary_dists[1, ii * bound_stride] * np.cos(theta)
            right_bound.poses.append(pose_stamped)
        left_bound.header.frame_id = 'map'
        left_bound.header.seq = 0
        left_bound.header.stamp = bound_time
        self.boundary_pub.publish(left_bound)
        right_bound.header.frame_id = 'map'
        right_bound.header.seq = 0
        right_bound.header.stamp = bound_time
        self.boundary_pub.publish(right_bound)

        # boundary_dists = boundary_dists[:, ::-1]
        # print('boundary dists:', boundary_dists)
        return boundary_dists

    def update_solution(self, x_0, us, D, K=None):
        # x_0, us, D = queue
        if not self.load_k:
            K = np.zeros((self.m*self.N, self.n*self.N))
        xs = self.roll_out_trajectory(x_0, us, self.N)
        # print(self.x_target[0::self.n, 0])
        # self.update_target_speed(xs)
        A, B, d = self.ar.linearize_dynamics(xs, us, dt=self.dt_linearization)
        A = A.reshape((self.n, self.n, self.N), order='F')
        B = B.reshape((self.n, self.m, self.N), order='F')
        d = d.reshape((self.n, 1, self.N), order='F')
        D = np.tile(D.reshape((self.n, self.l, 1)), (1, 1, self.N))
        A, B, d, D = self.ar.form_long_matrices_LTV(A, B, d, D)
        sigma_0 = np.zeros((self.n, self.n))
        sigma_N_inv = np.zeros((self.n, self.n))
        # self.obstacle_callback()
        boundary_dists = self.convert_obs_to_constraints(xs)
        # boundary_dists = 999
        # self.x_target[5:self.n*self.N:self.n, 0] = nom_path

        self.solver.populate_params(A, B, d, D, xs[:, 0], sigma_0, sigma_N_inv, self.Q_bar, self.R_bar, us, self.x_target, self.mu_N, boundary_dists, K=K)
        try:
            V, K = self.solver.solve()
            K = K.reshape((self.m * self.N, self.n * self.N))
        except RuntimeError:
            V = np.tile(np.array([0, 0.1]).reshape((-1, 1)), (self.N, 1)).flatten()
            X_bar = np.dot(A, xs[:, 0]) + np.dot(B, V) + d.flatten()
            x_bar = X_bar.reshape((self.n, self.N), order='F')
            print(x_bar)
            K = np.zeros((self.m * self.N, self.n * self.N))
        finally:
            # print(x_0, us, D, K)
            pass
        X_bar = np.dot(A, xs[:, 0]) + np.dot(B, V) + d.flatten()
        x_bar = X_bar.reshape((self.n, self.N), order='F')
        # print(x_bar)
        print(xs)
        # xs_cartesian = np.zeros((self.n, self.N))
        # xs_cartesian[:, 0] = x_0.flatten()
        # xs_cartesian[3:, 0] = np.vstack((self.yaw, self.x, self.y)).flatten()
        # self.ar.map_coords = False
        # for ii in range(self.N - 1):
        #     xs_cartesian[:, ii + 1] = self.ar.update_dynamics(xs_cartesian[:, ii:ii + 1], us[:, ii:ii + 1], self.dt_linearization).flatten()
        # self.ar.map_coords = True
        trajectory = Path()
        path_stride = 1
        path_time = rospy.Time.now()
        for ii in range(int(self.N / path_stride)):
            pose_stamped = PoseStamped()
            dists = x_bar[-1, ii * path_stride] - self.ar.map_ca.s
            mini = np.argmin(np.abs(dists))
            p = self.ar.map_ca.p[:, mini]
            theta = np.arctan2(self.ar.map_ca.dif_vecs[1, mini], self.ar.map_ca.dif_vecs[0, mini])
            pose_stamped.pose.position.x = p[0] + dists[mini] * np.cos(theta) - x_bar[-2, ii * path_stride] * np.sin(theta)
            pose_stamped.pose.position.y = p[1] + dists[mini] * np.sin(theta) + x_bar[-2, ii * path_stride] * np.cos(theta)
            # pose_stamped.pose.position.x = xs_cartesian[4, ii * path_stride]
            # pose_stamped.pose.position.y = xs_cartesian[5, ii * path_stride]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.z = 1.0
            pose_stamped.header.seq = ii
            pose_stamped.header.stamp = path_time
            pose_stamped.header.frame_id = 'map'
            trajectory.poses.append(pose_stamped)
        trajectory.header.frame_id = 'map'
        trajectory.header.seq = 0
        trajectory.header.stamp = path_time
        self.path_pub.publish(trajectory)
        # queue.put((V, K, X_bar, (A, B, d)))
        return V, K, X_bar, (A, B, d)

    def update_control(self, V, K, X_bar, kk):
        y = self.state.flatten() - X_bar[kk * self.n:(kk + 1) * self.n]
        # if abs(y[7]) > 51.8453:  # need to use absolute s
        #     y[7] = 0
        u = V[kk * self.m:(kk + 1) * self.m] #+ np.dot(K[kk * self.m:(kk + 1) * self.m, kk * self.n:(kk + 1) * self.n], y)
        u = np.where(u > self.v_range[:, 1], self.v_range[:, 1], u)
        u = np.where(u < self.v_range[:, 0], self.v_range[:, 0], u)
        print(u)
        self.chassis_command.steer_cmd = u[0]
        if u[1] >= 0:
            self.chassis_command.throttle_cmd = u[1]
            self.chassis_command.brake_cmd = 0.00
        else:
            self.chassis_command.throttle_cmd = 0.00
            self.chassis_command.brake_cmd = -1.0 *u[1]
        self.chassis_command.header.stamp = rospy.Time.now()
        # self.chassis_command.sender = "CSSMPC"
        self.command_pub.publish(self.chassis_command)
        return y

    def system_id(self):
        throttle_range = np.arange(0.0, 1.0, step=0.01)
        steering_range = np.arange(-1.0, 1.0, step=0.01)

        # ramp
        # for ii in range(100):
        #     throttle = throttle_range[ii]
        #     self.chassis_command.throttle_cmd = throttle
        #     self.chassis_command.steer_cmd = 0.0
        #     self.chassis_command.brake_cmd = 0.0
        #     self.chassis_command.header.stamp = rospy.Time.now()
        #     self.command_pub.publish(self.chassis_command)
        #     rospy.sleep(0.1)
        # for ii in range(100):
        #     throttle = throttle_range[-1-ii]
        #     self.chassis_command.throttle_cmd = throttle
        #     self.chassis_command.steer_cmd = 0.0
        #     self.chassis_command.brake_cmd = 0.0
        #     self.chassis_command.header.stamp = rospy.Time.now()
        #     self.command_pub.publish(self.chassis_command)
        #     rospy.sleep(0.1)
        for ii in range(200):
            steer = steering_range[ii]
            self.chassis_command.throttle_cmd = 0.0
            self.chassis_command.steer_cmd = steer
            self.chassis_command.brake_cmd = 0.0
            self.chassis_command.header.stamp = rospy.Time.now()
            self.command_pub.publish(self.chassis_command)
            rospy.sleep(0.1)

        # random
        for ii in range(200):
            jj = int(100 * np.random.random())
            steering = steering_range[jj]
            self.chassis_command.throttle_cmd = 0.0
            self.chassis_command.steer_cmd = steering
            self.chassis_command.brake_cmd = 0.0
            self.chassis_command.header.stamp = rospy.Time.now()
            self.command_pub.publish(self.chassis_command)
            rospy.sleep(0.1)


            # def ltv_solve(self, queue):
    #     x_0, V, ltv_sys = queue.get()
    #     t2 = time.time()
    #     us = V.reshape((controller.m, controller.N), order='F')
    #     xs = self.roll_out_trajectory(x_0, us, controller.N, lin=ltv_sys)
    #     print('roll', time.time() - t2)
    #     # t2 = time.time()
    #     A, B, d = self.ar.linearize_dynamics(xs, us)
    #     print('ltv_solve', time.time() - t2)
    #     # print('lin time:', time.time()-t2)
    #     A = A.reshape((self.n, self.n, self.N), order='F')
    #     B = B.reshape((self.n, self.m, self.N), order='F')
    #     d = d.reshape((self.n, 1, self.N), order='F')
    #     D = np.zeros((self.n, self.l))
    #     D = np.tile(D.reshape((self.n, self.l, 1)), (1, 1, self.N))
    #     A, B, d, D = self.ar.form_long_matrices_LTV(A, B, d, D)
    #     ltv_sys = (A, B, d, D)
    #     sigma_0 = np.zeros((self.n, self.n))
    #     sigma_N_inv = np.zeros((self.n, self.n))
    #     print('here')
    #     queue.put(ltv_sys)
    #     print('here2')
    #     return


if __name__ == '__main__':
    t2 = time.time()
    controller = CS_SMPC()
    # controller.ltv_solver = cs_solver.CSSolver(controller.n, controller.m, controller.l, controller.N, controller.u_min, controller.u_max, mean_only=True)
    num_steps_applied = 1#int(controller.dt_solve / controller.dt_linearization)
    # solver_io = Queue(maxsize=1)
    # ltv_io = MQ()
    dt_control = controller.dt_solve
    control_update_rate = rospy.Rate(1/dt_control)
    linearization_step_rate = rospy.Rate(1/controller.dt_linearization)
    us = np.tile(np.array([0.0, 0.02]).reshape((-1, 1)), (1, controller.N))

    ks = np.zeros((2*10, 8*10, 25*20))
    ss = np.zeros((8, 25*20))
    iii = 0
    if controller.load_k:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('autorally_private_control')
        dictionary = np.load(package_path + "/src/CSSMPC/Ks_ltv_10N_9mps.npz")
        ks = dictionary['ks']
        ss = dictionary['ss']
        print("loaded feedback matrices")

    print('waiting for first pose estimate')
    while not controller.begin and not rospy.is_shutdown():
        rospy.sleep(0.1)
    # controller.state = np.array([1.5, -.0034, 0.0019, 0.0008, -2.0, 0.0036]).reshape((-1, 1))
    # obs_locs = np.array([[0.0, 18], [2., 15.0]])
    # controller.track_w = np.tile(np.ones((2, 1)), (1, controller.N))
    # controller.track_w[1, :] = 0.1
    # controller.track_w[1, 20:] = -0.2
    # controller.track_w[0, :] = 2
    # controller.state = np.array([0.09610, -.0, 0.0, 0.0, -3.04, 0.00]).reshape((-1, 1))
    # controller.obs_locs = np.array([[-10.0, -9.], [-10.0, -9.]])
    # buffer = 0.5
    # controller.obs_locs[:, 0] -= buffer
    # controller.obs_locs[:, 1] += buffer
    D = np.zeros((controller.n, controller.l))
    # solver_io.put((controller.state, us, D))
    # solve = DummyProcess(target=controller.update_solution, args=(solver_io,))
    # solve.start()
    # solve.join()
    # V, K, X_bar, lin_params = controller.update_solution(controller.state, us, D)
    # controller.system_id()
    # exit()
    while not rospy.is_shutdown():
        t0 = time.time()
        if controller.load_k:
            nearest = np.argmin(np.linalg.norm(controller.state - ss, axis=0))
            K = ks[:, :, nearest]
        mu_0 = controller.state.copy()
        print(mu_0)
        if controller.load_k:
            V, _K, X_bar, lin_params = controller.update_solution(controller.state, us, D, K=K)
        else:
            V, K, X_bar, lin_params = controller.update_solution(controller.state, us, D)
        # nearest = np.argmin(np.linalg.norm(controller.state[:, 0] - ss[:, :], axis=0))
        # K = ks[:, :, nearest]
        # try:
        #     ks[:, :, iii] = K[:, :]
        #     ss[:, iii] = controller.state[:, 0]
        #     iii += 1
        # except IndexError:
        #     np.savez('Ks_ltv_10N_9mps.npz', ks=ks, ss=ss)
        #     break
        # V, K, X_bar, lin_params = solver_io.get()
        us = V.reshape((controller.m, controller.N), order='F')
        # predicted_x_0 = controller.roll_out_trajectory(controller.state, us, num_steps_applied + 1, lin=lin_params)[:, -1].reshape((-1, 1))
        # print(controller.state, us, predicted_x_0)
        # us = np.hstack((us[:, num_steps_applied:], np.tile(us[:, -1].reshape((-1, 1)), (1, num_steps_applied))))
        # solver_io.put((predicted_x_0, us, D))
        # solve = DummyProcess(target=controller.update_solution, args=(solver_io,))
        # print('setup time:', time.time() - t0)
        # ltv_sys = lin_params
        for ii in range(num_steps_applied):
            # ltv_io.put((controller.state, V, ltv_sys))
            t1 = time.time()
            # ltv = Process(target=controller.ltv_solve, args=(ltv_io,))
            # ltv.start()
            # if ii == 0:
            #     solve.start()
            # controller.ltv_solve(ltv_io)
            # for jj in range(int(controller.dt_linearization / dt_control)):
            y = controller.update_control(V, K, X_bar, ii)
            print('time: ', time.time() - t2)
            t2 = time.time()
            # control_update_rate.sleep()
                # if ii == 1 and jj == 0:
            D = np.diag(y)
            # controller.goal_pub.publish(controller.goal)
            # controller.state = controller.ar.update_dynamics(controller.state, us[:, 0:1], 0.99)
            # ltv.join()
            # print('ltv time:', time.time() - t1)
            # ltv_sys = ltv_io.get()
            # A, B, d, script_D = ltv_sys
            # ltv_sys = (A, B, d)
            # controller.ltv_solver.populate_params(A, B, d, script_D, controller.state, np.zeros((controller.n, controller.n)), np.zeros((controller.n, controller.n)), controller.Q_bar, controller.R_bar,
            #                                 V[:controller.m])
            # V, _ = controller.ltv_solver.solve()
            # linearization_step_rate.sleep()
