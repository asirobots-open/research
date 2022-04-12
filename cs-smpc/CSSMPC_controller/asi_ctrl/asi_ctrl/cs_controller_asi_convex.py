#! /usr/bin/env python3
import numpy as np
import time
#import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import copy
from asi_ctrl import cs_model_asi as cs_model
from asi_ctrl import cs_solver_obstacle as cs_solver
import rclpy
import rclpy.node
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point32
from sensor_msgs.msg import PointCloud, ChannelFloat32
from rcl_interfaces.msg import SetParametersResult
from asi_msgs.msg import AsiTBSG, MotionVk, MotionVkFieldsPresent
import rospkg
from asi_msgs.msg import MapCA
from asi_ctrl import polylines_asi
from asi_msgs.msg import MapBounds


class CS_SMPC(rclpy.node.Node):
    def __init__(self):
        super().__init__('CSSMPC')
        self.begin = 0

        self.declare_parameter('max_speed', 5.0)
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.declare_parameter('terminal_speed', -1.0)
        self.terminal_speed = self.get_parameter('terminal_speed').get_parameter_value().double_value
        self.declare_parameter('horizon_length', 20)
        horizon_length = self.get_parameter('horizon_length').get_parameter_value().integer_value
        self.declare_parameter('time_step', 0.1)
        time_step = self.get_parameter('time_step').get_parameter_value().double_value
        self.declare_parameter('steer_min', -1.0)
        steer_min = self.get_parameter('steer_min').get_parameter_value().double_value
        self.declare_parameter('steer_max', 1.0)
        steer_max = self.get_parameter('steer_max').get_parameter_value().double_value
        self.declare_parameter('throttle_min', -0.2)
        throttle_min = self.get_parameter('throttle_min').get_parameter_value().double_value
        self.declare_parameter('throttle_max', 0.4)
        throttle_max = self.get_parameter('throttle_max').get_parameter_value().double_value
        self.declare_parameter('delta_steer_min', -0.3)
        delta_steer_min = self.get_parameter('delta_steer_min').get_parameter_value().double_value
        self.declare_parameter('delta_steer_max', 0.3)
        delta_steer_max = self.get_parameter('delta_steer_max').get_parameter_value().double_value
        self.declare_parameter('delta_throttle_min', -0.20)
        delta_throttle_min = self.get_parameter('delta_throttle_min').get_parameter_value().double_value
        self.declare_parameter('delta_throttle_max', 0.05)
        delta_throttle_max = self.get_parameter('delta_throttle_max').get_parameter_value().double_value
        self.declare_parameter('Q_speed', 10.0)
        Q_speed = self.get_parameter('Q_speed').get_parameter_value().double_value
        self.declare_parameter('Q_heading', 300.0)
        Q_heading = self.get_parameter('Q_heading').get_parameter_value().double_value
        self.declare_parameter('Q_lateral', 30.0)
        Q_lateral = self.get_parameter('Q_lateral').get_parameter_value().double_value
        self.declare_parameter('QN_speed', 0.0)
        QN_speed = self.get_parameter('QN_speed').get_parameter_value().double_value
        self.declare_parameter('QN_heading', Q_heading * horizon_length)
        QN_heading = self.get_parameter('QN_heading').get_parameter_value().double_value
        self.declare_parameter('QN_lateral', max(Q_heading, Q_lateral) * horizon_length)
        QN_lateral = self.get_parameter('QN_lateral').get_parameter_value().double_value
        self.declare_parameter('R_steer', 2.0)
        R_steer = self.get_parameter('R_steer').get_parameter_value().double_value
        self.declare_parameter('R_throttle', 2.0)
        R_throttle = self.get_parameter('R_throttle').get_parameter_value().double_value
        self.create_timer(1.0, self.dynamic_reconfigure)
        self.declare_parameter('bound_celing', 20.0)
        self.bound_ceiling = self.get_parameter('bound_celing').get_parameter_value().double_value
        self.declare_parameter('bound_length', 25.0)
        self.bound_length = self.get_parameter('bound_length').get_parameter_value().double_value
        self.declare_parameter('bound_stride', 1.0)
        self.bound_stride = self.get_parameter('bound_stride').get_parameter_value().double_value
        self.declare_parameter('command_topic', '')
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.declare_parameter('steering_gain', -1.0)
        self.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value
        self.declare_parameter('acceleration_gain', 10.0)
        self.acceleration_gain = self.get_parameter('acceleration_gain').get_parameter_value().double_value
        self.declare_parameter('use_vk', False)
        self.use_vk = self.get_parameter('use_vk').get_parameter_value().bool_value

        self.n = 6
        self.m = 2
        self.l = 6
        self.N = horizon_length
        self.dt_linearization = time_step
        self.dt_solve = 0.10

        self.target_speed = 5
        self.x_target = np.tile(np.array([self.target_speed, 0, 0, 0, 0, 0]).reshape((-1, 1)), (self.N - 1, 1))
        if self.terminal_speed < 0.0:
            terminal_speed = self.target_speed
        else:
            terminal_speed = self.terminal_speed
        self.x_target = np.vstack((self.x_target, np.array([terminal_speed, 0, 0, 0, 0, 0]).reshape((-1, 1))))
        self.mu_N = 10000 * np.array([5., 2., 4., 4., 4., 300.]).reshape((-1, 1))
        self.v_range = np.array([[steer_min, steer_max], [throttle_min, throttle_max]])
        self.slew_rate = np.array([[delta_steer_min, delta_steer_max], [delta_throttle_min, delta_throttle_max]])
        self.prob_lvl = 0.6
        self.load_k = 0
        self.track_w = 9999.0
        self.x = 0
        self.y = 0
        self.yaw = 0

        self.ar = cs_model.Model(self.N, vehicle_centric=False, map_coords=True, use_vk=self.use_vk)
        self.ar.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value
        self.ar.acceleration_gain = self.get_parameter('acceleration_gain').get_parameter_value().double_value

        if self.load_k:
            self.solver = cs_solver.CSSolver(self.n, self.m, self.l, self.N, self.u_min, self.u_max, mean_only=True, lti_k=False)
        else:
            self.solver = cs_solver.CSSolver(self.n, self.m, self.l, self.N, self.v_range, self.slew_rate, (False, 4*self.N), mean_only=True, k_form=1, prob_lvl=self.prob_lvl, chance_const_N=self.N, boundary_dim=-2, delta_slew_cost=1.0)

        Q = np.zeros((self.n, self.n))
        Q[0, 0] = Q_speed
        Q[1, 1] = 0.0
        Q[2, 2] = 0.0
        Q[3, 3] = Q_heading
        Q[4, 4] = Q_lateral
        self.Q_bar = np.kron(np.eye(self.N, dtype=int), Q)
        # self.Q_bar[-8, -8] = 30
        # self.Q_bar[-7, -7] = 1000
        self.Q_bar[-6, -6] = QN_speed
        self.Q_bar[-3, -3] = QN_heading
        self.Q_bar[-2, -2] = QN_lateral
        self.Q_bar[-1, -1] = 0.0
        R = np.zeros((self.m, self.m))
        R[0, 0] = R_steer  # 2
        R[1, 1] = R_throttle # 1
        self.R_bar = np.kron(np.eye(self.N, dtype=int), R)

        self.state = np.zeros((self.n, 1))  # np.array([5, 0, 0, 50, 50, 0, 0, 0])
        self.control = np.zeros((self.m, 1))
        self.obs_locs = np.empty(())
        self.us = np.tile(np.array([0.0, 0.02]).reshape((-1, 1)), (1, self.N))
        self.D = np.zeros((self.n, self.l))
        self.t2 = time.time()
        self.bounds_array = 100.0 * np.ones((2, int(self.bound_length / self.bound_stride)))

        if self.use_vk:
            chassis_cmd_msg_type = MotionVk
            self.motion_fields = MotionVkFieldsPresent()
        else:
            chassis_cmd_msg_type = AsiTBSG
        self.chassis_command = chassis_cmd_msg_type()
        self.command_pub = self.create_publisher(chassis_cmd_msg_type, self.command_topic, 1)
        self.path_pub = self.create_publisher(Path, "trajectory", 1)
        self.create_subscription(MapCA, "mapCA", self.odom_callback, 10)
        self.get_logger().info('subscribed to map odom')
        self.create_subscription(MapBounds, "bounds_array", self.bounds_callback, 2)

    def dynamic_reconfigure(self):
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.terminal_speed = self.get_parameter('terminal_speed').get_parameter_value().double_value
        Q_speed = self.get_parameter('Q_speed').get_parameter_value().double_value
        Q_heading = self.get_parameter('Q_heading').get_parameter_value().double_value
        Q_lateral = self.get_parameter('Q_lateral').get_parameter_value().double_value
        QN_speed = self.get_parameter('QN_speed').get_parameter_value().double_value
        QN_heading = self.get_parameter('QN_heading').get_parameter_value().double_value
        QN_lateral = self.get_parameter('QN_lateral').get_parameter_value().double_value
        R_steer = self.get_parameter('R_steer').get_parameter_value().double_value
        R_throttle = self.get_parameter('R_throttle').get_parameter_value().double_value

        Q = np.zeros((self.n, self.n))
        Q[0, 0] = Q_speed
        Q[1, 1] = 0.0
        Q[2, 2] = 0.0
        Q[3, 3] = Q_heading
        Q[4, 4] = Q_lateral
        self.Q_bar = np.kron(np.eye(self.N, dtype=int), Q)
        self.Q_bar[-6, -6] = QN_speed
        self.Q_bar[-3, -3] = QN_heading
        self.Q_bar[-2, -2] = QN_lateral
        R = np.zeros((self.m, self.m))
        R[0, 0] = R_steer  # 2
        R[1, 1] = R_throttle  # 1
        self.R_bar = np.kron(np.eye(self.N, dtype=int), R)

        self.ar.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value
        self.ar.acceleration_gain = self.get_parameter('acceleration_gain').get_parameter_value().double_value

    def odom_callback(self, map_msg):
        # self.get_logger().info('map odom received')
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
        self.target_speed = min(self.target_speed, self.max_speed)
        print('target speed: ', self.target_speed)
        self.x_target = np.tile(np.array([self.target_speed, 0, 0, 0, 0, 0]).reshape((-1, 1)), (self.N - 1, 1))
        if self.terminal_speed < 0.0:
            terminal_speed = self.target_speed
        else:
            terminal_speed = self.terminal_speed
        self.x_target = np.vstack((self.x_target, np.array([terminal_speed, 0, 0, 0, 0, 0]).reshape((-1, 1))))
        self.state = np.array([vx, vy, wz, epsi, ey, s]).reshape((-1, 1))
        print('state', self.state)
        self.begin = 1
        self.main_update()

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

    def convert_obs_to_constraints(self, xs):
        # boundary_dists = self.bound_ceiling * np.ones((2, self.N))
        bound_ss = np.arange(xs[-1, 0], xs[-1, 0] + self.bound_length, self.bound_stride)
        bound_ss = bound_ss[:self.bounds_array.shape[1]]
        min_ks = np.argmin(np.abs(xs[-1:, :] - bound_ss.reshape((-1, 1))), axis=0)
        boundary_dists = self.bounds_array[:, min_ks]
        # # boundary_dists = boundary_dists[:, ::-1]
        # # print('boundary dists:', boundary_dists)
        return boundary_dists

    def bounds_callback(self, msg):
        self.bounds_array = np.asarray(msg.array.data).reshape((2, int(self.bound_length / self.bound_stride)))

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

            X_bar = np.dot(A, xs[:, 0]) + np.dot(B, V) + d.flatten()
            x_bar = X_bar.reshape((self.n, self.N), order='F')
            # print(x_bar)
            # print(xs)
            # xs_cartesian = np.zeros((self.n, self.N))
            # xs_cartesian[:, 0] = x_0.flatten()
            # xs_cartesian[3:, 0] = np.vstack((self.yaw, self.x, self.y)).flatten()
            # self.ar.map_coords = False
            # for ii in range(self.N - 1):
            #     xs_cartesian[:, ii + 1] = self.ar.update_dynamics(xs_cartesian[:, ii:ii + 1], us[:, ii:ii + 1], self.dt_linearization).flatten()
            # self.ar.map_coords = True
            trajectory = Path()
            path_stride = 1
            path_time = self.get_clock().now().to_msg()
            for ii in range(int(self.N / path_stride)):
                pose_stamped = PoseStamped()
                dists = x_bar[-1, ii * path_stride] - self.ar.s
                mini = np.argmin(np.abs(dists))
                p = self.ar.p[:, mini]
                theta = np.arctan2(self.ar.dif_vecs[1, mini], self.ar.dif_vecs[0, mini])
                pose_stamped.pose.position.x = p[0] + dists[mini] * np.cos(theta) - x_bar[-2, ii * path_stride] * np.sin(theta)
                pose_stamped.pose.position.y = p[1] + dists[mini] * np.sin(theta) + x_bar[-2, ii * path_stride] * np.cos(theta)
                # pose_stamped.pose.position.x = xs_cartesian[4, ii * path_stride]
                # pose_stamped.pose.position.y = xs_cartesian[5, ii * path_stride]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.z = 1.0
                # pose_stamped.header.seq = ii
                pose_stamped.header.stamp = path_time
                pose_stamped.header.frame_id = 'map'
                trajectory.poses.append(pose_stamped)
            trajectory.header.frame_id = 'map'
            # trajectory.header.seq = 0
            trajectory.header.stamp = path_time
            self.path_pub.publish(trajectory)
        except RuntimeError:
            V = np.tile(np.array([0, 0.1]).reshape((-1, 1)), (self.N, 1)).flatten()
            X_bar = np.dot(A, xs[:, 0]) + np.dot(B, V) + d.flatten()
            x_bar = X_bar.reshape((self.n, self.N), order='F')
            print('error x_bar', x_bar)
            K = np.zeros((self.m * self.N, self.n * self.N))
        finally:
            # print(x_0, us, D, K)
            pass
        # queue.put((V, K, X_bar, (A, B, d)))
        return V, K, X_bar, (A, B, d)

    def update_control(self, V, K, X_bar, kk):
        y = self.state.flatten() - X_bar[kk * self.n:(kk + 1) * self.n]
        # if abs(y[7]) > 51.8453:  # need to use absolute s
        #     y[7] = 0
        u = V[kk * self.m:(kk + 1) * self.m] #+ np.dot(K[kk * self.m:(kk + 1) * self.m, kk * self.n:(kk + 1) * self.n], y)
        u = np.where(u > self.v_range[:, 1], self.v_range[:, 1], u)
        u = np.where(u < self.v_range[:, 0], self.v_range[:, 0], u)
        print('u', u)
        if self.use_vk:
            self.chassis_command.velocity = u[1]
            self.motion_fields.velocity = True
            self.chassis_command.curvature = -u[0]
            self.motion_fields.curvature = True
            self.chassis_command.direction = MotionVk.MOTION_VK_FORWARD
            self.motion_fields.direction = True
            self.chassis_command.fields_present = self.motion_fields
        else:
            self.chassis_command.steer_cmd = u[0]
            if u[1] >= 0:
                self.chassis_command.throttle_cmd = u[1]
                self.chassis_command.brake_cmd = 0.00
            else:
                self.chassis_command.throttle_cmd = 0.00
                self.chassis_command.brake_cmd = -1.0 *u[1]
            self.chassis_command.header.stamp = self.get_clock().now().to_msg()
            # self.chassis_command.sender = "CSSMPC"
            self.chassis_command.gear_cmd = 1
        self.command_pub.publish(self.chassis_command)
        return y

    def main_update(self):
        num_steps_applied = 1
        t0 = time.time()
        if self.load_k:
            nearest = np.argmin(np.linalg.norm(self.state - ss, axis=0))
            K = ks[:, :, nearest]
        mu_0 = self.state.copy()
        # print('mu_0', mu_0)
        if self.load_k:
            V, _K, X_bar, lin_params = self.update_solution(self.state, self.us, self.D, K=K)
        else:
            V, K, X_bar, lin_params = self.update_solution(self.state, self.us, self.D)
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
        self.us = V.reshape((self.m, self.N), order='F')
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
            y = self.update_control(V, K, X_bar, ii)
            print('time: ', time.time() - self.t2)
            # controller.get_logger().info('controlling')
            self.t2 = time.time()
            # control_update_rate.sleep()
            # if ii == 1 and jj == 0:
            self.D = np.diag(y)
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


def main(args=None):
    print('starting controller')
    rclpy.init(args=args)
    controller = CS_SMPC()
    controller.get_logger().info('subscribed to map odom')
    rate = controller.create_rate(0.1)
    # controller.ltv_solver = cs_solver.CSSolver(controller.n, controller.m, controller.l, controller.N, controller.u_min, controller.u_max, mean_only=True)
    num_steps_applied = 1  # int(controller.dt_solve / controller.dt_linearization)
    # solver_io = Queue(maxsize=1)
    # ltv_io = MQ()
    dt_control = controller.dt_solve
    # control_update_rate = rospy.Rate(1/dt_control)
    # linearization_step_rate = rospy.Rate(1/controller.dt_linearization)
    us = np.tile(np.array([0.0, 0.02]).reshape((-1, 1)), (1, controller.N))

    ks = np.zeros((2 * 10, 8 * 10, 25 * 20))
    ss = np.zeros((8, 25 * 20))
    iii = 0
    if controller.load_k:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('autorally_private_control')
        dictionary = np.load(package_path + "/src/CSSMPC/Ks_ltv_10N_9mps.npz")
        ks = dictionary['ks']
        ss = dictionary['ss']
        print("loaded feedback matrices")

    print('waiting for first pose estimate')
    # while not controller.begin and rclpy.ok():
    #     controller.get_logger().info('waiting')
    #     rate.sleep()
    controller.get_logger().info('starting')
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
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
