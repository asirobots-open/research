#! /usr/bin/env python3
import numpy as np
#import matplotlib.pyplot as plt
import time
import rospy
import scipy.interpolate
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
# from autorally_msgs.msg import wheelSpeeds
from scipy.spatial.transform import Rotation
from asi_msgs.msg import mapCA
from asi_msgs.msg import AsiClothoidPath
from asi_msgs.msg import AsiClothoid
import rospkg


class MapCA:

    def __init__(self):
        rospy.Subscriber("/ius0/planned_path", AsiClothoidPath, self.path_cb)
        self.path_pub = rospy.Publisher("/smoothed_path", Path, queue_size=10, latch=True)
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

        self.wf = 0.1
        self.wr = 0.1

        # plt.plot(p_x, p_y, '.-')
        # plt.plot(self.midpoints[0], self.midpoints[1], 'x')
        # plt.show()

        rospy.Subscriber("/ius0/odom_topic", Odometry, self.odom_cb)
        # rospy.Subscriber("/wheelSpeeds", wheelSpeeds, self.wheel_cb)
        self.mapca_pub = rospy.Publisher('/MAP_CA/mapCA', mapCA, queue_size=1)

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
        # start = time.time()
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        r = Rotation.from_quat([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        yaw, pitch, roll = r.as_euler('zyx', degrees=False)
        # vx = odom.twist.twist.linear.x * np.cos(yaw) + odom.twist.twist.linear.y * np.sin(yaw)
        # vy = odom.twist.twist.linear.x * -np.sin(yaw) + odom.twist.twist.linear.y * np.cos(yaw)
        # for whatever reason map_msg y velocity seems inverted so apply -
        vx = odom.twist.twist.linear.x * np.cos(yaw) - odom.twist.twist.linear.y * np.sin(yaw)
        vy = odom.twist.twist.linear.x * -np.sin(yaw) - odom.twist.twist.linear.y * np.cos(yaw)
        wz = odom.twist.twist.angular.z

        d_psi, n, s, speed = self.localize(np.asarray([x, y]), yaw)
        mapca = mapCA()
        mapca.header.stamp = rospy.Time.now()
        mapca.vx = vx
        mapca.vy = vy
        mapca.wz = wz
        mapca.wf = self.wf
        mapca.wr = self.wr
        mapca.s = s
        mapca.ey = n
        mapca.epsi = d_psi
        mapca.x = x
        mapca.y = y
        mapca.yaw = yaw
        mapca.path_s = s
        mapca.speed = speed
        self.mapca_pub.publish(mapca)
        # print(time.time() - start)

    def wheel_cb(self, speeds):
        self.wf = (speeds.lfSpeed + speeds.rfSpeed) / 2.0
        self.wr = (speeds.lbSpeed + speeds.rbSpeed) / 2.0

    def path_cb(self, path):
        num_segs = len(path.segments)
        p_x = np.zeros((1, num_segs))
        p_y = np.zeros((1, num_segs))
        p_theta = np.zeros((1, num_segs))
        p_curvature = np.zeros((1, num_segs))
        p_delta_curvature = np.zeros((1, num_segs))
        p_length = np.zeros((1, num_segs))
        p_speed = np.zeros((1, num_segs))

        for ii, clothoid in enumerate(path.segments):
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
        print(self.p, self.p_theta)
        print(self.p_speeds.shape)
        dif_vecs = self.p[:, 1:] - self.p[:, :-1]
        self.dif_vecs = dif_vecs.copy()
        self.slopes = dif_vecs[1, :] / (dif_vecs[0, :] + 1e-6)
        self.midpoints = self.p[:, :-1] + dif_vecs / 2
        self.s = np.cumsum(np.linalg.norm(dif_vecs, axis=0))
        print(self.p.shape)
        closed_ps = np.append(self.p, self.p[:, 0:1], axis=1)
        closed_speeds = np.append(self.p_speeds, self.p_speeds[:, 0:1], axis=1)
        tck, u = scipy.interpolate.splprep(np.vstack((closed_ps, closed_speeds)), u=None, s=0.0, per=1)
        num_steps = 100
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
        # plt.plot(self.p[0, :], self.p[1, :])
        # plt.plot(x_new, y_new)
        # plt.show()
        trajectory = Path()
        path_stride = 2
        path_time = rospy.Time.now()
        for ii in range(int(num_steps / path_stride)):
            pose_stamped = PoseStamped()
            pose_stamped.header.seq = ii
            pose_stamped.header.stamp = path_time
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = self.p[0, ii * path_stride]
            pose_stamped.pose.position.y = self.p[1, ii*path_stride]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.z = 1.0
            trajectory.poses.append(pose_stamped)
        trajectory.header.seq = 0
        trajectory.header.stamp = path_time
        trajectory.header.frame_id = 'map'
        self.path_pub.publish(trajectory)
        self.trajectory = trajectory
        return p_curvature, p_speed, self.s


def map_publish():
    map_pub = rospy.Publisher('/path', AsiClothoidPath, queue_size=2)
    path = AsiClothoidPath()
    package_path = '/autorally_private_control'
    file_name = 'CCRF_2021-01-10.npz'
    track_dict = np.load(package_path + '/src/maps/CCRF/' + file_name)
    p_x = track_dict['X_cen_smooth']
    p_y = track_dict['Y_cen_smooth']
    file_name = 'ccrf_track_optimal_1000s_15m.npz'
    track_dict = np.load(package_path + '/src/maps/CCRF/' + file_name)
    try:
        p_x = track_dict['X_cen_smooth']
        p_y = track_dict['Y_cen_smooth']
    except KeyError:
        p_x = track_dict['pts'][:, 0]
        p_y = track_dict['pts'][:, 1]
        rho = track_dict['curvature']
    num_segments = len(rho)
    for ii in range(num_segments):
        clothoid = AsiClothoid()
        clothoid.start_x_m = p_x[ii]
        clothoid.start_y_m = p_y[ii]
        clothoid.start_curvature_inv_m = rho[ii]
        path.segments.append(clothoid)
    path.header.stamp = rospy.Time.now()
    time.sleep(1)
    map_pub.publish(path)
    time.sleep(1)
    print('pubbed')


if __name__ == '__main__':
    rospy.init_node('map_ca', anonymous=True)
    map_ca = MapCA()
    # map_publish()
    rate = rospy.Rate(1)
    # while 1:
        # map_ca.path_pub.publish(map_ca.trajectory)
        # rate.sleep()
    rospy.spin()
