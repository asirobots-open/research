import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
import cs_controller_asi_convex
import cs_model_asi


if __name__ == '__main__':
    rclpy.init()
    steering_delay = 4
    throttle_delay = 12
    controller = cs_controller_asi_convex.CS_SMPC()
    model = cs_model_asi.Model(1, use_delays=True, map_coords=True)
    run_length = 300
    dt = 0.1
    state = np.zeros((6, 1))
    us = np.zeros((2, controller.N))
    D = np.zeros((6, 6))
    t2 = time.time()
    states = np.zeros((6, run_length))
    states[:, 0:1] = state
    times = np.linspace(0, dt*run_length, run_length)
    for ii in range(run_length-1):
        print(us)
        V, K, X_bar, lin_params = controller.update_solution(state, us, D)
        us = V.reshape((controller.m, controller.N), order='F')
        # print('us:, ', self.us)
        # y = controller.update_control(V, K, X_bar, ii)
        control = np.vstack((us[0, steering_delay:steering_delay+1], us[1, throttle_delay:throttle_delay+1]))
        state = model.update_dynamics(state, control, dt)
        us = np.hstack((us[:, 1:], us[:, -1:]))
        states[:, ii+1:ii+2] = state

        print('time: ', time.time() - t2)
        t2 = time.time()
        # self.control_update_rate.sleep()
        # D = np.diag(y)
    plt.subplot(3, 2, 1)
    plt.plot(times, states[0, :])
    plt.xlabel('t (s)')
    plt.ylabel('m/s')
    plt.legend(('vx',))
    plt.subplot(3, 2, 2)
    plt.plot(times, states[3, :])
    plt.xlabel('t (s)')
    plt.ylabel('rad')
    plt.legend(('heading error',))
    plt.subplot(3, 2, 3)
    plt.plot(times, states[4, :])
    plt.xlabel('t (s)')
    plt.ylabel('m')
    plt.legend(('lateral error',))
    # plt.subplot(3, 2, 4)
    # plt.plot(Xs[start_idx_state:], Ys[start_idx_state:])
    # # plt.plot(path_xs, path_ys, 'k--')
    # plt.xlabel('X (m)')
    # plt.ylabel('Y (m)')
    # plt.legend(('trajectory', 'path'))
    # plt.subplot(3, 2, 5)
    # plt.plot(t_control[start_idx_control:], deltas[start_idx_control:])
    # # plt.plot(t_steer, steerings, '--')
    # plt.xlabel('t (s)')
    # plt.ylabel('%')
    # plt.legend(('steering command', 'steering feedback'))
    # plt.subplot(3, 2, 6)
    # plt.plot(t_control[start_idx_control:],
    #          np.asarray(throttles[start_idx_control:]) - np.asarray(brakes[start_idx_control:]))
    # plt.xlabel('t (s)')
    # plt.ylabel('%')
    # plt.legend(('throttle command',))
    plt.show()
