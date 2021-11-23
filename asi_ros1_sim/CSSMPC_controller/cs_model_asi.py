import numpy as np
from numpy import sin, cos, tan, arctan as atan, sqrt, arctan2 as atan2, zeros, zeros_like, abs, pi
#import torch
#import throttle_model
#import rospkg
import polylines_asi


class Model:
    def __init__(self, N, vehicle_centric=False, map_coords=False):
        #self.throttle = throttle_model.Net()
        #rospack = rospkg.RosPack()
        # package_path = rospack.get_path('autorally_private_control')
        #package_path = '/home/user/catkin_ws/src/autorally_private/autorally_private_control'
        #self.throttle.load_state_dict(torch.load(package_path + '/src/CSSMPC/' + 'throttle_model1.pth'))
        self.N = N
        self.vehicle_centric = vehicle_centric
        self.map_coords = map_coords

        #ccrf_curv = np.load(package_path + '/src/maps/ccrf_2021/ccrf_track_optimal.npz')
        #self.s = ccrf_curv['s']
        #self.rho = ccrf_curv['curvature']
        self.map_ca = polylines_asi.MapCA()

    def get_curvature(self, s):
        while (s > self.s[-1]).any():
            s[s > self.s[-1]] -= self.s[-1]

        dif = np.abs(s.reshape((-1, 1)) - self.s.reshape((1, -1)))
        idx = np.argmin(dif, axis=1)
        rho = self.rho[idx]
        return rho.flatten()

    def update_dynamics(self, state, input, dt, nn=None, rho=0.0):
        state = state.T
        input = input.T
        m_Vehicle_m = 21.7562#1270
        m_Vehicle_Iz = 1.124#2000
        m_Vehicle_lF = 1#1.015
        lFR = 2.74#3.02
        m_Vehicle_lR = lFR-m_Vehicle_lF
        m_Vehicle_IwF = 0.1#8
        m_Vehicle_IwR = .0373
        m_Vehicle_rF = 0.095#0.325
        m_Vehicle_rR = 0.090#0.325
        m_Vehicle_h = 0.12#.54
        m_g = 9.80665

        tire_B = 4.0#10
        tire_C = 1.0
        tire_D = 1.0
        tire_E = 1.0
        tire_Sh = 0.0
        tire_Sv = 0.0

        N, dx = state.shape
        m_nu = 1

        vx = state[:, 0]
        vy = state[:, 1]
        wz = state[:, 2]
        psi = state[:, 3]
        X = state[:, 4]
        Y = state[:, 5]

        if (vx < 0.1).any():
            vx = np.maximum(vx, 0.1)

        m_Vehicle_kSteering = -0.06  # -pi / 180 * 18.7861
        m_Vehicle_cSteering = -0.001  # 0.0109
        throttle_factor = 0.38
        # delta = input[:, 0]
        steering = input[:, 0]
        delta = m_Vehicle_kSteering * steering + m_Vehicle_cSteering
        T = np.maximum(input[:, 1], 0)

        min_velo = 0.1
        deltaT = 0.01
        t = 0

        while t < dt:
            beta = atan2(vy, vx)
            beta = atan(m_Vehicle_lR / lFR * tan(delta))

            V = sqrt(vx * vx + vy * vy)
            # vFx = V * cos(beta - delta) + wz * m_Vehicle_lF * sin(delta)
            # vFy = V * sin(beta - delta) + wz * m_Vehicle_lF * cos(delta)
            # vRx = vx
            # vRy = vy - wz * m_Vehicle_lR

            # sEF = -(vFx - wF * m_Vehicle_rF) / (vFx) + tire_Sh
            # muFx = tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv
            # sEF = -(vRx - wR * m_Vehicle_rR) / (vRx) + tire_Sh
            # muRx = tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv
            #
            # sEF = atan(vFy / abs(vFx)) + tire_Sh
            # alpha = -sEF
            # muFy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv
            # sEF = atan(vRy / abs(vRx)) + tire_Sh
            # alphaR = -sEF
            # muRy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv

            # sFx = np.where(wF > 0, (vFx - wF * m_Vehicle_rF) / (wF * m_Vehicle_rF), 0)
            # sRx = np.where(wR > 0, (vRx - wR * m_Vehicle_rR) / (wR * m_Vehicle_rR), 0)
            # # sFy = np.where(vFx > 0, (1 + sFx) * vFy / vFx, 0)
            # # sRy = np.where(vRx > 0, (1 + sRx) * vRy / vRx, 0)
            # sFy = np.where(vFx > 0, vFy / (wF * m_Vehicle_rF), 0)
            # sRy = np.where(vRx > 0, vRy / (wR * m_Vehicle_rR), 0)
            #
            # sF = np.sqrt(sFx * sFx + sFy * sFy) + 1e-2
            # sR = np.sqrt(sRx * sRx + sRy * sRy) + 1e-2
            #
            # muF = tire_D * sin(tire_C * atan(tire_B * sF))
            # muR = tire_D * sin(tire_C * atan(tire_B * sR))
            #
            # muFx = -sFx / sF * muF
            # muFy = -sFy / sF * muF
            # muRx = -sRx / sR * muR
            # muRy = -sRy / sR * muR
            #
            # fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * muRx) / (
            #         m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (muFx * cos(delta) - muFy * sin(delta) - muRx))
            # # fFz = m_Vehicle_m * m_g * (m_Vehicle_lR / 0.57)
            # fRz = m_Vehicle_m * m_g - fFz
            #
            # fFx = fFz * muFx
            # fRx = fRz * muRx
            # fFy = fFz * muFy
            # fRy = fRz * muRy
            #
            # ax = ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz)
            #
            # dot_X =cos(psi)*vx - sin(psi)*vy
            # dot_Y = sin(psi)*vx + cos(psi)*vy

            next_state = zeros_like(state)
            next_state[:, 0] = vx + deltaT * 10.0 * T#+ deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz)
            next_state[:, 1] = vy #+ deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz)
            next_state[:, 2] = wz + deltaT * (V / m_Vehicle_lR * sin(beta))
            if self.map_coords:
                dists = np.abs(Y.reshape((-1, 1)) - self.map_ca.s.reshape((1, -1)))
                mini = np.argmin(dists, axis=1)
                rho = self.map_ca.rho[mini]
                next_state[:, 3] = psi + deltaT * (wz - (vx * cos(psi) - vy * sin(psi)) / (1 - rho * X) * rho)
                next_state[:, 4] = X + deltaT * (vx * sin(psi) + vy * cos(psi))
                next_state[:, 5] = Y + deltaT * (vx * cos(psi) - vy * sin(psi)) / (1 - rho * X)
            else:
                next_state[:, 3] = psi + deltaT * wz
                if self.vehicle_centric:
                    next_state[:, 4] = X + deltaT * (vx * cos(psi) - vy * sin(psi))
                    next_state[:, 5] = Y + deltaT * (vx * sin(psi) + vy * cos(psi))
                else:
                    next_state[:, 4] = X + deltaT * (V * cos(psi + beta))
                    next_state[:, 5] = Y + deltaT * (V * sin(psi + beta))

            t += deltaT
            vx = next_state[:, 0]
            vy = next_state[:, 1]
            wz = next_state[:, 2]
            psi = next_state[:, 3]
            X = next_state[:, 4]
            Y = next_state[:, 5]

        # print(t1-t0, t2-t1, t3-t2, t4-t3, t5-t4)

        return next_state.T

    def linearize_dynamics(self, states, controls, dt=0.1):
        nx = 6
        nu = 2
        nN = self.N

        delta_x = np.array([0.01, 0.001, 0.01, 0.001, 0.01, 0.01])
        delta_u = np.array([0.01, 0.01])
        delta_x_flat = np.tile(delta_x, (1, nN))
        delta_u_flat = np.tile(delta_u, (1, nN))
        delta_x_final = np.multiply(np.tile(np.eye(nx), (1, nN)), delta_x_flat)
        delta_u_final = np.multiply(np.tile(np.eye(nu), (1, nN)), delta_u_flat)
        xx = np.tile(states, (nx, 1)).reshape((nx, nx*nN), order='F')
        # print(delta_x_final, xx)
        ux = np.tile(controls, (nx, 1)).reshape((nu, nx*nN), order='F')
        x_plus = xx + delta_x_final
        # print(x_plus, ux)
        x_minus = xx - delta_x_final
        fx_plus = self.update_dynamics(x_plus, ux, dt)
        # print(fx_plus)
        fx_minus = self.update_dynamics(x_minus, ux, dt)
        A = (fx_plus - fx_minus) / (2 * delta_x_flat)

        xu = np.tile(states, (nu, 1)).reshape((nx, nu*nN), order='F')
        uu = np.tile(controls, (nu, 1)).reshape((nu, nu*nN), order='F')
        u_plus = uu + delta_u_final
        # print(xu)
        u_minus = uu - delta_u_final
        fu_plus = self.update_dynamics(xu, u_plus, dt)
        # print(fu_plus)
        fu_minus = self.update_dynamics(xu, u_minus, dt)
        B = (fu_plus - fu_minus) / (2 * delta_u_flat)

        state_row = np.zeros((nx*nN, nN))
        input_row = np.zeros((nu*nN, nN))
        for ii in range(nN):
            state_row[ii*nx:ii*nx + nx, ii] = states[:, ii]
            input_row[ii*nu:ii*nu+nu, ii] = controls[:, ii]
        d = self.update_dynamics(states, controls, dt) - np.dot(A, state_row) - np.dot(B, input_row)

        return A, B, d

    def form_long_matrices_LTI(self, A, B, D):
        nx = 8
        nu = 2
        N = self.N

        AA = np.zeros((nx*N, nx))
        BB = zeros((nx*N, nu * N))
        DD = zeros((nx, nx * N))
        B_i_row = zeros((nx, 0))
        # D_i_bar = zeros((nx, nx))
        for ii in np.arange(0, N):
            AA[ii*nx:(ii+1)*nx, :] = np.linalg.matrix_power(A, ii+1)

            B_i_cell = np.dot(np.linalg.matrix_power(A, ii), B)
            B_i_row = np.hstack((B_i_cell, B_i_row))
            BB[ii*nx:(ii+1)*nx, :(ii+1)*nu] = B_i_row

            # D_i_bar = np.hstack((np.dot(np.linalg.matrix_power(A, ii - 1), D), D_i_bar))
            # temp = np.hstack((D_i_bar, np.zeros((nx, max(0, nx * N - D_i_bar.shape[1])))))
            # DD = np.vstack((DD, temp[:, 0: nx * N]))

        return AA, BB, DD

    def form_long_matrices_LTV(self, A, B, d, D):
        nx = 6
        nu = 2
        nl = 6
        N = self.N

        AA = np.zeros((nx*N, nx))
        BB = zeros((nx*N, nu * N))
        dd = zeros((nx*N, 1))
        DD = zeros((nx*N, nl * N))
        AA_i_row = np.eye(nx)
        dd_i_row = np.zeros((nx, 1))
        # B_i_row = zeros((nx, 0))
        # D_i_bar = zeros((nx, nx))
        for ii in np.arange(0, N):
            AA_i_row = np.dot(A[:, :, ii], AA_i_row)
            AA[ii*nx:(ii+1)*nx, :] = AA_i_row

            B_i_row = B[:, :, ii]
            D_i_row = D[:, :, ii]
            for jj in np.arange(ii-1, -1, -1):
                B_i_cell = np.dot(A[:, :, ii], BB[(ii-1)*nx:ii*nx, jj*nu:(jj+1)*nu])
                B_i_row = np.hstack((B_i_cell, B_i_row))
                D_i_cell = np.dot(A[:, :, ii], DD[(ii-1)*nx:ii*nx, jj*nl:(jj+1)*nl])
                D_i_row = np.hstack((D_i_cell, D_i_row))
            BB[ii*nx:(ii+1)*nx, :(ii+1)*nu] = B_i_row
            DD[ii*nx:(ii+1)*nx, :(ii+1)*nl] = D_i_row

            dd_i_row = np.dot(A[:, :, ii], dd_i_row) + d[:, :, ii]
            dd[ii*nx:(ii+1)*nx, :] = dd_i_row

        return AA, BB, dd, DD
