import math
from boring_nodes import rk4

class AckermannModel():
    def __init__(self):
        self.max_steer_angle = 0.785;    # rad
        self.max_steer_rate = 1.5;       # rad/sec
        self.mass = 2000;                # kg
        self.drag_multiplier = 0.9375;   # Cd * air_density * area / 2   ==>  drag_multiplier * v^2 = drag force ==> 0.3 * 1.25 kg / m^3 * 5 m^2 / 2 = 0.9375 kg/m
        self.wheel_base = 4;             # m
        self.rolling_resistance = 0.01;  # unitless
        self.steer_pgain = 700.0;        # 
        self.steer_dgain = 40.0;         # slightly underdamped
        self.velocity_pgain = 1000.0
        self.velocity_igain = 5000

class AckermannCommand():
    def __init__(self):
        self.resistive_force = 0
        self.propulsive_force = 0
        self.target_steer_angle = 0

class AckermannState():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.longitudinal_velocity = 0
        self.steer_angle = 0
        self.dsteer_angledt = 0

    def setState(self,state):
        self.x = state[0]
        self.y = state[1]
        self.yaw = state[2]
        self.longitudinal_velocity = state[3]
        self.steer_angle = state[4]
        self.dsteer_angledt = state[5]

    def getState(self):
        return [self.x,self.y,self.yaw,self.longitudinal_velocity,self.steer_angle,self.dsteer_angledt]

class Ackermann():
    
    def __init__(self, model=None, state=None):
        self.t = 0
        self.min_impulse_time = 0.2; # 100ms
        self.model = AckermannModel() if model == None else model
        self.state = AckermannState() if state == None else state
        self.command = AckermannCommand()

    def derivs(self, t, state):
        self.state.setState(state)
        cq = math.cos(self.state.yaw)
        sq = math.sin(self.state.yaw)
        signv = 1 if self.state.longitudinal_velocity >= 0 else -1
        drag = -signv*abs(self.model.drag_multiplier * self.state.longitudinal_velocity**2)
        tangential_force = self.command.propulsive_force + drag
        stopping_force = abs(self.command.resistive_force)
        max_stopping_force = abs(self.state.longitudinal_velocity*self.model.mass/self.min_impulse_time)
        tangential_force -= signv*(max_stopping_force if stopping_force > max_stopping_force else stopping_force)
        d_longitudinal_velocity_dt  = tangential_force/self.model.mass
        d2steer_angledt2 = self.steer_acceleration(self.min_impulse_time)
        k = math.tan(self.state.steer_angle)/self.model.wheel_base
        dyawdt = k*self.state.longitudinal_velocity
        vxg = cq*self.state.longitudinal_velocity
        vyg = sq*self.state.longitudinal_velocity
        # Match derivatives to AckermannState
        return [vxg,vyg,dyawdt,d_longitudinal_velocity_dt,self.state.dsteer_angledt,d2steer_angledt2]

    def steer_acceleration(self, tau):
        pgain = self.model.steer_pgain
        dgain = self.model.steer_dgain
        p = self.state.steer_angle;     pmax = self.model.max_steer_angle
        v = self.state.dsteer_angledt;  vmax = self.model.max_steer_rate
        pdesired = self.command.target_steer_angle
        a = -pgain*(p - pdesired) - dgain*v
        if (v+tau*a) > vmax:
            a = (vmax-v)/tau
        elif (v + tau*a) < -vmax:
            a = (-vmax-v)/tau
        if p > pmax and v > 0:
            a = min(a,(pmax-p)/tau/tau)
        elif p < -pmax and v < 0:
            a = max(a,(-pmax-p)/tau/tau)
        return a

    def setTargetVelocity(self, target_longitudinal_velocity, dt):
        verr = target_longitudinal_velocity - self.state.longitudinal_velocity
        velocity_integral_term += verr*dt;
        force = self.model.velocity_pgain*verr + self.model.velocity_igain*velocity_integral_term
        self.command.propulsive_force = 0 if force <= 0 else force
        self.command.resistive_force = -force if force <= 0 else 0

    def step(self, dt):
        currentstate = self.state.getState()
        newstate = rk4.rk4(self.t, currentstate, dt, self.derivs)
        self.state.setState(newstate)
        self.t += dt

def test1():
    from matplotlib import pyplot as plt
    ack = Ackermann()
    ack.model.steer_pgain = 700.0;        # 
    ack.model.steer_dgain = 20.0;         # slightly underdamped

    ack.model.drag_multiplier = 40
    dt = 0.05
    state = []
    target_steer_angle = []
    t = []
    while ack.t < 20:
        if ack.t > 0 and ack.t < 10:
            ack.command.propulsive_force = ack.model.mass*0.5
            ack.command.resistive_force = 0
            ack.command.target_steer_angle = 0.1
        else:
            ack.command.resistive_force = ack.model.mass*0.4
            ack.command.propulsive_force = 0
            ack.command.target_steer_angle = 0
        # ack.command.target_steer_angle = 0.4*math.sin(ack.t)
        ack.step(dt)
        state.append(ack.state.getState())
        target_steer_angle.append(ack.command.target_steer_angle)
        t.append(ack.t)

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle('Ackermann Simulation')
    statet = list(map(list, zip(*state))) # transpose state
    ax1.plot(statet[0], statet[1])
    ax1.axis('equal')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax2.plot(t,statet[0], t,statet[1], t,statet[3])
    ax2.legend(['x (m)','y (m)','v (m/s)'])
    ax2.set_xlabel('time (s)')
    ax3.plot(t,statet[4], t,statet[5], t,target_steer_angle)    
    ax3.legend(['steer angle (rad)','steer rate (rad/sec)','target steer angle (rad)'])
    ax3.set_xlabel('time (s)')
    plt.show()

if __name__ == '__main__':
    test1()