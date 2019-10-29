#!/usr/bin/env python

"""
The goal is to drive the vehicle along the center line
"""

import rospy
from hkj_msgs.msg import RoadConditionVector, VehicleActuator
import numpy as np
from numpy.polynomial.polynomial import polyval
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from odeAdjoint import Ode1stExplicit
import time

class Vehicle1(Ode1stExplicit):
    """
    Vehicle model for track 1
    """
    def __init__(self):
        # For Integrator
        dim_y = 5
        dim_z = 2
        dim_p = 11   # 3rd order polynomial with one param fixed
        super().__init__(dim_y, dim_p, dim_z, method='RK45')
        self.p = np.zeros(dim_p)

        self.dt = rospy.get_param("integration_dt")
        self.mpc_dt = rospy.get_param("mpc_horizon")
        self.t = 0.0      # Current time, updated when publish actuation input
        self.acc = 0.0    # Initial condition - acceleration
        self.steer = 0.0  # Initial condition - steering angle
        self.planning_time = 0.0
        self.Lf = 3
        self.t_sample = np.linspace(0, self.dt, 11)
        
        # Spline: x-s, y-s, theta-s
        self.x_s = None
        self.y_s = None
        self.theta_s = None

        # ros publisher and subscriber
        self.sub = rospy.Subscriber("/perception", RoadConditionVector, self.callback, queue_size=10)
        self.pub = rospy.Publisher("/vehicle_actuator", VehicleActuator, queue_size=10)
        while self.pub.get_num_connections() < 1:
            continue

    def callback(self, data):
        """
        Run planning routine and publish the actuator input
        """
        if (len(data.road_condition_vt) < 10):
            rospy.signal_shutdown("Not enough waypoints ahead. Reach the end point.")

        # Write history
        with open("track2_continuous.csv", "a") as f:
            f.write("{T}, {X}, {Y}, {S}, {P}\n".format(
                T = self.t,
                X = data.pos_x,
                Y = data.pos_y,
                S = self.steer,
                P = self.planning_time,
            ))
        
        # Run optimization
        start = time.time()
        self.generateSpline(data)
        states = np.array(
            [data.pos_x, data.pos_y, data.vel_x, data.yaw_angle, 0.0]
        )
        p = self.p
        bounds = []
        for i in range(len(p)):
            if i == 0:
                bounds.append((self.steer, self.steer))
            else:
                bounds.append((-0.5, 0.5))

        res = minimize(
            fun = self.obj,
            x0 = p,
            bounds = bounds,
            args = (states,),
            method = 'SLSQP',
            jac = self.obj_jac,
            tol = 1e-6,
            options = dict(maxiter=50, disp=True)
        )

        # Compute acceleration
        steer = np.linspace(self.p[0], self.p[1], 11)
        self.planning_time = time.time() - start

        # Update parameters
        next_p = res.x[1:]
        self.p = np.append(next_p, 1)
        self.t += self.dt
        self.steer = steer[-1]   # The initial value of the next iteration is the last value in current iteration

        # Publish message  
        act_input = VehicleActuator()
        act_input.applied_force = [0.0] * 11
        act_input.steer_angle = steer
        self.pub.publish(act_input)

    def generateSpline(self, data):
        """
        Generate x_s, y_s and theta_s
        """
        x = []
        y = []
        theta = []
        
        for wp in data.road_condition_vt:
            x.append(wp.rc_wp[0])
            y.append(wp.rc_wp[1])
            theta.append(wp.theta)
        
        s = [0.0]
        for i in range(len(x) - 1):
            d = np.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
            s.append(s[-1] + d)

        self.x_s = CubicSpline(s, x)
        self.y_s = CubicSpline(s, y)
        self.theta_s = CubicSpline(s, theta)

    #- For integrator ---------------------------------------------------------
    def f(self, t, y):
        f = np.zeros(self.dim_y)
        f[0] = y[2] * np.cos(y[3])
        f[1] = y[2] * np.sin(y[3])
        f[2] = 0.0   # Constant velocity
        f[3] = y[2] / self.Lf * self.evalSpline(self.t_sample, self.p, t)
        f[4] = y[2] * np.cos(y[3]) * np.cos(self.theta_s(y[4])) + y[2] * np.sin(y[3]) * np.sin(self.theta_s(y[4]))
        return f

    def fj(self, t, y):
        jac = np.zeros((self.dim_y, self.dim_y))
        # x
        jac[0][2] = np.cos(y[3])
        jac[0][3] = -y[2]*np.sin(y[3])
        # y
        jac[1][2] = np.sin(y[3])
        jac[1][3] = y[2]*np.cos(y[3])
        # v

        # phi
        jac[3][2] = 1/self.Lf * polyval(t, [self.steer, self.p[0], self.p[1], self.p[2]])
        # s
        jac[4][2] = np.cos(y[3]) * np.cos(self.theta_s(y[4])) + np.sin(y[3]) * np.sin(self.theta_s(y[4]))
        jac[4][3] = -y[2] * np.sin(y[3]) * np.cos(self.theta_s(y[4])) + y[2] * np.cos(y[3]) * np.sin(self.theta_s(y[4]))
        jac[4][4] = -y[2] * np.cos(y[3]) * np.sin(self.theta_s(y[4])) * self.theta_s(y[4], 1) + y[2] * np.sin(y[3]) * np.cos(self.theta_s(y[4])) * self.theta_s(y[4], 1)

        return jac

    def fp(self, t, y):
        jac = np.zeros((self.dim_y, self.dim_p))
        # v
        # phi
        jac[3] = y[2] / self.Lf * self.evalSpline(self.t_sample, self.p, t, 1)

        return jac

    def I1(self, t, y):
        """
        Value of I1. Row vector with d = dim_z
        """
        return np.zeros(self.dim_z)

    def dI1dp(self, t, y):
        """
        Derivative of I1 w.r.t parameters p. 
        dI1dp[i][j] - derivative of I1_i w.r.t p_j
        """
        return np.zeros((self.dim_z, self.dim_p))

    def dI1dy(self, t, y):
        """
        Derivative of I1 w.r.t states y.
        dI1dy[i][j] - derivative of I1_i w.r.t y_j
        """
        return np.zeros((self.dim_z, self.dim_y))

    def I2(self, t, y):
        """
        Value of I2. Row vector with d = dim_z
        """
        i2 = np.zeros(self.dim_z)
        i2[0] = (y[0] - self.x_s(y[4]))**2 + (y[1] - self.y_s(y[4]))**2
        i2[1] = (y[3] - self.theta_s(y[4]))**2
        return i2

    def dI2dp(self, t, y):
        """
        Derivative of I2 w.r.t parameters p. 
        dI2dp[i][j] - derivative of I2_i w.r.t p_j
        """
        return np.zeros((self.dim_z, self.dim_p))

    def dI2dy(self, t, y):
        """
        Derivative of I1 w.r.t states y.
        dI2dy[i][j] - derivative of I2_i w.r.t y_j
        """
        jac = np.zeros((self.dim_z, self.dim_y))
        jac[0][0] = 2*(y[0] - self.x_s(y[4]))
        jac[0][1] = 2*(y[1] - self.y_s(y[4]))
        jac[0][4] = -2*(y[0]-self.x_s(y[4]))*self.x_s(y[4],1) - 2*(y[1] - self.y_s(y[4]))*self.y_s(y[4],1)
        jac[1][3] = 2*(y[3] - self.theta_s(y[4]))
        jac[1][4] = -2*(y[3] - self.theta_s(y[4]))*self.theta_s(y[4], 1)
        return jac
    
    def evalSpline(self, x, y, x_new, k=0):
        for i in range(len(x)-1):
            if x[i] <= x_new <= x[i+1]:
                if k == 0:
                    return y[i] + (y[i+1]-y[i])/(x[i+1]-x[i])*(x_new-x[i])
                else:
                    jac = np.zeros(len(x))
                    jac[i] = (x[i+1]-x_new)/(x[i+1]-x[i])
                    jac[i+1] = (x_new-x[i])/(x[i+1]-x[i])
                    return jac

    #- For Optimization -------------------------------------------------------
    def obj(self, p, states):
        self.p = p
        self.computeValue(0, self.dt, states)
        return 1e4 * self.z[0] + 1e3 * self.z[1]
    
    def obj_jac(self, p, states):
        self.p = p
        self.computeSensitivity(0, self.dt, states)
        return 1e4 * self.dzdp[0] + 1e3 * self.dzdp[1]

if __name__ == "__main__":
    rospy.init_node("Track1 Planner", disable_signals=True)
    vehicle = Vehicle1()
    rospy.spin()
