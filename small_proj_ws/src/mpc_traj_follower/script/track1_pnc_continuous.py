#!/usr/bin/env python

"""
The goal is to drive the vehicle along a straight line while
following the speed profile given.
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
        dim_y = 1
        dim_z = 1
        dim_p = 3   # 3rd order polynomial with one param fixed
        super().__init__(dim_y, dim_p, dim_z, method='RK45')
        self.p = np.zeros(dim_p)

        self.dt = rospy.get_param("integration_dt")
        self.mpc_dt = rospy.get_param("mpc_horizon")
        self.t = 0.0      # Current time, updated when publish actuation input
        self.acc = 0.0    # Initial condition - acceleration
        self.steer = 0.0  # Initial condition - steering angle
        self.planning_time = 0.0

        self.v_profile = self.generateSpeedProfile(
            t = [-1.0, 0.0, 10.0, 25.0, 30.0, 31.0],
            v = [0.0, 0.0, 16.67, 16.67, 0.0, 0.0],
            dt = 0.1
        )

        # ros publisher and subscriber
        self.sub = rospy.Subscriber("/perception", RoadConditionVector, self.callback, queue_size=10)
        self.pub = rospy.Publisher("/vehicle_actuator", VehicleActuator, queue_size=10)
        while self.pub.get_num_connections() < 1:
            continue
    
    def generateSpeedProfile(self, t, v, dt = 0.1):
        """
        Return a spline of speed profile.
            t = [t0, t1, ..., tn]
            v = [v0, v1, ..., vn]
            dt is the interval for interpolation
        """
        assert len(t) == len(v)
        t_sample = np.array([], dtype=np.single) 
        v_sample = np.array([], dtype=np.single)

        for i in range(len(t)-1):
            t0 = t[i]
            t1 = t[i+1]
            v0 = v[i]
            v1 = v[i+1]
            num_p = (t1-t0)//dt + 1
            t_sample = np.concatenate((t_sample, np.linspace(t0, t1, num_p, dtype=np.single)[1:]))
            v_sample = np.concatenate((v_sample, np.linspace(v0, v1, num_p, dtype=np.single)[1:]))

        return CubicSpline(t_sample, v_sample, bc_type = "clamped")

    def callback(self, data):
        """
        Run planning routine and publish the actuator input
        """
        if (self.t > 30):
            rospy.signal_shutdown("Simulation time larger than 30s")

        # Write history
        with open("track1_continuous.csv", "a") as f:
            f.write("{T}, {V}, {P}\n".format(
                T = self.t,
                V = data.vel_x,
                P = self.planning_time,
            ))
        
        # Run optimization
        start = time.time()
        v = np.array([data.vel_x])
        p = self.p
        res = minimize(
            fun = self.obj,
            x0 = p,
            bounds = ((-100, 100), (-1000, 1000), (-1000,1000)),
            args = (v,),
            method = 'SLSQP',
            jac = self.obj_jac,
            tol = 1e-6,
            options = dict(maxiter=20, disp=True)
        )

        # Compute acceleration
        acc_coef = [self.acc, res.x[0], res.x[1], res.x[2]]
        t = np.linspace(0, self.dt, 11)
        a = polyval(t, acc_coef)
        self.planning_time = time.time() - start

        # Update parameters
        self.p = res.x[:]
        self.t += self.dt
        self.acc = a[-1]   # The initial value of the next iteration is the last value in current iteration

        # Publish message  
        act_input = VehicleActuator()
        act_input.applied_force = a
        act_input.steer_angle = [0.0] * 11
        self.pub.publish(act_input)

    #- For integrator ---------------------------------------------------------
    def f(self, t, y):
        """
        We only need to compute v in track1, so return a vector of size 1
        """
        a_coef = [self.acc, self.p[0], self.p[1], self.p[2]]
        return np.array([polyval(t, a_coef)])

    def fj(self, t, y):
        return np.zeros((1,1))

    def fp(self, t, y):
        jac = np.zeros((1, self.dim_p))
        jac[0][0] = t
        jac[0][1] = t*t
        jac[0][2] = t*t*t
        return jac

    def I1(self, t, y):
        """
        Value of I1. Row vector with d = dim_z
        """
        return np.zeros(1)

    def dI1dp(self, t, y):
        """
        Derivative of I1 w.r.t parameters p. 
        dI1dp[i][j] - derivative of I1_i w.r.t p_j
        """
        return np.zeros((1,self.dim_p))

    def dI1dy(self, t, y):
        """
        Derivative of I1 w.r.t states y.
        dI1dy[i][j] - derivative of I1_i w.r.t y_j
        """
        return np.zeros((1, self.dim_y))

    def I2(self, t, y):
        """
        Value of I2. Row vector with d = dim_z
        """
        i2 = np.zeros(1)
        i2[0] = (y[0]-self.v_profile(t+self.t)) ** 2
        return i2

    def dI2dp(self, t, y):
        """
        Derivative of I2 w.r.t parameters p. 
        dI2dp[i][j] - derivative of I2_i w.r.t p_j
        """
        return np.zeros((1, self.dim_p))

    def dI2dy(self, t, y):
        """
        Derivative of I1 w.r.t states y.
        dI2dy[i][j] - derivative of I2_i w.r.t y_j
        """
        jac = np.zeros((1, self.dim_y))
        jac[0][0] = 2 * (y[0]-self.v_profile(t+self.t))
        return jac

    #- For Optimization -------------------------------------------------------
    def obj(self, p, states):
        self.p = p
        self.computeValue(0, self.dt, states)
        return 1e6 * self.z[0]
    
    def obj_jac(self, p, states):
        self.p = p
        self.computeSensitivity(0, self.dt, states)
        return 1e6 * self.dzdp

if __name__ == "__main__":
    rospy.init_node("Track1 Planner", disable_signals=True)
    vehicle = Vehicle1()
    rospy.spin()
