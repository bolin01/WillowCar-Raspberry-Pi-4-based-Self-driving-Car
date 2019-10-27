#!/usr/bin/env python

"""
The goal is to drive the vehicle along a straight line while
following the speed profile given.
"""

import rospy
from hkj_msgs.msg import RoadConditionVector, VehicleActuator
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from odeAdjoint import Ode1stExplicit

class Vehicle1(Ode1stExplicit):
    """
    Vehicle model for track 1
    """
    def __init__(self):
        self.dt = rospy.get_param("integration_dt")
        self.mpc_dt = rospy.get_param("mpc_horizon")
        self.t = 0.0      # Current time, updated when publish actuation input
        self.acc = 0.0    # Initial condition - acceleration
        self.steer = 0.0  # Initial condition - steering angle

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
        
        act_input = VehicleActuator()
        act_input.applied_force = [10.0] * 11
        act_input.steer_angle = [0.0] * 11
        self.pub.publish(act_input)

if __name__ == "__main__":
    rospy.init_node("Track1 Planner")
    vehicle = Vehicle1()
    rospy.spin()
