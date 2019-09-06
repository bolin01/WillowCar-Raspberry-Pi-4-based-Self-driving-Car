#!/usr/bin/env python
import rospy, csv
from hkj_msgs.msg import VehicleState
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from math import pi
try:
    # for Python 2.x
    from StringIO import StringIO
except ImportError:
    # for Python 3.x
    from io import StringIO

big = 1e9


class Visualiser:
    def __init__(self, waypoints):
        self.fig, self.ax = self.plot_track(waypoints)
    
    def plot_track(self, waypoints):
        """
        Plot the track in matplotlib. The waypoints are given in a csv file (text string)
        """
        # Convert it to list of points
        f = StringIO(waypoints)
        waypoints = list(csv.reader(f, quoting=csv.QUOTE_NONNUMERIC))
        left_x = []
        left_y = []
        right_x = []
        right_y = []
        l_x = l_y = big
        u_x = u_y = -big
        for wp in waypoints:
            left_x.append(wp[0])
            left_y.append(wp[1])
            right_x.append(wp[2])
            right_y.append(wp[3])
            l_x = min(l_x, wp[0], wp[2])
            l_y = min(l_y, wp[1], wp[3])
            u_x = max(u_x, wp[0], wp[2])
            u_y = max(u_y, wp[1], wp[3])

        # Plot track - This is too small when obstacle is included. We can have a subplot
        fig = plt.figure(figsize=[12, 7])
        ax = fig.add_subplot(111)
        ax.axis([l_x,u_x,l_y,u_y])
        ax.plot(left_x, left_y, "b-")
        ax.plot(right_x, right_y, "b-")
        return fig, ax
    
    def callback(self, data):
        """
        Callback function used in subscriber
        """
        # Remove all patches (car) from the plot
        [p.remove() for p in self.ax.patches]

        # Add a new car
        car = Rectangle(
            (data.pos_x, data.pos_y),
            5, 2,   # width, height
            edgecolor = 'r',facecolor = 'r',
            angle = data.yaw_angle * 180.0 / pi
            )
        self.ax.add_patch(car)
        self.ax.set_xlim(data.pos_x-30, data.pos_x+30)
        self.ax.set_ylim(data.pos_y-30, data.pos_y+30)

        # Update canvas
        self.fig.canvas.draw()
        plt.pause(1e-3)

if __name__ == '__main__':
    rospy.init_node("Vehicle Plotter")

    waypoints = rospy.get_param("roadmap_file")
    viz = Visualiser(waypoints)

    rospy.Subscriber("/vehicle_states", VehicleState, viz.callback)
    plt.show(block=True)   # Don't use ros.spin here