#!/usr/bin/env python
import rospy
import numpy as np
import sys
import os

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import time
parent_dir = os.path.dirname(os.path.realpath('/home/rushi/mobile_robotics/final_project/car-racing/src/car_racing/utils'))

# Add the parent directory to sys.path
sys.path.append(parent_dir)
from utils import racing_env

parent_dir = os.path.dirname(os.path.realpath('/home/rushi/mobile_robotics/final_project/car-racing/src/car_racing/racing'))

# Add the parent directory to sys.path
sys.path.append(parent_dir)
from racing import realtime
from car_racing.msg import VehicleControl, VehicleState
from car_racing.srv import AddNewVehicle


def start_visualization():
    rospy.init_node("visualization", anonymous=True)
    fig = plt.figure(figsize=(6, 6))
    vis = realtime.Visualization()
    vis.fig = fig
    vis.set_subscriber_track()
    ax = plt.gca()
    ax.set_axis_off()
    ax.set_xlim(-6, 6)
    ax.set_ylim(-12, 8)
    ax.axis("equal")
    vis.set_ax(ax)
    # determine if new vehicle is added
    s = rospy.Service("add_vehicle_visualization", AddNewVehicle, vis.add_vehicle)
    while not rospy.is_shutdown():
        ani = animation.FuncAnimation(fig, vis.update, init_func=vis.init)
        plt.show()


if __name__ == "__main__":
    try:
        start_visualization()
    except rospy.ROSInterruptException:
        pass
