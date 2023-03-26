#!/usr/bin/env python

from __future__ import print_function

from pycrazyswarm import *
import rospy
from geometry_msgs.msg import TransformStamped

class CrazyflieObstacleAvoidance(object):
    """ROS interface for avoiding obstacles in real time for crazyflie swarm"""

    def __init__(self):
        # subscriber to obstacle's vicon data
        sub_vicon_data = rospy.Subscriber('/vicon/danda/danda',
                                            TransformStamped,
                                            self.update_obstacle_position)
        # obstacle position
        self.obs_x = 0.0
        self.obs_y = 0.0
        self.obs_z = 0.0

        # command the crazyflie at the below frequency
        self.desired_cmd_frequency = 5
        self.command_crazyflies()
        # rospy.Timer(rospy.Duration(1 / self.desired_cmd_frequency), self.command_crazyflies)

    def update_obstacle_position(self, vicon_data):
        """updates the obstacle's position globally in the function"""
        self.obs_x = vicon_data.transform.translation.x
        self.obs_y = vicon_data.transform.translation.y
        self.obs_z = vicon_data.transform.translation.z

    def command_crazyflies(self):
        """main function for controlling the crazyflies"""
        swarm = Crazyswarm()
        timeHelper = swarm.timeHelper
        allcfs = swarm.allcfs

        for cf in allcfs.crazyflies:
            cf.takeoff(targetHeight = 1.0, duration = 3.5)

        print("X: ", self.obs_x)
        print("Y: ", self.obs_y)
        print("Z: ", self.obs_z)
        print("Press l to land")
        swarm.input.waitUntilButtonPressed()
        for cf in allcfs.crazyflies:
            cf.land(targetHeight = 0.04, duration = 3.5)

    

if __name__ == "__main__":
    CrazyflieObstacleAvoidance()
    rospy.spin()
