#!/usr/bin/env python
from __future__ import print_function

from pycrazyswarm import *
import rospy
import time 
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

        # variables for crazyflies
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        self.takeoff_height = 1.0
        self.takeoff_duration = 3.5
        self.takeoff_crazyflies()
        self.timeHelper = self.swarm.timeHelper

        # command the crazyflie at the below frequency
        print("Press s button to start object tracking")
        self.swarm.input.waitUntilButtonPressed()
        self.desired_cmd_frequency = 0.2

        self.command_crazyflies()
        # rospy.Timer(rospy.Duration(1 / self.desired_cmd_frequency), self.command_crazyflies)


    def update_obstacle_position(self, vicon_data):
        """updates the obstacle's position globally in the function"""
        self.obs_x = vicon_data.transform.translation.x
        self.obs_y = vicon_data.transform.translation.y
        self.obs_z = vicon_data.transform.translation.z

    def takeoff_crazyflies(self):
        """taking off crazyflies"""
        for cf in self.allcfs.crazyflies:
            cf.takeoff(targetHeight = self.takeoff_height, duration = self.takeoff_duration)
            cf.setGroupMask(1)


    def command_crazyflies(self, event=None):
        """main function for controlling the crazyflies"""
        while (1):
            
            for cf in self.allcfs.crazyflies:
                print(f'Drone {cf.id}s X position is: {cf.position()[0]}, Y position is: {cf.position()[1]}, Z position is: {cf.position()[2]}')
                print(f'Objects X position is: {self.obs_x}, Y position is: {self.obs_y}, Z position is: {self.obs_z}')
                cf.cmdPosition([cf.position()[0], cf.position()[1], self.obs_z])
                self.timeHelper.sleep(0.1)
            
        

if __name__ == "__main__":
    rospy.init_node('command_crazyflies')
    CrazyflieObstacleAvoidance()
    rospy.spin()
