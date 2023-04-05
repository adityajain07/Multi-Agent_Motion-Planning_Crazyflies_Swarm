#!/usr/bin/env python
from __future__ import print_function

from pycrazyswarm import *
import sys
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
import time

class CrazyflieObstacleAvoidance(object):
    """ROS interface for avoiding obstacles in real time for crazyflie swarm"""

    def __init__(self):
        # subscriber to obstacle's vicon data
        sub_vicon_data = rospy.Subscriber('/vicon/glove_right/glove_right',
                                            TransformStamped,
                                            self.update_obstacle_position)
        # object position position
        self.cur_object_x = 0.0
        self.cur_object_y = 0.0
        self.cur_object_z = 0.0
        
        # self.last_object_x = 0.0
        # self.last_object_y = 0.0
        # self.last_object_z = 0.0

        # TO ADD COMMENT
        self.threshold = 0.01

        # variables for crazyflies
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        self.takeoff_height = 1.0
        self.takeoff_duration = 3.5
        self.takeoff_crazyflies()
        self.timeHelper = self.swarm.timeHelper
        self.timePassed = 0

        # command the crazyflie at the below frequency
        print("Press s button to start object tracking")
        self.swarm.input.waitUntilButtonPressed()
        self.desired_cmd_frequency = 0.2

        self.command_crazyflies()
        # rospy.Timer(rospy.Duration(1 / self.desired_cmd_frequency), self.command_crazyflies)


    def update_obstacle_position(self, vicon_data):
        """updates the obstacle's position globally in the function"""
        self.cur_object_x = vicon_data.transform.translation.x
        self.cur_object_y = vicon_data.transform.translation.y
        self.cur_object_z = vicon_data.transform.translation.z

    def takeoff_crazyflies(self):
        """taking off crazyflies"""
        for cf in self.allcfs.crazyflies:
            cf.takeoff(targetHeight = self.takeoff_height, duration = self.takeoff_duration)
            cf.setGroupMask(1)
    
    def land_crazyflies(self):
        """Landing crazyflies"""
        for cf in self.allcfs.crazyflies:
            cf.land(targetHeight = 0.04, duration = 5.0)


    def command_crazyflies(self, event=None):
        """main function for controlling the crazyflies"""
        last_object_position = np.array([self.cur_object_x , self.cur_object_y , self.cur_object_z])
        start_time = time.time()
        
        while ((time.time()-start_time)<20):
            current_object_position  = np.array([self.cur_object_x , self.cur_object_y , self.cur_object_z])
            diff = current_object_position - last_object_position
            diff_x = 0.0
            diff_y = 0.0
            diff_z = 0.0
            position_changed = False
            for cf in self.allcfs.crazyflies:
                cf_x = cf.position()[0]; cf_y = cf.position()[1]; cf_z = cf.position()[2]; 
                if (abs(diff[0])>self.threshold):
                    diff_x = diff[0]
                    last_object_position[0] = last_object_position[0] + diff[0]
                    position_changed = True
                if (abs(diff[1])>self.threshold):
                    diff_y = diff[1]
                    last_object_position[1] = last_object_position[1] + diff[1]
                    position_changed = True
                if (abs(diff[2])>self.threshold):
                    diff_z = diff[2]
                    last_object_position[2] = last_object_position[2] + diff[2]
                    position_changed = True

                if position_changed:
                    cf.goTo([cf_x+diff_x, cf_y+diff_y , cf_z+diff_z],0, duration = 1.0)
                
                print('Last Object Position:', last_object_position)
            self.timeHelper.sleep(0.5)

        self.land_crazyflies()
        sys.exit()

if __name__ == "__main__":
    # rospy.init_node('command_crazyflies')
    CrazyflieObstacleAvoidance()
    rospy.spin()


