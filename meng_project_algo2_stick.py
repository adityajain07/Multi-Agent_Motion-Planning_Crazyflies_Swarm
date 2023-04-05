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
        sub_vicon_left_glove_data = rospy.Subscriber('/vicon/glove_left/glove_left',
                                            TransformStamped,
                                            self.update_left_glove_position)
        
        sub_vicon_right_glove_data = rospy.Subscriber('/vicon/glove_right/glove_right',
                                            TransformStamped,
                                            self.update_right_glove_position)
        # left glove position 
        self.cur_left_glove_x = 0.0
        self.cur_left_glove_y = 0.0
        self.cur_left_glove_z = 0.0

        # right glove position 
        self.cur_right_glove_x = 0.0
        self.cur_right_glove_y = 0.0
        self.cur_right_glove_z = 0.0

        # threshold change to trigger command to cf
        self.threshold = 0.01

        # variables for crazyflies
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        self.takeoff_height = 1.0
        self.takeoff_duration = 3.5
        self.takeoff_crazyflies()
        self.timeHelper = self.swarm.timeHelper
        self.timePassed = 0

        self.odd_list = [1,3,5,7]
        self.even_list = [2,4,6,91]

        # command the crazyflie at the below frequency
        print("Press s button to start object tracking")
        self.swarm.input.waitUntilButtonPressed()
        self.desired_cmd_frequency = 0.2

        self.command_crazyflies()
        # rospy.Timer(rospy.Duration(1 / self.desired_cmd_frequency), self.command_crazyflies)


    def update_left_glove_position(self, vicon_data):
        """updates the left glove's position globally in the class"""
        self.cur_left_glove_x = vicon_data.transform.translation.x
        self.cur_left_glove_y = vicon_data.transform.translation.y
        self.cur_left_glove_z = vicon_data.transform.translation.z

    def update_right_glove_position(self, vicon_data):
        """updates the right glove's position globally in the class"""
        self.cur_right_glove_x = vicon_data.transform.translation.x
        self.cur_right_glove_y = vicon_data.transform.translation.y
        self.cur_right_glove_z = vicon_data.transform.translation.z

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
        # last_object_position = np.array([self.cur_object_x , self.cur_object_y , self.cur_object_z])
        start_time = time.time()

        # set group mask
        for cf in self.allcfs.crazyflies:
            if cf.id in self.even_list:
                cf.setGroupMask(0)
            else:
                cf.setGroupMask(1)
        
        while ((time.time()-start_time)<20):
            # current_object_position  = np.array([self.cur_object_x , self.cur_object_y , self.cur_object_z])
            # diff = current_object_position - last_object_position
            self.allcfs.goTo([1, 1, 1],0, duration = 1.0, groupMask=0)
            self.allcfs.goTo([-1, -1, 1],0, duration = 1.0, groupMask=1)
            # diff_x = 0.0
            # diff_y = 0.0
            # diff_z = 0.0
            # position_changed = False
            # for cf in self.allcfs.crazyflies:
            #     cf_x = cf.position()[0]; cf_y = cf.position()[1]; cf_z = cf.position()[2]; 
            #     if (abs(diff[0])>self.threshold):
            #         diff_x = diff[0]
            #         last_object_position[0] = last_object_position[0] + diff[0]
            #         position_changed = True
            #     if (abs(diff[1])>self.threshold):
            #         diff_y = diff[1]
            #         last_object_position[1] = last_object_position[1] + diff[1]
            #         position_changed = True
            #     if (abs(diff[2])>self.threshold):
            #         diff_z = diff[2]
            #         last_object_position[2] = last_object_position[2] + diff[2]
            #         position_changed = True

            #     if position_changed:
            #         cf.goTo([cf_x+diff_x, cf_y+diff_y , cf_z+diff_z],0, duration = 1.0)
                
            #     print('Last Object Position:', last_object_position)
            self.timeHelper.sleep(2)
            # break

        self.land_crazyflies()
        sys.exit()

if __name__ == "__main__":
    rospy.init_node('command_crazyflies')
    CrazyflieObstacleAvoidance()
    rospy.spin()


