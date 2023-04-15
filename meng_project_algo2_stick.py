#!/usr/bin/env python
from __future__ import print_function

from pycrazyswarm import *
import sys
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
import time

class CrazyflieObjectFollowing(object):
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
        self.threshold = 0.2 # 20 cm

        # total flying time
        self.flying_time = 30

        # desired cf velocity
        self.go_to_velocity = 0.1 # 10 cm/sec

        # variables for crazyflies
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        self.takeoff_height = 1.0
        self.takeoff_duration = 3.5
        self.takeoff_crazyflies()
        self.timeHelper = self.swarm.timeHelper
        self.timePassed = 0

        self.odd_list = [1,3,5,7]
        self.even_list = [2,4,6,8,91]

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

        # print(f'Left glove z position {self.cur_left_glove_z }')

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
        print('Land command initiated.')
        for cf in self.allcfs.crazyflies:
            cf.land(targetHeight = 0.04, duration = 5.0)


    def _calculate_offset_and_goto_duration(self, last_object_pos, current_object_pos):
        """calculates and returns the offsets and goTo duration"""
        
        diff = current_object_pos - last_object_pos
        euclidean_dist = np.linalg.norm(diff)

        if euclidean_dist > self.threshold:
            go_to_duration =  round(euclidean_dist/self.go_to_velocity, 1)
            return diff, go_to_duration
        else:    
            return 0, 0

    def command_crazyflies(self, event=None):
        """main function for controlling the crazyflies"""
        last_left_glove_position  = np.array([self.cur_left_glove_x,self.cur_left_glove_y,self.cur_left_glove_z])
        last_right_glove_position = np.array([self.cur_right_glove_x,self.cur_right_glove_y,self.cur_right_glove_z])
        start_time = time.time()

        # set group mask
        for cf in self.allcfs.crazyflies:
            if cf.id in self.even_list:
                cf.setGroupMask(2)
            else:
                cf.setGroupMask(1)
        
        while ((time.time()-start_time)<self.flying_time):
            # left glove control
            current_left_glove_position = np.array([self.cur_left_glove_x,self.cur_left_glove_y,self.cur_left_glove_z])
            diff, go_to_duration_left = self._calculate_offset_and_goto_duration(last_left_glove_position, current_left_glove_position)
            if go_to_duration_left:
                self.allcfs.goTo([diff[0],diff[1],diff[2]],0, duration = go_to_duration_left, groupMask=1)
                last_left_glove_position = current_left_glove_position

            # right glove control
            current_right_glove_position = np.array([self.cur_right_glove_x,self.cur_right_glove_y,self.cur_right_glove_z])
            diff, go_to_duration_right = self._calculate_offset_and_goto_duration(last_right_glove_position, current_right_glove_position)
            if go_to_duration_right:
                self.allcfs.goTo([diff[0],diff[1],diff[2]],0, duration = go_to_duration_right, groupMask=2)
                last_right_glove_position = current_right_glove_position

            # self.timeHelper.sleep(max(go_to_duration_left, go_to_duration_right)+0.5)
            self.timeHelper.sleep(1)

        self.land_crazyflies()
        sys.exit()

if __name__ == "__main__":
    # rospy.init_node('command_crazyflies')
    CrazyflieObjectFollowing()
    rospy.spin()


