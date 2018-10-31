#! /usr/bin/env python
"""This script makes roboy bob his head to the beat in a seperate thread"""

import rospy
# import threading
import std_srvs.srv, geometry_msgs.msg, std_msgs.msg
import roboy_communication_middleware.srv
import tf
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


class HeadBobber():
    def __init__(self, bpm=120):
        self.bpm = bpm
        self.get_ik = rospy.ServiceProxy('/ik', roboy_communication_middleware.srv.InverseKinematics)

        self.links = ['/sphere_head_axis0/sphere_head_axis0/target ',
                      '/sphere_head_axis1/sphere_head_axis1/target ',
                      '/sphere_head_axis2/sphere_head_axis2/target ']
        self.publisher =[]
        for link in self.links:
            self.publisher.append(rospy.Publisher(link, std_msgs.msg.Float32, queue_size=1))
        print("Publishers for links were initialized")

    def move_head(self):
        # TODO move head to metronome or beat (based on magenta metronome or bpm)
        pass



if __name__ == '__main__':
    # make roboy head move to beat
    rospy.init_node('head_bobber')
    roboy_head = HeadBobber()
