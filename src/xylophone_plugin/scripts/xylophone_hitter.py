#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

#import threading

import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg

import tf
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import pyquaternion

def show_marker(pose, publisher, rate=100, color=(1,0,0,1), sphereSize=0.2):

    rate3 = rospy.Rate(rate)
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = 'test'
        marker.type = Marker.SPHERE
        marker.pose.position.x = -pose[0][0]
        marker.pose.position.y = -pose[0][1]
        marker.pose.position.z = -pose[0][2]
        marker.pose.orientation.x = pose[1][0]
        marker.pose.orientation.y = pose[1][1]
        marker.pose.orientation.z = pose[1][2]
        marker.pose.orientation.w = pose[1][3]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.scale.x = sphereSize
        marker.scale.y = sphereSize
        marker.scale.z = sphereSize
        marker.lifetime = rospy.Duration(10);
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.id = 101

        publisher.publish(marker);
        # print("marker published")
        rate3.sleep()

class Xylophone():
    def __init__(self):
        #TODO change C_sharo_0 to C_sharp_0
        self.notes_list = ['C_0','C_sharo_0', 'D_0', 'D_sharp_0', 'E_0', 'F_0', 'F_sharp_0', 'G_0', 'G_sharp_0', 'A_0', 'A_sharp_0', 'H_0',
                            'C_1','C_sharp_1', 'D_1', 'D_sharp_1', 'E_1', 'F_1', 'F_sharp_1', 'G_1', 'G_sharp_1', 'A_1', 'A_sharp_1', 'H_1',
                            'C_2','C_sharp_2', 'D_2', 'D_sharp_2', 'E_2', 'F_2', 'F_sharp_2', 'G_2', 'G_sharp_2', 'A_2', 'A_sharp_2', 'H_2']

    def hit_key(self, key_pos):
        #action to make roboy hit key
        pass

        # Signal handler
        #rospy.spin()

class TF_Listener():
    #get one key position and move to key
    #TODO get all key positions
    def __init__(self, *args):
        self.listener = tf.TransformListener()

    def get_key_pos(self, key):
        #get key positions
        position = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(key, "world", now, rospy.Duration(4.0))
            position = self.listener.lookupTransform(key, "world", now)
        except:
            print("couldn't find lookup transform of %s frame" %key)
        return position

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('xylophone_hitter')
    marker_publisher = rospy.Publisher('visualization', Marker, queue_size=100)

    xyl = Xylophone()
    listener = TF_Listener()


    for note in xyl.notes_list[:]:
        print(note)
        current_key_pos = listener.get_key_pos(note)
        print("position of %s" %note, current_key_pos)
        show_marker(current_key_pos, marker_publisher)
        #xyl.hit_key(current_key_pos)