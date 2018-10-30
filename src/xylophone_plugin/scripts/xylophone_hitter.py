#! /usr/bin/env python

import rospy
#import threading
import std_srvs.srv, geometry_msgs.msg, std_msgs.msg
import roboy_communication_middleware.srv
import tf
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


def show_marker(pose, publisher, color=(1,0,0,1), sphereSize=0.05):
    rate3 = rospy.Rate(10)
    rate3.sleep()
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.ns = 'keys_visualization'
    marker.type = Marker.SPHERE
    marker.pose.position.x = pose[0][0]
    marker.pose.position.y = pose[0][1]
    marker.pose.position.z = pose[0][2]
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
    marker.lifetime = rospy.Duration(5.);
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.ADD
    marker.id = 101

    publisher.publish(marker);
    print("marker published")


class Xylophone():
    def __init__(self):
        # TODO change C_sharo_0 to C_sharp_0
        self.notes_list = ['C_0','C_sharo_0', 'D_0', 'D_sharp_0', 'E_0', 'F_0', 'F_sharp_0', 'G_0', 'G_sharp_0', 'A_0', 'A_sharp_0', 'H_0',
                            'C_1','C_sharp_1', 'D_1', 'D_sharp_1', 'E_1', 'F_1', 'F_sharp_1', 'G_1', 'G_sharp_1', 'A_1', 'A_sharp_1', 'H_1',
                            'C_2','C_sharp_2', 'D_2', 'D_sharp_2', 'E_2', 'F_2', 'F_sharp_2', 'G_2', 'G_sharp_2', 'A_2', 'A_sharp_2', 'H_2',
                            'C_3']
        self.notes_dict = {48: 'C_0', 49: 'C_sharo_0', 50: 'D_0', 51: 'D_sharp_0', 52: 'E_0', 53: 'F_0',
                           54: 'F_sharp_0', 55: 'G_0', 56: 'G_sharp_0', 57: 'A_0', 58: 'A_sharp_0', 59: 'H_0',
                           60: 'C_1', 61: 'C_sharp_1', 62: 'D_1', 63: 'D_sharp_1', 64: 'E_1', 65: 'F_1',
                           66: 'F_sharp_1', 67: 'G_1', 68: 'G_sharp_1', 69: 'A_1', 70: 'A_sharp_1', 71: 'H_1',
                           72: 'C_2', 73: 'C_sharp_2', 74: 'D_2', 75: 'D_sharp_2', 76: 'E_2', 77: 'F_2',
                           78: 'F_sharp_2', 79: 'G_2', 80: 'G_sharp_2', 81: 'A_2', 82: 'A_sharp_2', 83: 'H_2', 84: 'C_3'}

        self.key_pos_listener = tf.TransformListener()
        self.get_ik = rospy.ServiceProxy('/ik', roboy_communication_middleware.srv.InverseKinematics)
        """
        #ROBOY XYLOPHONE MODEL, left arm only
        self.links = ['/hip_joint/hip_joint/target', '/sphere_axis0/sphere_axis0/target', '/sphere_axis1/sphere_axis1/target',
                      '/sphere_axis2/sphere_axis2/target', '/elbow_left/elbow_left/target', '/wrist_left/wrist_left/target']
        self.publisher =[]
        for link in self.links:
            self.publisher.append(rospy.Publisher(link, std_msgs.msg.Float32,queue_size=1))
        """
        #ROBOY UPPER BODY MODEL
        # hip
        self.hip_joint_publ = rospy.Publisher('/hip_joint/hip_joint/target', std_msgs.msg.Float32, queue_size=1)
        # left arm
        self.sphere_left_axis0_publ = rospy.Publisher('/sphere_left_axis0/sphere_left_axis0/target', std_msgs.msg.Float32, queue_size=1)
        self.sphere_left_axis1_publ = rospy.Publisher('/sphere_left_axis1/sphere_left_axis1/target', std_msgs.msg.Float32, queue_size=1)
        self.sphere_left_axis2_publ = rospy.Publisher('/sphere_left_axis2/sphere_left_axis2/target', std_msgs.msg.Float32, queue_size=1)
        self.elbow_left_rot0_publ = rospy.Publisher('/elbow_left_rot0/elbow_left_rot0/target', std_msgs.msg.Float32, queue_size=1)
        self.elbow_left_rot1_publ = rospy.Publisher('/elbow_left_rot1/elbow_left_rot1/target', std_msgs.msg.Float32, queue_size=1)
        self.left_wrist_0_publ = rospy.Publisher('/left_wrist_0/left_wrist_0/target', std_msgs.msg.Float32, queue_size=1)
        self.left_wrist_1_publ = rospy.Publisher('/left_wrist_1/left_wrist_1/target', std_msgs.msg.Float32, queue_size=1)
        # right arm
        self.sphere_right_axis0_publ = rospy.Publisher('/sphere_right_axis0/sphere_right_axis0/target', std_msgs.msg.Float32, queue_size=1)
        self.sphere_right_axis1_publ = rospy.Publisher('/sphere_right_axis1/sphere_right_axis1/target', std_msgs.msg.Float32, queue_size=1)
        self.sphere_right_axis2_publ = rospy.Publisher('/sphere_right_axis2/sphere_right_axis2/target', std_msgs.msg.Float32, queue_size=1)
        self.elbow_right_rot0_publ = rospy.Publisher('/elbow_right_rot0/elbow_right_rot0/target', std_msgs.msg.Float32, queue_size=1)
        self.elbow_right_rot1_publ = rospy.Publisher('/elbow_right_rot1/elbow_right_rot1/target', std_msgs.msg.Float32, queue_size=1)
        self.right_wrist_0_publ = rospy.Publisher('/right_wrist_0/right_wrist_0/target', std_msgs.msg.Float32, queue_size=1)
        self.right_wrist_1_publ = rospy.Publisher('/right_wrist_1/right_wrist_1/target', std_msgs.msg.Float32, queue_size=1)

        self.left_arm_publisher = [self.hip_joint_publ,
                                   self.sphere_left_axis0_publ, self.sphere_left_axis1_publ,self.sphere_left_axis2_publ,
                                   self.elbow_left_rot0_publ, self.elbow_left_rot1_publ,
                                   self.left_wrist_0_publ, self.left_wrist_1_publ]
        self.right_arm_publisher = [self.sphere_right_axis0_publ, self.sphere_right_axis1_publ,self.sphere_right_axis2_publ,
                                    self.elbow_right_rot0_publ, self.elbow_right_rot1_publ,
                                    self.right_wrist_0_publ, self.right_wrist_1_publ]
        print("Publishers for links were initialized")

    def get_key_pos(self, key):
        # get key positions
        position = None
        try:
            now = rospy.Time.now()
            self.key_pos_listener.waitForTransform("world", key, now, rospy.Duration(10.0))
            position = self.key_pos_listener.lookupTransform("world", key, now)
        except:
            print("couldn't find lookup transform of %s frame" %key)
        return position

    def move_arm(self, ik_values, arm_publisher):
        for angle, publ in zip(ik_values.angles, arm_publisher):
            temp_msg = std_msgs.msg.Float32()
            temp_msg.data = angle
            publ.publish(temp_msg)
        print("PUBLISHED IK TO JOINTS")

    def hit_key(self, note, key_pos):
        #TODO compute distance to key from each endeffector
        # action to make roboy hit key
        current_pose = geometry_msgs.msg.Pose()
        current_pose.position.x = key_pos[0][0]
        current_pose.position.y = key_pos[0][1]
        current_pose.position.z = key_pos[0][2]
        print(current_pose)

        if note < 67:
            current_ik = self.get_ik.call("palm", 1, 'palm', current_pose)
            print(current_ik)
            self.move_arm(current_ik, self.left_arm_publisher)
        else:
            current_ik = self.get_ik.call("right_palm", 1, 'right_palm', current_pose)
            print(current_ik)
            self.move_arm(current_ik, self.right_arm_publisher)


if __name__ == '__main__':
    rospy.init_node('xylophone_hitter')
    marker_publisher = rospy.Publisher('keys_visualization', Marker, queue_size=100)

    xyl = Xylophone()
    rate = rospy.Rate(1)
    rate.sleep()  # wait 1 second for init

    for i in range(20):
        rand_note = np.random.randint(48, 84)
        current_key_pos = xyl.get_key_pos(xyl.notes_dict[rand_note])
        print("position of %s" %xyl.notes_dict[rand_note], current_key_pos)
        show_marker(current_key_pos, marker_publisher)
        xyl.hit_key(rand_note, current_key_pos)
        rate.sleep()

    """
    for note in xyl.notes_list[20:]:
        print(note)
        current_key_pos = xyl.get_key_pos(note)
        print("position of %s" %note, current_key_pos)
        show_marker(current_key_pos, marker_publisher)
        xyl.hit_key(note, current_key_pos)
        rate.sleep()
    """