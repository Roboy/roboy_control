#! /usr/bin/env python

import rospy
#import threading
import std_srvs.srv, geometry_msgs.msg, std_msgs.msg
import roboy_communication_middleware.srv
import tf
from visualization_msgs.msg import Marker, MarkerArray


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
        self.key_pos_listener = tf.TransformListener()
        self.get_ik = rospy.ServiceProxy('/ik', roboy_communication_middleware.srv.InverseKinematics)
        self.links = ['/hip_joint/hip_joint/target', '/sphere_axis0/sphere_axis0/target', '/sphere_axis1/sphere_axis1/target',
                      '/sphere_axis2/sphere_axis2/target', '/elbow_left/elbow_left/target', '/wrist_left/wrist_left/target']
        self.publisher =[]
        for link in self.links:
            self.publisher.append(rospy.Publisher(link, std_msgs.msg.Float32,queue_size=1))
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

    def left_arm_publisher(self, ik_values):
        for angle, publ in zip(ik_values.angles, self.publisher):
            temp_msg = std_msgs.msg.Float32()
            temp_msg.data = angle
            publ.publish(temp_msg)
        print("PUBLISHED IK TO JOINTS")

    def hit_key(self, key_pos):
        # action to make roboy hit key
        current_pose = geometry_msgs.msg.Pose()
        current_pose.position.x = key_pos[0][0]
        current_pose.position.y = key_pos[0][1]
        current_pose.position.z = key_pos[0][2]

        print(current_pose)

        current_ik = self.get_ik.call("hand_left", 1, 'hand_left', current_pose)
        print("\ncurrent ik")
        print(current_ik)
        print("\n")

        self.left_arm_publisher(current_ik)


if __name__ == '__main__':
    rospy.init_node('xylophone_hitter')
    marker_publisher = rospy.Publisher('keys_visualization', Marker, queue_size=100)

    xyl = Xylophone()
    rospy.Rate(1).sleep()  # wait 1 second for init

    for note in xyl.notes_list[4:5]:
        print(note)
        current_key_pos = xyl.get_key_pos(note)
        print("position of %s" %note, current_key_pos)
        show_marker(current_key_pos, marker_publisher)
        xyl.hit_key(current_key_pos)
