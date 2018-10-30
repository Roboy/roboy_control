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
        self.hip_joint_publ = rospy.Publisher('/hip_joint/hip_joint/target', std_msgs.msg.Float32, queue_size=1)
        self.sphere_left_axis0_publ = rospy.Publisher('/sphere_left_axis0/sphere_left_axis0/target', std_msgs.msg.Float32, queue_size=1)
        self.sphere_left_axis1_publ = rospy.Publisher('/sphere_left_axis1/sphere_left_axis1/target', std_msgs.msg.Float32, queue_size=1)
        self.sphere_left_axis2_publ = rospy.Publisher('/sphere_left_axis2/sphere_left_axis2/target', std_msgs.msg.Float32, queue_size=1)
        self.elbow_left_rot0_publ = rospy.Publisher('/elbow_left_rot0/elbow_left_rot0/target', std_msgs.msg.Float32, queue_size=1)
        self.elbow_left_rot1_publ = rospy.Publisher('/elbow_left_rot1/elbow_left_rot1/target', std_msgs.msg.Float32, queue_size=1)
        self.left_wrist_0_publ = rospy.Publisher('/left_wrist_0/left_wrist_0/target', std_msgs.msg.Float32, queue_size=1)
        self.left_wrist_1_publ = rospy.Publisher('/left_wrist_1/left_wrist_1/target', std_msgs.msg.Float32, queue_size=1)

        # self.left_arm_publisher_list = [self.hip_joint_publ,
        #                            self.sphere_left_axis0_publ,
        #                            self.sphere_left_axis1_publ,
        #                            self.sphere_left_axis2_publ,
        #                            self.elbow_left_rot0_publ,
        #                            self.elbow_left_rot1_publ]

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
        #TODO improve
        msg0 = std_msgs.msg.Float32()
        msg0.data = ik_values.angles[0]
        msg1 = std_msgs.msg.Float32()
        msg1.data = ik_values.angles[1]
        msg2 = std_msgs.msg.Float32()
        msg2.data = ik_values.angles[2]
        msg3 = std_msgs.msg.Float32()
        msg3.data = ik_values.angles[3]
        msg4 = std_msgs.msg.Float32()
        msg4.data = ik_values.angles[4]
        msg5 = std_msgs.msg.Float32()
        msg5.data = ik_values.angles[5]
        msg6 = std_msgs.msg.Float32()
        msg6.data = ik_values.angles[6]
        msg7 = std_msgs.msg.Float32()
        msg7.data = ik_values.angles[7]

        self.hip_joint_publ.publish(msg0)
        self.sphere_left_axis0_publ.publish(msg1)
        self.sphere_left_axis1_publ.publish(msg2)
        self.sphere_left_axis2_publ.publish(msg3)
        self.elbow_left_rot0_publ.publish(msg4)
        self.elbow_left_rot1_publ.publish(msg5)
        self.left_wrist_0_publ.publish(msg6)
        self.left_wrist_1_publ.publish(msg7)
        print("PUBLISHED IK TO JOINTS")

    def hit_key(self, key_pos):
        # action to make roboy hit key
        current_pose = geometry_msgs.msg.Pose()
        current_pose.position.x = key_pos[0][0]
        current_pose.position.y = key_pos[0][1]
        current_pose.position.z = key_pos[0][2]

        # not needed for type 1 message which assumes position in world frame
        # current_pose.orientation.x = key_pos[1][0]
        # current_pose.orientation.y = key_pos[1][1]
        # current_pose.orientation.z = key_pos[1][2]
        # current_pose.orientation.w = key_pos[1][3]

        print(current_pose)

        current_ik = self.get_ik.call("palm", 1, 'palm', current_pose)
        print("\ncurrent ik")
        print(current_ik)
        print("\n")

        self.left_arm_publisher(current_ik)


if __name__ == '__main__':
    rospy.init_node('xylophone_hitter')
    marker_publisher = rospy.Publisher('visualization', Marker, queue_size=100)

    xyl = Xylophone()
    rospy.Rate(1).sleep()  # for init

    for note in xyl.notes_list[:]:
        print(note)
        current_key_pos = xyl.get_key_pos(note)
        print("position of %s" %note, current_key_pos)
        show_marker(current_key_pos, marker_publisher)
        xyl.hit_key(current_key_pos)
