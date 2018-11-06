#! /usr/bin/env python

import rospy
#import threading
import std_srvs.srv, geometry_msgs.msg, std_msgs.msg
import roboy_communication_middleware.srv
import tf
from visualization_msgs.msg import Marker
import numpy as np
import pyquaternion


def show_marker(pose, publisher, color=(1,0,0,1), sphereSize=0.05, frame = 'world' , is_pose=False):
    rate3 = rospy.Rate(10)
    rate3.sleep()
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = 'keys_visualization'
    marker.type = Marker.SPHERE
    if frame==('world') and is_pose==False:
        print('world + no pose')
        marker.pose.position.x = pose[0][0]
        marker.pose.position.y = pose[0][1]
        marker.pose.position.z = pose[0][2]
        marker.pose.orientation.x = pose[1][0]
        marker.pose.orientation.y = pose[1][1]
        marker.pose.orientation.z = pose[1][2]
        marker.pose.orientation.w = pose[1][3]
    if ((frame=='world') and is_pose):
        if frame=='left_hand':
            marker.pose = pose
        if frame=='right_hand':
            print('yup')
            marker.pose = pose

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

    publisher.publish(marker)
    # print("marker published")


class Xylophone():
    def __init__(self):
        # TODO change C_sharo_0 to C_sharp_0
        self.notes_list = ['C_0','C_sharo_0', 'D_0', 'D_sharp_0', 'E_0', 'F_0', 'F_sharp_0', 'G_0', 'G_sharp_0', 'A_0', 'A_sharp_0', 'H_0',
                           'C_1','C_sharp_1', 'D_1', 'D_sharp_1', 'E_1', 'F_1', 'F_sharp_1', 'G_1', 'G_sharp_1', 'A_1', 'A_sharp_1', 'H_1',
                           'C_2','C_sharp_2', 'D_2', 'D_sharp_2', 'E_2', 'F_2', 'F_sharp_2', 'G_2', 'G_sharp_2', 'A_2', 'A_sharp_2', 'H_2',
                           'C_3']
        # ints correspond to midi notes
        self.notes_dict = {48: 'C_0', 49: 'C_sharo_0', 50: 'D_0', 51: 'D_sharp_0', 52: 'E_0', 53: 'F_0',
                           54: 'F_sharp_0', 55: 'G_0', 56: 'G_sharp_0', 57: 'A_0', 58: 'A_sharp_0', 59: 'H_0',
                           60: 'C_1', 61: 'C_sharp_1', 62: 'D_1', 63: 'D_sharp_1', 64: 'E_1', 65: 'F_1',
                           66: 'F_sharp_1', 67: 'G_1', 68: 'G_sharp_1', 69: 'A_1', 70: 'A_sharp_1', 71: 'H_1',
                           72: 'C_2', 73: 'C_sharp_2', 74: 'D_2', 75: 'D_sharp_2', 76: 'E_2', 77: 'F_2',
                           78: 'F_sharp_2', 79: 'G_2', 80: 'G_sharp_2', 81: 'A_2', 82: 'A_sharp_2', 83: 'H_2', 84: 'C_3'}
        self.key_pos_listener = tf.TransformListener()

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


class Robot(Xylophone):
    def __init__(self):
        Xylophone.__init__(self)
        self.get_ik = rospy.ServiceProxy('/ik', roboy_communication_middleware.srv.InverseKinematics)
        self.tf_listener = tf.TransformListener()
        self.marker_publ = rospy.Publisher('keys_visualization', Marker, queue_size=1)
        # hip
        #self.hip_joint_publ = rospy.Publisher('/hip_joint/hip_joint/target', std_msgs.msg.Float32, queue_size=1)
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

        self.left_arm_publisher = [#self.hip_joint_publ,
                                   self.sphere_left_axis0_publ, self.sphere_left_axis1_publ,self.sphere_left_axis2_publ,
                                   self.elbow_left_rot0_publ, self.elbow_left_rot1_publ,
                                   self.left_wrist_0_publ, self.left_wrist_1_publ]
        self.right_arm_publisher = [self.sphere_right_axis0_publ, self.sphere_right_axis1_publ,self.sphere_right_axis2_publ,
                                    self.elbow_right_rot0_publ, self.elbow_right_rot1_publ,
                                    self.right_wrist_0_publ, self.right_wrist_1_publ]
        print("Publishers for links were initialized")

    def get_transform_pose(self, frame1, frame2):
        position = None
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(frame1, frame2, now, rospy.Duration(10.0))
            position = self.tf_listener.lookupTransform(frame1, frame2, now)
        except:
            print("Couldn't get transform of hand")
        return position

    def move_arm(self, ik_values, arm_publisher):
        for angle, publ in zip(ik_values.angles, arm_publisher):
            temp_msg = std_msgs.msg.Float32()
            temp_msg.data = angle
            publ.publish(temp_msg)
        # print("PUBLISHED IK TO JOINTS")

    def hit_motion(self, angle, arm_publisher):
        angle_rad = (angle*np.pi)/180
        msg = std_msgs.msg.Float32()
        msg.data = angle_rad
        arm_publisher.publish(msg)

    def hit_key(self, note, key_pos, how_high=0.2, how_y=0.3):
        # TODO compute distance to key from each endeffector
        # action to make roboy hit key
        current_pose = geometry_msgs.msg.Pose()
        current_pose.position.x = key_pos[0][0]
        current_pose.position.y = key_pos[0][1]+how_y
        current_pose.position.z = key_pos[0][2]+how_high

        if note < 67:
            # left hand
            try:
                current_ik = self.get_ik.call("left_hand", 1, 'left_hand', current_pose)
                # print(current_ik)
                self.move_arm(current_ik, self.left_arm_publisher)

                # hit_ik = self.get_ik.call()
                # self.hit_motion(32.4, self.left_wrist_0_publ)
            except:
                print("Couldn't solve ik for note %s" % self.notes_dict[note])
        else:
            # right hand
            try:
                current_ik = self.get_ik.call("right_hand", 1, 'right_hand', current_pose)
                # print(current_ik)
                self.move_arm(current_ik, self.right_arm_publisher)
                #self.hit_motion(32.4, self.right_wrist_1_publ)
            except:
                print("Couldn't solve ik for note %s" % self.notes_dict[note])

    def home(self, positions, how_high=0.2, how_y=0.2):
        # moves arms to home position above xylophone
        (left_arm_pos, right_arm_pos) = positions
        # initialize poses
        left_arm_home_pose = geometry_msgs.msg.Pose()
        left_arm_home_pose.position.x = left_arm_pos[0][0]
        left_arm_home_pose.position.y = left_arm_pos[0][1]+how_y
        left_arm_home_pose.position.z = left_arm_pos[0][2]+how_high
        right_arm_home_pose = geometry_msgs.msg.Pose()
        right_arm_home_pose.position.x = right_arm_pos[0][0]
        right_arm_home_pose.position.y = right_arm_pos[0][1]+how_y
        right_arm_home_pose.position.z = right_arm_pos[0][2]+how_high

        # get ik and move to home
        left_arm_ik = self.get_ik.call("left_hand", 1, "left_hand", left_arm_home_pose)
        right_arm_ik = self.get_ik.call("right_hand", 1, "right_hand", right_arm_home_pose)
        self.move_arm(left_arm_ik, self.left_arm_publisher)
        self.move_arm(right_arm_ik, self.right_arm_publisher)


if __name__ == '__main__':
    rospy.init_node('xylophone_hitter')
    marker_publisher = rospy.Publisher('keys_visualization', Marker, queue_size=1)

    xyl = Xylophone()
    roboy = Robot()
    rate = rospy.Rate(1)
    short_rate = rospy.Rate(5)
    rate.sleep()  # wait 1 second for init

    # show current ball position
    left_ball_pose =  [0., -0.2645, -0.07]
    right_ball_pose = [-0.0025, -0.27, -0.073]

    # show_marker(right_ball_pose, marker_publisher, frame='right_hand', sphereSize=0.02)
    # show_marker(left_ball_pose, marker_publisher, frame='left_hand', sphereSize=0.02)

    # right_hand_pose = roboy.get_transform_pose("world", "right_hand")
    right_wrist_pose = roboy.get_transform_pose("world", "wrist_right_1")
    # print('right_hand_pose:'); print(right_hand_pose);print('\n')
    print('right_wrist_pose:'); print(right_wrist_pose); print('\n')
    # show_marker(right_hand_pose, marker_publisher, frame='world', sphereSize=0.03, is_pose=True)
    # show_marker(right_wrist_pose, marker_publisher, frame='world', sphereSize=0.03, is_pose=True)

    # wrist_to_hand = roboy.get_transform_pose("right_hand", "wrist_right_1")
    # print("wrist_to_hand:"); print(wrist_to_hand); print("\n")
    wrist_to_hand = roboy.get_transform_pose("wrist_right_1","right_hand")
    print("wrist_to_hand:"); print(wrist_to_hand); print("\n")
    hand_quat = pyquaternion.Quaternion(wrist_to_hand[1])
    rotated_right_hand = hand_quat.rotate(wrist_to_hand[0])
    print(rotated_right_hand)

    right_hand_pos = [[right_wrist_pose[0][0]+rotated_right_hand[0],
                       right_wrist_pose[0][1]+rotated_right_hand[1],
                       right_wrist_pose[0][2]+rotated_right_hand[2]], [right_wrist_pose[1][0],right_wrist_pose[1][1],
                        right_wrist_pose[1][2], right_wrist_pose[1][3]]]

    show_marker(right_hand_pos, marker_publisher, sphereSize=0.05)
    #wrist_to_hand.inverseTimes(right_wrist_pos)

    # # define home pos
    # home_pos = (xyl.get_key_pos('F_0'), xyl.get_key_pos('F_2'))
    # print(home_pos)
    # roboy.home(home_pos)
    #
    # for i in range(10):
    #     rand_note = np.random.randint(48, 66)  # 48-84
    #     current_key_pos = xyl.get_key_pos(xyl.notes_dict[rand_note])
    #     # print("position of %s" %xyl.notes_dict[rand_note], current_key_pos)
    #     show_marker(current_key_pos, marker_publisher)
    #     roboy.hit_key(rand_note, current_key_pos)
    #     rate.sleep()
    #     # roboy.home(home_pos)
    #     # short_rate.sleep()

    """
    # good for testing if ik for all notes can be solved
    for note in xyl.notes_list[20:]:
        print(note)
        current_key_pos = xyl.get_key_pos(note)
        print("position of %s" %note, current_key_pos)
        show_marker(current_key_pos, marker_publisher)
        xyl.hit_key(note, current_key_pos)
        rate.sleep()
    """