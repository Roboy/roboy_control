#! /usr/bin/env python

import rospy
import threading
import std_srvs.srv, geometry_msgs.msg, std_msgs.msg
import roboy_middleware_msgs.srv
from roboy_control_msgs.msg import MoveEndEffectorAction, MoveEndEffectorResult, MoveEndEffectorGoal
import tf
from visualization_msgs.msg import Marker
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray
import actionlib
import mido
import time


class Xylophone():
    def __init__(self):
        # TODO change C_sharo_0 to C_sharp_0 and change note names to the correct octave
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
        self.key_positions = []

    def get_key_pos(self, key):
        # get key positions
        position = None
        try:
            now = rospy.Time.now()
            self.key_pos_listener.waitForTransform("world", key, now, rospy.Duration(10))
            position = self.key_pos_listener.lookupTransform("world", key, now)
        except:
            print("couldn't find lookup transform of %s frame" %key)
        return position

    def save_key_positions(self):
        for key in self.notes_list:
            self.key_positions.append(self.get_key_pos(key))


class Robot(Xylophone):
    def __init__(self):
        Xylophone.__init__(self)
        self.queue_size = 10
        self.get_ik = rospy.ServiceProxy('/ik', roboy_middleware_msgs.srv.InverseKinematics)
        self.tf_listener = tf.TransformListener()
        self.marker_publ = rospy.Publisher('stick_visualization', Marker, queue_size=100)
        self.rate = rospy.Rate(0.5)
        self.hit_rate = rospy.Rate(0.1)
        self.sequence = np.zeros((96,60))
        self.midi_msg = 0
        self.hit_thread_left = threading.Thread(target=self.hit_key, args=('left',))
        self.hit_thread_left.setDaemon(True)
        self.hit_thread_right = threading.Thread(target=self.hit_key, args=('right',))
        self.hit_thread_right.setDaemon(True)
        self.home_pos = 0
        self.action_client_left = actionlib.SimpleActionClient('CARDSflow/MoveEndEffector/left_stick_tip',
                                                                    MoveEndEffectorAction)
        self.action_client_right = actionlib.SimpleActionClient('CARDSflow/MoveEndEffector/right_stick_tip',
                                                                    MoveEndEffectorAction)
        self.midi_outport = None
        # left arm
        self.sphere_left_axis0_publ = rospy.Publisher('/sphere_left_axis0/sphere_left_axis0/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.sphere_left_axis1_publ = rospy.Publisher('/sphere_left_axis1/sphere_left_axis1/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.sphere_left_axis2_publ = rospy.Publisher('/sphere_left_axis2/sphere_left_axis2/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.elbow_left_rot0_publ = rospy.Publisher('/elbow_left_rot0/elbow_left_rot0/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.elbow_left_rot1_publ = rospy.Publisher('/elbow_left_rot1/elbow_left_rot1/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.left_wrist_0_publ = rospy.Publisher('/left_wrist_0/left_wrist_0/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.left_wrist_1_publ = rospy.Publisher('/left_wrist_1/left_wrist_1/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.left_stick_joint_publ = rospy.Publisher('/left_stick_tip_joint/left_stick_tip_joint/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        # right arm
        self.sphere_right_axis0_publ = rospy.Publisher('/sphere_right_axis0/sphere_right_axis0/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.sphere_right_axis1_publ = rospy.Publisher('/sphere_right_axis1/sphere_right_axis1/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.sphere_right_axis2_publ = rospy.Publisher('/sphere_right_axis2/sphere_right_axis2/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.elbow_right_rot0_publ = rospy.Publisher('/elbow_right_rot0/elbow_right_rot0/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.elbow_right_rot1_publ = rospy.Publisher('/elbow_right_rot1/elbow_right_rot1/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.right_wrist_0_publ = rospy.Publisher('/right_wrist_0/right_wrist_0/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.right_wrist_1_publ = rospy.Publisher('/right_wrist_1/right_wrist_1/target', std_msgs.msg.Float32, queue_size=self.queue_size)
        self.right_stick_joint_publ = rospy.Publisher('/right_stick_tip_joint/right_stick_tip_joint/target', std_msgs.msg.Float32, queue_size=self.queue_size)

        self.left_stick_publisher = [self.sphere_left_axis0_publ, self.sphere_left_axis1_publ,self.sphere_left_axis2_publ,
                                    self.elbow_left_rot0_publ, self.elbow_left_rot1_publ,
                                    self.left_wrist_0_publ, self.left_wrist_1_publ, self.left_stick_joint_publ]
        self.right_stick_publisher = [self.sphere_right_axis0_publ, self.sphere_right_axis1_publ,self.sphere_right_axis2_publ,
                                    self.elbow_right_rot0_publ, self.elbow_right_rot1_publ,
                                    self.right_wrist_0_publ, self.right_wrist_1_publ, self.right_stick_joint_publ]
        self.left_hand_publisher = self.left_stick_publisher[:-1]
        self.right_hand_publisher = self.right_stick_publisher[:-1]
        print("Publishers for links were initialized")

    def open_outport(self):
        # TODO autoroute midi port to virtual synth possible??
        avail_out_ports = mido.get_output_names()
        ports_dict = {i: avail_out_ports[i] for i in range(len(avail_out_ports))}
        port = None
        for i in range(len(avail_out_ports)):
            if "Synth input" in ports_dict[i]:  # Better way than looking for this string?
                port = ports_dict[i]
        if port:
            self.midi_outport = mido.open_output(port)
            print("Found FLUID Synth and autoconnected!")
        else:
            self.midi_outport = mido.open_output("Robot port", virtual=True)
            print("Could not find FLUID Synth, created virtual midi port called 'Robot port'")

    def get_transform(self, frame1, frame2):
        trans, rot = [0, 0, 0], [0, 0, 0, 1]
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(frame1, frame2, now, rospy.Duration(10.0))
            trans, rot = self.tf_listener.lookupTransform(frame1, frame2, now)
        except:
            print("Couldn't get transform of hand")
        return np.array(trans), np.array(rot)

    def move_arm(self, ik_values, arm_publisher):
        for angle, publ in zip(ik_values.angles, arm_publisher):
            temp_msg = std_msgs.msg.Float32()
            temp_msg.data = angle
            publ.publish(temp_msg)
        # print("PUBLISHED IK TO JOINTS")

    def hit_motion(self, publishers, angles=(0., 0.)):
        (angle1, angle2) = angles
        print(angle1, angle2)
        (publisher1, publisher2) = publishers
        angle1_rad = (angle1*np.pi)/180
        angle2_rad = (angle2*np.pi)/180
        msg1 = std_msgs.msg.Float32()
        msg2 = std_msgs.msg.Float32()
        msg1.data = angle1_rad
        msg2.data = angle2_rad
        # publish hit
        publisher2.publish(msg2)
        publisher1.publish(msg1)

    def hit_key(self, left_or_right):
        # action to make roboy hit key
        note = self.midi_msg[1]
        key_pos = self.key_positions[note-48]
        prepare_height = 0.#0.4
        prepare_y = 0.#0.15
        prepare_hit_pose = geometry_msgs.msg.Pose()
        prepare_hit_pose.position.x = key_pos[0][0]
        prepare_hit_pose.position.y = key_pos[0][1] + prepare_y
        prepare_hit_pose.position.z = key_pos[0][2] + prepare_height
        hit_pose = geometry_msgs.msg.Pose()

        # left hand
        if left_or_right == 'left':
            self.action_client_left.wait_for_server()
            # hit key
            goal = MoveEndEffectorGoal(endeffector='left_stick_tip',
                                            type=0, ik_type=1, pose=prepare_hit_pose,
                                            target_frame='left_stick_tip',
                                            timeout=30, tolerance=0.1)
            # goal.note = note
            self.action_client_left.send_goal(goal)
            self.action_client_left.wait_for_result(rospy.Duration.from_sec(5.))
            prepare_hit_pose.position.z += 0.1
            goal = MoveEndEffectorGoal(endeffector='left_stick_tip',
                                            type=0, ik_type=1, pose=prepare_hit_pose,
                                            target_frame='left_stick_tip',
                                            timeout=30, tolerance=0.1, id=note)
            self.action_client_left.send_goal(goal)
            self.action_client_left.wait_for_result(rospy.Duration.from_sec(0.5))

        # right hand
        else:
            self.action_client_right.wait_for_server()
            # hit key
            goal = MoveEndEffectorGoal(endeffector='right_stick_tip',
                                            type=0, ik_type=1, pose=prepare_hit_pose,
                                            target_frame='right_stick_tip',
                                            timeout=30, tolerance=0.1)
            self.action_client_right.send_goal(goal)
            self.action_client_right.wait_for_result(rospy.Duration.from_sec(5.))
            # rospy.Subscriber("/CARDSflow/MoveEndEffector/right_stick_tip/result")
            prepare_hit_pose.position.z += 0.1
            goal = MoveEndEffectorGoal(endeffector='right_stick_tip',
                                            type=0, ik_type=1, pose=prepare_hit_pose,
                                            target_frame='right_stick_tip',
                                            timeout=30, tolerance=0.1, id=note)
            self.action_client_right.send_goal(goal)
            self.action_client_right.wait_for_result(rospy.Duration.from_sec(0.5))


    def home(self, how_high=0.15, how_y=0.1):
        # moves arms to home position above xylophone
        (left_arm_pos, right_arm_pos) = self.home_pos
        left_arm_home_pose = geometry_msgs.msg.Pose()
        left_arm_home_pose.position.x = left_arm_pos[0][0]
        left_arm_home_pose.position.y = left_arm_pos[0][1] + how_y
        left_arm_home_pose.position.z = left_arm_pos[0][2] + how_high
        right_arm_home_pose = geometry_msgs.msg.Pose()
        right_arm_home_pose.position.x = right_arm_pos[0][0]
        right_arm_home_pose.position.y = right_arm_pos[0][1] + how_y
        right_arm_home_pose.position.z = right_arm_pos[0][2] + how_high

        # get ik and move to home
        left_arm_ik = self.get_ik.call("left_stick_tip", 1, "left_stick_tip", left_arm_home_pose)
        right_arm_ik = self.get_ik.call("right_stick_tip", 1, "right_stick_tip", right_arm_home_pose)
        self.move_arm(left_arm_ik, self.left_hand_publisher)
        self.move_arm(right_arm_ik, self.right_hand_publisher)

    def play_note_callback(self, msg):
        midi_note = msg.result.id
        # print(msg)
        if(midi_note):
            # print("play note = {}".format(msg.result.note))
            self.midi_outport.send(mido.Message('note_on',
                                note=midi_note, velocity=40, time=10))

            # self.midi_outport.send(mido.Message('note_off',
                                # note=midi_note, velocity=64, time=400))

    def play_note_thread(self, left_or_right):
        if left_or_right == 'left':
            rospy.Subscriber("/CARDSflow/MoveEndEffector/left_stick_tip/result", MoveEndEffectorResult,
                                                    self.play_note_callback)
            rospy.spin()
        if left_or_right == 'right':
            rospy.Subscriber("/CARDSflow/MoveEndEffector/right_stick_tip/result", MoveEndEffectorResult,
                                                    self.play_note_callback)
            rospy.spin()

    def midi_callback(self, midi_msg):
        self.midi_msg = list(midi_msg.data)
        #transpose lowest highest to xylophone range
        if self.midi_msg[1] >= 36 and self.midi_msg[1] <= 48:
            self.midi_msg[1]+=12
        if self.midi_msg[1] >= 86 and self.midi_msg[1] < 98:
            self.midi_msg[1]-=12

        if self.midi_msg[1] < 66:
            if not self.hit_thread_left.is_alive():
                self.left_play_thread = threading.Thread(target=self.play_note_thread, args=('left',))
                self.left_play_thread.setDaemon(True)
                self.left_play_thread.start()
                self.hit_thread_left = threading.Thread(target=self.hit_key, args=('left',))
                self.hit_thread_left.setDaemon(True)
                self.hit_thread_left.start()
        else:
            if not self.hit_thread_left.is_alive():
                self.right_play_thread = threading.Thread(target=self.play_note_thread, args=('right',))
                self.right_play_thread.setDaemon(True)
                self.right_play_thread.start()
                self.hit_thread_left = threading.Thread(target=self.hit_key, args=('right',))
                self.hit_thread_left.setDaemon(True)
                self.hit_thread_left.start()


    def play(self):
        rospy.Subscriber("midi_publisher", Int32MultiArray, self.midi_callback, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('xylophone_hitter')

    roboy = Robot()
    roboy.open_outport()
    rate = rospy.Rate(1)
    short_rate = rospy.Rate(5)
    long_rate = rospy.Rate(0.2)
    rate.sleep()  # wait 1 second for init

    # get and save all key positions
    roboy.save_key_positions()
    # print(roboy.key_positions)

    # define home pos
    roboy.home_pos = (roboy.get_key_pos('F_0'), roboy.get_key_pos('F_2'))
    roboy.home()
    rate.sleep()

    print("I am ready to receive MIDI notes.")
    roboy.play()

    # for i in range(6):
    #     rand_note = np.random.randint(48, 65)  # 48-84
    #     # rand_note = 48+i
    #     roboy.hit_key(rand_note)
    #     rate.sleep()
    #     roboy.home()
    #     rate.sleep()
