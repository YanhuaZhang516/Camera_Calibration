#!/usr/bin/env python
import rospy
import tf

from geometry_msgs.msg import TransformStamped, PoseStamped

from utils import makedirs

import sys, termios, tty, select
import numpy as np

import argparse
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState

from json_tricks import dumps
from std_msgs.msg import String


import os
from datetime import datetime

#ActionLib
import actionlib
from ias_robot_msgs.msg import SimpleGoToAction, SimpleGoToGoal, KinesteticAction, KinesteticGoal
from ias_robot_msgs.msg import State

import matplotlib.pyplot as plt


repo_dir = os.path.abspath(__file__ + "/../../")
data_dir = os.path.join(repo_dir,'data')


## Optitrack Elements
list_of_elements_env = ['greenDrum', 'blueDrum']
list_of_elements_human = ['stick']

list_of_elements_env = ['glass']

elements = [list_of_elements_env, list_of_elements_human]
parent_tf = 'world'
ee_tf = 'R_endeffector_link'

def parse_arguments():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--save_opt', type=bool, default=True, help='save_optitrack')
    parser.add_argument('--save_cartesian', type=bool, default=True, help='save_cartesian')
    args = parser.parse_args()
    return args

elements_list = []
def from_H_2_JSON(data):
    msg = []

    msg_json = dumps(data)

    msg_pub = String()
    msg_pub.data = msg_json
    return msg_pub

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


class ImitationLearningRecord():

    def __init__(self):
        # Saving Folder
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y_%H:%M:%S")
        self.savefolder = os.path.join(data_dir, dt_string)
        makedirs(self.savefolder)

        ########### Motion Capture Data ##############
        self.elements_env_pos = list_of_elements_env[:]
        self.elements_obj_pos = list_of_elements_human[:]
        self.obj_to_hand = None

        self.new_in = False
        self.tf = tf.TransformListener()

        ######### Kinestethis Control & GoTo Control ############
        self.client_goto = actionlib.SimpleActionClient('goto_node', SimpleGoToAction)
        self.client_goto.wait_for_server()
        self.client_kinest = actionlib.SimpleActionClient('kinesthetic_node', KinesteticAction)
        self.client_kinest.wait_for_server()

        self._goalKin = KinesteticGoal()
        group = "RIGHT_ARM"
        stateList = []
        st = State()
        st.duration = 5.
        st.destination = [-0.3, 0.9, 2.0, -1.4, 2.0, -0.5, 1.0]
        stateList.append(st)
        self._goHome = SimpleGoToGoal(group=group, states=stateList)

        ####### Cartesian Position Recording #######
        jnt_pos_topic = '/joint_states'
        rospy.Subscriber(jnt_pos_topic, JointState, self.callback_jnts)
        self.joint_state = np.zeros(7)
        self.joint_vel   = np.zeros(7)


    def callback_jnts(self, data):
            ## I will only save Right Arm joint position 15 - 21
            self.joint_state = np.array(data.position[15:22])
            self.joint_vel   = np.array(data.velocity[15:22])


    def update_motion_capture(self, elements):

        for index in range(len(elements[0])):
            elem_i = elements[0][index]
            try:
                (trans, rot) = self.tf.lookupTransform(parent_tf, elem_i, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.elements_env_pos[index] = np.array([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])

        for index in range(len(elements[1])):
            elem_i = elements[1][index]
            try:
                (trans, rot) = self.tf.lookupTransform(parent_tf, elem_i, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.elements_obj_pos = np.array([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])

            try:
                (trans, rot) = self.tf.lookupTransform(ee_tf, elem_i, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.obj_to_hand = np.array([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])



    def record(self,topic_name, args):
        rate = rospy.Rate(20.0)

        full_traj = []
        traj = []
        traj_r = []
        record = False

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        #new = termios.tcgetattr(fd)

        print("Robot Movement recording Tool \n Press c: Record conditioning elements "
              "\n Press k: Activate gravity compensation \n Press r: Start recording trajectories "
              "\n Press s: Save trajectory \n Press h: Send Robot Home Position \n Press esc: Close project")
        try:
            tty.setcbreak(sys.stdin.fileno())
            j = 0
            tr_i = 0
            counter = 0
            while not rospy.is_shutdown():
                self.update_motion_capture(elements)

                if record == True:# and self.new_in==True: #todo: Look for imposing the record only if new data in
                    traj.append(self.elements_obj_pos)
                    traj_r.append(self.joint_state)
                    self.new_in=False

                if isData():
                    c = sys.stdin.read(1)
                    if c == 'c':
                        print("Save conditioning information.")
                        j += 1
                        filename = str(j) + '_conditional.npy'
                        np.save(self.savefolder + '/' + filename, self.elements_env_pos)
                        ## SAVE END-EFFECTOR TO OBJ ##
                        filename = str(j) + '_obj_2_ee.npy'
                        np.save(self.savefolder + '/' + filename, self.obj_to_hand)

                        tr_i = 0

                    if c == 'k':
                        print("Activating kinesthetic mode.")
                        self.client_kinest.send_goal(self._goalKin)

                    if c == 'r':
                        print("Activate recording of data.")
                        record = True
                        traj = []
                        traj_r = []

                    if c == 's':
                        print("Record of trajectory finished")
                        print("Adding a new trajectory of length {}, You have now {} trajectories "
                              .format(len(traj), len(full_traj) + 1))
                        record = False
                        self.client_kinest.cancel_goal()

                        filename = str(j) + '_' + str(tr_i) + '_stick_traj.npy'
                        np.save(self.savefolder + '/' + filename, traj)
                        filename = str(j) + '_' + str(tr_i) + '_robot_traj.npy'
                        np.save(self.savefolder + '/' + filename, traj_r)

                        tr_i +=1

                    if c == 'h':
                        print("Move Home Position")
                        self.client_kinest.cancel_goal()
                        self.client_goto.send_goal(self._goHome)
                        self.client_goto.wait_for_result()

                    if c == '\x1b':  # x1b is ESC
                        print("Cancel all the robot controllers")
                        break

                rate.sleep()

        finally:
            print("Closing all data recorders")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


if __name__ == '__main__':
    args = parse_arguments()
    rospy.init_node('save_data', anonymous=True)
    try:
        topic_name = 'tf'
        recorder = ImitationLearningRecord()
        recorder.record(topic_name, args)
    except rospy.ROSInterruptException:
        pass
