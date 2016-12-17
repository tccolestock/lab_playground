#!/usr/bin/env python

"""
emg_UR10move_NSF.py: A simple script to move the Universal Robot 10 (UR10) through a range of motion
 that brings it to a pick up location, and then to a drop off location. The grasping and releasing
 of the object (TBD) will be executed by the Shadow Hand (SH) from a different script.

BioRobotics Lab, Florida Atlantic University, 2016
"""

from __future__ import division

__author__ = "Thomas Colestock"
__version__ = "1.0.0"

import time

import rospy

from std_msgs.msg import String, Float32, UInt8
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("ur10_move", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()


# --------------- Arm Positions ---------------
arm_start = {
    'ra_shoulder_pan_joint': -1.6755197683917444,
    'ra_elbow_joint': 2.391160726547241,
    'ra_wrist_1_joint': 2.303798198699951,
    'ra_shoulder_lift_joint': -1.5533440748797815,
    'ra_wrist_3_joint': -3.10664946237673,
    'rh_WRJ2': -0.08740126759572807,
    'rh_WRJ1': -0.009642963029241673,
    'ra_wrist_2_joint': -1.5882452170001429,
            }

arm_pickup = {
    'ra_shoulder_pan_joint': -0.575897518788473,
    'ra_elbow_joint': 2.86228346824646,
    'ra_wrist_1_joint': 1.6754974126815796,
    'ra_shoulder_lift_joint': -1.2914817968951624,
    'ra_wrist_3_joint': -1.5357773939715784,
    'rh_WRJ2': 0.05646164732737393,
    'rh_WRJ1': -0.10736475895393359,
    'ra_wrist_2_joint': -1.5881970564471644,
            }

arm_exit_pickup = {
    'ra_shoulder_pan_joint': -0.575909439717428,
    'ra_elbow_joint': 2.7576346397399902,
    'ra_wrist_1_joint': 1.8324915170669556,
    'ra_shoulder_lift_joint': -1.4485862890826624,
    'ra_wrist_3_joint': -1.5358369986163538,
    'rh_WRJ2': -0.008102103551746979,
    'rh_WRJ1': -0.10673035727744258,
    'ra_wrist_2_joint': -1.5882094542132776,
                }

arm_midway = {
    'ra_shoulder_pan_joint': -1.780236546193258,
    'ra_elbow_joint': 2.7576465606689453,
    'ra_wrist_1_joint': 1.8324674367904663,
    'ra_shoulder_lift_joint': -1.4485982100116175,
    'ra_wrist_3_joint': -1.5358369986163538,
    'rh_WRJ2': -0.008395394812687014,
    'rh_WRJ1': -0.10545759885212826,
    'ra_wrist_2_joint': -1.5882094542132776,
            }

arm_release = {
    'ra_shoulder_pan_joint': -3.027827803288595,
    'ra_elbow_joint': 2.6113691329956055,
    'ra_wrist_1_joint': 1.8882097005844116,
    'ra_shoulder_lift_joint': -1.3068426291095179,
    'ra_wrist_3_joint': -1.4986370245562952,
    'rh_WRJ2': -0.103164843927744,
    'rh_WRJ1': -0.10998772922135532,
    'ra_wrist_2_joint': -1.595231835042135,
            }

# =============== Main ===============
if __name__ == '__main__':
    # Move arm and hand to start position
    joint_goals = arm_start
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)

    # Move arm to pickup location
    joint_goals = arm_pickup
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)

    time.sleep(1)

    # Exit the pickup zone
    joint_goals = arm_exit_pickup
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3, True)

    # Go to the midway point
    joint_goals = arm_midway
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, False)

    # Go to the release area
    joint_goals = arm_release
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 5, True)
