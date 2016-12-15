#!/usr/bin/env python

# use ros_myo myo-rawNode-right.py for emg readings with this program.
#   - otherwise need to respecify the emg topic being subscribed to.

from __future__ import division
import time
import rospy
from std_msgs.msg import String, Float32, UInt8
from ros_myo.msg import EmgArray
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("emg_grasp", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

rate_handle = rospy.Rate(50) #hz

emg_min = 100
emg_max = 1000




class MovingFilter(object):

    def __init__(self, windowSize):
        self.w = windowSize
        self.window = [0]*self.w
        self.mvavg = 0.0

    def new(self, x):
        self.window.pop()
        self.window.insert(0,x)
        self._update_mvavg()

    def _update_mvavg(self):
        self.mvavg = sum(self.window)/self.w


class JointDictionaries(object):

    max_rad_sec = 1

    # hand_start = {
    #         'rh_FFJ1': -0.013387468694274651,   'rh_FFJ2': 0.10550124582950798,
    #         'rh_FFJ3': -0.07913645703418956,    'rh_FFJ4': -0.020790969983510318,
    #         'rh_THJ4': 0.8987090669167258,      'rh_THJ5': -1.0529838245665772,
    #         'rh_THJ1': 0.36613957472880915,     'rh_THJ2': -0.3099264451304632,
    #         'rh_THJ3': 0.04339213288734181,     'rh_LFJ2': 0.31856120196799154,
    #         'rh_LFJ3': -0.13247924347682977,    'rh_LFJ1': 0.020856552138779016,
    #         'rh_LFJ4': 0.006156109478006114,    'rh_LFJ5': 0.030368858695598477,
    #         'rh_RFJ4': -0.017502072148899307,   'rh_RFJ1': 0.04862574836081379,
    #         'rh_RFJ2': 0.23106641618794493,     'rh_RFJ3': -0.040169677117662395,
    #         'rh_MFJ1': 0.0061621824517631985,   'rh_MFJ3': -0.03814186780706377,
    #         'rh_MFJ2': 0.28535536916148746,     'rh_MFJ4': 0.005735133335643892,
    #         }

    hand_start = {
            'rh_RFJ3': 0.040169677117662395
    }

    hand_close = {
            'rh_RFJ3': 0.6358242015720096
    }

    # hand_close = {
    #         'rh_FFJ1': 0.5366228138727492,      'rh_FFJ2': 1.3707472622836295,
    #         'rh_FFJ3': 0.6104890181588297,      'rh_FFJ4': -0.020790969983510318,
    #         'rh_THJ4': 1.1494816044032174,      'rh_THJ5': -0.25236240595266746,
    #         'rh_THJ1': 1.0564478227578378,      'rh_THJ2': 0.5591902548242037,
    #         'rh_THJ3': 0.3010860128238289,      'rh_LFJ2': 1.1510589476677358,
    #         'rh_LFJ3': 0.3496450123403709,      'rh_LFJ1': 0.2812655031286765,
    #         'rh_LFJ4': 0.006156109478006114,   'rh_LFJ5': 0.030368858695598477,
    #         'rh_RFJ4': -0.017502072148899307,   'rh_RFJ1': 0.2252787835450361,
    #         'rh_RFJ2': 1.1696882711839942,      'rh_RFJ3': 0.6358242015720096,
    #         'rh_MFJ1': 0.18990725919524606,     'rh_MFJ3': 0.6792600589796994,
    #         'rh_MFJ2': 1.3251573950327318,      'rh_MFJ4': 0.005735133335643892,
    #         }

    def __init__(self):
        self.move_dict = dict()
        self.old_dict = self.hand_start
        self.diff_dict = dict()
        self.duration = 0.01

    def newMoveDict(self, scale):
        # move_dict = dict()
        for k in self.hand_start.keys():
            self.move_dict[k] = scale * (self.hand_close[k] - self.hand_start[k]) + self.hand_start[k]
        # return(move_dict)
        self._detSpeed()

    def _detSpeed(self):
        for k in self.move_dict.keys():
            self.diff_dict[k] = abs(self.move_dict[k] - self.old_dict[k])
        self.duration = max(self.diff_dict.values())/self.max_rad_sec
        if self.duration < 0.01:
            self.duration = 0.01
        self.old_dict = self.move_dict


# # =============== Define Position Dictionaries ===============
# # --------------- Hand Positions ---------------
# hand_start = {
#         'rh_FFJ1': -0.013387468694274651,   'rh_FFJ2': 0.10550124582950798,
#         'rh_FFJ3': -0.07913645703418956,    'rh_FFJ4': -0.020790969983510318,
#         'rh_THJ4': 0.8987090669167258,      'rh_THJ5': -1.0529838245665772,
#         'rh_THJ1': 0.36613957472880915,     'rh_THJ2': -0.3099264451304632,
#         'rh_THJ3': 0.04339213288734181,     'rh_LFJ2': 0.31856120196799154,
#         'rh_LFJ3': -0.13247924347682977,    'rh_LFJ1': 0.020856552138779016,
#         'rh_LFJ4': 0.006156109478006114,    'rh_LFJ5': 0.030368858695598477,
#         'rh_RFJ4': -0.017502072148899307,   'rh_RFJ1': 0.04862574836081379,
#         'rh_RFJ2': 0.23106641618794493,     'rh_RFJ3': -0.040169677117662395,
#         'rh_MFJ1': 0.0061621824517631985,   'rh_MFJ3': -0.03814186780706377,
#         'rh_MFJ2': 0.28535536916148746,     'rh_MFJ4': 0.005735133335643892,
#         }
hand_start = {
        'rh_FFJ1': 0.01,                    'rh_FFJ2': 0.10550124582950798,
        'rh_FFJ3': 0.01,                    'rh_FFJ4': 0,
        'rh_THJ4': 0.8987090669167258,      'rh_THJ5': -0.90,
        'rh_THJ1': 0.36613957472880915,     'rh_THJ2': -0.3099264451304632,
        'rh_THJ3': 0.04339213288734181,     'rh_LFJ2': 0.31856120196799154,
        'rh_LFJ3': 0.01,                    'rh_LFJ1': 0.020856552138779016,
        'rh_LFJ4': 0,                       'rh_LFJ5': 0.030368858695598477,
        'rh_RFJ4': 0,                       'rh_RFJ1': 0.04862574836081379,
        'rh_RFJ2': 0.23106641618794493,     'rh_RFJ3': 0.04,
        'rh_MFJ1': 0.0061621824517631985,   'rh_MFJ3': 0.03,
        'rh_MFJ2': 0.28535536916148746,     'rh_MFJ4': 0,
        }

hand_close = {
        'rh_FFJ1': 0.5366228138727492,      'rh_FFJ2': 1.3707472622836295,
        'rh_FFJ3': 0.6104890181588297,      'rh_FFJ4': -0.1693188654196813,
        'rh_THJ4': 1.1494816044032174,      'rh_THJ5': -0.25236240595266746,
        'rh_THJ1': 1.0564478227578378,      'rh_THJ2': 0.5591902548242037,
        'rh_THJ3': 0.3010860128238289,      'rh_LFJ2': 1.1510589476677358,
        'rh_LFJ3': 0.3496450123403709,      'rh_LFJ1': 0.2812655031286765,
        'rh_LFJ4': 0.0007317935784767475,   'rh_LFJ5': 0.038378063907728126,
        'rh_RFJ4': -0.030822436892029084,   'rh_RFJ1': 0.2252787835450361,
        'rh_RFJ2': 1.1696882711839942,      'rh_RFJ3': 0.6358242015720096,
        'rh_MFJ1': 0.18990725919524606,     'rh_MFJ3': 0.6792600589796994,
        'rh_MFJ2': 1.3251573950327318,      'rh_MFJ4': -0.007377111269187729,
        }

def listen():
    rospy.Subscriber("/myo_emg_right", EmgArray, callback)
    print("spinning")
    rospy.spin()


def callback(data):
    emg = data.data
    flex = emg[7]
    print("raw flex reading: %d" % flex)
    flex = flexLimits(flex)
    mvgfilt.new(flex)
    # print(mvgfilt.window)
    flex_avg = flexLimits(mvgfilt.mvavg)
    print("applied avg value: %f" % flex_avg)
    proportion = (flex_avg - emg_min) / (emg_max - emg_min)
    print("linear proportion: %f" %proportion)
    jointdict.newMoveDict(proportion)
    pub_handle.publish(jointdict.move_dict['rh_RFJ3'])
    # print(jointdict.move_dict)
    # tempdict = jointdict.move_dict
    # print(tempdict)
    # print("flex avg: %d" % flex_avg)
    # print("mvavg from class: %d" % mvgfilt.mvavg)
    # joint_goals = makeMoveDict(hand_start, hand_close, proportion)
    hand_commander.move_to_joint_value_target_unsafe(jointdict.move_dict, jointdict.duration, False)

def flexLimits(value):
    if value < emg_min:
        return(emg_min)
    elif value > emg_max:
        return(emg_max)
    else:
        return(value)

# def makeMoveDict(start, end, scale):
#     move_dict = dict()
#     for k in start.keys():
#         move_dict[k] = scale * (end[k] - start[k]) + start[k]
#     return(move_dict)


if __name__ == '__main__':
    mvgfilt = MovingFilter(50);
    jointdict = JointDictionaries();
    hand_commander.move_to_joint_value_target_unsafe(hand_start, 3.0, True)
    # hand_commander.move_to_joint_value_target_unsafe(hand_close, 3.0, True)

    # for i in range(90, 1010, 10):
    #     i = flexLimits(i)
    #     p = (i - emg_min) / (emg_max - emg_min)
    #     jointdict.newMoveDict(p)
    #     hand_commander.move_to_joint_value_target_unsafe(jointdict.move_dict, jointdict.duration, False)
    #     print("i: %d" % i)
    #     print("p: %f" % p)
    #     print("dict: \n")
    #     print(jointdict.move_dict)
    #     print("durration: %f" % jointdict.duration)
    #     print("\n")
    #     # time.delay(0.1)

    pub_handle = rospy.Publisher("dict_value", Float32, queue_size=10)
    listen()
