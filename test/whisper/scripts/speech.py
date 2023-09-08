#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from playsound import playsound
from std_msgs.msg import String

def callback(data):
    if "M101" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M101.wav")
    elif "M102" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M102.wav")
    elif "M103" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M103.wav")
    elif "M104" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M104.wav")
    elif "M105" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M105.wav")
    elif "M106" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M106.wav")
    elif "M107" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M107.wav")
    elif "M108" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M108.wav")
    elif "M109" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M109.wav")
    elif "M110" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M110.wav")
    elif "M111" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M111.wav")
    elif "M112" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M112.wav")
    elif "M113" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M113.wav")
    elif "M114" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M114.wav")
    elif "M115" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M115.wav")
    elif "M116" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M116.wav")
    elif "M117" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M117.wav")
    elif "M118" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/M118.wav")
    elif "elevator" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/elevator.wav")
    elif "toilet" in data.data:
        playsound("/home/junho/catkin_ws/src/whisper/speeches/toilet.wav")

def sound_player():
    rospy.init_node('sound_player')
    rospy.Subscriber("sound_play", String, callback)
    rate = rospy.Rate(1)

    rospy.spin()

if __name__ == '__main__':
    sound_player()

