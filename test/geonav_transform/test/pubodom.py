#!/usr/bin/env python
'''
hack to publish some 0 odometry
'''

# Python
import rospy
from sensor_msgs.msg import NavSatFix
import sys
from math import *
import tf
import math

# ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

lat = 36.76905957
lon = 126.93517886



def clamp(val):
    while val < -pi/2.0:
        val += 2.0*pi
    while val > pi/2.0:
        val -= 2.0*pi
    return val

def gps_callback(msg):
    global lat, lon
    lat = msg.latitude
    lon = msg.longitude
   

new_msg = PoseWithCovarianceStamped()
call_pose = False
def pose_callback(msg):

    global new_msg, call_pose

    new_msg.header.stamp = rospy.Time.now()
    new_msg.header.frame_id = "map"

    new_msg.pose.pose.position = msg.pose.pose.position

    new_msg.pose.pose.orientation.z = 0.00
    new_msg.pose.pose.orientation.w = 1.0

    call_pose = True





def talker():
    global lat, lon, pose_sub, call_pose
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)



    
    pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_callback)


    rospy.Subscriber('/filtered_gps_topic', NavSatFix, gps_callback)   

    rospy.init_node('odomtester')
    rate = rospy.Rate(2) # 10hz
    #lat = 36.76905957
    dlat = 0.0 #0.000001
    #lon = 126.93517886
    dlon = 0.0 #0.000001

    roll = 0.0
    droll = 0.0 #0.01
    pitch = 0.0
    dpitch = 0.0 #0.01
    yaw = pi/4.0
    dyaw = 0.0 #0.03

    N = 0.0
    while not rospy.is_shutdown():
        omsg = Odometry()
        omsg.header.seq += 1
        omsg.header.stamp = rospy.get_rostime()
        omsg.header.frame_id = 'odom'
        omsg.child_frame_id = 'base_footprint'
        omsg.pose.pose.position.x = lon+N*dlon
        omsg.pose.pose.position.y = lat+N*dlat
        omsg.pose.pose.position.z = 0.0 #1.0

        r = clamp(roll+N*droll)
        p = clamp(pitch+N*dpitch)
        y = clamp(yaw+N*dyaw)
        q = tf.transformations.quaternion_from_euler(r,p,y)
        omsg.pose.pose.orientation.x = q[0]
        omsg.pose.pose.orientation.y = q[1]
        omsg.pose.pose.orientation.z = q[2]
        omsg.pose.pose.orientation.w = 20 #q[3]

        omsg.pose.covariance = range(36)

        omsg.twist.twist.linear.x=1.0
        omsg.twist.twist.linear.y=2.0
        omsg.twist.twist.linear.z=3.0
        omsg.twist.twist.angular.x=4.0
        omsg.twist.twist.angular.y=5.0
        omsg.twist.twist.angular.z=6.0

        omsg.twist.covariance = range(36,36+36)

        if call_pose == True :
            initpose_pub = rospy.Publisher('/initialpose_modified', PoseWithCovarianceStamped, queue_size=10)
            initpose_pub.publish(new_msg)
            call_pose = False

        #lat += 0.00001
        #lon += 0.00001

        rospy.loginfo("publishing odom (%.10f, %.10f, %.2f)"%(omsg.pose.pose.position.y,omsg.pose.pose.position.x,omsg.pose.pose.position.z))
        pub.publish(omsg)
        N += 1.0
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
