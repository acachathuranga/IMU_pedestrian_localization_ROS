#!/usr/bin/python3

# Install packages to use Python3 with ROS 
# sudo apt-get install python3-yaml
# sudo pip3 install rospkg catkin_pkg
import os
# Constrain OPENBLAS Multithreading (To solve Numpy Performance Issues)
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import rospy
import sys
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry
from INS.tools.geometry_helpers import euler2quat
import numpy as np

from pedestrian_localizer import pedestrian_localizer

localizer = None
odom_pub = None

def publish_odom(x, p, header):
    """ Publish Odometry Message

        :param x: State
                       pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
    """
    if x is not None:
        x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw = x
        qw, qx, qy, qz =  euler2quat(roll, pitch, yaw, axes='sxyz')
        odom = Odometry()
        odom.header = header
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z 
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.linear.z = vel_z

        pose_covariance = np.zeros((6,6))
        pose_covariance[:3, :3] = p[:3, :3]
        pose_covariance[:3, 3:] = p[:3, 6:]
        pose_covariance[3:, :3] = p[6:, :3]
        pose_covariance[3:, 3:] = p[6:, 6:]
        odom.pose.covariance = pose_covariance.reshape(-1).tolist()

        twist_covariance = np.zeros((6,6))
        twist_covariance[:3, :3] = p[3:6, 3:6]
        odom.twist.covariance = twist_covariance.reshape(-1).tolist()

        odom_pub.publish(odom)

def callback(imu_reading):
    localizer.update_odometry(imu_reading)
    
    
def imu_odometry():
    print ("IMU Odometry Publisher")
    rospy.init_node('imu_odometry_publisher', anonymous=True)

    calibrate_yaw = rospy.get_param(rospy.get_name()+'/calibrate_yaw', True)
    calibration_distance = rospy.get_param(rospy.get_name()+'/calibration_distance', 2)
    imu_topic = rospy.get_param(rospy.get_name()+'/imu_topic', "/vectornav/IMU")
    yaw_pub_method = rospy.get_param(rospy.get_name()+ '/yaw_pub_method', 'Stable')
    yaw_pub_latch = rospy.get_param(rospy.get_name()+ '/yaw_pub_latch', True)

    global localizer, odom_pub
    localizer = pedestrian_localizer(calibrate_yaw=calibrate_yaw, calibration_distance=calibration_distance, yaw_method=yaw_pub_method, yaw_latch=yaw_pub_latch, callback=publish_odom)

    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)
    rospy.Subscriber(imu_topic, Imu, callback)
    print ("Subscribed to: ", imu_topic)
    print ("Human Orientation Approximation Method: ", yaw_pub_method)
    print ("Human Orientation Latch: ", yaw_pub_latch)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imu_odometry()

    

