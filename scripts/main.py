#!/usr/bin/python3.5

# Install packages to use Python3 with ROS 
# sudo apt-get install python3-yaml
# sudo pip3 install rospkg catkin_pkg
import rospy
import sys
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry
from INS.INS import INS

import numpy as np
import math
from INS.tools.csv_parser import readCSV, readROSBagCSV, writeCSV 
from INS.tools.data_visualizer import show3Dposition, show2Dposition
from INS.tools.geometry_utils import rotateOutput

ins = None
odom_pub = None

def callback(imu_reading):
    G_opt_shoe = 2.5e8
    x = ins.baseline(imu_reading=imu_reading, G=G_opt_shoe)

    x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw = x
    odom = Odometry()
    odom.header = imu_reading.header
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = z 
    odom.pose.pose.orientation.x = roll
    odom.pose.pose.orientation.y = pitch
    odom.pose.pose.orientation.z = yaw
    odom_pub.publish(odom)
    
def imu_odometry():
    rospy.init_node('imu_odometry_publisher', anonymous=True)

    global ins, odom_pub, odom
    ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=5)
    ins.init()
    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)
    rospy.Subscriber("/vectornav/IMU", Imu, callback)

    print ("IMU Odometry Publisher")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imu_odometry()

    

