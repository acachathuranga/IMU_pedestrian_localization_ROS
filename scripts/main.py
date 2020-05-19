#!/usr/bin/python3.5

# Install packages to use Python3 with ROS 
# sudo apt-get install python3-yaml
# sudo pip3 install rospkg catkin_pkg
import rospy
import sys
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry
from INS.tools.geometry_helpers import euler2quat

<<<<<<< HEAD
import numpy as np
import math
from INS.tools.csv_parser import readCSV, readROSBagCSV, writeCSV 
from INS.tools.data_visualizer import show3Dposition, show2Dposition
from INS.tools.geometry_utils import rotateOutput
from INS.tools.geometry_helpers import euler2quat
=======
from pedestrian_localizer import pedestrian_localizer
>>>>>>> 1beea90e3bb83e51e499df4c2e52937163f8c366

localizer = None
odom_pub = None

def publish_odom(x, header):
    """ Publish Odometry Message

        :param x: State
                       pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
    """
    if x is not None:
        x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw = x
        qw, qx, qy, qz =  euler2quat(roll, pitch, yaw, axes='sxyz')
        odom = Odometry()
        odom.header = header
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
        odom_pub.publish(odom)

def callback(imu_reading):
<<<<<<< HEAD
    G_opt_shoe = 2.5e8
    x = ins.baseline(imu_reading=imu_reading, G=G_opt_shoe)

    # Publish Odometry Message
    x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw = x
    qw, qx, qy, qz =  euler2quat(roll, pitch, yaw, axes='sxyz')
    odom = Odometry()
    odom.header = imu_reading.header
    odom.child_frame_id = "pedestrian"
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
    odom_pub.publish(odom)
=======
    localizer.update_odometry(imu_reading)
    
>>>>>>> 1beea90e3bb83e51e499df4c2e52937163f8c366
    
def imu_odometry():
    print ("IMU Odometry Publisher")
    rospy.init_node('imu_odometry_publisher', anonymous=True)

<<<<<<< HEAD
    global ins, odom_pub, odom
    ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=5)
    ins.init()
    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)
    rospy.Subscriber("/footIMU/IMU", Imu, callback)
=======
    calibrate_yaw = rospy.get_param(rospy.get_name()+'/calibrate_yaw', True)
    calibration_steps = rospy.get_param(rospy.get_name()+'/calibration_steps', True)
    imu_topic = rospy.get_param(rospy.get_name()+'/imu_topic', "/vectornav/IMU")
>>>>>>> 1beea90e3bb83e51e499df4c2e52937163f8c366

    global localizer, odom_pub
    localizer = pedestrian_localizer(calibrate_yaw=calibrate_yaw, calibration_steps=calibration_steps, callback=publish_odom)

    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)
    rospy.Subscriber(imu_topic, Imu, callback)
    print ("Subscribed to: ", imu_topic)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imu_odometry()

    

