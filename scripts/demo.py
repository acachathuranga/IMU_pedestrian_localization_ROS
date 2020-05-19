#!/usr/bin/python3.5

# Install packages to use Python3 with ROS 
# sudo apt-get install python3-yaml
# sudo pip3 install rospkg catkin_pkg
import sys 
from pedestrian_localizer import pedestrian_localizer

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import math
from INS.tools.csv_parser import readCSV, readROSBagCSV, writeCSV 
from INS.tools.data_visualizer import show3Dposition, show2Dposition, interactive2Dposition_init, update2Dposition
from INS.tools.geometry_utils import rotateOutput
from INS.tools.geometry_helpers import euler2quat

def publish_odom(x, header):
    """ Publish Odometry Message

        :param x: State
                       pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
    """
    global x_out
    if x is not None:
        x_out.append([x, header])

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



if __name__ == '__main__':
    rospy.init_node('imu_odometry_publisher', anonymous=True)
    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)

    localizer = pedestrian_localizer(callback=publish_odom)

    print ("IMU Odometry Publisher")

    data_dir = "/home/achala/catkin_ws/src/imu_odometry/scripts/ros_data/"
    result_dir = "/home/achala/catkin_ws/src/imu_odometry/scripts/results/"
    fileName = "2020-03-23-17-15-36"

    fieldNames= ["field.header.stamp", 
                    "field.linear_acceleration.x", "field.linear_acceleration.y", "field.linear_acceleration.z",
                    "field.angular_velocity.x", "field.angular_velocity.y", "field.angular_velocity.z"]
    dataTypes = [int, float, float, float, float, float, float]

    status, userData = readROSBagCSV(data_dir+fileName+'.csv', fields=fieldNames, dtype=dataTypes)
    ros_data = userData.view((float, len(userData.dtype.names)))
    ros_data[:,0] = userData['field.header.stamp']
    
    print("SUTD Demo: " + fileName)

    print ("Input shape: ", ros_data.shape)
    # Initialize odometry msg
    odom = Odometry()
    x_out = []
    data = Imu()
    for i, imu_reading in enumerate(ros_data):
        # Create IMU Message
        data.header.stamp.secs = int(imu_reading[0] * 1e-9)
        data.header.stamp.nsecs = int(imu_reading[0] % 1e9)
        data.linear_acceleration.x = imu_reading[1]
        data.linear_acceleration.y = imu_reading[2]
        data.linear_acceleration.z = imu_reading[3]
        data.angular_velocity.x = imu_reading[4]
        data.angular_velocity.y = imu_reading[5]
        data.angular_velocity.z = imu_reading[6]

        # Estimate Odometry
        localizer.update_odometry(data)
    
    output = np.zeros((len(x_out), 10))
    output[:,0] = ros_data[:,0]   # Time Stamps

    # Rotate and generate output
    for i in range(len(x_out)):
        output[i,1:10] = rotateOutput(x_out[i][0], roll=math.pi, pitch=0, yaw=0)

    writeCSV(output, result_dir+fileName+'.csv', fields=['time', 
                                                        'x_position', 'y_position', 'z_position', 
                                                        'vel_x', 'vel_y', 'vel_z',
                                                        'roll', 'pitch', 'yaw',])
    show2Dposition(output[:,1:], result_dir+fileName)

    rospy.loginfo("Execution Complete")

