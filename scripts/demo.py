#!/usr/bin/python3.5

# Install packages to use Python3 with ROS 
# sudo apt-get install python3-yaml
# sudo pip3 install rospkg catkin_pkg
import sys 
from INS.INS import INS

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import math
from INS.tools.csv_parser import readCSV, readROSBagCSV, writeCSV 
from INS.tools.data_visualizer import show3Dposition, show2Dposition, interactive2Dposition_init, update2Dposition
from INS.tools.geometry_utils import rotateOutput

if __name__ == '__main__':
    rospy.init_node('imu_odometry_publisher', anonymous=True)
    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)

    global ins
    ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=5)
    ins.init()
    G_opt_shoe = 2.5e8

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

    data = Imu()
    x_out = np.zeros((ros_data.shape[0], 9))

    for i, imu_reading in enumerate(ros_data):
        # Crease IMU Message
        data.header.stamp.secs = int(imu_reading[0] * 1e-9)
        data.header.stamp.nsecs = int(imu_reading[0] % 1e9)
        data.linear_acceleration.x = imu_reading[1]
        data.linear_acceleration.y = imu_reading[2]
        data.linear_acceleration.z = imu_reading[3]
        data.angular_velocity.x = imu_reading[4]
        data.angular_velocity.y = imu_reading[5]
        data.angular_velocity.z = imu_reading[6]

        # Estimate Odometry
        x_out[i,:] = ins.baseline(imu_reading=data, G=G_opt_shoe)

        # Create Odometry Message
        odom.header.stamp.secs = int(imu_reading[0] * 1e-9)
        data.header.stamp.nsecs = int(imu_reading[0] % 1e9)
        odom.header.frame_id = "imu"
        odom.pose.pose.position.x = x_out[i,0]
        odom.pose.pose.position.y = x_out[i,1]
        odom.pose.pose.position.z = x_out[i,2]
        odom.pose.pose.orientation.x = x_out[i,6]
        odom.pose.pose.orientation.y = x_out[i,7]
        odom.pose.pose.orientation.z = x_out[i,8]

        odom_pub.publish(odom)

    output = np.zeros((x_out.shape[0], 11))
    output[:,0] = ros_data[:,0]   # Time Stamps
    # Rotate and generate output
    output[:,1:11] = rotateOutput(x_out, roll=math.pi, pitch=0, yaw=0)


    writeCSV(output, result_dir+fileName+'.csv', fields=['time', 
                                                        'x_position', 'y_position', 'z_position', 
                                                        'roll', 'pitch', 'yaw',
                                                        'quaternion_w', 'quaternion_x', 'quaternion_y', 'quaternion_z'])
    show2Dposition(output[:,1:], result_dir+fileName)

    rospy.loginfo("Execution Complete")
