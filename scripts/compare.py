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

def createIMU_Msg(imu_reading):
    """ Create IMU ROS msg from IMU reading
        :param imu_reading: timeStamp (nanoseconds), lin.acc.x, lin.acc.y, lin.acc.z, ang.vel.x, ang.vel.y, ang.vel.z
        :return sensor_msgs/IMU message
    """
    msg = Imu()
    # Create IMU Message
    msg.header.stamp.secs = int(imu_reading[0] * 1e-9)
    msg.header.stamp.nsecs = int(imu_reading[0] % 1e9)
    msg.linear_acceleration.x = imu_reading[1]
    msg.linear_acceleration.y = imu_reading[2]
    msg.linear_acceleration.z = imu_reading[3]
    msg.angular_velocity.x = imu_reading[4]
    msg.angular_velocity.y = imu_reading[5]
    msg.angular_velocity.z = imu_reading[6]
    return msg

def readIMUCSV(file):
    """ Return IMU data from a IMU ROS Bag Export CSV

        :param file: file_path/filename.csv
        :return     Numpy array with columns:
                    -   timeStamp(nanoseconds), lin.acc.x, lin.acc.y, lin.acc.z, ang.vel.x, ang.vel.y, ang.vel.z
    """
    fieldNames= ["field.header.stamp", 
                    "field.linear_acceleration.x", "field.linear_acceleration.y", "field.linear_acceleration.z",
                    "field.angular_velocity.x", "field.angular_velocity.y", "field.angular_velocity.z"]
    dataTypes = [int, float, float, float, float, float, float]

    status, userData = readROSBagCSV(file+'.csv', fields=fieldNames, dtype=dataTypes)
    ros_data = userData.view((float, len(userData.dtype.names)))
    ros_data[:,0] = userData['field.header.stamp']
    return ros_data

def get_zv(file):
    """ Returns a numpy array of Zupt updates.

        :param file: filePath to IMU data CSV file
        :return Numpy Array. 
                Columns -  timeStamp, ZuptState (True/False)
    """
    global ins
    ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=3)  # Default 5
    ins.init()
    G_opt_shoe = 2.5e8

    print ("IMU Odometry Publisher")
    ros_data = readIMUCSV(file)
    
    x_out = np.zeros((ros_data.shape[0], 9))
    zvList = np.zeros((ros_data.shape[0], 2))
    zvList[:,0] = ros_data[:,0]

    for i, imu_reading in enumerate(ros_data):
        # Estimate Odometry
        x_out[i,:], zv = ins.baseline(imu_reading=createIMU_Msg(imu_reading), G=G_opt_shoe, return_zv=True)
        zvList[i,1] = zv

    output = np.zeros((x_out.shape[0], 11))
    output[:,0] = ros_data[:,0]   # Time Stamps
    # Rotate and generate output
    output[:,1:11] = rotateOutput(x_out, roll=math.pi, pitch=0, yaw=0)


    # writeCSV(output, result_dir+fileName+'.csv', fields=['time', 
    #                                                     'x_position', 'y_position', 'z_position', 
    #                                                     'roll', 'pitch', 'yaw',
    #                                                     'quaternion_w', 'quaternion_x', 'quaternion_y', 'quaternion_z'])
   
    show2Dposition(output[:,1:], file)
    return zvList

if __name__ == '__main__':
    print ("ZV Comparision")
    data_dir = "/home/achala/catkin_ws/src/imu_odometry/data/comparision/data/"
    result_dir = "/home/achala/catkin_ws/src/imu_odometry/results/comparision/results/"

    foot_suffix = "_foot"
    waist_suffix = "_waist"

    file1 = "2020-03-27-14-13-37"
    file2 = "2020-03-27-14-39-27"
    file3 = "2020-03-27-14-55-56"
    file4 = "2020-03-27-16-06-30"

    fileName = file1
    zv = get_zv(data_dir+fileName+foot_suffix)
    data = readIMUCSV(data_dir+fileName+waist_suffix)
    
    import matplotlib.pyplot as plt

    plt.plot(zv[:,0], zv[:,1], label='Zupt', linewidth=4)
    plt.plot(data[:,0], data[:,1], label='lin.accX')
    plt.plot(data[:,0], data[:,2], label='lin.accY')
    plt.plot(data[:,0], data[:,3], label='lin.accZ')
    plt.plot(data[:,0], data[:,4], label='ang.velX')
    plt.plot(data[:,0], data[:,5], label='ang.velY')
    plt.plot(data[:,0], data[:,6], label='ang.velZ')
    plt.legend()
    plt.show()

    # get_zv(data_dir+file1)
    # get_zv(data_dir+file2)
    # get_zv(data_dir+file3)
    # get_zv(data_dir+file4)



   

