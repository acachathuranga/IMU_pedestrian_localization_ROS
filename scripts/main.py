#!/usr/bin/python3.5

# Install packages to use Python3 with ROS 
# sudo apt-get install python3-yaml
# sudo pip3 install rospkg catkin_pkg
import rospy
import sys
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry

from pedestrian_localizer import pedestrian_localizer

localizer = None
odom_pub = None

def callback(imu_reading):
    
    x = localizer.update(imu_reading)
    if x is not None:
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
    auto_orient = rospy.get_param('auto_orient', 'rpy')

    global localizer, odom_pub
    localizer = pedestrian_localizer()

    odom_pub = rospy.Publisher('imu_odometry', Odometry, queue_size=100)
    rospy.Subscriber("/footIMU/IMU", Imu, callback)
    

    print ("IMU Odometry Publisher")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imu_odometry()

    

