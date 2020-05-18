import numpy as np
import math
from INS.INS import INS
import rospy
from sensor_msgs.msg import Imu 

from INS.tools.geometry_utils import rotateOutput

class pedestrian_localizer():
    # Global constants
    STATIONARY = 0
    STARTING = 1
    MOVING = 2
    STOPPING = 3
    INVALID = 4

    def __init__(self, calibrate_yaw=False):
        """ Pedestrian Localization system

            :param calibrate_yaw: If set to True, initial yaw will be calculated from the first few steps within a fixed displacement
            Example:
                pedestrian_localizer('rpy') # Create localizer object with initial roll, pitch, yaw correction
        """

        self.ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=5)
        self.ins.init()

        self.step_count = 0
        self.calibration_readings = 0
        self.avg_a = np.array([0.0, 0.0, 0.0]) 

        self.calibrated = False
        self.calibrate_roll_pitch = True # Roll and pitch will be always
        self.calibrate_yaw = calibrate_yaw

        self.step_state = self.INVALID
        self.state_timeStamp = rospy.get_time()
        self.step_counter_threshold = 0.2 # Seconds

        self.yaw_calibration_distance = 2 # Meters

    def update_step_count(self, zv):

        if (self.step_state == self.STATIONARY):
            if (zv == False):
                self.step_state = self.STARTING
                self.state_timeStamp = rospy.get_time()

        elif (self.step_state == self.STARTING):
            if (zv == True):
                self.step_state = self.STATIONARY
            elif (rospy.get_time() - self.state_timeStamp) > self.step_counter_threshold:
                self.step_state = self.MOVING
                # Record Step (Other foot completed step)
                self.step_count += 1

        elif (self.step_state == self.MOVING):
            if (zv == True):
                self.step_state = self.STOPPING
                self.state_timeStamp = rospy.get_time()

        elif (self.step_state == self.STOPPING):
            if (zv == False):
                self.step_state = self.MOVING
            elif (rospy.get_time() - self.state_timeStamp) > self.step_counter_threshold:
                self.step_state = self.STATIONARY
                # Record Step (foot completed step)
                self.step_count += 1

        elif (self.step_state == self.INVALID):
            if (zv == True):
                self.step_state = self.STOPPING
            else:
                self.step_state = self.STARTING

            self.state_timeStamp = rospy.get_time()

        else: 
            self.step_state = self.INVALID


    def calibrate(self, imu_reading, x):
        """  Calculates Roll, Pitch and Yaw initial values

            :param imu_reading: IMU reading ros msg
            :param x: State
                       pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw

            Roll, Pitch angles are calculated perpendicular to gravity vector
            Yaw angle is caculated according to the direction of initial motion
        """
        # Calibrate roll and pitch until first step is complete
        if(self.calibrate_roll_pitch):
            a_x = imu_reading.linear_acceleration.x
            a_y = imu_reading.linear_acceleration.y
            a_z = imu_reading.linear_acceleration.z

            # Averaging linear acceleration
            self.avg_a = (self.avg_a * self.calibration_readings + np.array([a_x, a_y, a_z])) / (self.calibration_readings + 1) 
            self.calibration_readings += 1

            if (self.step_count >= 1):
                avg_x, avg_y, avg_z = self.avg_a
                roll = np.arctan2(avg_y, avg_z)
                pitch = np.arctan2(avg_x,np.sqrt(avg_y*avg_y + avg_z*avg_z))

                # self.ins.initialize_pose(position=(0,0,0), orientation=(roll, pitch, 0))
                # Roll, Pitch calibration complete
                print ("Calibration done")
                print (roll, pitch)
                self.calibrate_roll_pitch = False
                self.calibrated = True
        
        
        # Correct roll angle
        # if ('r' in config):
            # Calculating gravity angle to Y axis in YZ plane
            # alpha = np.arctan2(lin_acc[1], lin_acc[2])
            # print (alpha)
            # 

        # Correct pitch angle
        # if ('p' in config):
            # Calculating gravity angle to X axis in XZ plane
            # beta = np.arctan2(lin_acc[0], np.sqrt(lin_acc[1]*lin_acc[1] + lin_acc[2]*lin_acc[2]))
            # print (beta)
            # self.pitch = math.pi/2 - beta   # gravity should be in -Z direction (perpendicular to X axis)
        
        
        # roll = np.arctan2(-avg_y,-avg_z)
        # pitch = np.arctan2(avg_x,np.sqrt(avg_y*avg_y + avg_z*avg_z))
           
        # attitude = np.array([roll, pitch, heading])
        # x[0, 6:9] = attitude
        # q[0, :] = euler2quat(roll, pitch, heading, 'sxyz')


    def update(self, imu_reading):

        G_opt_shoe = 2.5e8
        x, zv = self.ins.baseline(imu_reading=imu_reading, G=G_opt_shoe, return_zv=True)
        self.update_step_count(zv)

        if self.calibrated == False:      
            self.calibrate(imu_reading, x)
            return None
        else:
            return x
        

    


    
        