import rospy
import copy
import numpy as np
import math
from INS.INS import INS
import rospy
from sensor_msgs.msg import Imu 

from INS.tools.geometry_utils import rotateOutput
from INS.tools.geometry_helpers import euler2mat

class pedestrian_localizer():

    def __init__(self, calibrate_yaw=False, calibration_distance=2, yaw_method='Stable', yaw_latch=True, callback=None):
        """ Pedestrian Localization system

            :param calibrate_yaw: If set to True, initial yaw will be calculated from the first few steps within a fixed displacement
            :param calibration_distance: Distance to walk until Calibration is complete. 
                                        Collection data points upto this time will be used for calibration.
            :param yaw_method: Yaw Publishing Method
                                Human Yaw is different from Foot Yaw. Foot Yaw keeps changing drastically even when walking straight. 
                                Hence a conditioning method is used to approximate Human Yaw.

                                Real : IMU Yaw will be published. This will be the foot Yaw, not the Human Yaw
                                Zupt : Yaw will be updated only at Foot Zero Velocity instances
                                Stable : Yaw will be updated when the foot is approximately horizontal 
                                        This will reduce Yaw distortions when foot inclined with respect to flow
            :param yaw_latch: Yaw Publisher Latch
                                If true, odometry will keep publishing using last known Orientation, until an orientation update is available
                                If false, odometry is published only when an Orientation update is available
            :param callback: Callback function that accepts calculated state of pedestrian and a standard message header
                                at each imu_reading
                                - State
                                    pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
                                - State Covariance
                                - Current IMU Reading Header
        """

        self.ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=5)
        self.ins.init()

        self.callback = callback

        self.calibration_distance = calibration_distance
        self.calibration_readings = []

        self.rp_calibrated = False
        self.calibrated = False
        self.calibrate_yaw = calibrate_yaw
        self.yaw_correction_matrix = np.identity(3)

        # IMU roll, pitch and yaw are determined through calibration
        self.IMU_orientation = np.zeros(3)

        self.yaw_method = yaw_method
        self.yaw_latch = yaw_latch
        self.human_orientation = np.zeros(3)

    def update_foot_state(self, imu_reading, return_zv=False, return_quarternion=False):
        """ Estimates IMU odometry and returns current state

            :param imu_reading: ROS sensor_msgs.Imu message
            :param return_zv: Return calculated Zero Velocity status for current imu_reading
            :return  State
                        tf - Homogeneous transformation matrix for IMU frame (in navigation frame)
                        twist - IMU velocity in navigation frame
                     State Covariance

                     (optional) zv
                        True / False
        """
        return self.ins.baseline(imu_reading=imu_reading, return_zv=return_zv)

    def calibrate(self, imu_reading, TF, Twist, p, zv):
        """  Calculates Roll, Pitch and Yaw initial values

            :param imu_reading: IMU reading ros msg
            :param x: State
                       pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
            :param p: State Covariance
            :param zv: Zero velocity status

            Roll, Pitch angles are calculated perpendicular to gravity vector
            Yaw angle is caculated according to the direction of initial motion

            Updated Note: Now calibration is only done for zero velocity readings only
        """
        self.calibration_readings.append([copy.deepcopy(imu_reading), TF, Twist, p, zv])

        # Start roll / pitch calibration if 100 zero velocity readings has been registered
        if ((not self.rp_calibrated) and (len(self.calibration_readings) > 100)):
            # Roll and Pitch Calibration
            a_x = 0.0
            a_y = 0.0
            a_z = 0.0
            for reading, x, _, zv in self.calibration_readings:
                if zv:
                    a_x += reading.linear_acceleration.x
                    a_y += reading.linear_acceleration.y
                    a_z += reading.linear_acceleration.z
            
            roll = np.arctan2(-a_y, -a_z)
            pitch = np.arctan2(-a_x, -np.sqrt(a_y*a_y + a_z*a_z))
            yaw = 0.0
             
            # Resetting INS and initializing pose
            self.ins.init_pose(position=(0,0,0), orientation=(roll, pitch, yaw))
            self.IMU_orientation[0:2] = roll, pitch

            # Calibration complete
            print ("Roll-Pitch Calibration Successfull")
            print ("IMU on foot: roll= ", int(roll*180/np.pi), "degrees     pitch= ", int(pitch*180/np.pi), "degrees")
            self.rp_calibrated = True

            # Flush initial calibration data points
            self.calibration_readings.clear()

        if ((self.rp_calibrated) and ((np.linalg.norm(x[:2]) >= self.calibration_distance) or (not self.calibrate_yaw))):
            # Yaw Calibration
            _, TF, Twist, _, _ = self.calibration_readings[-1]
            if self.calibrate_yaw:
                yaw = math.atan2(TF[3,1], TF[3,0])
                self.IMU_orientation[2] = -yaw
                self.yaw_correction_matrix = euler2mat(0, 0, -yaw)

            # Recalculating previous odometry
            for reading, TF, Twist, p, zv in self.calibration_readings:
                # x_, p, zv_ = self.update_foot_state(reading, return_zv=True)
                status, TF_ = self.update_human_yaw(TF, zv)
                if status and (self.callback is not None):
                    self.callback(TF_, Twist, p, reading.header)

            # Calibration complete
            if self.calibrate_yaw:
            	print ("Yaw Calibration Successfull")
            	print ("IMU on foot: yaw= ", int(self.IMU_orientation[2]*180/np.pi), "degrees")
            else:
                print ("Yaw Calibration disabled")
            self.calibrated = True

    def update_human_yaw(self, TF, zv):
        """ Approximates Human Yaw using Foot Yaw

            :param x: State
                       Transformation matrix
            :parm zv: Zero velocity staus. (True: Foot is at zero velocity)
            :return   Status, State
                       Status: True is valid state data is available. False otherwise
                       State: Transformation matrix
            Note: Refer yaw_method and yaw_latch descriptions for details  
            :warning: Use this method only for publishing. Do not use the return value of this method for any INS processing 
        """
        status = False
        TF_ = copy.copy(TF)
        x_[6:9] = self.angleNormalizer(x[6:9] + self.IMU_orientation)
        if self.yaw_method == 'Real':
            status = True
        elif self.yaw_method == 'Zupt':
            if zv:
                # Zero Velocity. Updating Yaw
                self.human_orientation = x_[6:9]
                status = True
            elif self.yaw_latch:
                # Nonzero Velocity. Latched yaw : returning last known valid yaw
                x_[6:9] = self.human_orientation
                status = True

            # Else: Nonezero Velocity. Not using past yaw data. status = False
        elif self.yaw_method == 'Stable':
            if np.max(np.fabs(x_[7])) < 0.15: # Approximately 8.6 Degrees
                # Stable Foot roll & pitch. Updating Yaw
                self.human_orientation = x_[6:9]
                status = True
            elif self.yaw_latch:
                # Unstable Foot roll & pitch. Latched yaw : returning last known valid yaw
                x_[6:9] = self.human_orientation
                status = True
            # Else: Unstalbe Foot yaw. Not using path yaw data. status = False
        else:
            rospy.logerr("Pedestrian_Localizer: Unknown yaw_method")
        
        # Yaw correction
        x_[:3] = np.dot(self.yaw_correction_matrix, x_[:3])
        return status, x_

    def angleNormalizer(self, angle):
        """Converting angle between -PI and PI
        """
        angle = np.fmod(angle, 2*np.pi)

        mask = angle > np.pi
        angle = angle - 2 * np.pi * mask 
        mask = angle < -np.pi
        angle = angle + 2 * np.pi * mask
        return angle

    def update_odometry(self, imu_reading, odometry_publisher=None):
        TF, Twist, P, zv = self.update_foot_state(imu_reading, return_zv=True, return_quarternion=True)

        if self.calibrated == False:      
            self.calibrate(imu_reading, TF, Twist, P, zv)
        else:
            status, TF = self.update_human_yaw(TF, zv)
            if status and (self.callback is not None):
                self.callback(TF, Twist, P, imu_reading.header)
        

    


    
        
