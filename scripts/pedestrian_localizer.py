import rospy
import copy
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

    def __init__(self, calibrate_yaw=False, calibration_steps=2, yaw_method='Stable', yaw_latch=True, callback=None):
        """ Pedestrian Localization system

            :param calibrate_yaw: If set to True, initial yaw will be calculated from the first few steps within a fixed displacement
            :param calibration_steps: Number for initial steps to wait until Calibration is started. 
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
        """

        self.ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=5)
        self.ins.init()

        self.callback = callback

        self.step_count = 0
        self.calibration_steps = calibration_steps
        self.calibration_readings = [] 

        self.calibrated = False
        self.calibrate_yaw = calibrate_yaw

        self.step_state = self.INVALID
        self.state_timeStamp = 0.0
        self.step_counter_threshold = 0.3 # Seconds (Required minimum static state time, to record state : moving / stationary)

        # IMU roll, pitch and yaw are determined through calibration
        self.IMU_orientation = np.zeros(3)

        self.yaw_method = yaw_method
        self.yaw_latch = yaw_latch
        self.human_orientation = np.zeros(3)

    def update_foot_state(self, imu_reading, return_zv=False):
        """ Estimates IMU odometry and returns current state

            :param imu_reading: ROS sensor_msgs.Imu message
            :param return_zv: Return calculated Zero Velocity status for current imu_reading
            :return  State
                        pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw

                     zv
                        True / False
        """
        if return_zv:
            x, zv = self.ins.baseline(imu_reading=imu_reading, return_zv=True)
            return x, zv
        else:
            x = self.ins.baseline(imu_reading=imu_reading)
            return x

    def update_step_count(self, zv, imu_reading):
        """ Updates and returns current step count

            :param zv: Zero Velocity update (True/False)
                        If foot is stationary, condition should be set to true, otherwise false.
            :param imu_reading: sensor_msgs/Imu message

            :return Step count
            
            @warning: To track step count correctly, this function should be called at each time an imu reading is received 
                        (Per each iteration)
        """
        time = imu_reading.header.stamp.secs +  imu_reading.header.stamp.nsecs * 1e-9

        if (self.step_state == self.STATIONARY):
            if (zv == False):
                self.step_state = self.STARTING
                self.state_timeStamp = time

        elif (self.step_state == self.STARTING):
            if (zv == True):
                self.step_state = self.STATIONARY
            elif (time - self.state_timeStamp) > self.step_counter_threshold:
                self.step_state = self.MOVING
                # Record Step (Other foot completed step)
                self.step_count += 1

        elif (self.step_state == self.MOVING):
            if (zv == True):
                self.step_state = self.STOPPING
                self.state_timeStamp = time

        elif (self.step_state == self.STOPPING):
            if (zv == False):
                self.step_state = self.MOVING
            elif (time - self.state_timeStamp) > self.step_counter_threshold:
                self.step_state = self.STATIONARY
                # Record Step (foot completed step)
                self.step_count += 1

        elif (self.step_state == self.INVALID):
            if (zv == True):
                self.step_state = self.STOPPING
            else:
                self.step_state = self.STARTING

            self.state_timeStamp = time

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
        self.calibration_readings.append(copy.deepcopy(imu_reading))

        # Start calibration if calibration step count has reached
        if (self.step_count >= self.calibration_steps):

            # Roll and Pitch Calibration
            a_x = 0.0
            a_y = 0.0
            a_z = 0.0
            for reading in self.calibration_readings:
                a_x += reading.linear_acceleration.x
                a_y += reading.linear_acceleration.y
                a_z += reading.linear_acceleration.z

            roll = np.arctan2(a_y, a_z)
            pitch = np.arctan2(a_x, np.sqrt(a_y*a_y + a_z*a_z))
            yaw = 0.0

            # Resetting INS and initializing pose
            self.ins.init_pose(position=(0,0,0), orientation=(roll, pitch, yaw))
            self.IMU_orientation[0:2] = roll, pitch

            # Yaw Calibration
            if self.calibrate_yaw:
                pass
                # TODO 

            # Recalculating previous odometry
            for reading in self.calibration_readings:
                x_, zv = self.update_foot_state(reading, return_zv=True)
                status, x_ = self.update_human_yaw(x_, zv)
                if status and (self.callback is not None):
                    self.callback(x_, reading.header)

            # Calibration complete
            print ("Calibration Successfull")
            print ("IMU Orientation on foot: roll= ", int(roll*180/np.pi), "degrees     pitch= ", int(pitch*180/np.pi), "degrees")
            self.calibrated = True

    def update_human_yaw(self, x, zv):
        """ Approximates Human Yaw using Foot Yaw

            :param x: State
                       pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
            :parm zv: Zero velocity staus. (True: Foot is at zero velocity)
            :return   Status, State
                       Status: True is valid state data is available. False otherwise
                       State: pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
            Note: Refer yaw_method and yaw_latch descriptions for details  
            :warning: Use this method only for publishing. Do not use the return value of this method for any INS processing 
        """
        status = False
        x[6:9] = self.angleNormalizer(x[6:9] + self.IMU_orientation)
        if self.yaw_method == 'Real':
            status = True
        elif self.yaw_method == 'Zupt':
            if zv:
                # Zero Velocity. Updating Yaw
                self.human_orientation = x[6:9]
                status = True
            elif self.yaw_latch:
                # Nonzero Velocity. Latched yaw : returning last known valid yaw
                x[6:9] = self.human_orientation
                status = True

            # Else: Nonezero Velocity. Not using past yaw data. status = False
        elif self.yaw_method == 'Stable':
            if np.max(np.fabs(x[7])) < 0.15: # Approximately 8.6 Degrees
                # Stable Foot roll & pitch. Updating Yaw
                self.human_orientation = x[6:9]
                status = True
            elif self.yaw_latch:
                # Unstable Foot roll & pitch. Latched yaw : returning last known valid yaw
                x[6:9] = self.human_orientation
                status = True
            # Else: Unstalbe Foot yaw. Not using path yaw data. status = False
        else:
            rospy.logerr("Pedestrian_Localizer: Unknown yaw_method")
        
        return status, x

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
        x, zv = self.update_foot_state(imu_reading, return_zv=True)
        self.update_step_count(zv=zv, imu_reading=imu_reading)

        if self.calibrated == False:      
            self.calibrate(imu_reading, x)
        else:
            status, x = self.update_human_yaw(x, zv)
            if status and (self.callback is not None):
                self.callback(x, imu_reading.header)
        

    


    
        
