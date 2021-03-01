import numpy as np
from collections import deque
from sensor_msgs.msg import Imu as IMU_Msg
from .EKF import Localizer

class INS():
    def __init__(self, sigma_a=0.01, sigma_w=0.1*np.pi/180, detector="shoe", W=5, dt=None):
        """ Initialize Inertial Navigation System
        """
        self.config = {
        "sigma_a": sigma_a,
        "sigma_w": sigma_w,
        "g": 9.575
            }
        # "g": 9.8029   # Default
        # "g": 9.775    # Waist IMU
        # "g": 9.575    # Foot IMU

        sigma_a = self.config["sigma_a"]
        sigma_w = self.config["sigma_w"]
        self.config["var_a"] = np.power(sigma_a,2)
        self.config["var_w"] = np.power(sigma_w,2)

        ##process noise in body frame
        self.sigma_acc = 0.5*np.ones((1,3))
        self.var_acc = np.power(self.sigma_acc,2)
        self.sigma_gyro = 0.5*np.ones((1,3))*np.pi/180
        self.var_gyro = np.power(self.sigma_gyro,2)
    
        self.Q = np.zeros((6,6))  ##process noise covariance matrix Q
        self.Q[0:3,0:3] = self.var_acc*np.identity(3)
        self.Q[3:6,3:6] = self.var_gyro*np.identity(3)
        self.config["Q"] = self.Q
        
        self.sigma_vel = 0.01 #0.01 default
        R = np.zeros((3,3))
        R[0:3,0:3] = np.power(self.sigma_vel,2)*np.identity(3)   ##measurement noise, 0.01 default
        self.config["R"] = R
        
        H = np.zeros((3,9))
        H[0:3,3:6] = np.identity(3)
        self.config["H"] = H

        self.detector = detector
        self.config["detector"] = self.detector

        self.W = W
        self.config["W"] = self.W 

        self.x_in = deque(maxlen=W)    
        
        self.Localizer = Localizer(self.config)

    def init(self):
        """ Initialize state variables x, q, P with initial position 0,0 and orientation 0,0,0
        """
        self.x, self.q, self.P = self.Localizer.init() #initialize state
        self.x_in.clear()
    
    def init_pose(self, position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0)):
        """ Initializes position and orientation (x, q). Resets covariance matrix (P)

            :param position: Initial position x,y,z coordinates
            :param orientation: Initial orientation euler angles in roll, pitch, yaw format (xyz)
            :return None
        """
        self.x, self.q , self.P = self.Localizer.init_variables(position=position, attitude=orientation)
        self.x_in.clear()

    def input_window(self, imu_reading):
        """ Insert and update input data window

            :param imu_reading: IMU reading ROS msg
                                type: sensor_msgs/IMU
            :return IMU data window of size W
                        W is a configuration parameter of the INS class object
                        Dimension: window_size x [time, acc.x, acc.y, acc.z, vel.x, vel.y, vel.z]
        """
        # Time stamp is scaled since ROS message time stamp is in nanoseconds
        data = np.array([(imu_reading.header.stamp.secs +  imu_reading.header.stamp.nsecs * 1e-9),
                        imu_reading.linear_acceleration.x, imu_reading.linear_acceleration.y, imu_reading.linear_acceleration.z,
                        imu_reading.angular_velocity.x, imu_reading.angular_velocity.y, imu_reading.angular_velocity.z])
        self.x_in.append(data)
        return self.x_in

        
    def baseline(self, imu_reading, G=5e8, zv=None, return_zv=False):
        """ Estimates IMU odometry and returns current state

            :param imu_reading: ROS sensor_msgs.Imu message
            :param G: Detector threshold
            :param zv: Zero Velocity Update status (True - Manually set this reading to a Zupt state)

            :return  State
                        pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
                     State Covariance Matrix
                     ZV (Optional)

        """
        imudata = self.input_window(imu_reading)

        # TODO call initial heading register function
        
        if zv is None and len(imudata) == self.W:  
            # Compute the trial's zero-velocity detection using the specified detector
            zv = self.Localizer.compute_zv_lrt(np.asarray(imudata))
        else:
            # Use a pre-computed zero-velocity estimate provided by arguments
            zv = zv
     
        if (len(imudata) >= 2):
            time = imudata[-1][0] 
            prev_time = imudata[-2][0]
            dt = time - prev_time    # dt = time difference between last and current readings
        else:
            # Input data window not sufficient for processing
            if return_zv:
                return self.x, self.P, zv
            else:
                return self.x, self.P

        x_data = imudata[-1][1:]

        # Fetch variables
        x = self.x
        q = self.q
        P = self.P

        #update state through motion model
        x, q, Rot = self.Localizer.nav_eq(x, x_data, q, dt)
        F,G = self.Localizer.state_update(x_data, q, dt) 

        P = (F.dot(P)).dot(F.T) + (G.dot(self.Q)).dot(G.T)
        P = (P + P.T)/2 #make symmetric

        #corrector
        if zv == True: 
            x, P, q = self.Localizer.corrector(x, P, Rot) 


        # x[2] = -x[2] # TODO
        
        # Save variables
        self.x = x
        self.q = q
        self.P = P

        if return_zv:
            return self.x, self.P, zv
        else:
            return self.x, self.P
    

if __name__ == '__main__':
    config = {}
    config["W"] = 5
    sigma_a = 0.01
    sigma_w = 0.1*np.pi/180
    config["var_a"] = np.power(sigma_a,2)
    config["var_w"] = np.power(sigma_w,2)
    config["g"] = 9.8029
    G_opt_shoe = 2.5e8

    # imu_data = np.array([[0, 1, 1, 1, 2, 2, 2],
                        #  [1, 1, 1, 1, 2, 2, 2],
                        #  [2, 1, 1, 1, 2, 2, 2],
                        #  [3, 1, 1, 1, 2, 2, 2],
                        #  [4, 1, 1, 1, 2, 2, 2]])
    
    imu_reading = IMU_Msg()
    imu_reading.header.stamp.secs = 0
    imu_reading.header.stamp.nsecs = 0
    imu_reading.linear_acceleration.x = 1
    imu_reading.linear_acceleration.y = 1
    imu_reading.linear_acceleration.z = 1
    imu_reading.angular_velocity.x = 2
    imu_reading.angular_velocity.y = 2
    imu_reading.angular_velocity.z = 2

    ins = INS(sigma_a = 0.00098, sigma_w = 8.7266463e-5, detector="shoe", W=5)
    ins.init()

    for i in range (10):
        x, p = ins.baseline(imu_reading=imu_reading, G=G_opt_shoe)
        print (x[0])
        imu_reading.header.stamp.secs += 1
    
        