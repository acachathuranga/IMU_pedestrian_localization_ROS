import numpy as np
from numpy import linalg as LA
from .detectors.shoe import SHOE
# from .detectors.lstm import LSTM
from .tools.geometry_helpers import euler2mat, VecTose3, MatrixExp6, Adjoint
import sys

class Localizer():
    def __init__(self, config):
        """ Initializes inertial navigation class
            
            :param config: Dictionary of parameters
                            required:
                                - 'detector' : Valid detector name string
                                - All the parameters required by the specified detector class
                                - 'W' : Window size for temporal data processing

        """
        self.config = config

    def init_variables(self, position=(0,0,0), attitude=(0,0,0)):
        tf = np.zeros((4,4))   # initialize homogenious transformation matrix
        twist = np.zeros((6,1))   # initialize twist matrix (lin.x, lin.y, lin.z, ang.x, ang.y, ang.z)

        tf[3, :3] = position
        tf[3,3] = 1

        # Note: Since initial heading is explicitly set, Initial odometry will have a slight Z plane drift
        tf[:3, :3] = euler2mat(attitude[0], attitude[1], attitude[2]) 

        P = np.zeros((9,9)) #initial covariance matrix P
        P[0:3,0:3] = np.power(1e-5,2)*np.identity(3) #position (x,y,z) variance
        P[3:6,3:6] = np.power(1e-5,2)*np.identity(3) #velocity (x,y,z) variance
        P[6:9,6:9] = np.power(0.1*np.pi/180,2)*np.identity(3) #np.power(0.1*np.pi/180,2)*np.identity(3)

        return tf, twist, P

    def init(self):
        # Initialize Detector
        if (self.config['detector'] == "shoe"):
            #### SHOE Detector Thresholds ####
            # G_opt_shoe = 2.5e8
            G_opt_shoe = 2.5e8 #8e8
            self.config["G_opt_shoe"] = G_opt_shoe
            self.detector = SHOE(self.config)

        if (self.config['detector'] == 'lstm'):
            pass
            # self.detector = LSTM()
            # self.detector = self.detector.cuda()
        # TODO if more detectors are added, add respective functions to create suitable detector object

        # Initialize states
        tf, twist, P_hat = self.init_variables()
        return tf, twist, P_hat
          
    def nav_eq(self, tf, twist, imu, dt):
        # New transform is calculated using previous twist
        # print(twist)
        vel_se3 = VecTose3(twist.reshape(6,))
        print(vel_se3 * dt)
        step_transform = MatrixExp6(vel_se3 * dt)
        tf = np.dot(tf, step_transform)
        
        # transform gravity to IMU frame
        g_imu = np.dot(tf[:3, :3].T, np.array([0,0,self.config["g"]]))
        """ Z axis is considered as pointing upwards. Hence gravity is positive acceleration """

        acc_n = imu[0:3] - g_imu    # Gravity removal
        lin_vel = np.dot(tf[:3, :3].T, twist[0:3])  # Previous linear velocity in IMU frame
        lin_vel += (acc_n * dt).reshape(3,1)   # new velocity calculation
        
        # IMU Frame twist
        twist[0:3] = lin_vel
        twist[3:6] = imu[3:6].reshape(3,1)  # Angular velocity reading

        # Navigation Frame twist
        twist = Adjoint(tf).dot(twist)
        
        return tf, twist  

    def state_update(self, imu,tf, dt):
        F = np.identity(9)
        F[0:3,3:6] = dt*np.identity(3)

        imu_r = tf[:3, :3].dot(imu[0:3])
        f_skew = np.array([[0,-imu_r[2],imu_r[1]],[imu_r[2],0,-imu_r[0]],[-imu_r[1],imu_r[0],0]])
        F[3:6,6:9] = -dt*f_skew 
        
        G = np.zeros((9,6))
        G[3:6,0:3] = dt*tf[:3, :3]
        G[6:9,3:6] = -dt*tf[:3, :3]
        return F,G

    def corrector(self, tf, twist, P_check):
        eye3 = np.identity(3)
        eye9 = np.identity(9)
        omega = np.zeros((3,3))        
        
        K = (P_check.dot(self.config["H"].T)).dot(LA.inv((self.config["H"].dot(P_check)).dot(self.config["H"].T) + self.config["R"]))
        z = twist[3:6] ### true state is 0 velocity, current velocity is error
        # Only angular velocity is taken for error calculation since, our measurement is only angular velocity  
        dx = K.dot(z) 
        #inject position and velocity error
        tf[3, :3] += dx[0:3].reshape(3,)
        twist[:3] += dx[3:6]

        omega[0:3,0:3] = [[0,-dx[8], dx[7]],[dx[8],0,-dx[6]],[-dx[7],dx[6],0]] 
        tf[:3, :3] = (eye3+omega).dot(tf[:3, :3]) # Inject rotational error

        P_check = (eye9-K.dot(self.config["H"])).dot(P_check)
        P_check = (P_check + P_check.T)/2
        return tf, twist, P_check
       
    def compute_zv_lrt(self, x_in, return_zv=True):
        """ Calculates Zero Velocity Update binary value (True / False)

            :param x_in: IMU Data time window
                        - Columns: [timeStamp, lin.acc.x, lin.acc.y, lin.acc.z, ang.vel.x, ang.vel.y, ang.vel.z]
                        - Rows : Number of records in window
            :return ZuPT boolean value (True - Zero Velocity)
        """
        zv = self.detector(x_in)

        if return_zv:
            return zv

        
