import numpy as np
from numpy import linalg as LA
from .detectors.shoe import SHOE
from .tools.geometry_helpers import quat2mat, mat2quat, euler2quat, quat2euler
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

    def init_variables(self):
        if (self.config['detector'] == "shoe"):
            x = np.zeros((9))   # initialize state window
            q = np.zeros((4))   # initialize quaternion window

            # TODO : Init heading
            roll, pitch, heading = (0.0, 0.0, 0.0)
            q = euler2quat(roll, pitch, heading, 'sxyz')

            P = np.zeros((9,9)) #initial covariance matrix P
            P[0:3,0:3] = np.power(1e-5,2)*np.identity(3) #position (x,y,z) variance
            P[3:6,3:6] = np.power(1e-5,2)*np.identity(3) #velocity (x,y,z) variance
            P[6:9,6:9] = np.power(0.1*np.pi/180,2)*np.identity(3) #np.power(0.1*np.pi/180,2)*np.identity(3)
            return x, q, P

    def init(self):
        # Initialize Detector
        if (self.config['detector'] == "shoe"):
            self.detector = SHOE(self.config)

        # TODO if more detectors are added, add respective functions to create suitable detector object

        # Initialize states
        x, q, P_hat = self.init_variables()
        return x, q, P_hat
          
    def nav_eq(self, xin, imu, qin, dt):
        #update Quaternions
        x_out = np.copy(xin) #initialize the output
        omega = np.array([[0,-imu[3], -imu[4], -imu[5]],  [imu[3], 0, imu[5], -imu[4]],  [imu[4], -imu[5], 0, imu[3]],  [imu[5], imu[4], -imu[3], 0]])
    
        norm_w = LA.norm(imu[3:6])
        if(norm_w*dt != 0):
            q_out = (np.cos(dt*norm_w/2)*np.identity(4) + (1/(norm_w))*np.sin(dt*norm_w/2)*omega).dot(qin) 
        else:
            q_out = qin

        attitude = quat2euler(q_out,'sxyz')#update euler angles
        x_out[6:9] = attitude    
        
        Rot_out = quat2mat(q_out)   #get rotation matrix from quat
        acc_n = Rot_out.dot(imu[0:3])       #transform acc to navigation frame,  
        acc_n = acc_n + np.array([0,0,self.config["g"]])   #removing gravity (by adding)
        
        x_out[3:6] += dt*acc_n #velocity update
        x_out[0:3] += dt*x_out[3:6] +0.5*np.power(dt,2)*acc_n #position update
        
        return x_out, q_out, Rot_out  

    def state_update(self, imu,q, dt):
#        return F,G
        F = np.identity(9)
        F[0:3,3:6] = dt*np.identity(3)

        Rot = quat2mat(q)
        imu_r = Rot.dot(imu[0:3])
        f_skew = np.array([[0,-imu_r[2],imu_r[1]],[imu_r[2],0,-imu_r[0]],[-imu_r[1],imu_r[0],0]])
        F[3:6,6:9] = -dt*f_skew 
        
        G = np.zeros((9,6))
        G[3:6,0:3] = dt*Rot
        G[6:9,3:6] = -dt*Rot
       
        return F,G
    def corrector(self, x_check, P_check, Rot):
        eye3 = np.identity(3)
        eye9 = np.identity(9)
        omega = np.zeros((3,3))        
        
        K = (P_check.dot(self.config["H"].T)).dot(LA.inv((self.config["H"].dot(P_check)).dot(self.config["H"].T) + self.config["R"]))
        z = -x_check[3:6] ### true state is 0 velocity, current velocity is error
        q=mat2quat(Rot)   
        dx = K.dot(z) 
        x_check += dx  ###inject position and velocity error
         
        omega[0:3,0:3] = [[0,-dx[8], dx[7]],[dx[8],0,-dx[6]],[-dx[7],dx[6],0]] 
        Rot = (eye3+omega).dot(Rot)
        q = mat2quat(Rot)
        attitude = quat2euler(q,'sxyz')
        x_check[6:9] = attitude    #Inject rotational error           
        P_check = (eye9-K.dot(self.config["H"])).dot(P_check)
        P_check = (P_check + P_check.T)/2
        return x_check, P_check, q
       
    def compute_zv_lrt(self, x_in, G=3e8, return_zv=True):
        """ Calculates Zero Velocity Update binary value (True / False)

            :param x_in: IMU Data time window
                        - Columns: [timeStamp, lin.acc.x, lin.acc.y, lin.acc.z, ang.vel.x, ang.vel.y, ang.vel.z]
                        - Rows : Number of records in window
            :return ZuPT boolean value (True - Zero Velocity)
        """

        zv = self.detector.estimate(x_in)

        if return_zv:
            zv=zv<G
        return zv

        
