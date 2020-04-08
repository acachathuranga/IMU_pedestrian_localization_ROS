import numpy as np
from numpy import linalg as LA
import sys

class SHOE():
    def __init__(self, config):
        """ Initialize SHOE detector 

            :param config: Dictionary of configuration values
                                - W : IMU data window size
                                - var_a : Linear acceleration variance
                                - var_w : Angular velocity variance
        """
        self.config = config

    def estimate(self, imu_data):
        """ Estimate Zero Velocity Update condition given a IMU data window

            :param imu_data: Time window of IMU data
                        - 2D array of data where each row is an IMU data reading
                        - Columns are : timeStamp, lin.acc.x, lin.acc.y, lin.acc.z, ang.vel.x, ang.vel.y, ang.vel.z
            :return Estimated value (0 to 1)
        """
        window_size = self.config["W"]
        inv_a = (1/self.config["var_a"])
        inv_w = (1/self.config["var_w"])
        g = self.config["g"]
        
        acc = imu_data[:,1:4]
        gyro = imu_data[:,4:7]
        smean_a = np.mean(acc, axis=0)
        g_comp = g * smean_a/LA.norm(smean_a)    # Gravity compensation term

        zupt = 0.0
        # T = np.sum( inv_a*np.dot((acc-g_comp), (acc-g_comp).T) + inv_w*np.dot(gyro, gyro.T) )

        for s in range(window_size):
            zupt += inv_a*np.dot((acc[s]-g_comp), (acc[s]-g_comp).T) + inv_w*np.dot(gyro[s], gyro[s].T)

        zupt = zupt/window_size
        return zupt
        
    
if __name__ == '__main__':
    config = {}
    config["W"] = 5
    sigma_a = 0.01
    sigma_w = 0.1*np.pi/180
    config["var_a"] = np.power(sigma_a,2)
    config["var_w"] = np.power(sigma_w,2)
    config["g"] = 9.8029

    imu_data = np.array([[0, 1, 1, 1, 2, 2, 2],
                         [1, 1, 1, 1, 2, 2, 2],
                         [2, 1, 1, 1, 2, 2, 2],
                         [3, 1, 1, 1, 2, 2, 2],
                         [4, 1, 1, 1, 2, 2, 2]])

    detector = SHOE(config)
    print(detector.estimate(imu_data))
        
