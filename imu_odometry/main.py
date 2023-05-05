import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import Imu 
import numpy as np
from .INS.tools.geometry_helpers import euler2quat
from .pedestrian_localizer import pedestrian_localizer

class IMUOdometry(Node):
    
    def __init__(self):
        super().__init__('imu_odometry_publisher')
        self.get_logger().info("IMU Odometry Publisher")
      
        self.declare_parameter('calibrate_yaw',True)
        calibrate_yaw = self.get_parameter('calibrate_yaw').get_parameter_value().bool_value 

        self.declare_parameter('calibration_distance',2.0)
        calibration_distance = self.get_parameter('calibration_distance').get_parameter_value().double_value 
                
        self.declare_parameter('imu_topic',"/vectornav/IMU")
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value 
        
        self.declare_parameter('yaw_pub_method','Stable')
        yaw_pub_method = self.get_parameter('yaw_pub_method').get_parameter_value().string_value 
        
        self.declare_parameter('yaw_pub_latch',True)
        yaw_pub_latch = self.get_parameter('yaw_pub_latch').get_parameter_value().bool_value 
        
        
        
        self.localizer = pedestrian_localizer(calibrate_yaw=calibrate_yaw, calibration_distance=calibration_distance, yaw_method=yaw_pub_method, yaw_latch=yaw_pub_latch, callback=self.publish_odom)
        
        self.odom_pub =  self.create_publisher(Odometry, 'imu_odometry', 100)   
        
        self.subscription = self.create_subscription(Imu,imu_topic, self.callback,10) 
        
        self.get_logger().info("Subscribed to: "+ imu_topic)
        self.get_logger().info("Human Orientation Approximation Method: "+ yaw_pub_method)
        self.get_logger().info("Human Orientation Latch: "+ str(yaw_pub_latch))
        
        self.total = 0
        self.count = 0        


    def callback(self,imu_reading): 
        self.localizer.update_odometry(imu_reading)        
        # x=imu_reading
        # acc = (float(x.linear_acceleration.x)**2+float(x.linear_acceleration.y)**2+float(x.linear_acceleration.z)**2)**0.5
        #self.get_logger().info(str(acc))
         
        # self.total += acc
        # self.count+=1
        # if self.count==200:
        #     self.get_logger().info(str(self.total/self.count))
        #     self.count,self.total=0,0
        
    def publish_odom(self,x, p, header):
        """ Publish Odometry Message

            :param x: State
                        pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw
        """
        if x is not None:
            x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw = x
            qw, qx, qy, qz =  euler2quat(roll, pitch, yaw, axes='sxyz')
            odom = Odometry()
            odom.header = header
            odom.header.frame_id = 'odom'
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = z 
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = vel_x
            odom.twist.twist.linear.y = vel_y
            odom.twist.twist.linear.z = vel_z

            pose_covariance = np.zeros((6,6))
            pose_covariance[:3, :3] = p[:3, :3]
            pose_covariance[:3, 3:] = p[:3, 6:]
            pose_covariance[3:, :3] = p[6:, :3]
            pose_covariance[3:, 3:] = p[6:, 6:]
            odom.pose.covariance = pose_covariance.reshape(-1).tolist()

            twist_covariance = np.zeros((6,6))
            twist_covariance[:3, :3] = p[3:6, 3:6]
            odom.twist.covariance = twist_covariance.reshape(-1).tolist()

            self.odom_pub.publish(odom)
    
def main(args=None):
    rclpy.init(args=args)

    node = IMUOdometry()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except :
        print("Error")   
