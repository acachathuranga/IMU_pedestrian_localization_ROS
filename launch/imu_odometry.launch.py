import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

 
 
    
def generate_launch_description():
    
    pkg_dir = get_package_share_directory("imu_odometry")

    # Create Launch configuratios
    #namespace = LaunchConfiguration('namespace')
        
    imu_node = Node(
            package='imu_odometry',
            # namespace='turtlesim1',
            executable='exec_py',
            name='imu_odometry_publisher',
            #namespace=namespace,
            output="screen",
            parameters=[  ParameterFile(os.path.join(pkg_dir, 'config', 'parameters.yaml'), allow_substs=True)],
            #prefix=['xterm -e gdb -ex run --args']
        # arguments=['--ros-args', '--log-level', 'debug'],
        #emulate_tty=True)
            
        )
   
    
    ld = LaunchDescription()
    ld.add_action(imu_node)

    
    return ld