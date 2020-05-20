# IMU Based Pedestrian Localization for ROS

Foot mounted IMU based localization using dead reckoning and Zero-Velocity updates. SHOE detecor is used to detect zero velocity time instances. 

Some parts of the EKF algorithm were derived from https://github.com/utiasSTARS/pyshoe.git

## Dependencies
* ROS
* Python 3.5
* Numpy
* Matplotlib
  
  
**Install following packages** <br /> 
>      sudo apt-get install python3-yaml
>      sudo apt-get install python3-tk
>      sudo pip3 install rospkg catkin_pkg numpy matplotlib

#### Subscribed Topics
- sensor_msgs/IMU : /footIMU/IMU
  
#### Published Topics
- nav_msgs/Odometry: /imu_odometry

The topics can be remapped or changed in the launch file.

Compile and run the package
>       roslaunch imu_odometry imu_odometry.launch

### Calibration

The user can wear the IMU in any orientation on the foot. The published odometry values are orientation calibrated values.

* Roll and Pitch angle is calibrated perpendicular to gravity vector
* Yaw angle calibration will be added in future

#### Note
The number of initial readings used for calibration is set by the 'calibration_steps' parameter in the launch file. 
> Example: calibration_steps = 2
>> Odometry will not be published until user walks two steps. Once the 2 steps are completed, calibrated odometry values (including time-stamped previous values) will be published.

### Demo

A demo application to read and publish odometry data from a CSV file is also provided. Configure the input CSV file directory and name in the demo.py file and run.
>      python3 ./scripts/demo.py

