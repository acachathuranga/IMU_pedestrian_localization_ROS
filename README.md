# IMU Based Pedestrian Localization for ROS

Foot mounted IMU based localization using dead reckoning and Zero-Velocity updates. SHOE detecor is used to detect zero velocity time instances. 

Some parts of the EKF algorithm were derived from https://github.com/utiasSTARS/pyshoe.git

## Dependencies
* ROS
* Python 3.5 / 3.6
* Numpy
* Matplotlib

#### Note:
>  Make sure correct python version is mentioned in main.py line 1
  
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

### Human Orientation Approximation

The orientation of the human is approximated using Foot orientation. Three approximation methods are provided. 

 * 'Real' : IMU Yaw will be published. This will be the foot Yaw, not the Human Yaw
 * 'Zupt' : Yaw will be updated only at Foot Zero Velocity instances
 * 'Stable' : Yaw will be updated when the foot is approximately horizontal. This will reduce Yaw distortions when foot inclined with respect to floor


## Parameters
Refer 'yaw_pub_method' parameter in parameters.yaml file within config directory.
* **imu_topic** : Subscribed IMU topic. (Default: /footIMU/IMU)

* **yaw_pub_method**: Human Orientation Approximation Method. (Default: 'Stable')
* **yaw_pub_latch**: If true, odometry will keep publishing using last known Orientation, until an orientation update is available. Else, odometry is published only when an Orientation update is available (Default: true)
* **calibrate_yaw**: Calibrate yaw angle according to initial walking direction. (Default: false)
* **calibration_steps**: Calibration will be performed considering readings upto this many walking steps. (Default: 2)

#### Note
 
> Example: calibration_steps = 2
>> Odometry will not be published until user walks two steps. Once the 2 steps are completed, calibrated odometry values (including time-stamped previous values) will be published.


### Demo

A demo application to read and publish odometry data from a CSV file is also provided. Configure the input CSV file directory and name in the demo.py file and run.
>      python3 ./scripts/demo.py

