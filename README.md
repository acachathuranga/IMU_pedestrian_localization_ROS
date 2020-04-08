# IMU Based Pedestrian Localization for ROS

Foot mounted IMU based localization using dead reckoning and Zero-Velocity updates. SHOE detecor is used to detect zero velocity time instances. 

Some parts of the EKF algorithm were derived from https://github.com/utiasSTARS/pyshoe.git

## Dependencies
* ROS
* Python 3.5
* Numpy
  
  
**Install following packages** <br /> 
>      sudo apt-get install python3-yaml
>      sudo pip3 install rospkg catkin_pkg

#### Subscribed Topics
- sensor_msgs/IMU : /vectornav/IMU
  
#### Published Topics
- nav_msgs/Odometry: /imu_odometry

The topics can be remapped or edited in the main.py file. 

Compile and run the package
>       rosrun imu_localization main.py

### Demo

A demo application to read and publish odometry data from a CSV file is also provided. Configure the input CSV file directory and name in the demo.py file and run.
>      python3 demo.py

