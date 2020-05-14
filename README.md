# ros_simulation
## 1.Install ROS melodic
### Follow: http://wiki.ros.org/melodic/Installation

## 2.Install neccesary packages
```
mkdir -p ~/tb3_catkin_ws/src 
cd ~/tb3_catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/tb3_catkin_ws && catkin_make
source ~/tb3_catkin_ws/devel/setup.bash
```

## 3. Test simulation & control robot
```
export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种
roslaunch turtlebot3_fake turtlebot3_fake.launch
```
#### New terminal
```
export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
## Use w,a,d,x key to control
```
``````
``````
``````
``````
``````
``````
``````
``````
``````
