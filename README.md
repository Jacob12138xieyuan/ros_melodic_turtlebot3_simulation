# ros_simulation
#
# Part 1 Preparation
## 1.Install ROS melodic
#### Follow: http://wiki.ros.org/melodic/Installation

## 2.Install neccesary packages
```
mkdir -p ~/tb3_catkin_ws/src ## Create working station
cd ~/tb3_catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/tb3_catkin_ws && catkin_make ## Compile
source ~/tb3_catkin_ws/devel/setup.bash
```

## 3. Start simulation in rviz
#### Start turtlebot3 simulation
```
export TURTLEBOT3_MODEL=burger ## TURTLEBOT3 have burger, waffle and waffle_pi, choose the one you like
roslaunch turtlebot3_fake turtlebot3_fake.launch
```
#### Control robot in rviz
```
## Open new terminal
export TURTLEBOT3_MODEL=burger 
source ~/tb3_catkin_ws/devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch ## Use w,a,d,x key to control
```
#
# Part 2 Start Gazebo
## 1. Load TurtleBot3 World Gazebo environment
```
export TURTLEBOT3_MODEL=burger  
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
![alt text](http://images.ncnynl.com/ros/2017/turtlebot3_world_bugger.png)
## 2. Load TurtleBot3 House Gazebo environment
```
$ export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_house.png)
## 3. Control robot in Gazebo

#### Start keyboard control
```
## Open new termianl, 
export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种    
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
#### Load Gezebo env
```
## Open new termianl
export TURTLEBOT3_MODEL=burger  
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
#### Start simulation
```
## Open new termianl
export TURTLEBOT3_MODEL=burger 
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```
#### Start rviz GUI
``` 
## Open new termianl
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
![alt text](http://images.ncnynl.com/ros/2017/turtlebot3_gazebo_rviz.png)
#
# Part 3 Build map with SLAM
## 1. Load gezebo env
```
sudo apt-get install ros-melodic-gmapping
cd ~/tb3_catkin_ws && catkin_make ## Compile
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
## 2. Start SLAM mapping simulation
```
## New terminal
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_virtual_slam.png)
## 3. Control & move it
```
## New terminal
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
## 4. Save map
```
## New terminal
rosrun map_server map_saver -f ~/map
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_virtual_slam_map.png)

#
# Part 4 Navigation using map
## 1. Load gazebo env
```
## new termial
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
## 2. start navigation program
```
## new termial
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_virtual_navigation.png)

#
# Part 4 AutoRace 
## 1. Install autorace package
```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git
cd ~/catkin_ws && catkin_make
```
## 2. Load autorace gezebo env
```
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```
## 3. Set Traffic Light，Parking and Toll Gate tasks
```
roslaunch turtlebot3_gazebo turtlebot3_autorace_mission.launch
```
## 4. Start turtlebot3_autorace_core
```
## new terminal
export AUTO_EX_CALIB=action
export AUTO_DT_CALIB=action
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch
## new terminal
rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_autorace_map_mission.png)
``````
``````
``````
``````
``````
``````

