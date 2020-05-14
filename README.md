# ros_simulation
#
# Part 1
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
export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种
roslaunch turtlebot3_fake turtlebot3_fake.launch
```
#### Control robot in rviz
```
## Open new terminal
export TURTLEBOT3_MODEL=burger 
source ~/tb3_catkin_ws/devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch ## Use w,a,d,x key to control
```
# Part 2
## 1. Load TurtleBot3 World Gazebo environment
```
export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种    
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
## 2. Load TurtleBot3 House Gazebo environment
```
$ export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
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
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种   
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
#### Start simulation
```
## Open new termianl
export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种  
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```
#### Start rviz GUI
``` 
## Open new termianl
export TURTLEBOT3_MODEL=burger
## TURTLEBOT3_MODEL有burger, waffle或waffle_pi三种  
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
``````
``````
``````
