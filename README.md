# ros_melodic_turtlebot3_simulation
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
![alt text](https://github.com/Jacob12138xieyuan/ros_simulation/blob/master/1.png?raw=true)
#### Control robot in rviz
```
## Open new terminal
export TURTLEBOT3_MODEL=burger 
source ~/tb3_catkin_ws/devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch ## Use w,a,d,x key to control
```
#
# Part 2 Start Gazebo
## 1. Test TurtleBot3 World Gazebo environment
```
export TURTLEBOT3_MODEL=burger  
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
![alt text](http://images.ncnynl.com/ros/2017/turtlebot3_world_bugger.png)
## 2. Test TurtleBot3 House Gazebo environment
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_house.png)
## 3. Control robot in Gazebo

#### Start keyboard control
```
## Open new termianl, 
export TURTLEBOT3_MODEL=burger 
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
#### Load Gezebo env
```
## Open new termianl
export TURTLEBOT3_MODEL=burger  
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
#### Start turtlebot3 simulation
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
## 1. Install pakages
```
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-gmapping
```
## 1. Load gezebo env
```
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
## 3. Control & move robot
```
## New terminal
export TURTLEBOT3_MODEL=burger
source ~/tb3_catkin_ws/devel/setup.bash
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
## 1. Install pakages
```
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-move-base
```
## 2. Load gazebo env
```
## new termial
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
## 3. start navigation program
```
## new termial
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

## 4. Set destination in rviz
```
## new terminal
export TURTLEBOT3_MODEL=burger
rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz
```
![alt text](https://github.com/Jacob12138xieyuan/ros_simulation/blob/master/2.png?raw=true)
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_virtual_navigation.png)

#
# Part 4 AutoRace 
## 1. Install autorace package
```
cd ~/tb3_catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git
cd ~/tb3_catkin_ws && catkin_make
```
## 2. Load autorace gezebo env
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```
## 3. Set Traffic Light，Parking and Toll Gate tasks
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_autorace_mission.launch
```
## 4. Calibrate camera
```
export GAZEBO_MODE=true
export AUTO_IN_CALIB=action
roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```
## 5. Start turtlebot3_autorace_core
```
## new terminal
export AUTO_EX_CALIB=action
export AUTO_DT_CALIB=action
export TURTLEBOT3_MODEL=burger
source ~/tb3_catkin_ws/devel/setup.bash
roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch
```
## 6.Start all nodes
```
## new terminal
rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_autorace_map_mission.png)
#
# Part 5 Turtlebot3 with manipulator
## 1. Install packages
```
cd ~/tb3_catkin_ws/src/
sudo apt-get install ros-melodic-moveit
sudo apt-get install ros-melodic-moveit-msgs
sudo apt-get install ros-melodic-joint-trajectory-controller
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
cd ~/tb3_catkin_ws && catkin_make
```
## 2. Load gezebo env
```
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
## Press [▶] button in Gazebo
```
![alt text](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/tb3_omx_gazebo.png)
## 3. Run move_group node 
```
## new terminal
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
## Untill see "You can start planning now!"
```
## 4. Control manipulator in rviz
```
## new terminal 
roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```
![alt text](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/tb3_omx_rviz.png)
## 5. Run ROBOTIS GUI Controller
```
roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
![alt text](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/tb3_omx_gui_controller.png)
#
# Part 6 Walk in a maze
## 1. Install package
```
cd ~/tb3_catkin_ws/src && git clone https://github.com/EndlessLoops/fira_maze
cd ~/tb3_catkin_ws && catkin_make
```
## 2. Start roscore
```
roscore
```
## 3. Load gezebo maze env
```
## new terminal
export TURTLEBOT3_MODEL=burger
source ~/tb3_catkin_ws/devel/setup.bash
roslaunch fira_maze maze_1_world.launch
```
## 4. Start maze explore simulation
```
source ~/tb3_catkin_ws/devel/setup.bash
rosrun fira_maze maze_explorer.py
```
![alt text](http://images.ncnynl.com/ros/2019/turtlebot3_virtual_maze_solution.png)
#
# Part 7 ROSPlan mission planner
## 1. Install packages
```
sudo apt install flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet
cd ~/tb3_catkin_ws/src && git clone https://github.com/KCL-Planning/rosplan
cd ~/tb3_catkin_ws && catkin_make
sudo apt install ros-${ROS_DISTRO}-turtlebot3-gazebo ros-${ROS_DISTRO}-turtlebot3-navigation ros-${ROS_DISTRO}-move-base-msgs
cd ~/tb3_catkin_ws/src
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone https://github.com/KCL-Planning/rosplan_demos.git
cd ~/tb3_catkin_ws && catkin_make
```
## 2. Start rviz demo
```
roslaunch rosplan_turtlebot3_demo turtlebot.launch
```
![alt text](https://github.com/Jacob12138xieyuan/ros_simulation/blob/master/3.png?raw=true)
## 3. Generate a plan
```
## new terminal
source ~/tb3_catkin_ws/devel/setup.bash
rosrun rosplan_turtlebot3_demo turtlebot_explore.bash
```
## 4. Custom waypoints
```
## Alternatively instead of step 3, you can test your own custom waypoint from yaml file:
## new terminal
rosed rosplan_turtlebot3_demo waypoints.yaml
rosrun rosplan_turtlebot3_demo turtlebot_explore_wp_from_file.bash
```
![alt text](https://github.com/Jacob12138xieyuan/ros_simulation/blob/master/4.png?raw=true)
