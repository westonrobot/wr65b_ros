## About
This repository contains ROS packages for the WR65-B robot arm

Operating System: Ubuntu 18.04, 20.04</br>
ROS Version: Melodic, Noetic

## Packages
rm_65_description
* Robot models and configuration files
* rm_65.urdf.xarco: RM65-B model without end tool

rm_65_moveit_config
* launch and configuration files to start MoveIt motion planning

rm_gazebo
* parameters and configuration files for Gazebo robot model

rm_65_demo
* MoveIt examples (obstacle avoidance, pick & play)

rm_msgs
* control and status messages

rm_control
* robot controller
* send cubic spline interpolated robot arm trajectories planned by MoveIt to rm_driver node every 20ms. Time period is adjustable but should be >10ms

rm_driver
* Establish socket connection with the robot arm through the ethernet port. Robot arm default IP address is 192.168.1.18
* subscribe to topics and update the joint angles of the robotic arm in RVIZ

rm_bringup
* automatically run rm_driver, rm_control, and RVIZ (MoveIt). Control real robot by dragging the simulated robot model

## Setup Workspace
1. Create workspace and change directory to the src folder
   ```
   $ mkdir -p <your_workspace>/src
   $ cd <your_workspace>/src
   ```
2. Git clone into the src folder
   ```
   git clone https://github.com/westonrobot/
   ```
3. Install system dependencies
   ```
   rosdep install -y --from-paths . --ignore-src --rosdistro <distro> -r
   ```
4. Build packages
   ```
   $ cd ..
   $ catkin init
   $ catkin build rm_msgs
   $ catkin build
   ```

## Visualize in RVIZ
Source robot workspace and launch rm_65_description package
```
roslaunch rm_65_description display.launch
```

If model is not shown, manually change the **Fixed Frame** to **base_link**, and add the robot model.
<img src="./docs/base_link.png"/>
<img src="./docs/robot_model.png"/>

## Motion Planning using MoveIt
MoveIt is a motion planning library that integrates component packages related to mobile operations in the ROS system. Through the MoveIt plug-in, you can control the robot arm to complete 
functions such as drag planning, random target planning, initial position and pose update, and collision detection.

Run MoveIt demo of the RM65B robotic arm in simulation
```
roslaunch rm_65_moveit_config demo.launch
```

1. Drag Planning
   * Drag the front end of the robot arm to change its position and pose. Then click the "Plan & Execute" button on the Planning tab
2. Target Planning
   * Click the Goal State drop-down list in the Planning tab to select the target position and pose of the robot arm, and then click the "Plan & Execute" button
   <img src="./docs/target_planning.png" />

## MoveIt with Gazebo Simulation
1. Modify the **rm_65_moveit_controller_manager.launch.xml** in the **rm_65_moveit_config** package to load the **controllers_gazebo.yaml** file 
<img src="./docs/moveit_controller_manager_sim.png"/>

2. Start RVIZ (MoveIt) and Gazebo
   ```
   roslaunch rm_gazebo arm_65_bringup_moveit.launch
   ```
3. Set **Fixed Frame** to **base_link**. Then add **MotionPlanning** into the panel. In RVIZ, drag the arm to a new position, then click **Plan & Execute**. The arm in Gazebo will then move to the position set in RVIZ. 

## Examples (Simulation)
1. Obstacles Avoidance
2. Pick & Place

### Obstacles Avoidance
1. Launch robotic arm in RVIZ with MoveIt
   ```
   roslaunch rm_65_moveit_config demo.launch
   ```
2. Launch python file containing obstacles in another terminal
   ```
   rosrun rm_65_demo moveit_obstacles_demo.py
   ```

   **Note**: Run "sudo chmod +x <python_file>.py" if the python files are not executable

### Pick & Place
1. Launch robotic arm in RVIZ with MoveIt
   ```
   roslaunch rm_65_moveit_config demo.launch
   ```
2. Launch pick_place_demo cpp file in another terminal
   ```
   rosrun rm_65_demo pick_place_demo
   ```

## MoveIt with real robot
1. Before operating it, make sure the robot and PC are connected to the same WiFi. Ping the robot and make sure there is a connection.
   ```
   ping 192.168.1.18
   ```
2. Modify the **rm_65_moveit_controller_manager.launch.xml** in the **rm_65_moveit_config** package to load the **controllers.yaml** file
<img src="./docs/moveit_controller_manager_real.png"/>

3. Launch rm_control package
   ```
   roslaunch rm_control rm_control.launch
   ```
3. Launch rm_driver package and MoveIt
   ```
   roslaunch rm_bringup rm_robot.launch
   ```
4. In RVIZ, drag the arm to a new position or select **Goal State**, then click **Plan & Execute**. The robotic arm will then move to the target location.

## Move Command Examples
1. MoveJ Command
2. MoveJ_P Command
3. MoveL Command

</br>

* MoveJ and MoveJ_P: 
  * The intermediate trajectory is not constrained.

    <img src="./docs/movejp.png" />

* MoveL:
  * The trajectory between the two points is a linear motion.

    <img src="./docs/movel.png" />

### MoveJ Command
ROS node **api_moveJ_demo** deliver MoveJ commands via **/rm_driver/MoveJ_Cmd** topic. The demo code was written in c++, located at **rm_65_demo/src/api_moveJ_demo.cpp**. The MoveJ command contains messages for each individual joints. The contents are encapsulated through the JSON protocol and pass to the robot using Socket.
1. Run roscore
   ```
   roscore
   ```
2. Launch rm_driver node
   ```
   rosrun rm_driver rm_driver
   ```
3. Launch api_moveJ_demo node
   ```
   rosrun rm_65_demo api_moveJ_demo
   ```

### MoveJ_P Command
ROS node **api_moveJ_P_demo** deliver MoveJ_P commands via **/rm_driver/MoveJ_P_Cmd** topic. The demo code was written in c++, located at **rm_65_demo/src/api_moveJ_P_demo.cpp**. The difference between MoveJ command is that MoveJ_P controls the movement through the end position of the robot arm.
1. Run roscore
   ```
   roscore
   ```
2. Launch rm_driver node
   ```
   rosrun rm_driver rm_driver
   ```
3. Launch api_moveJ_P_demo node
   ```
   rosrun rm_65_demo api_moveJ_P_demo
   ```

### MoveL Command
ROS node **api_moveL_demo** deliver MoveL commands via **/rm_driver/MoveL_Cmd** topic. The demo code was written in c++, located at **rm_65_demo/src/api_moveL_demo.cpp**.
1. Run roscore
   ```
   roscore
   ```
2. Launch rm_driver node
   ```
   rosrun rm_driver rm_driver
   ```
3. Launch api_moveL_demo node
   ```
   rosrun rm_65_demo api_moveL_demo
   ```
