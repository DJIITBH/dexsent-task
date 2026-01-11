# Fanuc CRX-10iA Robot Arm With Moveit2 in ROS 2

This repository contains the implementation of 2 tasks as given by company. 

## Features
- **Simulation of CRX-10iA robot arm in Gazebo**
- **MoveIt2 integration** for motion planning and visualization in Rviz
- **Dual Arm Planning** custom cartesian controller

## Requirements
- **ROS 2 (Humble)**
- **Gazebo**
- **MoveIt2**
- **ros2_control** and related plugins

## Setup
![image](https://drive.google.com/file/d/1Vhv7Y6WYol_J1OKm2VxVR3Wsv5PNvUAF/view?usp=sharing)

## TASK 1 : Robot arm with Robotiq gripper + Motion Planning (Moveit2)

### 1. Launch the Robot Arm in Gazebo
Start the simulated environment with robot arm in Gazebo:

```bash
ros2 launch ur_description_pkg gazebo.launch.py 
```

### 2. Launch MoveIt2 for Path Planning and Rviz Visualization
Start MoveIt2 for robot motion planning and visualization in Rviz:

```bash
ros2 launch fanuc_moveit_config moveit.launch.py
```
### 3. Give Goal via command line:
```bash
ros2 action send_goal \
  /arm_cont/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
      points: [{
        positions: [0.0, -0.5, 0.5, 0.0, 0.3, 0.0],
        time_from_start: { sec: 3, nanosec: 0 }
      }]
    }
  }"
```
## Or
### 3. Give Goal via C++ Script:
```bash
ros2 run robot_moveit simple_moveit_interface
```
### 4. Closing Gripper:
```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.4, max_effort: 100.0}}"
```


## TASK 2 - DUAL ARM MOTION PLANNING - Cartesian Controller

### 1. Launch the Robot Arms in Gazebo
Start the simulated environment with robot arms in Gazebo:

```bash
ros2 launch ur_description_pkg dual_gazebo.launch.py
```
### 2. Run the Cartesian Controller node

```bash
ros2 run robot_moveit cartesian_controller
```
### 2. Publish the desired pose goals for Both the arms - Goal
Format - [x_l, y_l, z_l, qx_l, qy_l, qz_l, qw_l, x_r, y_r, z_r, qx_r, qy_r, qz_r, qw_r] 
Where q-> quaternion 
```bash
ros2 topic pub --once /desired_pose std_msgs/msg/Float64MultiArray \
"{data: [0.2, 1.149, 0.28, -0.66, -0.52, -0.02, 0.52,
         -0.46, 1.179, 0.338, -0.7, 0.42, 0.113, 0.55]}"
```
### 2. Publish the desired pose goals for Both the arms - Home

```bash
ros2 topic pub --once /desired_pose std_msgs/msg/Float64MultiArray \
"{data: [1.23, -0.15, 0.44, 0.91, -0.0, 0.39, -0.0,
         -1.27, -0.15, 0.44, 0.91, -0.0, -0.39, 0.0]}"
```

## Challenges Faced
1. **Integrating the Robotiq Gripper:**
   - Took significant time to search for issue of mimic joints in ros2 control, modified gazebo ros2 control as moveit could not find mimic joints
   - Made 2 different planning groups for robot arm and gripper

2. **Dual Arm Cartesian Controller:**
   - First made a single URDF file for both robots, designed a triangular prism, mounting arm was a challenge.
   - Made 2 ros2 controllers for arms, then tried to make 2 move groups using Moveit for motion planning however, with that whenever i commanded move using moveit api, the robot arm moved sequentially one after the other, there was no synchronization whatsoever even with different move groups made. So I decided a new strategy to deal with this.
   - Made a custom cartesian controller using KDL-kinematics and dynamics library for doing off the shelf inverse kinematics.
   - Made a script consisting of subscriber for receiving the goal pose for both the arms, then calculating the respective joint angles for the arms and then moving the arms synchronously by publishing the joint angles with joint trajectory controller in a single callback function.

## Video Demonstration (TASK 1)
Watch the video demonstrating the implementation of Single arm planning with moveit2 and robotiq gripper:
[Watch Video](https://drive.google.com/file/d/1Y_UGj7euWUwUPLbSSBUvMNqmEwy80Kb3/view?usp=sharing)

## Video Demonstration (TASK 2)
Watch the video demonstrating the implementation of Duak arm planning with Cartesian Controller:
[Watch Video](https://drive.google.com/file/d/1KiJ8Zgyk8HmjuDJhHmEUsnXrvC-3C7LA/view?usp=sharing)

