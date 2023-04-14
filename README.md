# agilex_scout_mini
ROS-based SLAM and trajectory planning for AgileX Scout Mini Robot

![Alt Text](https://i.ibb.co/yfj2PVP/ros.gif)


### How to Run Simulation

1. Run the following command in your catkin_ws folder:

   ```shell
   roslaunch agilex_scout_mini scout_world.launch
   ```
2. Run the following command in the new tab:

   ```shell
   roslaunch gazebo_ros empty_world.launch
   ```

3. Finally, Run the following command in the new tab:

   ```shell
   roslaunch scout_bringup scout_teleop_keyboard.launch
   ```

Now you can control the robot in gazebo using the keys
