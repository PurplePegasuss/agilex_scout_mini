# agilex_scout_mini
ROS-based SLAM and trajectory planning for AgileX Scout Mini Robot

### SLAM in Robotics Lab
<p align = "center">
  <img src = "images/fov.gif" height = "240px" style="margin:10px 10px">
  <img src = "images/ros.gif" height = "240px" style="margin:10px 10px">
</p>

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
