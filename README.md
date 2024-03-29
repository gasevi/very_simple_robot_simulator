# <span style="color:#00f; font-size: 4em;"> A Very Simple Robot Simulator </span>

A very simple robot simulator that uses cmd_vel_mux nodelet and implements a Kinect like simulator

<p align="center">
  <img src="images/very_simple_robot_nav_interface.png" width=500 >
</p>

## <span style="font-size: 4em;">Install</span>

```sh
cd <your_catkin_ws>/src
git clone https://github.com/gasevi/yocs_cmd_vel_mux.git
git clone https://github.com/gasevi/very_simple_robot_simulator.git
cd ..
sudo apt-get install python3-pil python3-pil.imagetk
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin_make
```

## <span style="font-size: 4em;">Include package in your ROS environment</span>

```sh
echo "source <your_catkin_ws>/devel/setup.bash" >> .bashrc 
source .bashrc
```

## <span style="font-size: 4em;">Operation Modes</span>

 * Idle mode: non interactive interface
 * Add Wall mode: press 'w' key to draw a wall on the map with the mouse pointer.
 * Delete Wall mode: press 'd' key to delete a wall.
 * Set Robot Pose mode: press 'p' key to change the robot pose. Drag and drop the robot to change the position. Click outside the robot to change its orientation.

## <span style="font-size: 4em;">Architecture</span>

<p align="center">
  <img src="images/very_simple_robot_nav_design.png" width=800 >
</p>
