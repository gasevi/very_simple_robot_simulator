# <span style="color:#00f; font-size: 4em;"> A Very Simple Robot Simulator </span>

A very simple robot simulator that uses cmd_vel_mux nodelet and implements a Kinect like simulator

## <span style="font-size: 4em;">Install</span>

```sh
cd <your_catkin_ws>/src
git clone https://github.com/gasevi/very_simple_robot_simulator.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin_make
```

## <span style="font-size: 4em;">Include package in your ROS environment</span>

```sh
echo "source <your_catkin_ws>/devel/setup.bash" >> .bashrc 
source .bashrc
```

## <span style="font-size: 4em;">Architecture</span>

<img src="images/very_simple_robot_nav_design.png" width=800 >
