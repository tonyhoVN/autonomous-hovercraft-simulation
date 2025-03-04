# Autonomous Hovercraft Simulation
Simulation of autonomous hovercraft solving the maze. The simulation is runned on Gazebo and ROS2 env. The algorithm includes LQR optimal motion control, lidar-based SLAM, and A-star path planning 

<img src="media/maze.jpg" width="400">

## Hovercraft Model
<img src="media/CAD_HoverCraft.jpg" width="400">
<img src="media/Real_hover_craft.jpg" width="400">

## Installation 
- navigate to your ros2 work space to clone the folder.
```
cd ~/ros2_ws/src
git clone https://github.com/tonyhoVN/autonomous-hovercraft-simulation.git
cd .. & colcon build --packages-select hovercraft_simu
source install/local_setup.bash
```

## Run
```
ros2 launch hovercraft_simu start_final.launch.xml
```
