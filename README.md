# Autonomous Hovercraft Simulation
A simulation project on Gazebo to test autonomous hovercraft. The simulation is runned on ROS2

## Installation 
- navigate to your ros2 work space to clone the folder.
```
cd ~/ros2_ws/src
git clone https://github.com/tonyhoVN/autonomous-hovercraft-simulation.git
cd .. & colcon build --packages-select hovercraft_simu
```

## Run
```
ros2 launch hovercraft_simu start_final.launch
```
