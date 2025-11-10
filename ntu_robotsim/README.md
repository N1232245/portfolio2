# NTU_RobotSim_Nav2
NTU Robot Simulation for Nav2 using ROS 2 Humble and Gazebo Fortress

## Building the Project

```bash
cd ~/Projects/ntu_robotsim_ws
colcon build --symlink-install
source install/setup.bash
```

## Launching Simulations

### Launch Maze Simulation
```bash
ros2 launch ntu_robotsim maze.launch.py
```

### Then Launch Single Robot Simulation
```bash
ros2 launch ntu_robotsim single_robot_sim.launch.py
```
