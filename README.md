# tardigrade_ws 

This contains all of the code for Tardigrade to compete in Robosub 2025. It contains AUV descriptions, launch files, meshes, sub interfaces, control algorithms, and comptuer vision algorithms.

## Usage
> [!WARNING]  
> This section is under development. Launch scripts and some runtime configurations are not yet implemented.

1. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

2. Launch the robot (example):
   ```bash
   ros2 launch tardigrade tardigrade_launch.py
   ```

3. Run tests or control scripts using `ros2 run`.