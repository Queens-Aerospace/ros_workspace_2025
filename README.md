# ROS Workspace 2025

## Instructions for Running Simulation and Code

### MicroDDS
To start the MicroDDS agent:
```bash
MicroXRCEAgent udp4 -p 8888
```

### Gazebo Simulation
To launch the Gazebo simulation:
1. Navigate to the `PX4_Autopilot` directory.
2. Run the following command:
   ```bash
   make px4_sitl gz_x500
   # For plane
   make px4_sitl gz_advanced_plane
   ```

### QGroundControl
To start QGroundControl:
1. Navigate to the directory where QGroundControl is installed.
2. Run:
   ```bash
   ./QGroundControl.AppImage
   ```

### Running a ROS Node
To run a ROS node:
1. Navigate to the directory containing your `.py` ROS node.
2. Run your node with:
   ```bash
   python3 your_node_name.py
   ```

#### Example
If using `px4_ros_com`:
1. Go to the example directory:
   ```bash
   cd px4_ros_com/src/example/offboard_py
   ```
2. Run the offboard control script:
   ```bash
   python3 offboard_control.py
   ```

### Building and Sourcing
Whenever you add new code or make changes to the workspace, always build and source your environment:
1. Build the workspace:
   ```bash
   colcon build
   ```
2. Source the setup file:
   ```bash
   source install/setup.bash
   ```

This ensures that all changes are properly incorporated into your workspace before running any nodes.
