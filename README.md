# Table of Contents
### 1. [Introduction and Basics](#ros2-humble-basics)
### 2. [Package and Directory Management](#package-management)
### 4. [Control Turtle With Gamepad](#troubleshooting)
### 5. [Resources](#resources)
 
# ROS2 Humble Basics

This guide provides an overview of ROS2 setup and package management. For installation and detailed instructions, please refer to the official documentation:
- [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

### System Requirements

- Operating System: Ubuntu 22.04 (Jammy Jellyfish) - Recommended for ROS2 Humble
- RAM: Minimum 4GB (8GB+ recommended)
- Disk Space: At least 20GB free space

### Installation

Please follow the official ROS2 installation guide for your specific operating system:
- [Ubuntu Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install.html)
- [Windows Installation](https://docs.ros.org/en/humble/Installation/Windows-Install.html)
- [macOS Installation](https://docs.ros.org/en/humble/Installation/macOS-Install.html)

### Basic ROS2 Commands

```bash
# List all available nodes
ros2 node list

# List all topics
ros2 topic list

# List all services
ros2 service list

# Launch a ROS2 package
ros2 launch <package_name> <launch_file>

# Create a new ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

# Package Management

## Creating Packages
1. Create a workspace and navigate to the src directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Create a new package:
```bash
# For Python packages
ros2 pkg create --build-type ament_python <package_name>

# For C++ packages
ros2 pkg create --build-type ament_cmake <package_name>
```

## Package Structure
```
<package_name>/
├── CMakeLists.txt           # (C++) Build system instructions
├── package.xml              # Package metadata and dependencies
└── setup.py                 # (Python) Build system instructions
```

## Important Files
1. **package.xml**: Package metadata and dependencies
2. **CMakeLists.txt**: Build instructions (C++)
3. **setup.py**: Package installation configuration (Python)

## Building Packages
```bash
# Build all packages in workspace
cd ~/ros2_ws
colcon build --symlink-install

# Build specific package
colcon build --packages-select <package_name>
```

## Directory Structure
After building, your workspace will contain:
```
ros2_ws/
├── build/                # Build files
├── install/              # Installation files
├── log/                  # Build and test logs
└── src/                  # Source code
```
1. **build/** directory:
   - Contains intermediate build files
   - CMake cache and configuration
   - Object files and compilation products

2. **install/** directory:
   - Contains the final installed files
   - Executable files in `lib/<package_name>`
   - Python modules in `lib/python3.x/site-packages`
   - Launch files in `share/<package_name>`
   - Setup scripts for environment configuration

3. **log/** directory:
   - Build logs
   - Test results
   - Debug information

## Running Packages
```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Run a node
ros2 run <package_name> <node_name>

# Launch using a launch file
ros2 launch <package_name> <launch_file>
```
# Control Turtle with a Controller/Gamepad

Control the ROS2 turtlesim using a gamepad/joystick controller. This package provides an interface between your game controller and the turtle simulator, allowing for intuitive control of the turtle's movement.

## Prerequisites

- ROS2 Humble installed on Ubuntu 22.04
- USB gamepad/joystick (Xbox, PlayStation, or similar controllers)
- Required ROS2 packages:
  ```bash
  sudo apt-get install ros-humble-joy ros-humble-teleop-twist-joy
  ```

## Installation

1. Create a ROS2 workspace (if you don't have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone this repository:
   ```bash
   git clone git@github.com:Arnav-28/rosify.git
   cd ~/ros2_ws
   ```

3. Build the package:
   ```bash
   colcon build --packages-select rosify
   source install/setup.bash
   ```

## Package Structure

```
rosify/
├── CMakeLists.txt           # Build system instructions
├── package.xml              # Package metadata and dependencies
├── src/
│   └── turtleJoy.py  # Main controller node
├── launch/
│   └── turtleJoy.launch.py      # Launch file
└── README.md
```

## Usage

Launch all required nodes (turtlesim, joy node, and controller) with:
```bash
ros2 launch rosify turtleJoy.launch.py
```

### Controller Mapping

- Left stick vertical: Forward/backward movement
- Left stick horizontal: Rotation
- Default speeds can be adjusted in the code:
  - `linear_speed = 2.0`  # Forward/backward speed
  - `angular_speed = 2.0` # Rotation speed

## Node Architecture

The system consists of three main nodes:

1. `joy_node`: Reads raw controller input
2. `turtleJoy`: Converts joystick commands to turtle movement
3. `turtlesim_node`: The turtle simulator

Data flow:
```
USB Controller → joy_node → turtleJoy → turtlesim_node
```

## Topics

The package uses the following ROS2 topics:

- `/joy` (sensor_msgs/Joy): Raw joystick data
  - `axes[]`: Joystick axis values (-1.0 to 1.0)
  - `buttons[]`: Button states (0 or 1)
- `/turtle1/cmd_vel` (geometry_msgs/Twist): Turtle velocity commands
  - `linear`: Linear velocity (x, y, z)
  - `angular`: Angular velocity (x, y, z)

## Troubleshooting

1. **Command not found**: Source the setup file
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Build failures**: Install dependencies
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Controller not detected**:
   ```bash
   # Check available joysticks
   ls /dev/input/js*
   
   # Verify joy node output
   ros2 topic echo /joy
   ```

4. **Verify node connections**:
   ```bash
   # List all nodes
   ros2 node list
   
   # Check topic connections
   ros2 topic info /turtle1/cmd_vel
   ```

4. **Common issues**:
   - If the controller isn't responding, ensure it's properly connected and recognized by the system
   - If the turtle isn't moving, check that all nodes are running (`ros2 node list`)
   - For incorrect movements, verify axis mappings by monitoring the joy topic

## Customization

You can modify the controller behavior by adjusting these parameters in `turtleJoy.py`:

```python
self.linear_speed = 2.0    # Forward/backward speed
self.angular_speed = 2.0   # Rotation speed
self.linear_axis = 1       # Axis number for forward/backward
self.angular_axis = 0      # Axis number for rotation
```

## Troubleshooting

## Acknowledgments

- ROS2 Development Team
- Turtlesim Package Developers
- Joy Package Maintainers

## Resources
### Internet Gems:

- [Playlist for ROS2 Basics](https://docs.ros.org/en/humble/Tutorials.html)
- [Hands-on tutorial](https://design.ros2.org/)
- [TurtleBot3 Simulation Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
- [TurtleBot3 Repository](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

### Official Resources:

- [Official ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Design Documentation](https://design.ros2.org/)
- [ROS2 GitHub Repository](https://github.com/ros2)
- [ROS Answers](https://answers.ros.org/)

### Contributing:

Feel free to submit issues and enhancement requests!
