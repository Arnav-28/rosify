# ROS2 Setup and Package Guide

## Table of Contents
1. [Introduction](#introduction)
2. [Package Management](#package-management)
3. [Directory Structure](#directory-structure)
4. [Troubleshooting](#troubleshooting)
5. [Resources](#resources)

## Introduction

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

## Package Management

### Creating Packages
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

### Package Structure
```
<package_name>/
├── CMakeLists.txt           # (C++) Build system instructions
├── package.xml              # Package metadata and dependencies
└── setup.py                 # (Python) Build system instructions
```

### Important Files
1. **package.xml**: Package metadata and dependencies
2. **CMakeLists.txt**: Build instructions (C++)
3. **setup.py**: Package installation configuration (Python)

### Building Packages
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

## Troubleshooting

1. **Command not found**: Source the setup file
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Build failures**: Install dependencies
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
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