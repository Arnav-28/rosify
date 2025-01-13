# ROS2 Installation and Setup Guide

This guide provides instructions for installing and setting up ROS2 (Robot Operating System 2) on Ubuntu. For the most up-to-date and detailed information, please refer to the [official ROS2 documentation](https://docs.ros.org/en/humble/Installation.html).

## System Requirements

- Operating System: Ubuntu 22.04 (Jammy Jellyfish) - Recommended for ROS2 Humble
- RAM: Minimum 4GB (8GB+ recommended)
- Disk Space: At least 20GB free space

## Installation Steps

### 1. Set Locale

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Setup Sources

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS2 Packages

```bash
sudo apt update
sudo apt upgrade

# Install ROS2 Desktop (full installation with GUI tools)
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

### 4. Environment Setup

Add the following to your `~/.bashrc` file:

```bash
source /opt/ros/humble/setup.bash
```

Then reload the terminal or run:

```bash
source ~/.bashrc
```

## Verifying Installation

### 1. Test Publisher/Subscriber

Open two terminal windows:

Terminal 1 (Publisher):
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

Terminal 2 (Subscriber):
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages being published and received between the nodes.

## Basic ROS2 Commands

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

## Troubleshooting

Common issues and solutions:

1. **Command not found**: Ensure you've sourced the setup file:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Permission issues**: Use `sudo` for installation commands

3. **Build failures**: Make sure all dependencies are installed:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Additional Resources

- [Official ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Design Documentation](https://design.ros2.org/)
- [ROS2 GitHub Repository](https://github.com/ros2)
- [ROS Answers](https://answers.ros.org/)

## Contributing

Feel free to submit issues and enhancement requests!

## License

This project is licensed under the Apache License 2.0 - see the [ROS2 License](https://github.com/ros2/ros2/blob/master/LICENSE) for details.

