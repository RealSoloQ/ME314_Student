# ME314 XArm Control Package
### Overview
The ME314_XArm package provides Python-based control and teleoperation functionalities for the xArm7 robotic arm, integrating with ROS2 for Stanford University's ME 314: Robotic Dexterity taught by Dr. Monroe Kennedy III.

### Caution
Please stay away from the robot arm during operation to avoid personal injury or equipment damage.
Ensure e-stop is close and accessible before controlling arm.

### Installation (if using native Linux Ubuntu 22.04 System)

#### Install ros2 humble (for Ubuntu 22.04)
Follow instructions for ros2 humble (desktop) install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html or copy and paste the below commands in terminal:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

#### Install Gazebo

```bash
sudo apt install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
```

#### Install Realsense2 SDK and ROS2 Wrapper

a. Install librealsense (source: https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu step #2 option #2)

```bash
# Configure Ubuntu Repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# Install librealsense2 debian package
sudo apt install ros-humble-librealsense2*
```

b. Install RealSense Wrapper (source: https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu)

```bash
# Assuming Ubuntu Repositories are already configured from previous step, install realsense2 wrapper debian package
sudo apt install ros-humble-realsense2-*
```

#### Install Moveit2

```bash
sudo apt install ros-humble-moveit
```

#### Create xarm_ros2_ws and Clone Custom ME314 Package

```bash
cd ~
mkdir -p xarm_ros2_ws/src
cd ~/xarm_ros2_ws/src
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
git clone https://github.com/RealSoloQ/ME314_XArm.git
```

#### Build Workspace

```bash
cd ~/xarm_ros2_ws
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Commands
#### Navigate to Workspace and Source Install Before Running Any Commands

```bash
cd ~/xarm_ros2_ws
source install/setup.bash
```

#### Tele-Operation with Spacemouse

```bash
ros2 run me314 xarm_spacemouse_ros2.py
```

#### Option #1: Control XArm using XArm Planner (with MoveIt API) (RECOMMENDED)

1. Control in Gazebo

a. In one terminal run the following command:

```bash
ros2 launch me314 me314_xarm_gazebo.launch.py
```

2. Control in Real

a. In one terminal run the xarm planner launch command:

```bash
ros2 launch me314 me314_xarm_real.launch.py
```

b. In another terminal run script (example):

```bash
ros2 run me314 xarm_a2b_example.py
```

c. In another terminal run 

#### Option #2: Control XArm in Gazebo (using pymoveit2 python binding)

1. Launch gazebo with moveit configuration

```bash
ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py add_gripper:=true
```

2. Run example pymoveit2 script

```bash
ros2 run me314 move_A_to_B_pymoveit.py --ros-args -p dof:=7 -p robot_type:=xarm
```

#### Control XArm in Real (using XArm API)

```bash
ros2 run me314 move_A_to_B.py
```

### Using Docker Image (for Windows and Mac users)
1. Install Docker Desktop (or Docker Engine for CLI only)
2. If using terminal: docker pull
3. If using Docker Desktop
