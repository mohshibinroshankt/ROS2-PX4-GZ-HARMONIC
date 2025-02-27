# ROS2-PX4-GZ-HARMONIC

### To setup ROS 2 for use with PX4:

- Install PX4 (to use the PX4 simulator)
```bash
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
- Install ROS 2
  ```bash
      sudo apt update && sudo apt install locales
      sudo locale-gen en_US en_US.UTF-8
      sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
      export LANG=en_US.UTF-8
      sudo apt install software-properties-common
      sudo add-apt-repository universe
      sudo apt update && sudo apt install curl -y
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >       /dev/null
      sudo apt update && sudo apt upgrade -y
      sudo apt install ros-humble-desktop
      sudo apt install ros-dev-tools
      source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
  
  ```
- Setup Micro XRCE-DDS Agent & Client
  ```bash
      git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
      cd Micro-XRCE-DDS-Agent
      mkdir build
      cd build
      cmake ..
      make
      sudo make install
      sudo ldconfig /usr/local/lib/
  
  ```

- Build & Run ROS 2 Workspace

This is a rough structure, refer **https://docs.px4.io/main/en/ros2/user_guide.html** for the actual PX-ROS2 User guide


- Adding Custom world 
1. After proper installation and making the PX4-Autopilot, go to "/home/shibin/PX4-Autopilot/Tools/simulation/gz/worlds"
   Add the custom world to this, make sure the world name in the world file is same as the file name.
2. Then add the world name to /home/shibin/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeList.txt
3. Then build px4 again using ""make px4_sitl_gz_x500""

Build & Run ROS 2 Workspace
# PX4 ROS 2 Workspace Setup

This section provides instructions on how to create and build a ROS 2 workspace for PX4 integration using `px4_ros_com`(an example repo) and `px4_msgs` packages.

## Prerequisites
Ensure you have installed:
- ROS 2 Humble
- PX4 Firmware
- Colcon build tools
- Git

## Creating the Workspace

1. Open a terminal and create a new workspace directory:
   ```bash
   mkdir -p ~/ws_sensor_combined/src/
   cd ~/ws_sensor_combined/src/
   ```

2. Clone the required repositories:
   ```bash
   git clone https://github.com/PX4/px4_msgs.git
   git clone https://github.com/PX4/px4_ros_com.git
   ```

## Building the Workspace

1. Navigate to the workspace root:
   ```bash
   cd ~/ws_sensor_combined/
   ```

2. Source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace using Colcon:
   ```bash
   colcon build
   ```

## Running the Example

1. Open a new terminal and navigate to the workspace:
   ```bash
   cd ~/ws_sensor_combined/
   ```

2. Source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Source the workspace setup file:
   ```bash
   source install/local_setup.bash
   ```

4. Launch the example node:
   ```bash
   ros2 launch px4_ros_com sensor_combined_listener.launch.py
   ```
OR

5. Running an example offboard_control node
```bash
   ros2 run px4_ros_com offboard_control.py 
   ```
**Also, before running this node, make sure that**
1. **The gz sim with drone mode is spawned.**
2. **Communication is established with MicroXRCEDDS running in another terminal.**
3. **And also qground control in installed and runnning.**

# Installing QGroundControl

QGroundControl is the ground control software used to interface with PX4-based drones. Follow the steps below to install and run it on your system.

## Prerequisites

Before installing QGroundControl, run the following commands to configure necessary system permissions and dependencies:

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```

After running these commands, **log out and log back in** to apply the user permission changes.

## Installing and Running QGroundControl

1. **Download** `QGroundControl.AppImage` from the [official QGroundControl releases page](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).
2. **Make the file executable**:
   ```bash
   chmod +x ./QGroundControl.AppImage
   ```
3. **Run QGroundControl**:
   ```bash
   ./QGroundControl.AppImage
   ```
   Alternatively, you can double-click the AppImage file to launch it.

## Notes
- Ensure that your system meets the required dependencies for smooth operation.
- If you experience display issues, try running QGroundControl with `--disable-gpu` as an argument.
- Use a stable release version for best compatibility with PX4.

For more details, visit the [QGroundControl Documentation](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu).


## Notes
- Ensure the PX4 firmware and `px4_msgs` package have matching message definitions.
- If the message definitions differ, you may need to run a message translation node.
  ie, If you get error similar to
```bash
[RTPS_READER_HISTORY Error] Change payload size of '220' bytes is larger than the history payload size of '219' bytes and cannot be resized. -> Function can_change_be_added_nts
```
this means that the message defenition differ in PX4-Autopilot/msgs and px4_msgs/msgs(inside our workspace)
You can find which message is differ by running the [PX4-ros2-interface-lib](https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html) .

To manually verify that two local versions of PX4 and px4_msgs have matching message sets, you can use the following script:

```sh
./scripts/check-message-compatibility.py -v path/to/px4_msgs/ path/to/PX4-Autopilot/
```
- Follow ROS 2 best practices by using separate terminals for building and running nodes.

## License
This section follows the PX4 open-source license. Refer to the original repositories for licensing details.

For more details, visit the [PX4 ROS 2 Integration Guide](https://docs.px4.io/main/en/ros2/user_guide.html#build-ros-2-workspace).



   

### work in progress
