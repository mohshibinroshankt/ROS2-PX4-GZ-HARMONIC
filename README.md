# ROS2-PX4-GZ-HARMONIC

### To setup ROS 2 for use with PX4:

- Install PX4 (to use the PX4 simulator)
  ``` cd
      git clone https://github.com/PX4/PX4-Autopilot.git --recursive
      bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
      cd PX4-Autopilot/
      make px4_sitl ```
- Install ROS 2
- Setup Micro XRCE-DDS Agent & Client
- Build & Run ROS 2 Workspace
