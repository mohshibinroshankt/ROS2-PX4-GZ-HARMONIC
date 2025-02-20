# ROS2-PX4-GZ-HARMONIC

### To setup ROS 2 for use with PX4:

- Install PX4 (to use the PX4 simulator)
```
      cd
      git clone https://github.com/PX4/PX4-Autopilot.git --recursive
      bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
      cd PX4-Autopilot/
      make px4_sitl
```
- Install ROS 2
  ```
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
  ```
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
3. Then build px4 again using ""make px4_sit"'

Build & Run ROS 2 Workspace
1. In this created a new node for path planning(to a fixed target)
   

###work in progress
