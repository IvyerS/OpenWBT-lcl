<div align="center">
  <h1 align="center"> OpenWBT Hardware List and Environment Configuration</h1>
  <h3 align="center"> GALBOT · Tsinghua </h3>

[中文](installation_zh.md) |[英文](installation.md)

</div>


# Hardware List

| Device Name      | Description                        | Official Link                                                                |
| ---------------- | ---------------------------------- | ---------------------------------------------------------------------------- |
| G1-EDU           | Unitree Robot Development Platform | [G1-EDU Developer Support](https://support.unitree.com/home/zh/G1_developer) |
| Apple Vision Pro | Apple Mixed Reality Device         | [https://www.apple.com/apple-vision-pro/](https://www.apple.com/apple-vision-pro/)   |
| Joystick         | For Device Control                 | Please contact openwbt@galbot.com                                                                            |
| User Computer    | Ubuntu 20.04 Recommended           | -                                                                           |
# Environment Configuration
## Conda Environment Configuration

### Clone the Repository
```bash
git clone https://github.com/GalaxyGeneralRobotics/OpenWBT.git
```
The default working directory is `~/OpenWBT`. Please modify it according to your actual setup.

### Create and Activate Conda Environment

```bash
conda create -n OpenWBT python=3.8
conda activate OpenWBT
```

### Install Core Dependencies

```bash
# Install pinocchio, version must be 3.1.0
conda install pinocchio -c conda-forge
pip install meshcat
pip install casadi
pip install onnxruntime
pip install pyserial
pip install mujoco
```

For the remaining dependencies, refer to the requirements.txt file in https://github.com/unitreerobotics/avp_teleoperate/tree/main to configure:
```bash
pip install -r requirements.txt
```

### unitree_sdk2_python Installation
Ensure you are in the `OpenWBT` Conda environment and execute the following commands:

```bash
# Clone the SDK repository
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
# Enter the repository directory
cd unitree_sdk2_python
# Install the SDK
pip install -e .
```

### Isaac Gym Installation
Download the Isaac Gym installation package from the [Isaac Gym download page]: (https://developer.nvidia.com/isaac-gym/download)and extract it. Then navigate to the `IsaacGym_Preview_4_Package/isaacgym/python` directory and execute the following command:

```bash
pip install -e .
```

## ROS 2 Environment Configuration
Installation steps refer to [Unitree G1 ROS2 Communication](https://support.unitree.com/home/en/G1_developer/ros2_communication_routine)

### Install CMake Build Tools and GCC 9.4.0

```bash
# Install CMake 3.23.3 (compile from source downloaded from the official website). Refer to the official documentation for compilation steps.
wget https://cmake.org/files/v3.23/cmake-3.23.3.tar.gz

tar -zxvf cmake-3.23.3.tar.gz
cd cmake-3.23.3
sudo apt-get -y install libssl-dev
sudo ./configure
sudo make -j8
sudo make install

cmake --version # Verify successful installation:

# Install GCC 9.4.0
sudo apt-get install gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
```

### Install ROS 2 Foxy
The following uses ROS2 Foxy as an example. If you are using another version of ROS2, simply replace `foxy` with the name of your current ROS2 version wherever applicable.

For ROS2 Foxy installation, please refer to:  
[https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/fo)

**Note**: After installation, it is recommended to comment out `source /opt/ros/foxy/setup.bash` in `.bashrc` to prevent conflicts.

### Install and Compile unitree_ros2

- Installation:
```bash
# Clone the repository
git clone https://github.com/unitreerobotics/unitree_ros2
# Install dependencies
pip install empy==3.3.4 catkin_pkg==1.0.0 lark==1.2.2
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl # Comment out "source /opt/ros/foxy/setup.bash" in ~/.bashrc
```

- Compile cyclonedds (ensure ROS 2 environment is not sourced)
Since the G1 uses CycloneDDS version 0.10.2, it is necessary to change the default DDS implementation in ROS2. For more details, refer to:  
[https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html)

Before compiling CycloneDDS, make sure no ROS2 environment variables are sourced when launching the terminal. Otherwise, CycloneDDS may fail to compile.  
You can check your `~/.bashrc` file for any lines such as `source /opt/ros/foxy/setup.bash` and comment them out or remove them if present.

- Compile:
```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```
- Enable the ROS 2 environment and build the `unitree_ros2` project
```bash
source /opt/ros/foxy/setup.bash
colcon build
```

### Network Configuration and Verification

1. Connect the robot, and check your network interface name and IP using `ifconfig`, e.g., name `eno1`, IP `192.168.123.161`.

2. Configure your host network to be in the same subnet as the robot, e.g.:
   - IP: `192.168.123.222`
   - Netmask: `255.255.255.0`
   - Gateway: `192.168.123.1`

3. Modify the network interface configuration: In `~/unitree_ros2/setup.sh`, change `NetworkInterface name="eno1"` to match your actual interface name.

4. Switch to the ROS 2 environment:
```bash
source /opt/ros/foxy/setup.sh; source ~/unitree_ros2/setup.sh
```
5. Run `ros2 topic list` to verify the connection. If robot-related topics appear, the configuration is successful.

## Apple Vision Pro Communication Configuration
Apple does not allow WebXR to be used over non-HTTPS connections. To test the application locally, you need to use mkcert to create a self-signed certificate and install it on the client.

### Install mkcert with brew

Install brew

```bash
sudo apt-get install build-essential procps curl file git
cd \~
/bin/bash -c "\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Install mkcert

```bash
brew install mkcert
```

### Generate and Configure Certificates

- Check local IP address

```bash
ifconfig | grep inet
```

- Create certificates

Assuming the local IP address is 192.168.123.2:

```bash
mkcert -install && mkcert -cert-file cert.pem -key-file key.pem 192.168.123.2 localhost 127.0.0.1 # Replace with your actual wifi IP address
```

- Copy the generated cert.pem and key.pem files to the teleop directory of the project

```bash
cp cert.pem key.pem \~/avp\_teleoperate/teleop/
```

- Configure system firewall
Open TCP port 8012

```bash
sudo ufw allow 8012
```

### Install Certificate on Apple Vision Pro
Execute the `mkcert -CAROOT` command to get the path of `rootCA.pem`. Then use Apple's AirDrop function to send the `rootCA.pem` file to Apple Vision Pro for installation.

**Note**: Apple Vision Pro needs to enable file reception before receiving files. Method: Settings >> General >> AirDrop >> Everyone (10 minutes).

### Enable WebXR-Related Features on Apple Vision Pro
Enable path: Settings >> Apps >> Safari Browser >> Advanced >> Feature Flags >> Enable WebXR-related features.

### Entering VR for Teleoperation with Real-Time Video
**Prerequisites**: The teleoperation program has been executed on the user's computer and the robot's image service is enabled.

Open the browser on Apple Vision Pro and enter:
```https://192.168.123.2:8012?ws=wss://192.168.123.2:8012```
Click "Enter VR" and allow tracking to view the camera feed.
*Note: Replace with your actual IP address.*

