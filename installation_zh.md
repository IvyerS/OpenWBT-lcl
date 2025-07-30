<div align="center">
  <h1 align="center"> OpenWBT 硬件清单与环境配置 </h1>
  <h3 align="center"> 银河通用机器人 · 清华大学 </h3>

[中文](installation_zh.md) | [英文](installation.md)

</div>


# 硬件清单

| 设备名称             | 说明        | 官方链接                                        |
| --------------------- | -------------- | ---------------------------------------------------------------- |
| G1-EDU           | 宇树机器人开发平台 | [G1-EDU 开发者支持](https://support.unitree.com/home/zh/G1_developer) |
| Apple Vision Pro | 苹果混合现实设备  | [https://www.apple.com/apple-vision-pro/](https://www.apple.com/apple-vision-pro/) |
| 手柄               | 用于控制设备    | 请联系 openwbt@galbot.com                                                           |
| 用户电脑              | 推荐使用ubuntu 20.04   | - |

# 环境配置

## Conda 环境配置

### 克隆本代码
```bash
git clone https://github.com/GalaxyGeneralRobotics/OpenWBT.git
```
下面默认工作路径为`~/OpenWBT`，请根据实际情况修改。

### 创建并激活 Conda 环境

```bash
conda create -n OpenWBT python=3.8
conda activate OpenWBT
```

### 安装核心依赖

```bash
# 安装pinocchio，版本需为3.1.0
conda install pinocchio -c conda-forge
pip install meshcat
pip install casadi
pip install onnxruntime
pip install pyserial
pip install mujoco
```

其余依赖可参考[https://github.com/unitreerobotics/avp\_teleoperate/tree/main](https://github.com/unitreerobotics/avp_teleoperate/tree/main)中的`requirements.txt`文件来配置：
```bash
pip install -r requirements.txt
```

### unitree_sdk2_python 安装

确保处于`OpenWBT` Conda 环境，执行以下命令：

```bash
# 克隆SDK仓库
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
# 进入仓库目录
cd unitree_sdk2_python
# 安装SDK
pip install -e .
```

### Isaac Gym 安装

在 [Isaac Gym 下载页面](https://developer.nvidia.com/isaac-gym/download)下载 Isaac Gym 安装包并解压，然后到`IsaacGym_Preview_4_Package/isaacgym/python`目录下执行下面命令：
```
pip install -e .
```


## ROS 2 环境配置
以下安裝步驟參考[宇树G1 ROS2通信](https://support.unitree.com/home/zh/G1_developer/ros2_communication_routine)

### 安装 CMake 编译工具以及GCC 9.4.0

```bash
# 建议安装CMake 3.23.3（从官网下载源码编译）编译安装步骤参考官方文档
wget https://cmake.org/files/v3.23/cmake-3.23.3.tar.gz

tar -zxvf cmake-3.23.3.tar.gz
cd cmake-3.23.3
sudo apt-get -y install libssl-dev
sudo ./configure
sudo make -j8
sudo make install

cmake --version # 验证是否安装成功：

# 安装GCC 9.4.0
sudo apt-get install gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
```

### 安装 ROS 2 Foxy
下文以 ROS2 foxy 为例，如需使用其他版本的 ROS2，在相应的地方替换 foxy 为当前的 ROS2 版本名称即可。

ROS2 foxy 的安装可参考: [https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

**注意**：安装完成后注释掉`.bashrc`中的`source /opt/ros/foxy/setup.bash`以防冲突。


### 安装并编译 unitree_ros2
- 安装：
```bash
# 克隆仓库
git clone https://github.com/unitreerobotics/unitree_ros2
# 安装依赖
pip install empy==3.3.4 catkin\_pkg==1.0.0 lark==1.2.2
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl # 在\~/.bashrc里注释掉"source /opt/ros/foxy/setup.bash"
```

- 编译 cyclonedds（确保未 source ROS 2 环境）

由于 G1 使用的是 cyclonedds 0.10.2 版本，因此需要先更改 ROS2 的 DDS 实现。详见：[https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html)

编译 cyclonedds 前确保在启动终端时没有 source ros2 相关的环境变量，否则会导致 cyclonedds 编译报错，可打开`~/.bashrc`文件检查是否有 "source /opt/ros/foxy/setup.bash" 存在则将其删除或注释。

- 编译:
```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```

- 启用 ROS 2 环境并编译unitree_ros2项目
```bash
source /opt/ros/foxy/setup.bash
colcon build
```

### 网络配置与验证
1. 连接机器人，通过`ifconfig`查看网口名称和ip，如：名称 eno1，ip 192.168.123.161
2. 配置网络使得与机器人在同一子网下，如：
   - IP: `192.168.123.222`
   - Netmask: `255.255.255.0`
   - Gateway: `192.168.123.1`

3. 修改网口配置：`~/unitree_ros2/setup.sh` 中 `NetworkInterface name="eno1"` 改为正确的网口名称
4. 切换 ROS 2 环境：
```bash
source /opt/ros/foxy/setup.sh; source ~/unitree_ros2/setup.sh
```
5. 执行`ros2 topic list`显示机器人相关话题，出现则配置成功



## Apple Vision Pro 通信配置

苹果不允许在非 HTTPS 连接上使用 WebXR。要在本地测试应用程序，需要利用`mkcert`创建一个自签名证书并在客户端上安装它。

### 使用 brew 安装 mkcert

- 安装 brew

```bash
sudo apt-get install build-essential procps curl file git
cd \~
/bin/bash -c "\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

- 安装 mkcert
```bash
brew install mkcert
```

### 生成并配置证书

- 查看本机 IP 地址
```bash
ifconfig | grep inet
```

- 创建证书

假设本机 IP 地址是 192.168.123.2:

```bash
mkcert -install && mkcert -cert-file cert.pem -key-file key.pem 192.168.123.2 localhost 127.0.0.1 # 注意替换为你的实际 IP 地址
```
- 把生成的 cert.pem 和 key.pem 两个文件拷贝到项目的 teleop 目录下
```bash
cp cert.pem key.pem ~/OpenWBT/
```

- 设置系统防火墙
开放 TCP 8012 端口
```bash
sudo ufw allow 8012
```

### 在 Apple Vision Pro 中安装证书

执行`mkcert -CAROOT`命令可以获取到`rootCA.pem`的路径。再通过苹果的隔空投送功能把`rootCA.pem`文件发送到 Apple Vision Pro进行安装。

**注意**：Apple Vision Pro 接收文件前需要启动文件接收功能，方法是：设置 >> 通用 >> 隔空投送 >> 所有人 (10 分钟)。


### 启用 Apple Vision Pro 的 WebXR 相关功能

启用路径为：设置 >> Apps >> Safari 浏览器 >> 高级 >> 功能标志 >> 启用 WebXR 相关功能。


### 进入 VR 由实时画面进行遥操

**前提条件**：用户电脑端已经执行遥操作程序并且机器人已开启图像服务

打开 Apple Vision Pro 的浏览器输入:
```https://192.168.123.2:8012?ws=wss://192.168.123.2:8012```
点击 "Enter VR" 并且允许跟踪，即可看到画面。
*注意替换为你的实际 IP 地址。*