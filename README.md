# SenseSimulation

高颢嘉

# 注意事项

本代码为 SenseLabRobo 仿真环境，用于在没有实车的环境下测试上层代码。本代码只适合部署在个人开发环境中，请勿将此代码部署在实车中。

# 安装驱动

```bash
cd ~/
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

# 下载与配置

```bash
cd ~/
git clone --recursive https://github.com/gaohaojia/SenseSimulation
cd SenseSimulation
sudo apt update
sudo apt install ros-humble-desktop-full python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

# 编译

目前只测试过在 x86 架构上编译本代码。

```bash
bash ~/SenseSimulation/toBuild.sh
```

# 启动

将 [robot_count] 替换成要仿真的机器人数量（当前支持最大为 5）。

```bash
bash ~/SenseSimulation/run.sh [robot_count]
```