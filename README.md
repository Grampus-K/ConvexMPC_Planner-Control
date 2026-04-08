# 🎓 复杂环境下四旋翼无人机轨迹规划与跟踪控制 (Ke Yincheng 毕业设计)

本项目基于香港大学 MARS 实验室的开源框架 [IPC (Integrated Planning and Control)](https://github.com/hku-mars/IPC) 进行深度优化与二次开发。通过对底层算法的精简、反馈机制的重构以及控制接口的完善，成功将其调整为**高度稳定、可直接用于实机高频飞行的可用状态**。

---

## 🌟 核心改进与特性

### 1. 算法精简与去噪优化
* **移除冗余模块**：去除了原版工程中的油门估计（Thrust Estimation）与微分平坦映射（Differential Flatness Mapping），降低了系统复杂度。
* **加速度反馈重构（核心去噪）**：由于去除了油门估计，直接使用 IMU 原始加速度反馈会导致系统噪声极大。本项目创新性地**将反馈加速度替换为上一帧 MPC 输出的最优加速度指令**，在保证状态机闭环的同时，从根本上消除了高频震荡噪声。

### 2. 深度定制的 `px4ctrl` 飞控接口
* **前馈控制直接下发**：将 MPC 优化出的最优速度直接作为前馈（Feedforward）发送给 PX4，大幅提升了轨迹跟踪的响应速度。
* **平滑悬停逻辑（Bug Fix）**：修复了原版的急刹车 Bug。当飞手松开遥控器摇杆时，系统会先下发速度为 `0` 的指令，待无人机实际速度降至安全阈值以下后，再将当前位置设为悬停点，实现了丝滑的刹车悬停。
* **里程计驱动状态机**：`px4ctrl` 的整体状态机全面放弃了不可靠的定时器（Timer）驱动，改为完全依赖 `odom` 里程计反馈驱动，状态切换更加严格、安全。

---

## ⚠️ 重要注意事项

### 🧭 坐标系转换说明 (极其重要)
* **MPC 输出**：MPC 规划解算出的最优速度是在 **世界坐标系 (World Frame)** 下的。
* **Odom 反馈**：由 MAVROS 和 FAST-LIO2 提供给系统的里程计速度反馈均为 **机体坐标系 (Body Frame)**。
* **数据流转机制**：
  1. 在 MPC 的回调函数中，已自动将输入的机体系速度转化为世界系速度供算法规划。
  2. 最终下发给 PX4 执行的速度指令，已重新转换为 **机体坐标系**。

### 🚨 飞行安全与紧急接管
* **最高优先级打杆接管**：如果飞行中感觉轨迹异常或有炸机风险，**不需要切换任何飞行模式，直接拨动遥控器摇杆即可强行降落/接管**，打杆动作在 `px4ctrl` 中具有最高优先级！

---

## 🛠️ 安装指南

### 1. 安装 MAVROS 及其依赖
```bash
sudo apt-get install ros-noetic-mavros
sudo apt-get install ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
```

### 2. 安装 IPC 算法依赖
本项目的底层优化求解器依赖项，请参考原版 IPC 工程进行配置：
👉 [HKU-MARS IPC Github 仓库](https://github.com/hku-mars/IPC)

### 3. 安装 FAST-LIO2 与 Livox Mid-360 驱动
雷达点云与里程计环境配置，请严格参考以下教程：
👉 [FAST-LIO2 + Mid-360 配置指南](https://blog.csdn.net/m0_55117804/article/details/142644882)

### 4. 编译工程
> **💡 编译提示**：在首次编译时，**请务必先单独编译 `fast_lio` 相关的包**，否则可能会因依赖顺序问题报错。

```bash
cd ~/your_workspace
catkin_make
```

---

## 🚀 运行与飞行指南

1. **环境初始化**  
   运行脚本启动基础环境：
   ```bash
   sh shflies/ready.sh
   ```
2. **启动控制器**  
   等待终端输出 `校准完成！参考位置已设置为初始位姿` 类似字样后，启动自定义控制器：
   ```bash
   roslaunch px4ctrl run_ctrl.launch
   ```
3. **启动 IPC 规划器**  
   ```bash
   roslaunch ipc ipc_real.launch
   ```
4. **切入 Offboard 模式**  
   在遥控器上拨动开关进入 `Offboard` 模式。此时观察 `px4ctrl` 的终端窗口，确认出现 `switch to POSTL mode` 的提示。
5. **一键起飞逻辑**  
   * 遥控器打杆解锁（内八或直接拨杆解锁）。
   * **起飞操作**：直接将油门杆推满，然后立刻回中。
   * *（说明：本项目重写了起飞状态机，只要检测到油门高于设定阈值，无人机会无视打杆幅度，自动平稳起飞并悬停至 `1.2m` 高度）。*
6. **自主导航飞行**  
   待无人机稳定悬停后，即可在 `RViz` 界面中使用 `3D Nav Goal` 工具直接打点，无人机将自主规划轨迹并飞往目标点。

---

## 🔧 实用工具与命令

**提高 MAVROS 里程计频率**  
为了让 MPC 规划器获得更高频的状态反馈，建议在起飞前运行以下命令，强制提升 PX4 的里程计回传频率：
```bash
rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0
```
