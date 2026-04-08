🎓 复杂环境下四旋翼无人机轨迹规划与跟踪控制

Ke Yincheng 毕业设计

本项目基于香港大学 MARS 实验室开源框架 IPC（Integrated Planning and Control） 进行深度优化与二次开发。

通过对底层算法精简、反馈机制重构以及控制接口优化，使系统达到高稳定性、高频率、可实机部署飞行的工程级可用状态。

🌟 核心改进与特性
1. 算法精简与去噪优化
移除冗余模块
删除油门估计（Thrust Estimation）
删除微分平坦映射（Differential Flatness Mapping）
👉 降低系统复杂度，提高鲁棒性
加速度反馈重构（核心改进）
原问题：直接使用 IMU 加速度 → 噪声极大
本项目方案：
使用上一帧 MPC 输出的最优加速度作为反馈
效果：
消除高频震荡
保持闭环一致性
显著提升控制稳定性
2. 深度定制的 px4ctrl 飞控接口
前馈控制优化
MPC 输出的最优速度 → 直接作为 Feedforward 下发 PX4
👉 提升轨迹跟踪响应速度
平滑悬停逻辑（Bug Fix）
修复原版“急刹车”问题
新逻辑：
松杆 → 先发送速度为 0
实际速度下降到阈值以下
当前位置设为悬停点
👉 实现丝滑刹车悬停
里程计驱动状态机（重要改进）
移除 Timer 驱动
全部基于 Odom 反馈驱动状态机
👉 状态切换更严格、安全
⚠️ 重要注意事项
🧭 坐标系转换说明（极其重要）
MPC 输出
世界坐标系（World Frame）
Odom 反馈
机体坐标系（Body Frame）
数据流机制
输入：Body → World（用于 MPC 计算）
输出：World → Body（发送给 PX4）
🚨 飞行安全与紧急接管
最高优先级遥控接管
无需切换模式
直接打杆即可强制接管 / 降落
👉 在 px4ctrl 中具有最高优先级
🛠️ 安装指南
1. 安装 MAVROS
sudo apt-get install ros-noetic-mavros
sudo apt-get install ros-noetic-mavros-extras

cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
2. 安装 IPC 依赖

👉 原项目仓库：
https://github.com/hku-mars/IPC

3. 安装 FAST-LIO2 与 Mid-360 驱动

👉 配置教程：
https://blog.csdn.net/m0_55117804/article/details/142644882

4. 编译工程

💡 重要提示：首次编译需先编译 fast_lio 相关包

cd ~/your_workspace
catkin_make
🚀 运行与飞行指南
1. 环境初始化
sh shflies/ready.sh
2. 启动控制器

等待终端出现类似 “校准完成” 提示：

roslaunch px4ctrl run_ctrl.launch
3. 启动 IPC 规划器
roslaunch ipc ipc_real.launch
4. 切入 Offboard 模式
使用遥控器切换到 Offboard
终端确认：
✅ “已切换到位置模式”
5. 一键起飞逻辑

操作步骤：

解锁（内八 / 拨杆）
油门推满 → 立即回中

说明：

检测到油门超过阈值后：
自动起飞
悬停至 1.2 m
6. 自主导航飞行
打开 RViz
使用 2D Nav Goal 打点
无人机将：
自动规划轨迹
飞向目标点
🔧 实用工具
提高 MAVROS 里程计频率
rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0

👉 作用：

提升 PX4 → MAVROS 回传频率
提高 MPC 状态反馈质量

💡 本项目 sh 脚本已内置该命令
