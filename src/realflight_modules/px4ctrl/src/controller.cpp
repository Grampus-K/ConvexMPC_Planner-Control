#include "controller.h"

using namespace std;
using namespace uav_utils;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

double LinearControl::wrapPi(double angle_rad)
{
  // 使用 fmod 实现稳定的包络：结果范围 (-pi, pi]
  static constexpr double kPi    = 3.14159265358979323846;
  static constexpr double kTwoPi = 6.28318530717958647692;

  double x = std::fmod(angle_rad + kPi, kTwoPi);
  if (x < 0) x += kTwoPi;
  return x - kPi;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{

}

quadrotor_msgs::Px4ctrlDebug 
LinearControl::velocityControl(const Desired_State_t &des,
    const Odom_Data_t &odom, Controller_Output_t &u, const uint8_t state)
{
    Eigen::Vector3d Kp, Kv, Kff;
    double Kyaw = 0.0;

    switch (state) {
      case 1: Kp.setZero()      ; Kv.setZero(); Kff.setZero(); Kyaw = 0.0; break;            // READY_TO_FLY
      case 2: Kp = {1.0,1.0,1.0}; Kv.setZero(); Kff.setZero(); Kyaw = 1.0; break;            // AUTO_HOVER
      case 3: Kp = {0.0,0.0,0.0}; Kv.setZero(); Kff.setOnes(); Kyaw = 1.0; break;            // CMD_CTRL
      case 4: Kp.setZero()      ; Kv.setZero(); Kff.setOnes(); Kyaw = 0.0; break;            // RC_CTRL
      case 5: Kp = {0.2,0.2,0.0}; Kv.setZero(); Kff = {0,0, 1}; Kyaw = 1.0; break;           // AUTO_TAKEOFF
      case 6: Kp = {0.2,0.2,0.0}; Kv.setZero(); Kff = {0,0,-1}; Kyaw = 1.0; break;           // AUTO_LAND
      default: Kp.setZero(); Kv.setZero(); Kff.setZero(); Kyaw = 0.0; break;
    }

    double yaw_odom = get_yaw_from_quaternion(odom.q);

    // 【优化核心】：利用 Eigen 自动构建 World 到 Body 坐标系的旋转矩阵
    // 旋转 -yaw 角度，即可将世界系向量转入机体系
    Eigen::Matrix3d R_world_to_body = Eigen::AngleAxisd(-yaw_odom, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d feedforward;
    Eigen::Vector3d error_p_body;

    // 【逻辑梳理】：把 state == 4 (纯遥控模式) 单独抽离，逻辑更清晰
    if (state == 4) 
    {
        feedforward = des.v;           // RC 发来的直接就是机体系速度
        error_p_body.setZero();        // 遥控模式下不闭环位置
        u.yaw_rate = des.yaw_rate;
    } 
    else 
    {
        // 1. 将世界系的期望速度转为机体系速度
        feedforward = R_world_to_body * des.v;

        // 2. 将世界系的位置误差转为机体系的位置误差
        Eigen::Vector3d error_p_world = des.p - odom.p;
        error_p_body = R_world_to_body * error_p_world;

        // 3. 计算 Yaw_rate
        u.yaw_rate = Kyaw * wrapPi(des.yaw - yaw_odom);
    }

    // 计算最终发给 PX4 的 机体系期望速度 (前馈 + 位置P控制)
    u.velocity = Kff.asDiagonal() * feedforward + Kp.asDiagonal() * error_p_body;

    // ----- Debug 赋值区 -----
    // 把机体系的期望速度重新转回世界系，仅为了发给 Rviz 或 PlotJuggler 看着直观
    // Eigen::Vector3d vel_world = Eigen::AngleAxisd(yaw_odom, Eigen::Vector3d::UnitZ()) * u.velocity;
    // debug_msg_.des_v_x = vel_world.x();
    // debug_msg_.des_v_y = vel_world.y();
    // debug_msg_.des_v_z = vel_world.z();

    //机体系
    debug_msg_.des_v_x = u.velocity.x();
    debug_msg_.des_v_y = u.velocity.y();
    debug_msg_.des_v_z = u.velocity.z();

    debug_msg_.des_a_x = des.p(0);
    debug_msg_.des_a_y = des.p(1);
    debug_msg_.des_a_z = des.p(2);

    return debug_msg_;
}
