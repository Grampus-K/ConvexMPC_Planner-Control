#include "planner.h"
#include "../include/polytope/emvp.hpp"

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

void PlannerClass::TimerCallback(const ros::TimerEvent &)
{
    if (!has_odom_flag_) return;

    // 检查是否到达目标点
    if (has_active_goal_) {
        double dist_to_goal = (odom_p_ - goal_p_).norm();
        if (dist_to_goal < goal_tolerance_) { 
            has_active_goal_ = false; // 到达目标点，关闭标志位
            ROS_INFO("\033[1;32m[Planner]: Goal reached! Stop sending cmd.\033[0m");
        }
    }

    // 如果当前没有激活的目标点，则不执行规划计算也不发送cmd
    if (!has_active_goal_) {
        return; 
    }

    timer_mutex_.lock();
    
    ros::Time t_start = ros::Time::now();
    ros::Time t_0 = odom_time_;

    if (new_goal_flag_) {
        new_goal_flag_ = false;
        replan_flag_ = false;
        PathReplan(true);
    }
    if (new_goal_flag_ == false && replan_flag_) {
        replan_flag_ = false;
        PathReplan(false);
    }
    log_times_[1] = (ros::Time::now() - t_start).toSec() * 1000.0;

    ros::Time sfc_start = ros::Time::now();
    if (follow_path_.size() > 0) { // follow A* path
        double min_dis = 10000.0;
        for (int i = 0; i < follow_path_.size(); i++) { // find the nearest point
            double dis = (odom_p_ - follow_path_[i]).norm();
            if (dis < min_dis) {
                min_dis = dis;
                astar_index_ = i;
            }
        }

        int goal_in_sfc = astar_index_;
        if (follow_path_.size() - astar_index_ <= ref_dis_) { // near the end, find the sfc at current point
            goal_in_sfc = follow_path_.size();
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes;

            // 修改为：用静止的最后一个路径点（或者 goal_p_）来生成终点盒子
            Eigen::Vector3d final_p = follow_path_.back();
            GenerateAPolytope(final_p, final_p, planes, 0); 

            // GenerateAPolytope(odom_p_, odom_p_, planes, 0);
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetFSC(planes, i);
            }
        } else { // find the biggest and the longest sfc
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes, last_planes;
            GenerateAPolytope(follow_path_[astar_index_], follow_path_[astar_index_], planes, 0); 
            last_planes = planes;
            int init_num = 0;
            if (mpc_->IsInFSC(odom_p_, planes) == false) { // if odom is not in initial sfc
                init_num = (odom_p_ - follow_path_[astar_index_]).norm() / path_dis_ / ref_dis_;
                ROS_INFO("\033[35m UAV is out sfc! init num is %d \033[0m", init_num);
            }
            int first_id = astar_index_, mpc_goal_index = 0;
            for (int i = first_id+1; i < follow_path_.size(); i++) { // find last pos in first sfc
                if (mpc_->IsInFSC(follow_path_[i], planes)) {
                    first_id = i;
                    goal_in_sfc = i;
                } else {
                    first_id -= 0.2 / path_dis_;
                    if (first_id < astar_index_) first_id = astar_index_;
                    break;
                }
            }
            mpc_goal_index = (first_id - astar_index_) / ref_dis_ + init_num + 1;
            assert(first_id >= astar_index_);
            for (int i = init_num; i <= mpc_goal_index && i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetFSC(planes, i);
            }
            int end_index = first_id + (mpc_->MPC_HORIZON-mpc_goal_index) * ref_dis_;
            if (end_index >= follow_path_.size()) end_index = follow_path_.size() - 1;
            for (int i = end_index; i >= first_id && mpc_goal_index < mpc_->MPC_HORIZON - 1; i--) {
                if (local_astar_->CheckPoint(follow_path_[i]) == false) continue;
                if (local_astar_->CheckLineObstacleFree(follow_path_[first_id], follow_path_[i]) == false) continue;
                i = i - 0.2 / path_dis_;
                if (i <= first_id) break;
                Eigen::Matrix<double, Eigen::Dynamic, 4> long_planes;
                GenerateAPolytope(follow_path_[first_id], follow_path_[i], long_planes, 1);
                if (long_planes.rows() > 0) {
                    for (int j = mpc_goal_index+1; j < mpc_->MPC_HORIZON; j++) {
                        mpc_->SetFSC(long_planes, j);
                    }
                    mpc_goal_index = mpc_->MPC_HORIZON;
                    for (int k = first_id; k < follow_path_.size(); k++) {
                        if (mpc_->IsInFSC(follow_path_[k], long_planes)) goal_in_sfc = k;
                        else break;
                    }
                    break;
                }
            }
            for (int i = mpc_goal_index+1; i < mpc_->MPC_HORIZON; i++) {
                if (last_planes.rows() > 0) mpc_->SetFSC(last_planes, i);
            }
        }

        // MPC set goal
        mpc_goals_.clear();
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            int index = astar_index_ + i * ref_dis_;
            
            // 钳制索引，防止越界
            if (index >= goal_in_sfc) index = goal_in_sfc;
            if (index >= follow_path_.size()) index = follow_path_.size() - 1;
            
            Eigen::Vector3d p_r = follow_path_[index];
            Eigen::Vector3d v_r = Eigen::Vector3d::Zero();
            
            // 【核心修改】：利用路径点自身的几何差分计算参考速度，坚决剔除 odom_p_
            if (index < follow_path_.size() - 1) {
                // 还没到终点，往前看一个点计算路径切向速度
                int next_index = index + ref_dis_;
                if (next_index >= follow_path_.size()) next_index = follow_path_.size() - 1;
                
                v_r = (follow_path_[next_index] - p_r) / mpc_->MPC_STEP;
            } else {
                // 只要当前参考点是最后一个点（终点），参考速度【必须】严格为 0
                v_r.setZero();
            }

            // 传给 MPC (期望位置，期望速度，期望加速度)
            mpc_->SetGoal(p_r, v_r, Eigen::Vector3d::Zero(), i);
            mpc_goals_.push_back(p_r);
    }
        AstarPublish(mpc_goals_, 3, 0.1);

        // yaw control
        if (astar_index_ < follow_path_.size() - 0.3 / path_dis_) {
            yaw_r_ = std::atan2(follow_path_.back().y()-odom_p_.y(), follow_path_.back().x()-odom_p_.x());
        }
    } else { // stay at initial position
        yaw_r_ = 0.0;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
        }
    }
    log_times_[2] = (ros::Time::now() - sfc_start).toSec() * 1000.0;

    // calculate model predict control algorithm
    ros::Time mpc_start = ros::Time::now();
    mpc_->SetStatus(odom_p_, odom_v_, last_cmd_a_);
    bool success_flag = mpc_->Run();
    log_times_[3] = (ros::Time::now() - mpc_start).toSec() * 1000.0;

    Eigen::Vector3d u_optimal, p_optimal, v_optimal, a_optimal, u_predict;
    Eigen::MatrixXd A1, B1;
    Eigen::VectorXd x_optimal = mpc_->X_0_;
    
    // 仅在MPC求解成功时发送命令，充当异常保护
    if (success_flag) {
        last_mpc_time_ = ros::Time::now();
        for (int i = 0; i <= ctrl_delay_/mpc_->MPC_STEP; i++) {
            mpc_->GetOptimCmd(u_optimal, i);
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
            x_optimal = A1 * x_optimal + B1 * u_optimal;
        }
        mpc_ctrl_index_ = ctrl_delay_/mpc_->MPC_STEP;

        p_optimal << x_optimal(0,0), x_optimal(1,0), x_optimal(2,0);
        v_optimal << x_optimal(3,0), x_optimal(4,0), x_optimal(5,0);
        a_optimal << x_optimal(6,0), x_optimal(7,0), x_optimal(8,0);

        last_cmd_a_ = a_optimal; 
        
        if (!perfect_simu_flag_) CmdPublish(odom_p_, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);

        std::vector<Eigen::Vector3d> path;
        x_optimal = mpc_->X_0_;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->GetOptimCmd(u_predict, i);
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
            x_optimal = A1 * x_optimal + B1 * u_predict;
            path.push_back(Eigen::Vector3d(x_optimal(0,0), x_optimal(1,0), x_optimal(2,0)));
        }
        MPCPathPublish(path);
    } else {
        ROS_WARN_THROTTLE(1.0, "[Planner]: MPC solve failed! Skip sending cmd.");
    }

    WriteLogTime();

    timer_mutex_.unlock();
}

void PlannerClass::GenerateAPolytope(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    planes.resize(0, 4);
    ros::Time start_t = ros::Time::now();
    Eigen::Vector3d box_max(10, 10, 3), box_min(-10, -10, -0.5);

    Eigen::Matrix<double, 6, 4> bd;
    bd.setZero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;
    bd(0, 3) = -p1.x()-box_max.x();
    bd(1, 3) =  p1.x()+box_min.x();
    bd(2, 3) = -p1.y()-box_max.y();
    bd(3, 3) =  p1.y()+box_min.y();
    bd(4, 3) = -box_max.z();
    bd(5, 3) = +box_min.z();

    Polytope p;
    if (local_pc_.empty()) { 
        planes.resize(6, 4); 
        planes.row(0) <<  1,  0,  0, -p1.x()-box_max.x();
        planes.row(1) <<  0,  1,  0, -p1.y()-box_max.y();
        planes.row(2) <<  0,  0,  1, -box_max.z();
        planes.row(3) << -1,  0,  0,  p1.x()+box_min.x();
        planes.row(4) <<  0, -1,  0,  p1.y()+box_min.y();
        planes.row(5) <<  0,  0, -1,  box_min.z();
        return ; 
    }
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(local_pc_[0].data(), 3, local_pc_.size());

    bool success = emvp::emvp(bd, pp, p1, p2, p, sfc_dis_, false, 1);
    if (success) {
        if (index == 0) p.Visualize(sfc_pub_, "emvp", true);
        p.Visualize(sfc_pub_, "emvp", false);
        planes = p.GetPlanes();
    } else {
        p.Reset();
    }
}

void PlannerClass::PathReplan(bool extend)
{
    astar_path_.clear();
    waypoints_.clear();
    follow_path_.clear();

    Eigen::Vector3d start_p, end_p;
    start_p = odom_p_;
    if (extend) {
        local_astar_->SetCenter(Eigen::Vector3d(odom_p_.x(), odom_p_.y(), 0.0));
    }
    local_astar_->setObsVector(local_pc_, expand_dyn_);

    bool add_goal_flag = false;
    end_p = goal_p_;
    double delta_x = goal_p_.x() - odom_p_.x();
    double delta_y = goal_p_.y() - odom_p_.y();
    if (std::fabs(delta_x) > map_upp_.x() || std::fabs(delta_y) > map_upp_.y()) {
        add_goal_flag = true;
        if (std::fabs(delta_x) > std::fabs(delta_y)) {
            end_p.x() = odom_p_.x() + (delta_x/std::fabs(delta_x)) * (map_upp_.x() - resolution_);
            end_p.y() = odom_p_.y() + ((map_upp_.x() - resolution_)/std::fabs(delta_x)) * delta_y;
        } else {
            end_p.x() = odom_p_.x() + ((map_upp_.y() - resolution_)/std::fabs(delta_y)) * delta_x;
            end_p.y() = odom_p_.y() + (delta_y/std::fabs(delta_y)) * (map_upp_.y() - resolution_);
        }
    }

    int point_num = 8;
    double r = expand_fix_ / 2;
    if (local_astar_->CheckStartEnd(start_p) == false) {
        ROS_INFO("\033[41;37m start point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << start_p.x() + r*sin(M_PI*2*i/point_num), start_p.y() + r*cos(M_PI*2*i/point_num), start_p.z();
                double dis_min = 10000.0;
                double dis = (odom_p_ - pt).norm();
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    start_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change start goal! %f %f %f\033[0m", start_p.x(), start_p.y(), start_p.z());
                break;
            }
            r += expand_fix_ / 2;
        }
    }
    r = expand_fix_ / 2;
    if (local_astar_->CheckStartEnd(end_p) == false) {
        ROS_INFO("\033[41;37m end point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << end_p.x() + r*sin(M_PI*2*i/point_num), end_p.y() + r*cos(M_PI*2*i/point_num), end_p.z();
                double dis_min = 10000.0;
                double dis = (odom_p_ - pt).norm();
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    end_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change end goal! %f %f %f\033[0m", end_p.x(), end_p.y(), end_p.z());
                break;
            }
            r += expand_fix_ / 2;
        }
    }

    bool search_flag = local_astar_->SearchPath(start_p, end_p);
    if (search_flag) {
        local_astar_->GetPath(astar_path_);
        AstarPublish(astar_path_, 0, 0.1);

        local_astar_->FloydHandle(astar_path_, waypoints_);
        if (add_goal_flag && local_astar_->CheckPoint(goal_p_)) waypoints_.push_back(goal_p_);
        AstarPublish(waypoints_, 1, 0.1);

        for (int i = 0; i < waypoints_.size()-1; i++) {
            Eigen::Vector3d vector = waypoints_[i+1] - waypoints_[i];
            int num = vector.norm() / path_dis_; 
            for (int j = 0; j < num; j++) {
                Eigen::Vector3d pt = waypoints_[i] + vector * j / num;
                follow_path_.push_back(pt);
            }
        }
        follow_path_.push_back(waypoints_.back());
        AstarPublish(follow_path_, 2, path_dis_);
    } else {
        ROS_INFO("\033[41;37m No path! Stay at current point! \033[0m");
        follow_path_.push_back(odom_p_);
    }

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = follow_path_.back().x();
    msg.pose.position.y = follow_path_.back().y();
    msg.pose.position.z = follow_path_.back().z();    
    goal_pub_.publish(msg);

    ros::Time now = ros::Time::now();
    local_astar_->Reset(); 
}

void PlannerClass::GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal_mutex_.lock();

    static Eigen::Vector3d last_goal;
    Eigen::Vector3d new_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    if (last_goal != new_goal) {
        goal_p_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        if (goal_p_.z() > map_upp_.z() - 0.5) goal_p_.z() = map_upp_.z() - 0.5;
        if (goal_p_.z() < 0.5) goal_p_.z() = 0.5;
        
        new_goal_flag_ = true;
        has_active_goal_ = true; // 激活目标点，允许发送cmd
        ROS_INFO("\033[1;32m[Planner]: Received new goal. Activating command stream.\033[0m");
    }
    last_goal = new_goal;

    goal_mutex_.unlock();
}

void PlannerClass::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_mutex_.lock();
    odom_time_ = msg->header.stamp;
    odom_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    
    odom_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
                                 
    //1. 提取 MAVROS 传回的机体系速度
    Eigen::Vector3d v_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    
    //2. 必须将其旋转到世界坐标系！
    odom_v_ = odom_q_ * v_body; 

    if (perfect_simu_flag_) {
        odom_v_.setZero();
    }
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    has_odom_flag_ = true;
    odom_mutex_.unlock();
}

void PlannerClass::IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_mutex_.lock();
    Eigen::Vector3d acc_body(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    
    // 把机体坐标系加速度转到世界坐标系
    Eigen::Vector3d acc_world = odom_q_ * acc_body;
    
    // 减去重力 (重力通常向上抵消或者向下，取决于你的坐标系，一般ENU下重力Z为 9.81)
    Eigen::Vector3d Gravity(0, 0, 9.81);
    Eigen::Vector3d acc_raw = acc_world - Gravity;
    
    // 必须要加低通滤波！(右边的系数是滤波系数，越小越平滑，但延迟越大)
    static Eigen::Vector3d acc_filtered = Eigen::Vector3d::Zero();
    acc_filtered = 0.8 * acc_filtered + 0.2 * acc_raw; 
    
    odom_a_ = acc_filtered; // 把滤波后的加速度给 MPC
    
    imu_mutex_.unlock();
}

void PlannerClass::LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    local_pc_mutex_.lock();

    ros::Time now = ros::Time::now();
    local_pc_.clear();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    vec_cloud_.push_back(cloud);
    if (vec_cloud_.size() > 10) vec_cloud_.pop_front();
    static_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < vec_cloud_.size(); i++) *static_cloud_ += vec_cloud_[i];
    pcl::CropBox<pcl::PointXYZ> cb; 
    cb.setMin(Eigen::Vector4f(odom_p_.x() - (map_upp_.x()-0.5), odom_p_.y() - (map_upp_.y()-0.5), 0.2, 1.0));
    cb.setMax(Eigen::Vector4f(odom_p_.x() + (map_upp_.x()-0.5), odom_p_.y() + (map_upp_.y()-0.5), map_upp_.z(), 1.0));
    cb.setInputCloud(static_cloud_);
    cb.filter(*static_cloud_);
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3f pos = odom_p_.cast<float>();
    vf.setLeafSize(0.2, 0.2, 0.2);
    vf.setInputCloud(static_cloud_);
    vf.filter(static_map_);
    for(auto &point: static_map_.points) {
        local_pc_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    static int obs_count = 0;
    local_astar_->SetCenter(Eigen::Vector3d(odom_p_.x(), odom_p_.y(), 0.0));
    local_astar_->setObsVector(local_pc_, expand_fix_);
    
    // 如果当前有跟随路径并且有激活的目标，再进行避障检查和重规划标记
    if (has_active_goal_ && follow_path_.size() > astar_index_) {
        std::vector<Eigen::Vector3d> remain_path;
        remain_path.insert(remain_path.begin(), follow_path_.begin()+astar_index_, follow_path_.end());
        if (local_astar_->CheckPathFree(remain_path) == false) replan_flag_ = true;
    }

    log_times_[0] = (ros::Time::now() - now).toSec() * 1000.0;

    local_pc_mutex_.unlock();
}

