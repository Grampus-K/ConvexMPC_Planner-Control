#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <mutex>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include "../include/astar.h"
#include "../include/mpc.h"
#include "../include/local_astar.h"

class PlannerClass {
public:
    PlannerClass(ros::NodeHandle &nh) {
        // load param
        double freq;
        nh.param("/ipc_node/simulation", simu_flag_, true);
        nh.param("/ipc_node/perfect_simu", perfect_simu_flag_, false);
        nh.param("/ipc_node/frequency", freq, 100.0);
        nh.param("/ipc_node/ctrl_delay", ctrl_delay_, 0.1);
        nh.param("/ipc_node/sfc_dis", sfc_dis_, 0.1);
        nh.param("/ipc_node/yaw_ctrl_flag", yaw_ctrl_flag_, false);
        nh.param("/ipc_node/yaw_gain", yaw_gain_, 0.1);
        nh.param("/ipc_node/goal_x", goal_p_.x(), 0.0);
        nh.param("/ipc_node/goal_y", goal_p_.y(), 0.0);
        nh.param("/ipc_node/goal_z", goal_p_.z(), 1.0);
        
        // Target arrival tolerance
        nh.param("/ipc_node/goal_tolerance", goal_tolerance_, 0.2); 

        Eigen::Vector3d map_size, map_low, map_upp;
        nh.param("/ipc_node/astar/resolution", resolution_, 0.1);
        nh.param("/ipc_node/astar/map_x_size", map_size.x(), 10.0);
        nh.param("/ipc_node/astar/map_y_size", map_size.y(), 10.0);
        nh.param("/ipc_node/astar/map_z_size", map_size.z(), 5.0);
        nh.param("/ipc_node/astar/expand_dyn", expand_dyn_, 0.25);
        nh.param("/ipc_node/astar/expand_fix", expand_fix_, 0.25);
        map_low << -map_size.x()/2.0, -map_size.y()/2.0, 0;
        map_upp <<  map_size.x()/2.0,  map_size.y()/2.0, map_size.z();
        map_upp_ = map_upp;
        nh.param("/ipc_node/fsm/ref_dis", ref_dis_, 1);
        nh.param("/ipc_node/fsm/path_dis", path_dis_, 0.1);

        // instantiation
        odom_p_ << 0, 0, 1;
        odom_v_.setZero();
        odom_a_.setZero();
        last_cmd_a_.setZero();

        mpc_   = std::make_shared<MPCPlannerClass>(nh);
        local_astar_ = std::make_shared<LoaclAstarClass>();
        local_astar_->InitMap(resolution_, map_low, map_upp);

        // ros topic
        astar_pub_ = nh.advertise<visualization_msgs::Marker>("astar_path", 1);
        gird_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map", 1);
        cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 1);
        mpc_path_pub_ = nh.advertise<nav_msgs::Path>("mpc_path", 1);
        sfc_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc", 1);
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_pub", 1);

        local_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("local_pc", 1, &PlannerClass::LocalPcCallback, this, ros::TransportHints().tcpNoDelay());
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &PlannerClass::OdomCallback, this, ros::TransportHints().tcpNoDelay());
        goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &PlannerClass::GoalCallback, this, ros::TransportHints().tcpNoDelay());
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("imu", 1, &PlannerClass::IMUCallback, this, ros::TransportHints().tcpNoDelay());

        timer_ = nh.createTimer(ros::Duration(1.0/freq), &PlannerClass::TimerCallback, this, false, true);

        std::string file = ros::package::getPath("ipc") + "/config";
        write_time_.open((file+"/time_consuming.csv"), std::ios::out | std::ios::trunc);
        log_times_.resize(4, 1); // 修改为4项日志
        write_time_ << "mapping" << ", " << "replan" << ", " << "sfc" << ", " << "mpc" << ", " << std::endl;
    }
    ~PlannerClass() {}

private:
    void AstarPublish(std::vector<Eigen::Vector3d>& nodes, uint8_t type, double scale) {
        visualization_msgs::Marker node_vis; 
        node_vis.header.frame_id = "world";
        node_vis.header.stamp = ros::Time::now();

        if (type == 0) {
            node_vis.ns = "astar_path";
            node_vis.color.a = 1.0;
            node_vis.color.r = 0.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 0.0;
        } else if (type == 1) {
            node_vis.ns = "floyd_path";
            node_vis.color.a = 1.0;
            node_vis.color.r = 1.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 0.0;
        } else if (type == 2) {
            node_vis.ns = "short_path";
            node_vis.color.a = 1.0;
            node_vis.color.r = 0.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 1.0;
        } else if (type == 3) {
            node_vis.ns = "set_points";
            node_vis.color.a = 1.0;
            node_vis.color.r = 0.0;
            node_vis.color.g = 1.0;
            node_vis.color.b = 0.0;
        }

        node_vis.type = visualization_msgs::Marker::CUBE_LIST;
        node_vis.action = visualization_msgs::Marker::ADD;
        node_vis.id = 0;
        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;
        
        node_vis.scale.x = scale;
        node_vis.scale.y = scale;
        node_vis.scale.z = scale;

        geometry_msgs::Point pt;
        for (int i = 0; i < int(nodes.size()); i++) {
            Eigen::Vector3d coord = nodes[i];
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);
            node_vis.points.push_back(pt);
        }
        astar_pub_.publish(node_vis);
    }
    
    void CmdPublish(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r) {
        quadrotor_msgs::PositionCommand msg;
        msg.header.frame_id = "world";
        msg.header.stamp    = ros::Time::now();
        msg.position.x      = p_r.x();
        msg.position.y      = p_r.y();
        msg.position.z      = p_r.z();
        msg.velocity.x      = v_r.x();
        msg.velocity.y      = v_r.y();
        msg.velocity.z      = v_r.z();
        msg.acceleration.x  = a_r.x();
        msg.acceleration.y  = a_r.y();
        msg.acceleration.z  = a_r.z();
        msg.jerk.x          = j_r.x();
        msg.jerk.y          = j_r.y();
        msg.jerk.z          = j_r.z();
        if (yaw_ctrl_flag_) {
            double yaw_error = yaw_r_ - yaw_;
            if (yaw_error >  M_PI) yaw_error -= M_PI * 2;
            if (yaw_error < -M_PI) yaw_error += M_PI * 2;
            msg.yaw     = yaw_ + yaw_error * 0.1;
            msg.yaw_dot = 0;
        } else {
            msg.yaw     = 0;
            msg.yaw_dot = 0;
        }
        cmd_pub_.publish(msg);
    }
    
    void MPCPathPublish(std::vector<Eigen::Vector3d> &pt) {
        nav_msgs::Path msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        for (int i = 0; i < pt.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = pt[i].x();
            pose.pose.position.y = pt[i].y();
            pose.pose.position.z = pt[i].z();
            msg.poses.push_back(pose);
        }
        mpc_path_pub_.publish(msg);
    }
    
    void WriteLogTime(void) {
        for (int i = 0; i < log_times_.size(); i++) {
            write_time_ << log_times_[i] << ", ";
            log_times_[i] = 0.0;
        }
        write_time_ << std::endl;
    }

    void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void IMUCallback(const sensor_msgs::ImuConstPtr& msg);
    void LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void TimerCallback(const ros::TimerEvent &);

    void PathReplan(bool extend);
    void GeneratePolyOnPath();
    void GenerateAPolytope(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index);

    ros::Timer      timer_;
    ros::Subscriber odom_sub_, goal_sub_, imu_sub_, local_pc_sub_;
    ros::Publisher  gird_map_pub_, astar_pub_, cmd_pub_, sfc_pub_, mpc_path_pub_, goal_pub_;
    ros::Time       odom_time_, last_mpc_time_;
    std::mutex  odom_mutex_, goal_mutex_, cloud_mutex_, timer_mutex_, local_pc_mutex_, imu_mutex_;

    bool simu_flag_, perfect_simu_flag_, pc_ctrl_flag_, yaw_ctrl_flag_;
    bool has_map_flag_{false}, has_odom_flag_{false}, replan_flag_{false}, new_goal_flag_{false};
    bool has_active_goal_{false}; // 核心：目标激活标志位

    double resolution_;
    double ctrl_delay_;
    double sfc_dis_, path_dis_, expand_dyn_, expand_fix_;
    double goal_tolerance_;
    int ref_dis_;
    int mpc_ctrl_index_;
    std::vector<Eigen::Vector3d> remain_nodes_;

    std::ofstream write_time_;
    std::vector<double> log_times_;

    Eigen::Vector3d goal_p_, map_upp_;
    Eigen::Vector3d odom_p_, odom_v_, odom_a_, last_cmd_a_ ;
    Eigen::Quaterniond odom_q_;
    double yaw_{0}, yaw_r_{0}, yaw_dot_r_{0}, yaw_gain_;

    int astar_index_{0};
    std::vector<Eigen::Vector3d> astar_path_;
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<Eigen::Vector3d> follow_path_;
    std::vector<Eigen::Vector3d> replan_path_;
    std::vector<Eigen::Vector3d> local_pc_;
    std::vector<Eigen::Vector3d> local_pc_buffer_[10];
    std::vector<Eigen::Vector3d> mpc_goals_;

    std::deque<pcl::PointCloud<pcl::PointXYZ>> vec_cloud_;
    pcl::PointCloud<pcl::PointXYZ> static_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_;

    std::shared_ptr<LoaclAstarClass> local_astar_;
    std::shared_ptr<MPCPlannerClass> mpc_;
};

#endif

