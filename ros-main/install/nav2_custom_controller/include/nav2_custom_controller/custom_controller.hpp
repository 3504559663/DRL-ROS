#ifndef NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_custom_controller {

class CustomController : public nav2_core::Controller {
public:
    CustomController() = default;
    ~CustomController() override = default;
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;
    geometry_msgs::msg::TwistStamped
    computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                            const geometry_msgs::msg::Twist &velocity,
                            nav2_core::GoalChecker * goal_checker) override;
    void setPlan(const nav_msgs::msg::Path &path) override;
    void setSpeedLimit(const double &speed_limit,
                        const bool &percentage) override;

protected:
    // 插件名称
    std::string plugin_name_;
    // 坐标变换缓存指针
    std::shared_ptr<tf2_ros::Buffer> tf_;
    // 代价地图
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    // 节点指针
    nav2_util::LifecycleNode::SharedPtr node_;
    // 全局代价地图
    nav2_costmap_2d::Costmap2D *costmap_;
    // 全局路径
    nav_msgs::msg::Path global_plan_;
    // 保存原始全局路径的副本
    nav_msgs::msg::Path original_global_plan_;
    // 参数
    double max_angular_speed_;
    double max_linear_speed_;
    double min_linear_speed_;     // 最小线速度
    double lookahead_distance_;   // 前视距离
    double rotate_to_heading_angular_vel_; // 原地旋转的角速度
    double yaw_tolerance_;        // 航向容差（弧度）
    double xy_tolerance_;         // 位置容差（米）
    double transform_tolerance_;  // 转换容差（秒）
    double control_frequency_;    // 控制频率（Hz）
    bool rotating_to_goal_;       // 是否正在旋转以对准目标
    double stop_on_goal_dist_;    // 接近目标时停止的距离
    bool moving_backwards_;       // 向后运动的标志

    // Ackermann parameters
    double wheelbase_;             // 轴距
    double max_steering_angle_;    // 最大转向角

    // PID控制器参数
    double kp_linear_;            // 线速度比例项
    double kp_angular_;           // 角速度比例项
    double ki_angular_;           // 角速度积分项
    double kd_angular_;           // 角速度微分项
    double prev_angular_error_;   // 上一次角度误差
    double integral_angular_error_; // 角度误差积分
    rclcpp::Time last_time_;      // 上次计算时间

    // 路径跟踪参数
    size_t path_progress_index_;    // 跟踪路径进度
    int off_track_counter_;         // 计算偏离轨道的程度
    double path_deviation_threshold_; // 允许偏离路径的最大程度

    // 前视距离参数
    double lookahead_time_;         // 基于时间的前视距离
    double min_lookahead_distance_ = 0.3;  // 最小前视距离
    double max_lookahead_distance_ = 1.5;  // 最大前视距离

    // 可视化
    bool enable_visualization_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

    // 查找路径上的目标点
    geometry_msgs::msg::PoseStamped
    getLookaheadPoint(const geometry_msgs::msg::PoseStamped &current_pose,
                    const double &lookahead_dist);

    // 计算目标点方向和当前位置的角度差
    double
    calculateAngleDifference(const geometry_msgs::msg::PoseStamped &current_pose,
                            const geometry_msgs::msg::PoseStamped &target_pose);

    // 平滑速度控制
    double applyPIDControl(const double &error, const double &dt);

    // 检查是否到达目标点附近
    bool isGoalReached(const geometry_msgs::msg::PoseStamped &current_pose);

    // 计算两点之间的距离
    double getEuclideanDistance(const geometry_msgs::msg::PoseStamped &pose1,
                                const geometry_msgs::msg::PoseStamped &pose2);

    // 速度平滑处理
    double limitVelocity(const double &velocity, const double &max_velocity);

    // 根据当前速度调整前视距离
    double adaptiveLookaheadDistance(const double &current_velocity);

    // 新的方法
    size_t findClosestPointOnPath(const geometry_msgs::msg::PoseStamped &pose);
    double calculatePathDeviation(const geometry_msgs::msg::PoseStamped &pose, size_t closest_idx);
    void publishVisualization(const geometry_msgs::msg::PoseStamped &robot_pose,
                            const geometry_msgs::msg::PoseStamped &target_pose,
                            size_t closest_idx);

    // Ackermann Steering Control
    double calculateSteeringAngle(double linear_velocity, double angular_velocity);

private:
    // Added normalizeAngle declaration
    double normalizeAngle(double angle);
};

} // namespace nav2_custom_controller

#endif // NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_