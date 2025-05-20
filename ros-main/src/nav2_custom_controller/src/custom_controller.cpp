#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <algorithm>
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/line_iterator.hpp"

namespace nav2_custom_controller {

void CustomController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    node_ = parent.lock();
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;

    // 声明并获取参数
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.3));
    node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".min_linear_speed", rclcpp::ParameterValue(0.1));
    node_->get_parameter(plugin_name_ + ".min_linear_speed", min_linear_speed_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
    node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.5));
    node_->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.8));
    node_->get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.1));
    node_->get_parameter(plugin_name_ + ".yaw_tolerance", yaw_tolerance_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".xy_tolerance", rclcpp::ParameterValue(0.1));
    node_->get_parameter(plugin_name_ + ".xy_tolerance", xy_tolerance_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    node_->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".stop_on_goal_dist", rclcpp::ParameterValue(0.2));
    node_->get_parameter(plugin_name_ + ".stop_on_goal_dist", stop_on_goal_dist_);

    // Ackermann parameters
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.3)); // 默认值，根据你的机器人修改
    node_->get_parameter(plugin_name_ + ".wheelbase", wheelbase_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".max_steering_angle", rclcpp::ParameterValue(0.7)); // 默认值，根据你的机器人修改
    node_->get_parameter(plugin_name_ + ".max_steering_angle", max_steering_angle_);

    // PID控制器参数
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".kp_linear", rclcpp::ParameterValue(1.0));
    node_->get_parameter(plugin_name_ + ".kp_linear", kp_linear_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".kp_angular", rclcpp::ParameterValue(1.0));
    node_->get_parameter(plugin_name_ + ".kp_angular", kp_angular_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".ki_angular", rclcpp::ParameterValue(0.0));
    node_->get_parameter(plugin_name_ + ".ki_angular", ki_angular_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".kd_angular", rclcpp::ParameterValue(0.0));
    node_->get_parameter(plugin_name_ + ".kd_angular", kd_angular_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".path_deviation_threshold", rclcpp::ParameterValue(0.5));
    node_->get_parameter(plugin_name_ + ".path_deviation_threshold", path_deviation_threshold_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
    node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".enable_visualization", rclcpp::ParameterValue(true));
    node_->get_parameter(plugin_name_ + ".enable_visualization", enable_visualization_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".min_lookahead_distance", rclcpp::ParameterValue(0.3));
    node_->get_parameter(plugin_name_ + ".min_lookahead_distance", min_lookahead_distance_);

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".max_lookahead_distance", rclcpp::ParameterValue(1.5));
    node_->get_parameter(plugin_name_ + ".max_lookahead_distance", max_lookahead_distance_);

    // 初始化可视化发布器
    if (enable_visualization_) {
        auto node = parent.lock();
        viz_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "path_following_markers", 1);
    }

    // 初始化路径跟踪变量
    path_progress_index_ = 0;
    off_track_counter_ = 0;

    // 初始化变量
    prev_angular_error_ = 0.0;
    integral_angular_error_ = 0.0;
    rotating_to_goal_ = false;
    moving_backwards_ = false;  // 初始化倒车运动标志
    control_frequency_ = 20.0;  // 默认20Hz
    last_time_ = node_->get_clock()->now();

    RCLCPP_INFO(
        node_->get_logger(),
        "CustomController initialized with parameters: max_linear=%.2f, max_angular=%.2f, lookahead=%.2f, wheelbase=%.2f, max_steering_angle=%.2f",
        max_linear_speed_, max_angular_speed_, lookahead_distance_, wheelbase_, max_steering_angle_);
}

void CustomController::cleanup() {
    RCLCPP_INFO(node_->get_logger(),
                "Cleaning up controller: %s of type nav2_custom_controller::CustomController",
                plugin_name_.c_str());
}

void CustomController::activate() {
    RCLCPP_INFO(node_->get_logger(),
                "Activating controller: %s of type nav2_custom_controller::CustomController",
                plugin_name_.c_str());
    prev_angular_error_ = 0.0;
    integral_angular_error_ = 0.0;
    rotating_to_goal_ = false;
    moving_backwards_ = false;  // 重置向后运动标志
}

void CustomController::deactivate() {
    RCLCPP_INFO(node_->get_logger(),
                "Deactivating controller: %s of type nav2_custom_controller::CustomController",
                plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) {

    // 准备输出速度命令
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel.header.stamp = node_->get_clock()->now();

    // 检查路径是否为空
    if (global_plan_.poses.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Received plan with zero length");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    // 转换机器人姿态到全局坐标系 - 移到前面来先初始化robot_pose
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!nav2_util::transformPoseInTargetFrame(
            pose, robot_pose, *tf_, global_plan_.header.frame_id, transform_tolerance_)) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform robot pose into global frame");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    // 检查是否到达目标前增加额外检查，避免过早认为目标达到
    if (global_plan_.poses.size() > 1) {
        const auto& goal = global_plan_.poses.back();
        double dist_to_goal = getEuclideanDistance(robot_pose, goal);
        
        // 只有当距离目标足够近时才检查是否到达
        if (dist_to_goal < stop_on_goal_dist_ * 2.0) {
            if (isGoalReached(pose)) {
                RCLCPP_INFO(node_->get_logger(), "Goal reached");
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.angular.z = 0.0;
                return cmd_vel;
            }
        }
    } else {
        // 路径太短，直接检查
        if (isGoalReached(pose)) {
            RCLCPP_INFO(node_->get_logger(), "Goal reached");
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            return cmd_vel;
        }
    }

    // 计算当前时间和上次时间的差值
    rclcpp::Time current_time = node_->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // 防止dt为0或异常小值
    if (dt < 0.01) {
        dt = 0.01;
    }

    // 选择目标点 - 使用自适应前视距离
    double adapted_lookahead = adaptiveLookaheadDistance(velocity.linear.x);
    auto target_pose = getLookaheadPoint(robot_pose, adapted_lookahead);

    // 计算角度差
    double angle_diff = calculateAngleDifference(robot_pose, target_pose);

    // 强化检测目标是否在后方的逻辑
    bool target_is_behind = false;

    // 检查目标点是否在车辆后方 - 当角度差的绝对值大于90度时目标在后方
    if (std::abs(angle_diff) > M_PI_2) {
        target_is_behind = true;
        // 当目标在后方时，我们反转角度差，以便可以倒车导航到目标
        // 但保留角度差的符号，以保持正确的转向方向
        if (angle_diff > 0) {
            angle_diff = angle_diff - M_PI;
        } else {
            angle_diff = angle_diff + M_PI;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Target is behind robot, adjusted angle diff: %.2f rad", angle_diff);
    }

    // 检查路径上是否有指示需要倒车的路径点
    bool path_indicates_reverse = false;
    if (path_progress_index_ < global_plan_.poses.size() - 1) {
        size_t next_idx = path_progress_index_ + 1;
        
        // 获取路径方向与点朝向
        double path_dx = global_plan_.poses[next_idx].pose.position.x - 
                        global_plan_.poses[path_progress_index_].pose.position.x;
        double path_dy = global_plan_.poses[next_idx].pose.position.y - 
                        global_plan_.poses[path_progress_index_].pose.position.y;
        double path_yaw = std::atan2(path_dy, path_dx);
        double point_yaw = tf2::getYaw(global_plan_.poses[path_progress_index_].pose.orientation);
        double dir_diff = std::abs(normalizeAngle(path_yaw - point_yaw));
        
        // 如果差异接近180度，说明路径是要求倒车的
        path_indicates_reverse = (dir_diff > 2.0);
    }

    // 修改：目标在后方时，优先选择倒车，忽略其他条件
    bool should_reverse = target_is_behind || 
                          (moving_backwards_ && std::abs(angle_diff) > M_PI/4) || // 降低倒车中的角度阈值
                          path_indicates_reverse;
    
    // 当倒车状态变化时输出信息
    if (should_reverse != moving_backwards_) {
        RCLCPP_INFO(
            node_->get_logger(), 
            "Switching direction: %s -> %s (angle_diff: %.2f rad, target_behind: %s)",
            moving_backwards_ ? "REVERSE" : "FORWARD",
            should_reverse ? "REVERSE" : "FORWARD",
            angle_diff,
            target_is_behind ? "true" : "false");
        
        moving_backwards_ = should_reverse;
    }

    // 覆盖原地旋转标志 - 当目标在后方时不使用原地旋转
    if (target_is_behind) {
        rotating_to_goal_ = false;
    }

    // *** 修改: 替换原地旋转逻辑为前进/后退配合转向的实际车辆行为 ***
    bool need_heading_adjustment = false;
    
    // 检测是否需要调整朝向 - 角度差大于30度但还不到倒车阈值
    if (!moving_backwards_ && std::abs(angle_diff) > 0.785) { // 约45度
        need_heading_adjustment = true;
        RCLCPP_INFO(node_->get_logger(), "Large heading error (%.2f rad), using steering maneuver", angle_diff);
    }
    
    // 使用变量跟踪调整方向的过程状态
    static bool in_heading_adjustment = false;
    static bool adjustment_forward_phase = true; // true=前进调整, false=后退调整
    static int adjustment_step_counter = 0;
    static double initial_heading_error = 0.0;
    
    // 当角度差显著减小时，我们认为调整已经足够
    if (in_heading_adjustment && (
        std::abs(angle_diff) < 0.17 || // 约10度
        std::abs(angle_diff) < std::abs(initial_heading_error) * 0.4)) { // 或角度减小了60%
        
        RCLCPP_INFO(node_->get_logger(), 
                   "Heading adjustment complete: reduced from %.2f to %.2f rad", 
                   initial_heading_error, angle_diff);
        in_heading_adjustment = false;
        adjustment_step_counter = 0;
    }
    
    // 如果需要调整且当前不在调整状态，开始一个新的调整过程
    if (need_heading_adjustment && !in_heading_adjustment && !rotating_to_goal_) {
        in_heading_adjustment = true;
        adjustment_forward_phase = true; // 从前进阶段开始
        adjustment_step_counter = 0;
        initial_heading_error = angle_diff;
        RCLCPP_INFO(node_->get_logger(), "Starting steering maneuver for heading adjustment");
    }
    
    // 重置旧的旋转标志，我们不再使用原地旋转
    rotating_to_goal_ = false;

    // 改进路径跟踪：检查是否显著偏离路径
    auto closest_pose_idx = findClosestPointOnPath(robot_pose);
    double deviation = calculatePathDeviation(robot_pose, closest_pose_idx);

    if (deviation > path_deviation_threshold_) {
        off_track_counter_++;
        if (off_track_counter_ > 10) { // 持续偏离
            RCLCPP_WARN(node_->get_logger(),
                        "Robot is significantly off-track (%.2fm). Attempting correction.", deviation);
            // 重置计数器以避免持续警告
            off_track_counter_ = 0;
            // 直接瞄准路径上的最近点以回到正轨
            target_pose = global_plan_.poses[closest_pose_idx];
        }
    } else {
        off_track_counter_ = 0; // 当回到正轨时重置计数器
    }

    // 改进前视逻辑：使用速度和时间进行更具预测性的控制
    if (!rotating_to_goal_ && !moving_backwards_) {
        // 根据速度和时间计算前视距离
        double velocity_based_lookahead = std::max(
            min_lookahead_distance_,
            std::abs(velocity.linear.x) * lookahead_time_
        );
        adapted_lookahead = std::min(velocity_based_lookahead, max_lookahead_distance_);
    }

    // 计算角速度
    double angular_vel = applyPIDControl(angle_diff, dt);
    
    // 确定线速度 - 根据是否在进行朝向调整
    double linear_vel = 0.0;
    
    if (in_heading_adjustment) {
        // 在调整阶段，使用中等线速度和更强的转向
        adjustment_step_counter++;
        
        // 角度调整过程使用交替的前进和后退，每10个周期切换一次方向
        if (adjustment_step_counter > 10) { // 约0.5秒，假设控制频率20Hz
            adjustment_forward_phase = !adjustment_forward_phase;
            adjustment_step_counter = 0;
            RCLCPP_DEBUG(node_->get_logger(), 
                      "Switching steering direction to %s", 
                      adjustment_forward_phase ? "FORWARD" : "REVERSE");
        }
        
        // 用中等线速度进行调整
        double adjust_speed = min_linear_speed_ * 0.9; // 提高调整速度
        
        // 根据当前阶段设置前进或后退
        linear_vel = adjustment_forward_phase ? adjust_speed : -adjust_speed;
        
        // 加强转向角 - 在调整时使用更大的角速度增益
        angular_vel = angle_diff * 2.0; // 提高转向响应
    } else {
        // 正常导航模式
        // 根据角度差调整线速度（角度差越大，速度越慢）
        double angle_factor = std::max(0.3, 1.0 - std::abs(angle_diff) / M_PI);
        
        // 当目标在后方时，使用较小的角度因子，确保倒车速度更稳定
        if (target_is_behind) {
            angle_factor = std::max(0.5, angle_factor); // 最小保持50%的速度
        }
        
        linear_vel = min_linear_speed_ + (max_linear_speed_ - min_linear_speed_) * angle_factor;
        
        // 限制最大线速度
        linear_vel = limitVelocity(linear_vel, max_linear_speed_);
        
        // 如果接近目标，减速
        double distance_to_goal = getEuclideanDistance(robot_pose, global_plan_.poses.back());
        if (distance_to_goal < stop_on_goal_dist_ * 1.5) {
            linear_vel *= (distance_to_goal / (stop_on_goal_dist_ * 1.5));
        }
        
        // 目标在后方时，强制使用倒车模式并应用更稳定的线速度
        if (moving_backwards_) {
            // 如果确定是后方目标，给予更稳定的速度
            if (target_is_behind) {
                // 使用更高的最小速度，确保倒车足够有力量
                linear_vel = -std::max(min_linear_speed_ * 1.2, std::abs(linear_vel));
            } else {
                linear_vel = -linear_vel;
            }
        }
    }

    // 当倒车时改进转向响应，保持更稳定的控制
    if (moving_backwards_ && target_is_behind) {
        // 减小转向幅度以避免倒车时过度转向
        angular_vel *= 0.8;
    }

    // Calculate steering angle from angular velocity for Ackermann steering
    double steering_angle = calculateSteeringAngle(linear_vel, angular_vel);
    
    // 在倒车模式时优化转向角
    if (moving_backwards_ && target_is_behind) {
        // 确保转向角足够大以有效转向，但不要过大导致不稳定
        double min_effective_steering = 0.1; // 约6度的最小有效转向
        if (std::abs(steering_angle) < min_effective_steering && std::abs(angle_diff) > 0.1) {
            steering_angle = (steering_angle >= 0) ? 
                             min_effective_steering : -min_effective_steering;
        }
    }

    // 设置速度命令 -  For Ackermann, angular.z is set to steering angle
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = steering_angle;

    RCLCPP_DEBUG(node_->get_logger(),
                "Velocity Commands: Linear Vel=%.2f, Angular Vel=%.2f (Steering Angle=%.2f), Angle Diff=%.2f, Rotating=%d, Reversing=%d",
                cmd_vel.twist.linear.x, angular_vel, steering_angle,
                angle_diff, rotating_to_goal_, moving_backwards_);

    // 如果启用了可视化，则可视化控制点
    if (enable_visualization_) {
        publishVisualization(robot_pose, target_pose, closest_pose_idx);
    }

    return cmd_vel;
}

void CustomController::setSpeedLimit(const double &speed_limit,
                                    const bool &percentage) {
    if (percentage) {
        // 按百分比设置速度限制
        max_linear_speed_ = original_global_plan_.poses.empty() ?
            max_linear_speed_ * speed_limit / 100.0 :
            max_linear_speed_ * speed_limit / 100.0;
    } else {
        // 直接设置速度限制
        max_linear_speed_ = speed_limit;
    }

    RCLCPP_INFO(node_->get_logger(),
              "Speed limit is set to %.2f m/s", max_linear_speed_);
}

void CustomController::setPlan(const nav_msgs::msg::Path &path) {
    // 保存原始路径的副本
    original_global_plan_ = path;
    global_plan_ = path;

    // 重置控制相关变量
    rotating_to_goal_ = false;
    moving_backwards_ = false;  // 重置倒车运动标志
    prev_angular_error_ = 0.0;
    integral_angular_error_ = 0.0;

    RCLCPP_INFO(node_->get_logger(),
              "New plan set, %zu waypoints", path.poses.size());
}

geometry_msgs::msg::PoseStamped CustomController::getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const double &lookahead_dist) {

    using nav2_util::geometry_utils::euclidean_distance;

    // 改进前视点选择
    if (global_plan_.poses.size() <= 1) {
        return global_plan_.poses.back();
    }

    // 找到路径上最近的点
    size_t closest_idx = findClosestPointOnPath(current_pose);

    // 始终从路径上的当前位置向前看，而不是向后
    size_t start_idx = closest_idx;

    // 沿路径段测量累计距离
    double accumulated_dist = 0.0;
    size_t lookahead_idx = start_idx;

    // 调整倒车时的前视距离 - 当倒车时使用较近的点以提高精度
    double effective_lookahead = lookahead_dist;
    if (moving_backwards_) {
        effective_lookahead = std::min(lookahead_dist * 0.7, 0.5); // 倒车时减小前视距离
    }

    for (size_t i = start_idx; i < global_plan_.poses.size() - 1; ++i) {
        double segment_dist = euclidean_distance(global_plan_.poses[i], global_plan_.poses[i+1]);
        accumulated_dist += segment_dist;

        if (accumulated_dist >= effective_lookahead) {
            lookahead_idx = i + 1;
            break;
        }
    }

    // 如果找不到足够远的点，则使用最后一个点
    if (accumulated_dist < effective_lookahead && lookahead_idx < global_plan_.poses.size() - 1) {
        lookahead_idx = global_plan_.poses.size() - 1;
    }

    // 如果接近目标，则直接瞄准目标
    double dist_to_goal = euclidean_distance(current_pose, global_plan_.poses.back());
    if (dist_to_goal < effective_lookahead * 1.5) {
        lookahead_idx = global_plan_.poses.size() - 1;
    }

    return global_plan_.poses[lookahead_idx];
}

double CustomController::calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &target_pose) {

    // 获取当前机器人航向角
    double current_yaw = tf2::getYaw(current_pose.pose.orientation);

    // 计算目标点相对于当前位置的方向角
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double target_angle = std::atan2(dy, dx);

    // 计算角度差并规范化到[-π, π]范围
    double angle_diff = target_angle - current_yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    return angle_diff;
}

// 修改PID控制器，降低角速度的突变性
double CustomController::applyPIDControl(const double &error, const double &dt) {
    // 减小比例增益，使转向更平缓
    double reduced_kp = kp_angular_ * 0.7; // 降低30%比例系数
    
    // 比例项
    double p_term = reduced_kp * error;

    // 积分项 - 减小积分作用，防止过度校正
    integral_angular_error_ += error * dt * 0.8; // 降低积分增量
    double i_term = ki_angular_ * integral_angular_error_;

    // 防止积分饱和
    const double max_i_term = max_angular_speed_ * 0.5; // 降低积分上限
    i_term = std::max(-max_i_term, std::min(i_term, max_i_term));

    // 增强微分作用，提供阻尼效果减少震荡
    double d_term = kd_angular_ * 1.5 * (error - prev_angular_error_) / dt;
    prev_angular_error_ = error;

    // 计算总控制输出
    double control = p_term + i_term + d_term;

    // 应用速度限制器，使角速度变化更平滑
    static double last_control = 0.0;
    double control_diff = control - last_control;
    
    // 限制单步变化率
    double max_control_change = 0.5 * dt; // 每秒最大变化率
    if (std::abs(control_diff) > max_control_change) {
        control = last_control + (control_diff > 0 ? max_control_change : -max_control_change);
    }
    
    last_control = control;

    // 限制角速度
    return limitVelocity(control, max_angular_speed_);
}

// 修改 isGoalReached 方法，解决目标变更问题
bool CustomController::isGoalReached(const geometry_msgs::msg::PoseStamped &current_pose) {
    if (global_plan_.poses.empty()) {
        return false;
    }

    // 获取目标位置
    const auto& goal = global_plan_.poses.back();

    // 计算距离
    double dist = getEuclideanDistance(current_pose, goal);

    // 获取当前机器人航向角和目标航向角
    double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    double goal_yaw = tf2::getYaw(goal.pose.orientation);

    // 计算航向差并规范化
    double yaw_diff = goal_yaw - current_yaw;
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

    // 在接近目标但未达到时输出更详细日志
    if (dist < xy_tolerance_ * 2.0) {
        RCLCPP_DEBUG(node_->get_logger(), 
                   "Close to goal: dist=%.3f (tol=%.3f), yaw_diff=%.3f (tol=%.3f), pos(%.2f,%.2f) vs goal(%.2f,%.2f)",
                   dist, xy_tolerance_, yaw_diff, yaw_tolerance_,
                   current_pose.pose.position.x, current_pose.pose.position.y,
                   goal.pose.position.x, goal.pose.position.y);
    }

    // 检查是否满足位置和航向容差
    bool position_reached = dist < std::min(xy_tolerance_, stop_on_goal_dist_);
    bool orientation_reached = fabs(yaw_diff) < yaw_tolerance_;

    // 添加延迟确认机制
    static rclcpp::Time goal_reach_start_time = node_->get_clock()->now();
    static bool goal_reach_timer_active = false;

    if (position_reached && orientation_reached) {
        if (!goal_reach_timer_active) {
            goal_reach_start_time = node_->get_clock()->now();
            goal_reach_timer_active = true;
        }

        double elapsed_time = (node_->get_clock()->now() - goal_reach_start_time).seconds();
        if (elapsed_time > 1.0) { // 确保机器人在目标区域停留至少1秒
            RCLCPP_INFO(node_->get_logger(), "Goal reached at (%.2f, %.2f)", 
                      goal.pose.position.x, goal.pose.position.y);
            goal_reach_timer_active = false;
            return true;
        }
    } else {
        goal_reach_timer_active = false; // 重置计时器
    }

    return false;
}

double CustomController::getEuclideanDistance(
    const geometry_msgs::msg::PoseStamped &pose1,
    const geometry_msgs::msg::PoseStamped &pose2) {

    double dx = pose2.pose.position.x - pose1.pose.position.x;
    double dy = pose2.pose.position.y - pose1.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double CustomController::limitVelocity(const double &velocity, const double &max_velocity) {
    if (velocity > max_velocity) {
        return max_velocity;
    } else if (velocity < -max_velocity) {
        return -max_velocity;
    }
    return velocity;
}

// 修改自适应前视距离计算，提高高速时的前视距离
double CustomController::adaptiveLookaheadDistance(const double &current_velocity) {
    // 更平滑的前视距离自适应
    double speed = std::abs(current_velocity);

    // 在非常低速时，使用最小前视距离
    if (speed < 0.1) {
        return min_lookahead_distance_;
    }

    // 使用曲线函数而非线性关系，提供更加平滑的前视距离变化
    // 在高速时，大幅增加前视距离，给小车更多反应时间
    double speed_factor = 1.0 + std::tanh(speed * 2.0);
    double adaptive_lookahead = lookahead_distance_ * speed_factor;

    return std::max(min_lookahead_distance_,
            std::min(adaptive_lookahead, max_lookahead_distance_));
}

size_t CustomController::findClosestPointOnPath(const geometry_msgs::msg::PoseStamped &pose) {
    size_t closest_pose_idx = 0;
    double min_distance = std::numeric_limits<double>::max();

    // 找到全局计划中最近的点
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        double distance = getEuclideanDistance(pose, global_plan_.poses[i]);
        if (distance < min_distance) {
            min_distance = distance;
            closest_pose_idx = i;
        }
    }

    // 存储以跟踪路径进度
    if (closest_pose_idx > path_progress_index_) {
        path_progress_index_ = closest_pose_idx;
    }

    return closest_pose_idx;
}

double CustomController::calculatePathDeviation(
    const geometry_msgs::msg::PoseStamped &pose,
    size_t closest_idx) {

    if (closest_idx >= global_plan_.poses.size() - 1) {
        return 0.0; // 在路径末端
    }

    // 获取机器人最近的路径段
    auto p1 = global_plan_.poses[closest_idx].pose.position;
    auto p2 = global_plan_.poses[closest_idx + 1].pose.position;
    auto robot_pos = pose.pose.position;

    // 使用直线方程计算到路径段的垂直距离
    double A = p2.y - p1.y;
    double B = p1.x - p2.x;
    double C = p2.x * p1.y - p1.x * p2.y;

    // 点到直线的距离：|Ax + By + C| / sqrt(A² + B²)
    return std::abs(A * robot_pos.x + B * robot_pos.y + C) /
          std::sqrt(A * A + B * B);
}

void CustomController::publishVisualization(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PoseStamped &target_pose,
    size_t closest_idx) {

    visualization_msgs::msg::MarkerArray marker_array;

    // 1. 当前路径标记
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = global_plan_.header.frame_id;
    path_marker.header.stamp = node_->get_clock()->now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.05;  // 线宽
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    path_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    for (const auto &pose : global_plan_.poses) {
        geometry_msgs::msg::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = 0.1;  // 略高于地面
        path_marker.points.push_back(p);
    }

    // 2. 目标点标记
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = global_plan_.header.frame_id;
    target_marker.header.stamp = node_->get_clock()->now();
    target_marker.ns = "lookahead_target";
    target_marker.id = 1;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.pose.position = target_pose.pose.position;
    target_marker.pose.position.z += 0.1;  // 略高于地面
    target_marker.pose.orientation.w = 1.0;
    target_marker.scale.x = 0.2;
    target_marker.scale.y = 0.2;
    target_marker.scale.z = 0.2;
    target_marker.color.r = 1.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 0.0;
    target_marker.color.a = 1.0;
    target_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    // 3. 最近点标记
    visualization_msgs::msg::Marker closest_marker;
    closest_marker.header.frame_id = global_plan_.header.frame_id;
    closest_marker.header.stamp = node_->get_clock()->now();
    closest_marker.ns = "closest_point";
    closest_marker.id = 2;
    closest_marker.type = visualization_msgs::msg::Marker::SPHERE;
    closest_marker.action = visualization_msgs::msg::Marker::ADD;
    closest_marker.pose.position = global_plan_.poses[closest_idx].pose.position;
    closest_marker.pose.position.z += 0.1;  // 略高于地面
    closest_marker.pose.orientation.w = 1.0;
    closest_marker.scale.x = 0.15;
    closest_marker.scale.y = 0.15;
    closest_marker.scale.z = 0.15;
    closest_marker.color.r = 0.0;
    closest_marker.color.g = 0.0;
    closest_marker.color.b = 1.0;
    closest_marker.color.a = 1.0;
    closest_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_array.markers.push_back(path_marker);
    marker_array.markers.push_back(target_marker);
    marker_array.markers.push_back(closest_marker);

    viz_pub_->publish(marker_array);
}

double CustomController::normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

// 修改计算转向角函数，使转向更平滑
double CustomController::calculateSteeringAngle(double linear_velocity, double angular_velocity)
{
    double abs_linear_vel = std::abs(linear_velocity);
    
    // 处理低速情况下的转向 - 更平滑的转向角增长
    if (abs_linear_vel < 0.1) {
        // 使用平方根函数使低速转向更平滑
        double speed_factor = std::sqrt(std::max(0.01, abs_linear_vel) / 0.1);
        double sign = (angular_velocity >= 0.0) ? 1.0 : -1.0;
        
        // 降低低速时的最大转向角，避免急转弯
        double max_low_speed_angle = max_steering_angle_ * 0.7;
        return sign * std::min(std::abs(angular_velocity * speed_factor), max_low_speed_angle);
    }
    
    // 标准Ackermann转向模型
    double steering_angle = std::atan2(wheelbase_ * angular_velocity, abs_linear_vel);
    
    // 当倒车时，同样的角速度需要反向的转向角以实现期望的轨迹
    if (linear_velocity < 0) {
        steering_angle = -steering_angle;
    }
    
    // 平滑限制最大转向角 - 使用平方根函数使接近最大值时增长变缓
    double max_angle_factor = 0.9; // 降至90%避免达到物理极限
    double normalized_angle = std::abs(steering_angle) / max_steering_angle_;
    
    if (normalized_angle > 0.7) { // 只在接近限制时应用平滑
        double smooth_factor = 0.7 + 0.3 * std::sqrt((normalized_angle - 0.7) / 0.3);
        normalized_angle = std::min(smooth_factor, 1.0);
        steering_angle = (steering_angle > 0 ? 1.0 : -1.0) * normalized_angle * max_steering_angle_ * max_angle_factor;
    } else {
        // 常规限制
        steering_angle = std::max(-max_steering_angle_ * max_angle_factor, 
                                std::min(steering_angle, max_steering_angle_ * max_angle_factor));
    }
    
    // 平滑转向角变化 - 避免突然变化
    static double prev_steering_angle = 0.0;
    double angle_diff = steering_angle - prev_steering_angle;
    
    // 限制单次转向角变化量
    double max_angle_change = 0.15; // 约8.6度
    if (std::abs(angle_diff) > max_angle_change) {
        steering_angle = prev_steering_angle + (angle_diff > 0 ? max_angle_change : -max_angle_change);
    }
    
    prev_steering_angle = steering_angle;

    // 增强小角度转弯能力
    if (std::abs(angular_velocity) < 0.3) { // 小角速度
        // 增加小角度转向的敏感度
        angular_velocity *= 1.5; // 提高50%的转向响应
    }

    // 增加低速时的转向能力
    if (abs_linear_vel < 0.15) { // 低速状态
        // 提高低速转向的有效性
        double steering_boost = 1.0 + (0.15 - abs_linear_vel) * 5.0; // 最多增加75%
        steering_angle *= steering_boost;
    }

    return steering_angle;
}

} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)
