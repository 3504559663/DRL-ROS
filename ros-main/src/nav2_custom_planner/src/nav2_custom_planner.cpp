// Modified nav2_custom_planner.cpp
#include "nav2_custom_planner/nav2_custom_planner.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <memory>
#include <limits>
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

// Register the planner as a plugin
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)

namespace nav2_custom_planner
{

CustomPlanner::CustomPlanner()
: costmap_(nullptr)
{
}

CustomPlanner::~CustomPlanner()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type CustomPlanner", name_.c_str());
}

void CustomPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  // Get parameters from node
  auto node = parent.lock();
  
  // Default values
  path_resolution_ = 0.05;       // Default resolution in meters
  obstacle_clearance_ = 0.3;     // Default clearance from obstacles in meters
  straight_line_preference_ = 2.0;  // Weight for preferring straight paths
  max_iterations_ = 10000;       // Maximum A* iterations to prevent infinite loops
  
  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_clearance", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".straight_line_preference", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(10000));
  
  node->get_parameter(name + ".path_resolution", path_resolution_);
  node->get_parameter(name + ".obstacle_clearance", obstacle_clearance_);
  node->get_parameter(name + ".straight_line_preference", straight_line_preference_);
  node->get_parameter(name + ".max_iterations", max_iterations_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".min_turning_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".is_ackermann", rclcpp::ParameterValue(false));
  
  node->get_parameter(name + ".min_turning_radius", min_turning_radius_);
  node->get_parameter(name + ".is_ackermann", is_ackermann_);

  // 添加机器人尺寸相关参数
  robot_width_ = 0.3;       // 默认机器人宽度 (m)
  robot_length_ = 0.75;     // 默认机器人长度 (m)
  inflation_radius_ = 0.4;  // 默认膨胀半径，考虑机器人尺寸的安全距离

  // 声明和获取机器人尺寸参数
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".robot_width", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".robot_length", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".inflation_radius", rclcpp::ParameterValue(0.4));
  
  node->get_parameter(name + ".robot_width", robot_width_);
  node->get_parameter(name + ".robot_length", robot_length_);
  node->get_parameter(name + ".inflation_radius", inflation_radius_);

  // 确保障碍物清除距离至少等于膨胀半径
  obstacle_clearance_ = std::max(obstacle_clearance_, inflation_radius_);

  // Initialize path stability parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_stability_timeout", rclcpp::ParameterValue(30.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_smoothing_factor", rclcpp::ParameterValue(0.92));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".turn_radius_factor", rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_improvement_threshold", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".turn_suppression_factor", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_simplify_aggressiveness", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angle_continuity_factor", rclcpp::ParameterValue(0.95));
  
  node->get_parameter(name + ".path_stability_timeout", path_stability_timeout_);
  node->get_parameter(name + ".path_smoothing_factor", path_smoothing_factor_);
  node->get_parameter(name + ".turn_radius_factor", turn_radius_factor_);
  node->get_parameter(name + ".path_improvement_threshold", path_improvement_threshold_);
  node->get_parameter(name + ".turn_suppression_factor", turn_suppression_factor_);
  node->get_parameter(name + ".path_simplify_aggressiveness", path_simplify_aggressiveness_);
  node->get_parameter(name + ".angle_continuity_factor", angle_continuity_factor_);
  
  // Initialize time and path
  last_plan_time_ = node->get_clock()->now();
  prev_goal_ = geometry_msgs::msg::PoseStamped();

  // 添加新的安全性参数
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_check_safety_factor", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".collision_check_points", rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_path_optimization", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".grid_resolution_multiplier", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".direct_path_preferred", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enhanced_heuristic", rclcpp::ParameterValue(true));
    
  node->get_parameter(name + ".obstacle_check_safety_factor", obstacle_check_safety_factor_);
  node->get_parameter(name + ".collision_check_points", collision_check_points_);
  node->get_parameter(name + ".enable_path_optimization", enable_path_optimization_);
  node->get_parameter(name + ".grid_resolution_multiplier", grid_resolution_multiplier_);
  node->get_parameter(name + ".direct_path_preferred", direct_path_preferred_);
  node->get_parameter(name + ".enhanced_heuristic", enhanced_heuristic_);

  RCLCPP_INFO(
    logger_, "CustomPlanner initialized with path_resolution=%.2f, obstacle_clearance=%.2f, "
    "straight_line_preference=%.2f, max_iterations=%d, min_turning_radius=%.2f, is_ackermann=%s, "
    "robot_width=%.2f, robot_length=%.2f, inflation_radius=%.2f", 
    path_resolution_, obstacle_clearance_, straight_line_preference_, max_iterations_,
    min_turning_radius_, is_ackermann_ ? "true" : "false", robot_width_, robot_length_, inflation_radius_);
}

void CustomPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type CustomPlanner", name_.c_str());
}

void CustomPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type CustomPlanner", name_.c_str());
}

void CustomPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type CustomPlanner", name_.c_str());
}

nav_msgs::msg::Path CustomPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(
    logger_, "Creating plan from (%.2f, %.2f) to (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // 检查起点和终点是否在碰撞中
  if (isPointInCollision(start.pose.position.x, start.pose.position.y)) {
    RCLCPP_WARN(logger_, "Start position is in collision with an obstacle.");
    return nav_msgs::msg::Path();  // 返回空路径
  }
  
  if (isPointInCollision(goal.pose.position.x, goal.pose.position.y)) {
    RCLCPP_WARN(logger_, "Goal position is in collision with an obstacle.");
    return nav_msgs::msg::Path();  // 返回空路径
  }

  nav_msgs::msg::Path path;
  path.header = start.header;
  
  // 计算直线距离，用于后续决策
  double direct_distance = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);

  // 修改策略：首先尝试直接路径，仅针对特别短的距离
  bool direct_path_possible = false;
  if (direct_distance < 0.5) {  // 只为非常短的路径（<50cm）优先考虑直线
    double direct_path_clearance = obstacle_clearance_ * 0.9;
    direct_path_possible = isLineCollisionFree(start, goal, direct_path_clearance);
    
    if (direct_path_possible) {
      RCLCPP_INFO(logger_, "Short direct path is safe, using it");
      auto direct_path = createStraightPath(start, goal);
      if (is_ackermann_) {
        return smoothPathForAckermann(direct_path);
      }
      return direct_path;
    }
  }
  
  // 使用A*寻找绕行路径
  RCLCPP_INFO(logger_, "Using A* to find path around obstacles");
  path = findPathAroundObstacles(start, goal);
  
  // 检查A*路径是否存在
  if (!path.poses.empty()) {
    RCLCPP_INFO(logger_, "A* found a path with %zu waypoints", path.poses.size());
    
    // 对于Ackermann系统，平滑路径以符合转弯约束
    if (is_ackermann_) {
      RCLCPP_INFO(logger_, "Smoothing A* path for Ackermann constraints");
      path = smoothPathForAckermann(path);
    }
    
    // A*路径存在，但执行宽松的碰撞检查
    if (isPathCollisionFree(path)) {
      RCLCPP_INFO(logger_, "A* path passed collision check");
      return path;
    } else {
      RCLCPP_WARN(logger_, "A* path has potential collisions, trying with more conservative parameters");
      path = findPathAroundObstacles(start, goal, true);
      
      if (!path.poses.empty() && isPathCollisionFree(path)) {
        RCLCPP_INFO(logger_, "Conservative A* path is collision-free");
        return path;
      }
    }
  }
  
  // 如果A*失败，再尝试直接路径（对于较长的距离）
  if (direct_distance >= 0.5) {
    double clearance = obstacle_clearance_ * 0.9;
    direct_path_possible = isLineCollisionFree(start, goal, clearance);
    
    if (direct_path_possible) {
      RCLCPP_INFO(logger_, "Direct path is safe, using it as fallback");
      auto straight_path = createStraightPath(start, goal);
      if (is_ackermann_) {
        return smoothPathForAckermann(straight_path);
      }
      return straight_path;
    }
  }
  
  // 对于Ackermann系统，如果前面的方法失败，尝试专门的处理
  if (is_ackermann_) {
    // 计算目标朝向和距离
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double target_angle = std::atan2(dy, dx);
    double start_yaw = tf2::getYaw(start.pose.orientation);
    double angle_diff = std::abs(normalizeAngle(target_angle - start_yaw));
    
    // 仅当角度非常大(超过65度)且距离适中时才考虑特殊处理
    bool is_large_angle = (angle_diff > 1.13); // 65度以上
    
    // 优先尝试双弧线路径
    if (is_large_angle && direct_distance > min_turning_radius_ && 
        direct_distance < min_turning_radius_ * 5.0) {
      
      RCLCPP_INFO(logger_, "Attempting special handling for large angle (%.1f deg)", 
                 angle_diff * 180.0/M_PI);
      
      // 首先尝试双弧线路径
      if (angle_diff < 2.09) { // 120度以下
        nav_msgs::msg::Path dual_arc_path = createDualArcPath(start, goal);
        if (!dual_arc_path.poses.empty() && isPathCollisionFree(dual_arc_path)) {
          RCLCPP_INFO(logger_, "Successfully created dual-arc path");
          return dual_arc_path;
        }
      }
      
      // 如果需要，尝试后退路径
      bool is_rear_target = (angle_diff > 2.79); // 160度以上
      if (is_rear_target && direct_distance < min_turning_radius_ * 2.0) {
        path = createReversePathToTarget(start, goal);
        if (!path.poses.empty() && isPathCollisionFree(path)) {
          RCLCPP_INFO(logger_, "Successfully created reverse path for rear target");
          return path;
        }
      }
      
      // 尝试大角度处理函数
      path = handleLargeAngleTarget(start, goal);
      if (!path.poses.empty() && isPathCollisionFree(path)) {
        RCLCPP_INFO(logger_, "Successfully created special path for large angle target");
        return path;
      }
    }
    
    // 尝试Dubins路径
    if (direct_distance > min_turning_radius_) {
      RCLCPP_INFO(logger_, "Trying Dubins path");
      path = createDubinsPath(start, goal);
      if (!path.poses.empty() && isPathCollisionFree(path)) {
        return path; 
      }
    }
  }
  
  // 最后的尝试：降低障碍物清除距离，再次尝试A*
  RCLCPP_WARN(logger_, "All standard methods failed - trying reduced clearance");
  double original_clearance = obstacle_clearance_;
  obstacle_clearance_ = std::max(0.05, obstacle_clearance_ * 0.6);
  
  path = findPathAroundObstacles(start, goal);
  
  // 恢复原始参数
  obstacle_clearance_ = original_clearance;
  
  if (path.poses.empty()) {
    RCLCPP_ERROR(logger_, "Failed to find any valid path");
    return nav_msgs::msg::Path(); // 返回空路径
  }
  
  // 增强路径稳定性检查
  auto current_time = node_.lock()->get_clock()->now();
  double time_since_last_plan = (current_time - last_plan_time_).seconds();
  
  // 检查是否需要重新规划
  bool need_replan = prev_path_.poses.empty();
  
  if (!prev_path_.poses.empty()) {
    // 计算新目标与旧目标的距离
    double goal_change = std::hypot(
      goal.pose.position.x - prev_goal_.pose.position.x,
      goal.pose.position.y - prev_goal_.pose.position.y);
      
    // 计算当前位置与路径的偏差
    double min_start_dev = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    for (size_t i = 0; i < prev_path_.poses.size(); ++i) {
      double dev = std::hypot(
        start.pose.position.x - prev_path_.poses[i].pose.position.x,
        start.pose.position.y - prev_path_.poses[i].pose.position.y);
      if (dev < min_start_dev) {
        min_start_dev = dev;
        closest_idx = i;
      }
    }
    
    // 检查前方路径质量
    bool path_ahead_valid = (closest_idx < prev_path_.poses.size() - 1);
    
    // 提高重规划门槛：只有在以下情况才重新规划
    need_replan = time_since_last_plan > path_stability_timeout_ ||
                 goal_change > 1.0 || 
                 min_start_dev > 1.2 ||
                 !path_ahead_valid;
    
    if (need_replan) {
      RCLCPP_INFO(logger_, "Replanning needed: timeout=%.1fs, goal_change=%.2fm, path_deviation=%.2fm",
                 time_since_last_plan, goal_change, min_start_dev);
    }
  }
  
  // 如果不需要重新规划，继续使用当前路径
  if (!need_replan) {
    RCLCPP_INFO(logger_, "Maintaining existing path for stability (age: %.1fs)", time_since_last_plan);
    return prev_path_;
  }
  
  // 更新历史路径
  last_plan_time_ = current_time;
  prev_goal_ = goal;
  prev_path_ = path;
  
  return path;
}

nav_msgs::msg::Path CustomPlanner::handleLargeAngleTarget(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  
  // 提取起点和目标数据
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double start_yaw = tf2::getYaw(start.pose.orientation);
  
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  
  // 计算从起点到目标的方向和距离
  double dx = goal_x - start_x;
  double dy = goal_y - start_y;
  double target_angle = std::atan2(dy, dx);
  double distance = std::hypot(dx, dy);
  
  // 计算角度差
  double angle_diff = normalizeAngle(target_angle - start_yaw);
  double abs_angle_diff = std::abs(angle_diff);
  
  // 如果双弧线路径失败或角度差太大，才考虑后退路径
  
  // 选择后退策略的初始调整距离 - 减小后退距离
  double reverse_distance;
  
  // 角度越接近90度，需要更大的后退距离
  if (abs_angle_diff > 1.57) { // 90度
    // 近90度，使用中等后退距离
    reverse_distance = std::max(robot_length_ * 1.0, min_turning_radius_ * 1.5);
  } else {
    // 其他角度，使用较小后退距离
    reverse_distance = std::max(robot_length_ * 0.8, min_turning_radius_ * 1.2);
  }
  
  // 确保后退距离不超过到目标的距离的一半
  reverse_distance = std::min(reverse_distance, distance * 0.5);
  
  RCLCPP_INFO(logger_, "Planning with angle diff %.1f deg, using reverse distance %.2f m", 
            abs_angle_diff * 180.0/M_PI, reverse_distance);
  
  // 创建后退点
  geometry_msgs::msg::PoseStamped reverse_point;
  reverse_point.header = start.header;
  reverse_point.pose.position.x = start_x - reverse_distance * std::cos(start_yaw);
  reverse_point.pose.position.y = start_y - reverse_distance * std::sin(start_yaw);
  
  // 后退点的朝向保持与起点相同
  reverse_point.pose.orientation = start.pose.orientation;
  
  // 检查后退路径是否无碰撞
  if (!isLineCollisionFree(start, reverse_point, obstacle_clearance_)) {
    RCLCPP_WARN(logger_, "Cannot reverse safely, trying alternate angles");
    
    // 尝试不同角度的后退方向
    bool found_safe_path = false;
    for (double angle_offset : {0.2, -0.2, 0.4, -0.4, 0.6, -0.6}) {
      double new_reverse_angle = start_yaw + angle_offset;
      
      reverse_point.pose.position.x = start_x - reverse_distance * std::cos(new_reverse_angle);
      reverse_point.pose.position.y = start_y - reverse_distance * std::sin(new_reverse_angle);
      
      // 更新后退点朝向
      tf2::Quaternion q;
      q.setRPY(0, 0, new_reverse_angle);
      reverse_point.pose.orientation.x = q.x();
      reverse_point.pose.orientation.y = q.y();
      reverse_point.pose.orientation.z = q.z();
      reverse_point.pose.orientation.w = q.w();
      
      if (isLineCollisionFree(start, reverse_point, obstacle_clearance_)) {
        found_safe_path = true;
        RCLCPP_INFO(logger_, "Found alternative reverse angle at %.1f deg", angle_offset * 180.0/M_PI);
        break;
      }
    }
    
    if (!found_safe_path) {
      // 如果无法找到安全的后退路径，尝试减小后退距离
      reverse_distance *= 0.6;
      reverse_point.pose.position.x = start_x - reverse_distance * std::cos(start_yaw);
      reverse_point.pose.position.y = start_y - reverse_distance * std::sin(start_yaw);
      
      if (!isLineCollisionFree(start, reverse_point, obstacle_clearance_)) {
        RCLCPP_ERROR(logger_, "Cannot find safe reverse path");
        return nav_msgs::msg::Path(); // 返回空路径
      }
    }
  }
  
  // 从后退点计算到目标的方向
  double to_goal_angle = std::atan2(goal_y - reverse_point.pose.position.y,
                                   goal_x - reverse_point.pose.position.x);
  
  // 创建朝向目标的点
  geometry_msgs::msg::PoseStamped facing_point = reverse_point;
  
  // 设置朝向目标的方向
  tf2::Quaternion qfacing;
  qfacing.setRPY(0, 0, to_goal_angle);
  facing_point.pose.orientation.x = qfacing.x();
  facing_point.pose.orientation.y = qfacing.y();
  facing_point.pose.orientation.z = qfacing.z();
  facing_point.pose.orientation.w = qfacing.w();
  
  // 从朝向目标的点到目标创建路径
  nav_msgs::msg::Path final_path;
  
  // 如果角度接近90度，使用Dubins路径更合适
  if (abs_angle_diff > 60.0 * M_PI/180.0) {
    final_path = createDubinsPath(facing_point, goal);
  } else {
    final_path = findPathAroundObstacles(facing_point, goal);
  }
  
  // 如果路径生成失败，尝试另一种方法
  if (final_path.poses.empty() || !isPathCollisionFree(final_path)) {
    RCLCPP_WARN(logger_, "Primary path planning failed, trying alternative");
    if (abs_angle_diff > 60.0 * M_PI/180.0) {
      final_path = findPathAroundObstacles(facing_point, goal);
    } else {
      final_path = createDubinsPath(facing_point, goal);
    }
  }
  
  if (final_path.poses.empty()) {
    RCLCPP_ERROR(logger_, "All path planning methods failed");
    return nav_msgs::msg::Path();
  }
  
  // 构建完整路径: 起点 -> 后退点 -> 朝向调整点 -> 最终路径
  path.poses.push_back(start);
  
  // 添加后退路径的中间点
  double segment_dist = std::hypot(reverse_point.pose.position.x - start.pose.position.x,
                                  reverse_point.pose.position.y - start.pose.position.y);
  
  if (segment_dist > path_resolution_ * 2.0) {
    int steps = std::max(2, static_cast<int>(segment_dist / path_resolution_));
    for (int i = 1; i < steps; i++) {
      double ratio = static_cast<double>(i) / steps;
      geometry_msgs::msg::PoseStamped interp_pose;
      interp_pose.header = start.header;
      interp_pose.pose.position.x = start_x + ratio * (reverse_point.pose.position.x - start_x);
      interp_pose.pose.position.y = start_y + ratio * (reverse_point.pose.position.y - start_y);
      interp_pose.pose.orientation = start.pose.orientation;
      path.poses.push_back(interp_pose);
    }
  }
  
  path.poses.push_back(reverse_point);
  
  // 如果转向角度较大，添加一个明确的转向帧
  if (std::abs(normalizeAngle(to_goal_angle - tf2::getYaw(reverse_point.pose.orientation))) > 0.35) {
    path.poses.push_back(facing_point);
  }
  
  // 添加最终路径，跳过第一个点（因为已经添加了facing_point）
  for (size_t i = 1; i < final_path.poses.size(); ++i) {
    path.poses.push_back(final_path.poses[i]);
  }
  
  // 确保最后一个点是目标点
  if (!path.poses.empty()) {
    path.poses.back() = goal;
  }
  
  RCLCPP_INFO(logger_, "Created maneuver path with %zu points for large angle target", path.poses.size());
  
  return path;
}

std::vector<geometry_msgs::msg::PoseStamped> CustomPlanner::generateDenseArcPoints(
  double cx, double cy, double radius, 
  double start_angle, double end_angle,
  const std_msgs::msg::Header & header,
  int num_points)
{
  std::vector<geometry_msgs::msg::PoseStamped> arc_points;
  
  // 标准化角度差以取最短路径
  double angle_diff = normalizeAngle(end_angle - start_angle);
  
  // 生成圆弧上的点 - 使用平滑的速度分布
  for (int i = 0; i < num_points; ++i) {
    // 使用S型曲线进行平滑过渡，使起始和结束更加平缓
    double base_t = static_cast<double>(i) / static_cast<double>(num_points - 1);
    double t;
    
    // 使用改进的平滑S型曲线函数，在弧线的开始和结束处更加平滑
    if (base_t < 0.5) {
      t = 0.5 * std::pow(2.0 * base_t, 2);
    } else {
      t = 1.0 - 0.5 * std::pow(2.0 * (1.0 - base_t), 2);
    }
    
    double angle = start_angle + t * angle_diff;
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = cx + radius * std::cos(angle);
    pose.pose.position.y = cy + radius *std::sin(angle);
    
    // 切线方向作为朝向
    double tangent_angle = angle + M_PI/2.0;
    if (angle_diff < 0) {
      tangent_angle = angle - M_PI/2.0;
    }
    
    tf2::Quaternion q;
    q.setRPY(0, 0, tangent_angle);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    arc_points.push_back(pose);
  }
  
  return arc_points;
}

nav_msgs::msg::Path CustomPlanner::createReversePathToTarget(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  
  // 提取起点和目标数据
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double start_yaw = tf2::getYaw(start.pose.orientation);
  
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  
  // 计算从起点到目标的方向和距离
  double dx = goal_x - start_x;
  double dy = goal_y - start_y;
  double distance = std::hypot(dx, dy);
  
  // 使用更宽松的障碍物检查 - 对于窄走廊专门降低清除距离
  double corridor_clearance = std::max(0.05, obstacle_clearance_ * 0.4);  // 大幅降低清除距离
  
  // 检查路径是否安全，使用宽松的清除要求
  if (!isLineCollisionFree(start, goal, corridor_clearance)) {
    // 尝试进一步降低清除要求
    corridor_clearance = std::max(0.01, obstacle_clearance_ * 0.2);  // 进一步降低清除距离
    if (!isLineCollisionFree(start, goal, corridor_clearance)) {
      RCLCPP_WARN(logger_, "Direct reverse path has obstacles, trying other methods");
      return nav_msgs::msg::Path(); // 返回空路径
    }
  }
  
  // 添加起点
  path.poses.push_back(start);
  
  // 根据路径长度添加中间点
  int steps = std::max(2, static_cast<int>(distance / path_resolution_));
  for (int i = 1; i < steps; i++) {
    double ratio = static_cast<double>(i) / static_cast<double>(steps);
    geometry_msgs::msg::PoseStamped interp_pose;
    interp_pose.header = start.header;
    interp_pose.pose.position.x = start_x + ratio * dx;
    interp_pose.pose.position.y = start_y + ratio * dy;
    
    // 中间点的朝向与起点相同但反向180度
    tf2::Quaternion q;
    // 倒车时车头方向与车辆前进方向相反
    q.setRPY(0, 0, normalizeAngle(start_yaw + M_PI));
    interp_pose.pose.orientation.x = q.x();
    interp_pose.pose.orientation.y = q.y();
    interp_pose.pose.orientation.z = q.z();
    interp_pose.pose.orientation.w = q.w();
    
    path.poses.push_back(interp_pose);
  }
  
  // 添加终点 - 确保使用指定的目标朝向
  path.poses.push_back(goal);
  
  RCLCPP_INFO(logger_, "Created direct reverse path with %zu points", path.poses.size());
  
  return path;
}

std::vector<std::pair<int, int>> CustomPlanner::optimizePath(
  const std::vector<std::pair<int, int>> & path,
  const std::vector<std::vector<int>> & grid)
{
  if (path.size() <= 3) {
    return path;
  }
  
  std::vector<std::pair<int, int>> optimized_path;
  optimized_path.push_back(path[0]);  // 添加起点
  
  // 确保开始区域安全性
  unsigned int safe_radius_cells = static_cast<int>(obstacle_clearance_ / costmap_->getResolution());
  
  size_t i = 0;
  while (i < path.size() - 1) {
    size_t next_point = i + 1;
    
    // 在长路径段中寻找可能的直接路径
    for (size_t j = i + 2; j < path.size(); ++j) {
      // 检查当前点与较远点之间是否有直接无碰撞路径
      if (hasLineOfSight(path[i], path[j], grid, safe_radius_cells)) {
        // 检查是否使路径更直
        if (j > i + 2) {
          next_point = j;
        }
      } else {
        // 一旦找不到直接路径，停止向前查找
        break;
      }
    }
    
    // 添加下一个点到优化路径
    optimized_path.push_back(path[next_point]);
    i = next_point;
  }
  
  // 确保终点被添加
  if (optimized_path.back() != path.back()) {
    optimized_path.push_back(path.back());
  }
  
  return optimized_path;
}

bool CustomPlanner::verifyPathQuality(
  const nav_msgs::msg::Path & path, 
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (path.poses.size() < 3) {
    return true; // 路径太短，无需验证
  }
  
  // 计算直线距离和路径长度
  double direct_distance = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
    
  double path_length = 0.0;
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    path_length += std::hypot(
      path.poses[i+1].pose.position.x - path.poses[i].pose.position.x,
      path.poses[i+1].pose.position.y - path.poses[i].pose.position.y);
  }
  
  // 如果路径长度超过直线距离太多，拒绝此路径
  // 允许的最大比例为直线距离的2.5倍
  const double max_length_ratio = 2.5;
  if (path_length > direct_distance * max_length_ratio) {
    RCLCPP_WARN(logger_, "Rejecting path: length (%.2f) too large compared to direct distance (%.2f)", 
               path_length, direct_distance);
    return false;
  }
  
  // 检测急转弯数量
  int sharp_turns = 0;
  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    double angle1 = std::atan2(
      path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,
      path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
      
    double angle2 = std::atan2(
      path.poses[i+1].pose.position.y - path.poses[i].pose.position.y,
      path.poses[i+1].pose.position.x - path.poses[i].pose.position.x);
      
    double turn_angle = std::abs(normalizeAngle(angle2 - angle1));
    
    // 统计大于30度的转弯
    if (turn_angle > 0.52) {
      sharp_turns++;
    }
  }
  
  // 如果急转弯数量过多，拒绝此路径
  // 每10米路径允许的最大急转弯数量
  const double max_turns_per_10m = 2.0;
  const double max_allowed_turns = std::max(1.0, path_length * max_turns_per_10m / 10.0);
  
  if (sharp_turns > max_allowed_turns) {
    RCLCPP_WARN(logger_, "Rejecting path: too many sharp turns (%d) for path length (%.2f)", 
               sharp_turns, path_length);
    return false;
  }
  
  return true;
}

nav_msgs::msg::Path CustomPlanner::createDualArcPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  
  // 提取位置和朝向
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double start_yaw = tf2::getYaw(start.pose.orientation);
  
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  
  // 距离和方向
  double dx = goal_x - start_x;
  double dy = goal_y - start_y;
  // Save distance for error checking
  double path_distance = std::hypot(dx, dy);
  double direct_angle = std::atan2(dy, dx);
  
  // 角度差
  double start_angle_diff = normalizeAngle(direct_angle - start_yaw);
  double goal_angle_diff = normalizeAngle(goal_yaw - direct_angle);
  
  // 确定转弯方向 (左转为正，右转为负)
  double start_turn_dir = (start_angle_diff > 0) ? 1.0 : -1.0;
  double goal_turn_dir = (goal_angle_diff > 0) ? 1.0 : -1.0;
  
  // 使用较大的转弯半径以确保平滑
  double turn_radius = min_turning_radius_ * 1.5;
  
  // 计算连接点 - 使用两个圆弧相接的方式
  // 第一个圆的中心
  double c1_x = start_x - turn_radius * std::sin(start_yaw) * start_turn_dir;
  double c1_y = start_y + turn_radius * std::cos(start_yaw) * start_turn_dir;
  
  // 第二个圆的中心
  double c2_x = goal_x - turn_radius * std::sin(goal_yaw) * goal_turn_dir;
  double c2_y = goal_y + turn_radius * std::cos(goal_yaw) * goal_turn_dir;
  
  // 两个圆心之间的距离
  double centers_dist = std::hypot(c2_x - c1_x, c2_y - c1_y);
  
  // 如果圆心距离小于两倍半径，无法构建有效的双弧线路径
  if (centers_dist < turn_radius * 0.8) {
    RCLCPP_WARN(logger_, "Cannot create dual-arc path: circle centers too close");
    return path; // 返回空路径
  }
  
  // 计算两个圆心连线的方向
  double centers_angle = std::atan2(c2_y - c1_y, c2_x - c1_x);
  
  // 计算两个弧的连接点 - 存储后面会使用
  double connection_x = (c1_x + c2_x) / 2.0;
  double connection_y = (c1_y + c2_y) / 2.0;
  
  // 为短路径显示警告
  if (path_distance < turn_radius * 2.0) {
    RCLCPP_WARN(logger_, "Path distance (%.2f) may be too short for smooth dual-arc path with radius %.2f", 
               path_distance, turn_radius);
  }
  
  // 计算连接角度
  double connect_angle1 = normalizeAngle(centers_angle + M_PI_2 * start_turn_dir);
  double connect_angle2 = normalizeAngle(centers_angle - M_PI_2 * goal_turn_dir);
  
  // 添加起点
  path.poses.push_back(start);
  
  // 生成第一段弧线
  auto arc1 = generateArcPoints(
    c1_x, c1_y, turn_radius,
    normalizeAngle(start_yaw + M_PI_2 * start_turn_dir),
    connect_angle1,
    start.header);
  
  // 添加第一段弧线但跳过第一个点(已经添加)
  for (size_t i = 1; i < arc1.size(); ++i) {
    path.poses.push_back(arc1[i]);
  }
  
  // 生成第二段弧线
  auto arc2 = generateArcPoints(
    c2_x, c2_y, turn_radius,
    connect_angle2,
    normalizeAngle(goal_yaw - M_PI_2 * goal_turn_dir),
    start.header);
  
  // 添加第二段弧线
  for (size_t i = 0; i < arc2.size(); ++i) {
    path.poses.push_back(arc2[i]);
  }
  
  // 确保终点使用正确的朝向
  path.poses.back() = goal;
  
  return path;
}

bool CustomPlanner::isPointInCollision(double wx, double wy)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return true;  // Outside the map is considered collision
  }
  unsigned char cost = costmap_->getCost(mx, my);
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

bool CustomPlanner::isPointSafe(double wx, double wy, double buffer)
{
  // 如果点本身在障碍物中，则不安全
  if (isPointInCollision(wx, wy)) {
    return false;
  }
  
  // 如果不需要缓冲区，仍执行基本的安全检查
  if (buffer <= 0.0) {
    buffer = robot_width_ * 0.2; // 确保至少有一个最小的安全距离
  }
  
  // 确保缓冲区至少包含机器人的半宽/半长再加上额外的安全边距
  double robot_radius = std::max(robot_width_, robot_length_) / 2.0;
  double safety_margin = 0.15; // 增加额外的安全边距
  double effective_buffer = std::max(buffer, robot_radius + safety_margin);
  
  // 在点周围的圆形区域内检查点，半径 = 有效缓冲区
  int steps = 24;  // 增加采样点数以获得更精确的检查
  for (int i = 0; i < steps; ++i) {
    double angle = 2.0 * M_PI * i / steps;
    double bx = wx + effective_buffer * std::cos(angle);
    double by = wy + effective_buffer * std::sin(angle);
    if (isPointInCollision(bx, by)) {
      return false;  // 在缓冲区内发现障碍物
    }
  }
  
  // 额外检查机器人对角线方向
  double diagonal_length = std::sqrt(robot_width_*robot_width_ + robot_length_*robot_length_) / 2.0 + 0.1;
  for (int i = 0; i < 4; i++) {
    double angle = i * M_PI_2 + M_PI_4; // 对角线方向
    double bx = wx + diagonal_length * std::cos(angle);
    double by = wy + diagonal_length * std::sin(angle);
    if (isPointInCollision(bx, by)) {
      return false;  // 对角线方向发现障碍物
    }
  }
  
  return true;
}

bool CustomPlanner::isLineCollisionFree(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double buffer)
{
  double x0 = start.pose.position.x;
  double y0 = start.pose.position.y;
  double x1 = end.pose.position.x;
  double y1 = end.pose.position.y;
  
  // 计算距离和方向
  double dx = x1 - x0;
  double dy = y1 - y0;
  double distance = std::sqrt(dx * dx + dy * dy);
  double path_angle = std::atan2(dy, dx);
  double perp_angle = path_angle + M_PI_2;
  
  // 应用安全因子，最小缓冲区设为0.12m
  buffer = std::max(buffer, 0.12);
  buffer *= obstacle_check_safety_factor_;
  
  // 合理的检查点数量
  int steps = std::max(collision_check_points_, 
                     static_cast<int>(distance / (path_resolution_ * 0.8))); // 增加检查频率
  
  // 限制步数以避免性能问题，但确保足够的检查
  steps = std::min(steps, 150); // 增加最大检查点数
  
  // 考虑车辆宽度进行垂直安全检查的距离
  double side_check_dist = std::max(robot_width_ * 0.55, 0.25); // 增加侧向安全检查距离
  
  // 检查沿线的点
  for (int i = 0; i <= steps; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(steps);
    double x = x0 + ratio * dx;
    double y = y0 + ratio * dy;
    
    // 每个点的基本碰撞检查
    if (!isPointSafe(x, y, buffer)) {
      return false;
    }
    
    // 垂直于路径方向的额外检查，考虑机器人宽度
    if (i % 3 == 0) { // 每三个点检查一次侧面
      for (double mult : {-1.0, 1.0}) {
        double side_x = x + mult * side_check_dist * std::cos(perp_angle);
        double side_y = y + mult * side_check_dist * std::sin(perp_angle);
        
        if (!isPointSafe(side_x, side_y, buffer * 0.7)) { // 侧面使用略小的缓冲区
          return false; // 侧面检测到可能碰撞
        }
      }
    }
  }
  
  return true; // 未检测到显著碰撞
}

nav_msgs::msg::Path CustomPlanner::createStraightPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  
  double x0 = start.pose.position.x;
  double y0 = start.pose.position.y;
  double x1 = goal.pose.position.x;
  double y1 = goal.pose.position.y;
  
  // Calculate the heading
  double dx = x1 - x0;
  double dy = y1 - y0;
  double distance = std::sqrt(dx * dx + dy * dy);
  double yaw = std::atan2(dy, dx);
  
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  
  // Number of points to add to the path
  int steps = std::max(1, static_cast<int>(distance / path_resolution_));
  for (int i = 0; i <= steps; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(steps);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;
    pose.pose.position.x = x0 + ratio * dx;
    pose.pose.position.y = y0 + ratio * dy;
    
    // Set orientation
    if (i < steps) {
      // Path points have orientation in the direction of travel
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    } else {
      // Last point uses the goal orientation
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  }
  return path;
}

std::vector<std::vector<double>> CustomPlanner::generateDistanceField(
  const std::vector<std::vector<int>> & grid)
{
  int width = grid.size();
  int height = grid[0].size();
  
  // Initialize distance field with infinity
  std::vector<std::vector<double>> distance_field(
    width, std::vector<double>(height, std::numeric_limits<double>::infinity()));
  
  // Queue for BFS
  std::queue<std::pair<int, int>> q;
  
  // Add all obstacle cells to the queue with distance 0
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      if (grid[x][y] == 1) {  // Obstacle
        distance_field[x][y] = 0.0;
        q.push({x, y});
      }
    }
  }
  
  // Directions for BFS (4-connected)
  int dx[4] = {-1, 0, 1, 0};
  int dy[4] = {0, -1, 0, 1};
  
  // BFS to compute distances
  while (!q.empty()) {
    auto [x, y] = q.front();
    q.pop();
    
    for (int i = 0; i < 4; ++i) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      
      if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
        double new_dist = distance_field[x][y] + 1.0;
        if (new_dist < distance_field[nx][ny]) {
          distance_field[nx][ny] = new_dist;
          q.push({nx, ny});
        }
      }
    }
  }
  
  // Convert distance field to meters
  double resolution = costmap_->getResolution();
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      if (distance_field[x][y] != std::numeric_limits<double>::infinity()) {
        distance_field[x][y] *= resolution;
      } else {
        // Set a maximum distance for free cells far from obstacles
        distance_field[x][y] = 10.0 * resolution; // 10 cells away
      }
    }
  }
  return distance_field;
}

double CustomPlanner::calculateDirection(int x1, int y1, int x2, int y2)
{
  return std::atan2(y2 - y1, x2 - x1);
}

double CustomPlanner::calculateTurningPenalty(
  int x1, int y1, int x2, int y2, int x3, int y3)
{
  if (x1 == x2 && y1 == y2) return 0.0;
  if (x2 == x3 && y2 == y3) return 0.0;
  
  double dir1 = calculateDirection(x1, y1, x2, y2);
  double dir2 = calculateDirection(x2, y2, x3, y3);
  
  // Calculate the absolute angle difference (in radians)
  double angle_diff = std::abs(normalizeAngle(dir2 - dir1));
  
  // Normalize to [0, pi]
  if (angle_diff > M_PI) {
    angle_diff = 2.0 * M_PI - angle_diff;
  }
  
  // Enhanced turning penalty with exponential increase for larger angles
  // This will strongly discourage large angle changes and unnecessary turns
  double penalty = straight_line_preference_;
  
  // Use exponential penalty for angles above a threshold
  double angle_threshold = 0.15; // 约8.6度
  if (angle_diff > angle_threshold) {
    // For small angles, linear penalty
    penalty = straight_line_preference_ * angle_diff;
    
    // For moderate angles, quadratic penalty
    if (angle_diff > 0.3) { // 约17度
      penalty = straight_line_preference_ * (angle_diff * angle_diff * 2.0);
    }
    
    // For large angles, exponential penalty
    if (angle_diff > 0.6) { // 约34度
      penalty = straight_line_preference_ * (std::exp(angle_diff * 2.0) - 1.0);
    }
  }
  
  return penalty;
}

bool CustomPlanner::hasLineOfSight(
  const std::pair<int, int> & start,
  const std::pair<int, int> & end,
  const std::vector<std::vector<int>> & grid,
  double buffer)
{
  int x0 = start.first;
  int y0 = start.second;
  int x1 = end.first;
  int y1 = end.second;
  
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (x0 < x1) ? 1 : -1;
  int err = dx - dy;
  
  // Convert buffer from meters to grid cells
  int buffer_cells = static_cast<int>(buffer / costmap_->getResolution());
  if (buffer_cells < 0) buffer_cells = 0;
  
  while (x0 != x1 || y0 != y1) {
    if (x0 < 0 || x0 >= static_cast<int>(grid.size()) || 
        y0 < 0 || y0 >= static_cast<int>(grid[0].size())) {
      return false;  // Out of bounds
    }
    
    // Check if current cell is an obstacle
    if (grid[x0][y0] == 1) {
      return false;
    }
    
    // Check cells within buffer distance
    if (buffer_cells > 0) {
      for (int bx = -buffer_cells; bx <= buffer_cells; ++bx) {
        for (int by = -buffer_cells; by <= buffer_cells;++by) {
          int cx = x0 + bx;
          int cy = y0 + by;
          
          // Skip cells outside the buffer circle
          if (bx*bx + by*by > buffer_cells*buffer_cells) continue;
          
          // Check if buffer cell is in bounds and is an obstacle
          if (cx >= 0 && cx < static_cast<int>(grid.size()) &&
              cy >= 0 && cy < static_cast<int>(grid[0].size()) &&
              grid[cx][cy] == 1) {
            return false;  // Found obstacle within buffer
          }
        }
      }
    }
    
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
  return true;
}

// Improve path simplification to reduce unnecessary turns
std::vector<std::pair<int, int>> CustomPlanner::simplifyPath(
  const std::vector<std::pair<int, int>> & path,
  const std::vector<std::vector<int>> & grid)
{
  if (path.size() <= 2) {
    return path;  // No simplification needed
  }
  
  // 增加更激进的距离参数，减少转弯点的数量
  double robot_radius = std::max(robot_width_, robot_length_) / 2.0;
  double buffer = std::max(obstacle_clearance_ * 0.8, robot_radius + 0.03);  // 降低安全裕度以减少转弯
  double buffer_grid_cells = buffer / costmap_->getResolution();
  
  std::vector<std::pair<int, int>> simplified;
  simplified.push_back(path[0]);  // 添加起点
  
  size_t i = 0;
  while (i < path.size() - 1) {
    // 对于起点，尝试直接连接到更远的点，以减少初始路径段的转弯
    size_t lookahead = (i == 0) ? 4 : 2;  // 对起点使用更远的直接视线检查
    
    size_t j = i + 1;
    size_t furthest = j;
    
    // 找到具有直接视线的最远点
    while (j < path.size() - 1 && j < i + 15) {  // 限制向前查找的范围但增加到15
      if (hasLineOfSight(path[i], path[j + 1], grid, buffer_grid_cells)) {
        furthest = j + 1;
        
        // 如果已经看到了足够远的点，可以提前跳出以节省计算
        if (j > i + lookahead) {
          break;
        }
      }
      j++;
    }
    
    // 检查跳跃点的角度 - 避免引入锐角转弯
    if (i > 0 && furthest < path.size() - 1) {
      // 计算之前的方向
      double prev_dir = std::atan2(
        path[i].second - path[i-1].second,
        path[i].first - path[i-1].first);
      
      // 计算新的潜在方向
      double new_dir = std::atan2(
        path[furthest].second - path[i].second,
        path[furthest].first - path[i].first);
      
      // 如果角度变化太大，尝试找到一个更平滑的中间点
      double angle_diff = std::abs(normalizeAngle(new_dir - prev_dir));
      if (angle_diff > 0.6) {  // 超过约34度
        // 检查中间点以获得更平滑的过渡
        for (size_t k = i+1; k < furthest; k++) {
          double mid_dir = std::atan2(
            path[k].second - path[i].second,
            path[k].first - path[i].first);
          
          if (std::abs(normalizeAngle(mid_dir - prev_dir)) < angle_diff * 0.6) {
            // 找到更好的平滑过渡点
            furthest = k;
            break;
          }
        }
      }
    }
    
    // 添加最远可见点
    simplified.push_back(path[furthest]);
    i = furthest;
  }
  
  return simplified;
}

nav_msgs::msg::Path CustomPlanner::findPathAroundObstacles(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // Initialize path
  nav_msgs::msg::Path path;
  path.header = start.header;

  // Convert start and goal to map coordinates
  unsigned int start_mx, start_my;
  unsigned int goal_mx, goal_my;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_mx, start_my) ||
      !costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
  {
    RCLCPP_WARN(
      logger_, "Start or goal is outside the map, cannot create a path.");
    return path;
  }

  // Check if the path is in a narrow corridor by sampling points along the direct line
  double corridor_width_check = 2.5 * inflation_radius_; // 增加检查宽度以更好地识别走廊
  double path_dx = goal.pose.position.x - start.pose.position.x;
  double path_dy = goal.pose.position.y - start.pose.position.y;
  double path_length = std::hypot(path_dx, path_dy);
  double path_angle = std::atan2(path_dy, path_dx);
  double side_angle = path_angle + M_PI_2; // 垂直于路径的方向
  
  bool is_narrow_corridor = false;
  
  // 只有当路径足够长时才检查走廊情况
  if (path_length > robot_length_ * 1.2) { // 减小最小路径长度阈值以更容易识别走廊
    is_narrow_corridor = true;
    int num_samples = std::min(12, static_cast<int>(path_length / 0.4) + 1); // 增加采样点数量
    double perpendicular_check_dist = std::max(robot_width_ * 1.8, 0.5); // 增加检查距离以更好地判断通道宽度
    
    // 沿路径采样
    for (int i = 0; i < num_samples; i++) {
      double ratio = static_cast<double>(i) / static_cast<double>(num_samples - 1);
      double x = start.pose.position.x + ratio * path_dx;
      double y = start.pose.position.y + ratio * path_dy;
      
      // 检查垂直方向上是否有障碍物 - 走廊检测
      bool has_obstacles_both_sides = true;
      for (double mult : {-1.0, 1.0}) {
        double check_x = x + mult * perpendicular_check_dist * std::cos(side_angle);
        double check_y = y + mult * perpendicular_check_dist * std::sin(side_angle);
        
        unsigned int mx, my;
        if (costmap_->worldToMap(check_x, check_y, mx, my)) {
          unsigned char cost = costmap_->getCost(mx, my);
          // 降低障碍物判定阈值，更容易识别出潜在障碍物
          if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE * 0.75) {
            has_obstacles_both_sides = false;
            break;
          }
        }
      }
      
      // 增加额外安全检查 - 检查多个点来确保完整性
      if (!has_obstacles_both_sides) {
        // 做二次验证检查 - 检查额外的点以提高准确性
        bool confirmed_open = true;
        
        for (double extra_dist = 0.2; extra_dist <= perpendicular_check_dist * 0.8; extra_dist += 0.2) {
          for (double mult : {-1.0, 1.0}) {
            double check_x = x + mult * extra_dist * std::cos(side_angle);
            double check_y = y + mult * extra_dist * std::sin(side_angle);
            
            if (isPointInCollision(check_x, check_y)) {
              confirmed_open = false;
              break;
            }
          }
          if (!confirmed_open) break;
        }
        
        if (confirmed_open) {
          is_narrow_corridor = false;
          break;
        }
      }
    }
  }

  if (is_narrow_corridor) {
    RCLCPP_INFO(logger_, "Narrow corridor detected, generating reverse path.");
    
    // 使用减小的清除距离生成走廊中的路径
    double original_clearance = obstacle_clearance_;
    obstacle_clearance_ = std::max(0.15, obstacle_clearance_ * 0.6); // 增加最小安全距离，从0.1增加到0.15
    
    nav_msgs::msg::Path reverse_path = createReversePathToTarget(start, goal);
    
    // Restore original clearance
    obstacle_clearance_ = original_clearance;
    
    if (!reverse_path.poses.empty()) {
      return reverse_path;
    }
    // If reverse path creation failed, fall back to regular planning
  }

  // Create grid for A* search
  // 0: free space, 1: obstacle
  int map_width = costmap_->getSizeInCellsX();
  int map_height = costmap_->getSizeInCellsY();
  std::vector<std::vector<int>> grid(
    map_width, std::vector<int>(map_height, 0));
  
  // 考虑机器人尺寸计算膨胀半径（单位：网格单元）
  double robot_max_dim = std::max(robot_width_, robot_length_);
  double safety_margin = 0.18;  // 从0.1增加到0.18的额外安全裕度
  int inflation_radius_cells = static_cast<int>((robot_max_dim/2.0 + safety_margin) / costmap_->getResolution());
  RCLCPP_INFO(logger_, "Using inflation radius of %d cells (%.2f meters) for path planning",  
             inflation_radius_cells, inflation_radius_cells * costmap_->getResolution());
  
  // 膨胀半径不应小于指定的obstacle_clearance_
  int clearance_cells = static_cast<int>(obstacle_clearance_ / costmap_->getResolution());
  inflation_radius_cells = std::max(inflation_radius_cells, clearance_cells);
  
  // 对膨胀区域应用额外的安全系数
  inflation_radius_cells = static_cast<int>(inflation_radius_cells * 1.15); // 增加15%的膨胀半径
  
  // 填充网格并膨胀障碍物
  std::vector<std::vector<int>> inflated_grid(
    map_width, std::vector<int>(map_height, 0));
  for (int x = 0; x < map_width; ++x) {
    for (int y = 0; y < map_height; ++y) {
      unsigned char cost = costmap_->getCost(x, y);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        grid[x][y] = 1;  // 标记为障碍物
        
        // 膨胀障碍物以考虑机器人尺寸和安全距离
        for (int nx = std::max(0, x - inflation_radius_cells); 
             nx < std::min(map_width, x + inflation_radius_cells + 1); ++nx) {
          for (int ny = std::max(0, y - inflation_radius_cells); 
               ny < std::min(map_height, y + inflation_radius_cells + 1); ++ny) {
            // 计算到障碍物的距离
            double dist = std::hypot(nx - x, ny - y);
            if (dist <= inflation_radius_cells) {
              inflated_grid[nx][ny] = 1;
            }
          }
        }
      }
    }
  }
  
  // 确保起点和终点不在膨胀的障碍物中
  inflated_grid[start_mx][start_my] = 0;
  inflated_grid[goal_mx][goal_my] = 0;
  
  // 创建成本场，考虑距离障碍物的距离
  std::vector<std::vector<double>> cost_field(
    map_width, std::vector<double>(map_height, 1.0));
  
  // 根据与障碍物的距离设置成本
  for (int x = 0; x < map_width; ++x) {
    for (int y = 0; y < map_height; ++y) {
      if (grid[x][y] == 1) {
        // 障碍物成本最高
        cost_field[x][y] = std::numeric_limits<double>::max();
      } else if (inflated_grid[x][y] == 1) {
        // 膨胀区域也有高成本，但可以在必要时通过
        cost_field[x][y] = 100.0;
      } else {
        // 在障碍物周围创建一个梯度成本场
        int min_dist_to_obstacle = inflation_radius_cells + 1;
        // 在一定范围内搜索最近的障碍物
        int search_radius = inflation_radius_cells * 2;
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
          for (int dy = -search_radius; dy <= search_radius; ++dy) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height && grid[nx][ny] == 1) {
              int dist = std::abs(dx) + std::abs(dy);  // Manhattan distance
              min_dist_to_obstacle = std::min(min_dist_to_obstacle, dist);
            }
          }
        }
        // 基于到障碍物的距离设置成本 - 距离越近，成本越高
        if (min_dist_to_obstacle <= inflation_radius_cells * 2) {
          double distance_factor = static_cast<double>(min_dist_to_obstacle) / (inflation_radius_cells * 2);
          cost_field[x][y] = 1.0 + 10.0 * (1.0 - distance_factor);
        }
      }
    }
  }
  
  // Generate distance field from obstacles
  std::vector<std::vector<double>> distance_field = generateDistanceField(grid);
  
  // Define node for enhanced A*
  struct Node {
    int x, y;              // Grid coordinates
    double g, h, f;        // Costs
    int parent_x, parent_y;// Parent coordinates
    int prev_x, prev_y;    // Previous waypoint (for direction calculation)
    Node(int _x, int _y, double _g, double _h, int _px, int _py, int _prev_x, int _prev_y)
    : x(_x), y(_y), g(_g), h(_h), f(_g + _h), 
      parent_x(_px), parent_y(_py), prev_x(_prev_x), prev_y(_prev_y) {}
    bool operator<(const Node & other) const {
      return f > other.f;  // Min heap
    }
  };
  
  std::priority_queue<Node> open_list;
  
  // Directions: left, down, right, up, diagonals
  int directions_x[8] = {-1, 0, 1, 0, -1, -1, 1, 1};
  int directions_y[8] = {0, -1, 0, 1, -1, 1, -1, 1};
  
  std::vector<std::vector<bool>> closed_list(
    map_width, std::vector<bool>(map_height, false));
  std::vector<std::vector<std::tuple<int, int, int, int>>> parent(
    map_width, std::vector<std::tuple<int, int, int, int>>(
    map_height, {-1, -1, -1, -1}));  // parent_x, parent_y, prev_x, prev_y
  std::vector<std::vector<double>> g_cost(
    map_width, std::vector<double>(map_height, std::numeric_limits<double>::infinity()));
  
  g_cost[start_mx][start_my] = 0;
  double h_start = std::hypot(start_mx - goal_mx, start_my - goal_my);
  
  // 计算起点到目标的方向角度 - 用于优化搜索方向
  double goal_direction = std::atan2(goal_my - start_my, goal_mx - start_mx);
  
  // Initial node (start position)
  open_list.push(Node(start_mx, start_my, 0, h_start, start_mx, start_my, start_mx, start_my));
  bool found_path = false;
  
  // A* search
  int iterations = 0;
  while (!open_list.empty() && iterations < max_iterations_) {
    iterations++;
    Node current = open_list.top();
    open_list.pop();
    
    int x = current.x;
    int y = current.y;
    
    if (closed_list[x][y]) {
      continue;
    }
    closed_list[x][y] = true;
    
    // Store parent information
    parent[x][y] = {current.parent_x, current.parent_y, current.prev_x, current.prev_y};
    
    // Check if we reached the goal
    if (static_cast<unsigned int>(x) == goal_mx && static_cast<unsigned int>(y) == goal_my) {
      found_path = true;
      break;
    }
    
    // 计算当前点与目标之间的方向
    double cur_to_goal_dir = std::atan2(goal_my - y, goal_mx - x);
    
    // 计算当前移动方向（如果有前一个点）
    double current_dir = 0.0;
    if (current.prev_x != x || current.prev_y != y) {
      current_dir = std::atan2(y - current.prev_y, x - current.prev_x);
    } else {
      // 如果是起点，使用到目标的方向作为初始方向
      current_dir = cur_to_goal_dir;
    }
    
    // 重排方向数组，优先考虑与当前移动方向更一致的方向
    std::vector<std::pair<int, double>> sorted_directions;
    for (int i = 0; i < 8; ++i) {
      double dir_angle = std::atan2(directions_y[i], directions_x[i]);
      // 计算方向变化的角度差
      double angle_diff = std::abs(normalizeAngle(dir_angle - current_dir));
      // 将方向和角度差作为对添加到向量中
      sorted_directions.push_back({i, angle_diff});
    }
    
    // 按角度差排序，优先选择角度变化小的方向
    std::sort(sorted_directions.begin(), sorted_directions.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });
    
    // Try each direction in sorted order
    for (const auto& [dir_idx, angle_diff] : sorted_directions) {
      int dir_x = directions_x[dir_idx];
      int dir_y = directions_y[dir_idx];
      int new_x = x + dir_x;
      int new_y = y + dir_y;
      
      // Check if new position is valid
      if (new_x >= 0 && new_x < map_width && new_y >= 0 && new_y < map_height &&
          !closed_list[new_x][new_y] && grid[new_x][new_y] == 0)
      {
        // 基本移动成本（直线vs对角线）
        double move_cost = (dir_idx < 4) ? 1.0 : 1.414;
        
        // 如果在膨胀区域内，大幅增加成本
        if (inflated_grid[new_x][new_y] == 1) {
          // 对于膨胀区域使用极高的成本
          // 这会减少路径穿过膨胀区域的可能性，但不会完全阻止
          // 当没有其他选择时仍然允许通过
          move_cost *= 10.0;
        } else {
          // 基于cost_field进一步调整成本
          move_cost *= cost_field[new_x][new_y];
        }
        
        // 增加距离场成本
        double clearance_cost = 0.0;
        if (distance_field[new_x][new_y] < obstacle_clearance_) {
          // 靠近障碍物的成本呈指数增长
          double dist_factor = distance_field[new_x][new_y] / obstacle_clearance_;
          clearance_cost = 5.0 * std::exp(5.0 * (1.0 - dist_factor));
        }
        
        // 计算新移动方向
        double new_dir = std::atan2(new_y - y, new_x - x);
        
        // Calculate turning penalty if we have history of at least 2 moves
        double turning_penalty = 0.0;
        if (current.prev_x != current.x || current.prev_y != current.y) {
          turning_penalty = calculateTurningPenalty(
            current.prev_x, current.prev_y, current.x, current.y, new_x, new_y);
          
          // 为Ackermann系统增加额外的转弯惩罚
          if (is_ackermann_) {
            // 计算两个向量之间的角度
            double dir1 = std::atan2(current.y - current.prev_y, current.x - current.prev_x);
            double dir2 = std::atan2(new_y - current.y, new_x - current.x);
            double angle_diff = std::abs(normalizeAngle(dir2 - dir1));
            
            // 如果角度变化太大，增加惩罚
            if (angle_diff > M_PI_4) {  // 45度
              turning_penalty *= (1.0 + angle_diff);
            }
            
            // 检测是否是S型转弯（先左后右或先右后左），这种情况下增加额外惩罚
            if (dir_idx < 4 && current.prev_x != -1) { // 非对角线移动且有前一个点
              int prev_dx = current.x - current.prev_x;
              int prev_dy = current.y - current.prev_y;
              int curr_dx = new_x - current.x;
              int curr_dy = new_y - current.y;
              
              // 如果前一个移动和当前移动构成S型
              if ((prev_dx * curr_dy - prev_dy * curr_dx) != 0) {
                // 这表示两个移动方向不同
                turning_penalty *= 1.5; // 额外增加50%的惩罚
              }
            }
          }
        }
        
        // 新增：检查新移动方向是否偏离目标方向
        double goal_alignment_penalty = 0.0;
        double goal_angle_diff = std::abs(normalizeAngle(new_dir - cur_to_goal_dir));
        
        // 偏离目标方向超过45度的移动将受到惩罚
        if (goal_angle_diff > M_PI_4) {
          goal_alignment_penalty = goal_angle_diff * 2.0; // 角度差越大，惩罚越重
        }
        
        // Total cost for this move
        double total_move_cost = move_cost + clearance_cost + turning_penalty + goal_alignment_penalty;
        double new_g = g_cost[x][y] + total_move_cost;
        
        if (new_g < g_cost[new_x][new_y]) {
          g_cost[new_x][new_y] = new_g;
          
          // Heuristic distance to goal
          double h = std::hypot(new_x - goal_mx, new_y - goal_my);
          
          // 添加方向引导以鼓励向目标移动
          double goal_dir = std::atan2(goal_my - new_y, goal_mx - new_x);
          double current_dir = std::atan2(new_y - y, new_x - x);
          double dir_diff = std::abs(normalizeAngle(goal_dir - current_dir));
          // 方向引导惩罚 - 偏离目标方向的节点获得较高的h值
          double direction_penalty = dir_diff * 0.15 * h; // 增加系数以更强烈地鼓励直线路径
          
          // 如果新位置在膨胀区域内，但不在实际障碍物中
          // 增加启发式以减少穿越这些区域的可能性
          if (inflated_grid[new_x][new_y] == 1) {
            h *= 1.5;  // 增加启发式值使这些点不太可能被选择
          }
          
          if (enhanced_heuristic_) {
            // 增强型启发式函数：添加到目标的直线路径偏好
            double direct_angle = std::atan2(goal_my - new_y, goal_mx - new_x);
            double direct_diff = std::abs(normalizeAngle(direct_angle - current_dir));
            // 偏离直线路径的惩罚 - 进一步加强 
            double direct_penalty = direct_diff * h * 0.3; // 增加系数
            // 如果新移动方向更接近目标方向，给予额外奖励
            if (direct_diff < 0.2) { // 约11.5度以内
              h *= 0.85; // 降低启发式值，更可能选择这条路径
            } else if (direct_diff > 0.7) { // 约40度以上
              h *= 1.2; // 增加启发式值，降低选择可能性
            }
            h += direct_penalty;
          }
          
          // Create new node with previous position for direction calculation
          open_list.push(Node(new_x, new_y, new_g, h + direction_penalty, x, y, current.x, current.y));
        }
      }
    }
  }
  
  if (!found_path) {
    RCLCPP_WARN(logger_, "No path found from start to goal after %d iterations.", iterations);
    return path;
  }
  
  RCLCPP_INFO(logger_, "Path found in %d iterations.", iterations);
  
  // Reconstruct path
  std::vector<std::pair<int, int>> path_cells;
  int x = goal_mx;
  int y = goal_my;
  while (!(static_cast<unsigned int>(x) == start_mx && static_cast<unsigned int>(y) == start_my)) {
    path_cells.push_back({x, y});
    auto [px, py, prev_x, prev_y] = parent[x][y];
    x = px;
    y = py;
  }
  path_cells.push_back({start_mx, start_my});
  std::reverse(path_cells.begin(), path_cells.end());
  
  // Simplify path to reduce unnecessary turns while maintaining clearance
  std::vector<std::pair<int, int>> simplified_path = simplifyPath(path_cells, grid);
  
  // 在将路径点转换为实际路径之前，添加Ackermann约束检查
  if (is_ackermann_) {
    // 对简化后的路径进行后处理，确保满足Ackermann转弯半径
    simplified_path = ensureAckermannConstraints(simplified_path, grid);
  }
  
  // 路径优化：在没有障碍物的开放区域简化路径
  if (enable_path_optimization_ && simplified_path.size() > 3) {
    simplified_path = optimizePath(simplified_path, grid);
  }
  
  // Convert grid coordinates to world coordinates
  for (size_t i = 0; i < simplified_path.size(); ++i) {
    double wx, wy;
    costmap_->mapToWorld(simplified_path[i].first, simplified_path[i].second, wx, wy);
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    
    // Set orientation to point toward the next waypoint
    if (i < simplified_path.size() - 1) {
      double next_wx, next_wy;
      costmap_->mapToWorld(
        simplified_path[i + 1].first, simplified_path[i + 1].second, 
        next_wx, next_wy);
      double yaw = std::atan2(next_wy - wy, next_wx - wx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    } else {
      // Use goal orientation for the last point
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path CustomPlanner::findPathAroundObstacles(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool use_conservative_params)
{
  // Store original parameters that we might modify
  double original_clearance = obstacle_clearance_;
  double original_safety_factor = obstacle_check_safety_factor_;
  
  // If using conservative parameters, increase safety margins
  if (use_conservative_params) {
    // Increase obstacle clearance by 50%
    obstacle_clearance_ *= 1.5;
    // Increase safety factor by 30%
    obstacle_check_safety_factor_ *= 1.3;
    
    RCLCPP_INFO(logger_, "Using conservative path planning parameters: clearance=%.2f, safety_factor=%.2f",
               obstacle_clearance_, obstacle_check_safety_factor_);
  }
  
  // Call the original implementation
  nav_msgs::msg::Path path = findPathAroundObstacles(start, goal);
  
  // Restore original parameters
  if (use_conservative_params) {
    obstacle_clearance_ = original_clearance;
    obstacle_check_safety_factor_ = original_safety_factor;
  }
  
  return path;
}

nav_msgs::msg::Path CustomPlanner::buildPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
{
  nav_msgs::msg::Path path;
  if (waypoints.empty()) {
    return path;
  }
  path.header = waypoints[0].header;
  path.poses = waypoints;
  return path;
}

bool CustomPlanner::isPathCollisionFree(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return true;
  }
  
  // 考虑机器人尺寸的缓冲区
  double robot_radius = std::max(robot_width_, robot_length_) / 2.0;
  double check_buffer = std::max(obstacle_clearance_, robot_radius);
  
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    if (!isLineCollisionFree(path.poses[i], path.poses[i+1], check_buffer)) {
      return false;
    }
  }
  return true;
}

double CustomPlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::vector<geometry_msgs::msg::PoseStamped> CustomPlanner::generateArcPoints(
  double cx, double cy, double radius, 
  double start_angle, double end_angle,
  const std_msgs::msg::Header & header)
{
  std::vector<geometry_msgs::msg::PoseStamped> arc_points;
  
  // 标准化角度差以取最短路径
  double angle_diff = normalizeAngle(end_angle - start_angle);
  
  // 生成圆弧上的点
  double arc_length = std::abs(angle_diff * radius);
  int num_points = std::max(2, static_cast<int>(arc_length / path_resolution_));
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
    double angle = start_angle + t * angle_diff;
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = cx + radius * std::cos(angle);
    pose.pose.position.y = cy + radius * std::sin(angle);
    
    // 切线方向作为朝向
    double tangent_angle = angle + M_PI/2.0;
    if (angle_diff < 0) {
      tangent_angle = angle - M_PI/2.0;
    }
    
    tf2::Quaternion q;
    q.setRPY(0, 0, tangent_angle);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    arc_points.push_back(pose);
  }
  return arc_points;
}

nav_msgs::msg::Path CustomPlanner::createDubinsPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  
  // 提取姿态数据
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double start_yaw = tf2::getYaw(start.pose.orientation);
  
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  
  // 计算从起点到目标的方向和距离
  double dx = goal_x - start_x;
  double dy = goal_y - start_y;
  double distance = std::hypot(dx, dy);
  double path_angle = std::atan2(dy, dx);
  
  // 计算起点和终点朝向与路径方向的角度差
  double start_angle_diff = normalizeAngle(path_angle - start_yaw);
  double goal_angle_diff = normalizeAngle(goal_yaw - path_angle);
  
  // 生成转弯弧线和连接直线
  std::vector<geometry_msgs::msg::PoseStamped> path_points;
  
  // 计算第一段转弯的中心点和角度
  double turn_direction = (start_angle_diff > 0) ? 1.0 : -1.0;  // 1.0左转，-1.0右转
  double r = min_turning_radius_;
  double center_x = start_x - r * std::sin(start_yaw) * turn_direction;
  double center_y = start_y + r * std::cos(start_yaw) * turn_direction;
  
  // 生成第一段转弯弧线
  double start_angle = normalizeAngle(start_yaw + M_PI/2.0 * turn_direction);
  double end_angle = normalizeAngle(path_angle + M_PI/2.0 * turn_direction);
  auto first_turn = generateArcPoints(center_x, center_y, r, start_angle, end_angle, start.header);
  
  // 添加第一段转弯
  for (const auto & pose : first_turn) {
    path.poses.push_back(pose);
  }
  
  // 直线段的长度
  double straight_length = distance - 2.0 * r * std::abs(std::sin(start_angle_diff / 2.0));
  
  // 如果直线段足够长，添加直线段
  if (straight_length > 0.1) {
    // 找到第一段转弯结束点
    double mid_x = path.poses.back().pose.position.x;
    double mid_y = path.poses.back().pose.position.y;
    
    // 沿路径方向添加直线段
    int steps = std::max(2, static_cast<int>(straight_length / path_resolution_));
    for (int i = 1; i <= steps; ++i) {
      double ratio = static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = start.header;
      pose.pose.position.x = mid_x + ratio * straight_length * std::cos(path_angle);
      pose.pose.position.y = mid_y + ratio * straight_length * std::sin(path_angle);
      
      // 直线段的方向保持不变
      tf2::Quaternion q;
      q.setRPY(0, 0, path_angle);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      
      path.poses.push_back(pose);
    }
  }
  
  // 计算第二段转弯（到终点朝向）
  double end_path_x = path.poses.back().pose.position.x;
  double end_path_y = path.poses.back().pose.position.y;
  turn_direction = (goal_angle_diff > 0) ? 1.0 : -1.0;  // 1.0左转，-1.0右转
  center_x = end_path_x - r * std::sin(path_angle) * turn_direction;
  center_y = end_path_y + r * std::cos(path_angle) * turn_direction;
  
  // 生成第二段转弯弧线
  start_angle = normalizeAngle(path_angle + M_PI/2.0 * turn_direction);
  end_angle = normalizeAngle(goal_yaw + M_PI/2.0 * turn_direction);
  auto second_turn = generateArcPoints(center_x, center_y, r, start_angle, end_angle, start.header);
  
  // 添加第二段转弯
  for (const auto & pose : second_turn) {
    path.poses.push_back(pose);
  }
  
  // 保证最后一个点是目标点
  path.poses.back() = goal;
  
  return path;
}

nav_msgs::msg::Path CustomPlanner::smoothPathForAckermann(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 3) {
    return path;  // 不需要平滑
  }
  
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = path.header;
  
  // 使用更大的转弯半径值 - 增加5倍以确保更平滑的转弯
  double enhanced_turning_radius = min_turning_radius_ * 5.0;
  
  // 预处理：对路径进行预分析，找出所有转弯点和直线段
  std::vector<bool> is_turning_point(path.poses.size(), false);
  std::vector<double> segment_angles(path.poses.size(), 0.0);
  
  // 首先分析路径并识别转弯点
  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    double angle1 = std::atan2(
      path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,
      path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
    
    double angle2 = std::atan2(
      path.poses[i+1].pose.position.y - path.poses[i].pose.position.y,
      path.poses[i+1].pose.position.x - path.poses[i].pose.position.x);
    
    double angle_diff = std::abs(normalizeAngle(angle2 - angle1));
    
    // 如果角度变化超过阈值，这是一个转弯点
    if (angle_diff > 0.25) {  // 大约14度
      is_turning_point[i] = true;
    }
    
    segment_angles[i] = angle2;
  }
  
  // 处理第一个点 - 确保方向与第一个路径段对齐
  if (path.poses.size() > 1) {
    double initial_angle = std::atan2(
      path.poses[1].pose.position.y - path.poses[0].pose.position.y,
      path.poses[1].pose.position.x - path.poses[0].pose.position.x);
    
    // 比较车辆方向与路径方向
    double start_yaw = tf2::getYaw(path.poses[0].pose.orientation);
    double angle_diff = std::abs(normalizeAngle(initial_angle - start_yaw));
    
    if (angle_diff > 0.2) {  // 如果角度差大于约11度
      // 修正第一个点的朝向以更好地对齐路径
      geometry_msgs::msg::PoseStamped fixed_first = path.poses[0];
      
      // 这里我们可以选择部分调整（70%路径方向，30%原始方向）
      double blended_angle = normalizeAngle(start_yaw + 0.7 * normalizeAngle(initial_angle - start_yaw));
      
      tf2::Quaternion q;
      q.setRPY(0, 0, blended_angle);
      fixed_first.pose.orientation.x = q.x();
      fixed_first.pose.orientation.y = q.y();
      fixed_first.pose.orientation.z = q.z();
      fixed_first.pose.orientation.w = q.w();
      
      smoothed_path.poses.push_back(fixed_first);
    } else {
      // 角度接近，保持原始方向
      smoothed_path.poses.push_back(path.poses[0]);
    }
  } else {
    smoothed_path.poses.push_back(path.poses[0]);
  }
  
  // 处理中间点 - 专注于平滑转弯
  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    // 如果识别为转弯点，添加平滑的转弯弧线
    if (is_turning_point[i]) {
      // 计算转弯半径 (可能更大以实现更平缓的转弯)
      double radius = calculateTurningRadius(
        path.poses[i-1], path.poses[i], path.poses[i+1]);
      
      // 降低对极小转弯半径的敏感度，使曲线更平滑
      if (radius < enhanced_turning_radius && radius > 0.001) {
        // 获取入弯和出弯方向
        double in_angle = segment_angles[i-1];
        double out_angle = segment_angles[i];
        
        // 根据转弯幅度调整半径 - 小转弯使用更大半径
        double turn_angle_diff = std::abs(normalizeAngle(out_angle - in_angle));
        double turn_radius = enhanced_turning_radius;
        
        // 使小转弯更加平滑
        if (turn_angle_diff < 0.3) {
          turn_radius *= (2.0 - turn_angle_diff/0.5);  // 更大的半径，最多2倍
        }
        
        // 确定转弯方向
        double cross_prod = (path.poses[i].pose.position.x - path.poses[i-1].pose.position.x) *
                            (path.poses[i+1].pose.position.y - path.poses[i].pose.position.y) -
                            (path.poses[i].pose.position.y - path.poses[i-1].pose.position.y) *
                            (path.poses[i+1].pose.position.x - path.poses[i].pose.position.x);
        double turn_dir = (cross_prod > 0) ? 1.0 : -1.0;  // 左转=1，右转=-1
        
        // 计算转弯中心
        double center_x = path.poses[i].pose.position.x - turn_radius * std::sin(in_angle) * turn_dir;
        double center_y = path.poses[i].pose.position.y + turn_radius * std::cos(in_angle) * turn_dir;
        
        // 计算弧线角度范围
        double start_angle = normalizeAngle(in_angle - M_PI/2.0 * turn_dir);
        double end_angle = normalizeAngle(out_angle - M_PI/2.0 * turn_dir);
        
        // 根据转弯角度和半径确定点的数量
        int arc_points;
        if (turn_angle_diff < 0.3) {
          arc_points = 5;  // 小转弯用较少点
        } else if (turn_angle_diff < 0.7) {
          arc_points = 10;  // 中等转弯
        } else {
          arc_points = 15 + static_cast<int>(turn_angle_diff * 10.0);  // 大转弯使用更多点
        }
        
        // 限制最大点数
        arc_points = std::min(arc_points, 30);
        
        // 生成平滑的弧线点
        auto arc_points_vec = generateDenseArcPoints(
          center_x, center_y, turn_radius, start_angle, end_angle, path.header, arc_points);
        
        // 添加弧线点
        for (const auto & pose : arc_points_vec) {
          smoothed_path.poses.push_back(pose);
        }
      } else {
        // 如果转弯不明显，简单添加原始点
        smoothed_path.poses.push_back(path.poses[i]);
      }
    } else {
      // 非转弯点直接添加
      smoothed_path.poses.push_back(path.poses[i]);
    }
  }
  
  // 处理最后一个点 - 确保朝向与目标朝向一致
  if (!path.poses.empty()) {
    smoothed_path.poses.push_back(path.poses.back());
  }
  
  // 应用进一步平滑和朝向修正
  return alignPathWithRobotOrientation(applyPathSmoothing(smoothed_path));
}

nav_msgs::msg::Path CustomPlanner::applyPathSmoothing(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 4) {
    return path;
  }
  
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = path.header;
  
  const int window_size = 7; // 增加窗口大小以获得更平滑的曲线
  const int half_window = window_size / 2;
  
  // 保留起点和终点不变
  smoothed_path.poses.push_back(path.poses.front());
  
  // 为中间点应用滑动窗口平均
  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    geometry_msgs::msg::PoseStamped smooth_pose = path.poses[i];
    
    // 计算窗口范围
    int start_idx = std::max(0, static_cast<int>(i) - half_window);
    int end_idx = std::min(static_cast<int>(path.poses.size()) - 1, static_cast<int>(i) + half_window);
    
    // 高斯权重和
    double sum_x = 0.0, sum_y = 0.0;
    double sum_weights = 0.0;
    for (int j = start_idx; j <= end_idx; ++j) {
      // 高斯权重 - 中心点权重最大
      double dist = std::abs(j - static_cast<int>(i));
      double weight = std::exp(-dist * dist / 4.0); // 降低衰减速率，使平滑更广泛
      
      // 中心点额外权重
      if (j == static_cast<int>(i)) {
        weight *= 2.0; // 增加中心权重以保持原始路径特征
      }
      
      sum_x += path.poses[j].pose.position.x * weight;
      sum_y += path.poses[j].pose.position.y * weight;
      sum_weights += weight;
    }
    
    // 计算加权平均
    smooth_pose.pose.position.x = sum_x / sum_weights;
    smooth_pose.pose.position.y = sum_y / sum_weights;
    
    // 计算平滑朝向 - 确保朝向变化平滑
    if (i < path.poses.size() - 2) {
      // 计算前向方向一个点的方向
      size_t next_idx = std::min(i + 2, path.poses.size() - 1);
      double dx = path.poses[next_idx].pose.position.x - smooth_pose.pose.position.x;
      double dy = path.poses[next_idx].pose.position.y - smooth_pose.pose.position.y;
      double desired_yaw = std::atan2(dy, dx);
      double prev_yaw = tf2::getYaw(smooth_pose.pose.orientation);
      
      // 计算标准化角度差
      double yaw_diff = normalizeAngle(desired_yaw - prev_yaw);
      double max_angle_change = 0.2; // 每步最大11.5度变化
      if (std::abs(yaw_diff) > max_angle_change) {
        yaw_diff = (yaw_diff > 0) ? max_angle_change : -max_angle_change;
      }
      
      // 应用平滑方向变化
      double smooth_yaw = prev_yaw + yaw_diff;
      tf2::Quaternion q;
      q.setRPY(0, 0, smooth_yaw);
      smooth_pose.pose.orientation.x = q.x();
      smooth_pose.pose.orientation.y = q.y();
      smooth_pose.pose.orientation.z = q.z();
      smooth_pose.pose.orientation.w = q.w();
    }
    
    smoothed_path.poses.push_back(smooth_pose);
  }
  
  // 添加最后一个点
  smoothed_path.poses.push_back(path.poses.back());
  
  return smoothed_path;
}

double CustomPlanner::calculateTurningRadius(
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2,
  const geometry_msgs::msg::PoseStamped & p3)
{
  // 计算三点间的两条边长
  double a = std::hypot(p2.pose.position.x - p1.pose.position.x,
                       p2.pose.position.y - p1.pose.position.y);
  double b = std::hypot(p3.pose.position.x - p2.pose.position.x,
                       p3.pose.position.y - p2.pose.position.y);
  double c = std::hypot(p3.pose.position.x - p1.pose.position.x,
                       p3.pose.position.y - p1.pose.position.y);
  
  // 使用海伦公式计算三角形面积
  double s = (a + b + c) / 2.0;
  double area = std::sqrt(s * (s - a) * (s - b) * (s - c));
  
  // 特殊情况处理：如果三点几乎共线
  if (area < 1e-6) {
    return std::numeric_limits<double>::infinity();
  }
  
  // 使用三角形面积公式计算半径：R = abc/(4A)
  double radius = (a * b * c) / (4.0 * area);
  return radius;
}

std::vector<std::pair<int, int>> CustomPlanner::ensureAckermannConstraints(
  const std::vector<std::pair<int, int>> & path,
  const std::vector<std::vector<int>> & grid)
{
  std::vector<std::pair<int, int>> new_path;
  if (path.size() < 3) {
    return path;
  }
  
  // 网格中的最小转弯半径（以格子数表示）
  double min_radius_cells = min_turning_radius_ / costmap_->getResolution();
  
  new_path.push_back(path[0]);
  for (size_t i = 1; i < path.size() - 1; ++i) {
    // 分析连续三个点形成的角度
    int x1 = path[i-1].first, y1 = path[i-1].second;
    int x2 = path[i].first, y2 = path[i].second;
    int x3 = path[i+1].first, y3 = path[i+1].second;
    
    // 计算向量长度
    int dx1 = x2 - x1, dy1 = y2 - y1;
    int dx2 = x3 - x2, dy2 = y3 - y2;
    double len1 = std::hypot(dx1, dy1);
    double len2 = std::hypot(dx2, dy2);
    
    // 如果段太短，直接添加原始点
    if (len1 < 1.0 || len2 < 1.0) {
      new_path.push_back(path[i]);
      continue;
    }
    
    // 计算角度阈值 - 根据角度变化自适应调整
    double angle1 = std::atan2(dy1, dx1);
    double angle2 = std::atan2(dy2, dx2);
    double angle_diff = normalizeAngle(angle2 - angle1);
    double angle_threshold = 0.2; // 默认值约11.5度
    
    // 如果偏转角较大，考虑添加转弯弧线
    if (std::abs(angle_diff) > angle_threshold) {
      double turn_dir = (angle_diff > 0) ? 1.0 : -1.0;
      
      // 只有对于大幅度转弯，我们才真正需要额外的弧线点
      if (std::abs(angle_diff) > 0.5) { // 约28.6度
        // 计算转弯中心
        double center_x = x2 + min_radius_cells * std::sin(angle1) * turn_dir;
        double center_y = y2 - min_radius_cells * std::cos(angle1) * turn_dir;
        
        // 计算入弯和出弯角度
        double start_angle = normalizeAngle(angle1 - M_PI/2.0 * turn_dir);
        double end_angle = normalizeAngle(angle2 - M_PI/2.0 * turn_dir);
        
        // 在网格中生成转弯弧线点 - 只在很大的转弯中才这样做
        int arc_points = std::max(3, static_cast<int>(std::abs(angle_diff) * min_radius_cells));
        for (int j = 1; j < arc_points - 1; ++j) {
          double t = static_cast<double>(j) / static_cast<double>(arc_points - 1);
          double arc_angle = start_angle + t * normalizeAngle(end_angle - start_angle);
          int arc_x = static_cast<int>(center_x + min_radius_cells * std::cos(arc_angle));
          int arc_y = static_cast<int>(center_y + min_radius_cells * std::sin(arc_angle));
          
          // 检查点是否在网格内且无碰撞
          if (arc_x >= 0 && arc_x < static_cast<int>(grid.size()) &&
              arc_y >= 0 && arc_y < static_cast<int>(grid[0].size()) &&
              grid[arc_x][arc_y] == 0) {
            new_path.push_back({arc_x, arc_y});
          }
        }
      }
    } else {
      // 如果转角不急，直接添加原始点
      new_path.push_back(path[i]);
    }
  }
  
  // 添加最后一个点
  new_path.push_back(path.back());
  return new_path;
}

nav_msgs::msg::Path CustomPlanner::alignPathWithRobotOrientation(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 3) {
    return path;
  }
  
  nav_msgs::msg::Path aligned_path = path;
  
  // 改进起点附近的路径对齐
  double start_yaw = tf2::getYaw(path.poses[0].pose.orientation);
  
  // 计算初始路径段的朝向
  double initial_path_angle = std::atan2(
    path.poses[1].pose.position.y - path.poses[0].pose.position.y,
    path.poses[1].pose.position.x - path.poses[0].pose.position.x);
  
  // 计算起点朝向与路径朝向的角度差
  double start_angle_diff = std::abs(normalizeAngle(initial_path_angle - start_yaw));
  
  // 如果起点朝向与路径朝向差距较大（超过20度），则插入额外的过渡点
  if (start_angle_diff > 0.35) {
    // 在原始路径的起点添加过渡点，使机器人能够平滑地转向路径
    geometry_msgs::msg::PoseStamped transition_pose = path.poses[0];
    
    // 计算过渡点位置 - 在起点前方的短距离处创建点
    double transition_distance = std::min(0.3, robot_length_ * 0.4);  // 使用较小距离，避免过远
    
    // 按照起始朝向放置过渡点
    transition_pose.pose.position.x += transition_distance * std::cos(start_yaw);
    transition_pose.pose.position.y += transition_distance * std::sin(start_yaw);
    
    // 过渡点的朝向使用起点朝向和路径朝向的混合
    double blend_factor = 0.4;  // 40%路径朝向，60%原始朝向
    double transition_yaw = normalizeAngle(start_yaw + blend_factor * normalizeAngle(initial_path_angle - start_yaw));
    
    tf2::Quaternion q;
    q.setRPY(0, 0, transition_yaw);
    transition_pose.pose.orientation.x = q.x();
    transition_pose.pose.orientation.y = q.y();
    transition_pose.pose.orientation.z = q.z();
    transition_pose.pose.orientation.w = q.w();
    
    // 创建新路径，首先添加原始起点，然后添加过渡点
    nav_msgs::msg::Path new_path;
    new_path.header = path.header;
    new_path.poses.push_back(path.poses[0]);
    new_path.poses.push_back(transition_pose);
    
    // 添加剩余的路径点
    for (size_t i = 1; i < path.poses.size(); ++i) {
      new_path.poses.push_back(path.poses[i]);
    }
    
    aligned_path = new_path;
  }
  
  // 确保路径末端与目标朝向平滑对齐
  size_t path_size = aligned_path.poses.size();
  if (path_size >= 3) {
    double goal_yaw = tf2::getYaw(aligned_path.poses.back().pose.orientation);
    
    // 获取最后一段路径的朝向
    double last_segment_angle = std::atan2(
      aligned_path.poses[path_size-1].pose.position.y - aligned_path.poses[path_size-2].pose.position.y,
      aligned_path.poses[path_size-1].pose.position.x - aligned_path.poses[path_size-2].pose.position.x);
    
    // 计算路径终点与目标朝向的角度差
    double end_angle_diff = std::abs(normalizeAngle(goal_yaw - last_segment_angle));
    
    // 如果终点朝向与路径朝向差距较大，添加过渡点
    if (end_angle_diff > 0.35) {
      // 倒数第二个点朝向应该介于路径方向和目标朝向之间
      double blend_factor = 0.6;  // 60%路径方向，40%目标朝向
      double second_last_yaw = normalizeAngle(last_segment_angle + 
                                          blend_factor * normalizeAngle(goal_yaw - last_segment_angle));
      
      // 设置倒数第二个点的朝向
      tf2::Quaternion q;
      q.setRPY(0, 0, second_last_yaw);
      aligned_path.poses[path_size-2].pose.orientation.x = q.x();
      aligned_path.poses[path_size-2].pose.orientation.y = q.y();
      aligned_path.poses[path_size-2].pose.orientation.z = q.z();
      aligned_path.poses[path_size-2].pose.orientation.w = q.w();
    }
  }
  
  // 确保所有路径点的朝向与行进方向一致，并限制角度变化率
  for (size_t i = 1; i < aligned_path.poses.size() - 1; ++i) {
    // 计算当前路径段的朝向
    double segment_angle = std::atan2(
      aligned_path.poses[i+1].pose.position.y - aligned_path.poses[i].pose.position.y,
      aligned_path.poses[i+1].pose.position.x - aligned_path.poses[i].pose.position.x);
    
    // 获取前一个点的朝向
    double prev_yaw = tf2::getYaw(aligned_path.poses[i-1].pose.orientation);
    
    // 计算角度差并限制变化率
    double angle_diff = normalizeAngle(segment_angle - prev_yaw);
    double max_change = 0.15; // 每点最大约8.6度变化
    
    if (std::abs(angle_diff) > max_change) {
      angle_diff = (angle_diff > 0) ? max_change : -max_change;
    }
    
    // 设置当前点的朝向，保持平滑过渡
    double smooth_yaw = normalizeAngle(prev_yaw + angle_diff);
    
    tf2::Quaternion q;
    q.setRPY(0, 0, smooth_yaw);
    aligned_path.poses[i].pose.orientation.x = q.x();
    aligned_path.poses[i].pose.orientation.y = q.y();
    aligned_path.poses[i].pose.orientation.z = q.z();
    aligned_path.poses[i].pose.orientation.w = q.w();
  }
  
  return aligned_path;
}

} // namespace nav2_custom_planner