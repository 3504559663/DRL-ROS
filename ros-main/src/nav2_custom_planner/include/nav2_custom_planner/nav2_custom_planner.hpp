// Modified nav2_custom_planner.hpp
#ifndef NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <utility>
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_custom_planner
{

class CustomPlanner : public nav2_core::GlobalPlanner
{
public:
  CustomPlanner();
  ~CustomPlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomPlanner")};
  
  // Planning parameters
  double path_resolution_;
  double obstacle_clearance_;        // Minimum clearance from obstacles
  double straight_line_preference_;  // Weight for preferring straight lines
  int max_iterations_;               // Maximum iterations for A* search
  
  // Ackermann specific parameters
  double min_turning_radius_;        // 最小转弯半径
  bool is_ackermann_;                // 是否使用Ackermann约束
  
  // Robot dimensions
  double robot_width_;               // 机器人宽度
  double robot_length_;              // 机器人长度
  double inflation_radius_;          // 膨胀半径
  
  // Helper method to check if a point is in collision
  bool isPointInCollision(double wx, double wy);
  
  // Helper method with buffer zone to avoid close proximity to obstacles
  bool isPointSafe(double wx, double wy, double buffer = 0.0);
  
  // Helper method to check if a line between two points is collision-free
  bool isLineCollisionFree(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double buffer = 0.0);
  
  // Helper method to create a straight-line path
  nav_msgs::msg::Path createStraightPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
  
  // Dubins路径生成函数
  nav_msgs::msg::Path createDubinsPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
    
  // 检查路径是否符合Ackermann约束
  bool isPathAckermannValid(const nav_msgs::msg::Path & path);
  
  // 平滑路径使其符合Ackermann约束
  nav_msgs::msg::Path smoothPathForAckermann(const nav_msgs::msg::Path & path);
  
  // 计算转弯半径
  double calculateTurningRadius(
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2,
    const geometry_msgs::msg::PoseStamped & p3);

  // 生成转弯弧线路径点
  std::vector<geometry_msgs::msg::PoseStamped> generateTurnArc(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double radius);
  
  // Generate distance field from obstacles
  std::vector<std::vector<double>> generateDistanceField(const std::vector<std::vector<int>> & grid);
  
  // Helper method to find a path around obstacles using enhanced A*
  nav_msgs::msg::Path findPathAroundObstacles(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
    
  // Overloaded version with conservative parameter option
  nav_msgs::msg::Path findPathAroundObstacles(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    bool use_conservative_params);
  
  // Enhanced path simplification
  std::vector<std::pair<int, int>> simplifyPath(
    const std::vector<std::pair<int, int>> & path,
    const std::vector<std::vector<int>> & grid);
  
  // Helper method to check if there's a direct line of sight
  bool hasLineOfSight(
    const std::pair<int, int> & start,
    const std::pair<int, int> & end,
    const std::vector<std::vector<int>> & grid,
    double buffer = 0.0);
  
  // Helper method to build a path from a list of waypoints
  nav_msgs::msg::Path buildPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints);
  
  // Calculate direction from two points
  double calculateDirection(int x1, int y1, int x2, int y2);
  
  // Calculate turning penalty between three points
  double calculateTurningPenalty(
    int x1, int y1, int x2, int y2, int x3, int y3);
  
  // 检查整条路径是否无碰撞
  bool isPathCollisionFree(const nav_msgs::msg::Path & path);
  
  // 使角度标准化到[-π, π]范围
  double normalizeAngle(double angle);
  
  // 生成沿圆弧的路径点
  std::vector<geometry_msgs::msg::PoseStamped> generateArcPoints(
    double cx, double cy, double radius, 
    double start_angle, double end_angle,
    const std_msgs::msg::Header & header);
    
  // 确保路径满足Ackermann约束
  std::vector<std::pair<int, int>> ensureAckermannConstraints(
    const std::vector<std::pair<int, int>> & path,
    const std::vector<std::vector<int>> & grid);
    
  // 增加新的大角度目标处理函数声明
  nav_msgs::msg::Path handleLargeAngleTarget(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
    
  // 添加创建直接倒车路径的函数声明
  nav_msgs::msg::Path createReversePathToTarget(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
    
  // 添加路径与机器人朝向对齐的函数声明
  nav_msgs::msg::Path alignPathWithRobotOrientation(const nav_msgs::msg::Path & path);
  
  // Path stability and smoothing parameters
  rclcpp::Time last_plan_time_;               // Last time a plan was created
  nav_msgs::msg::Path prev_path_;             // Previously generated path
  geometry_msgs::msg::PoseStamped prev_goal_; // Previous goal position
  double path_stability_timeout_;             // Timeout for path stability
  double path_smoothing_factor_;              // Path smoothing factor
  double turn_radius_factor_;                 // Turn radius factor
  double path_improvement_threshold_;         // Path improvement threshold
  double turn_suppression_factor_;            // Turn suppression factor
  double path_simplify_aggressiveness_;       // Path simplify aggressiveness
  double angle_continuity_factor_;            // Angle continuity factor

  // Additional parameters for enhanced path planning
  bool allow_unknown_;                        // Allow planning through unknown space
  bool consider_footprint_;                   // Consider robot footprint exactly in tight spaces
  int tight_space_retry_count_;               // Number of retries for tight spaces
  double tight_space_clearance_factor_;       // Reduced clearance factor for tight spaces
  bool use_adaptive_inflation_;               // Use adaptive inflation for difficult areas
  
  // Path smoothing functions
  nav_msgs::msg::Path applyPathSmoothing(const nav_msgs::msg::Path & path);
  
  // Generate dense arc points for smoother turns
  std::vector<geometry_msgs::msg::PoseStamped> generateDenseArcPoints(
    double cx, double cy, double radius, 
    double start_angle, double end_angle,
    const std_msgs::msg::Header & header,
    int num_points);
    
  // 新增安全性和优化参数
  double obstacle_check_safety_factor_; // 障碍物检查安全系数
  int collision_check_points_;          // 碰撞检查点数
  bool enable_path_optimization_;       // 是否启用路径优化
  double grid_resolution_multiplier_;   // 网格分辨率乘数
  bool direct_path_preferred_;          // 是否优先选择直接路径
  bool enhanced_heuristic_;             // 是否使用增强启发式函数
  
  // 新增路径优化方法
  std::vector<std::pair<int, int>> optimizePath(
    const std::vector<std::pair<int, int>> & path,
    const std::vector<std::vector<int>> & grid);
    
  /**
   * @brief 验证路径质量，检测是否存在过长路径或过多转弯
   * @param path 待验证的路径
   * @param start 起点位置
   * @param goal 目标位置
   * @return 如果路径质量可接受则返回true，否则返回false
   */
  bool verifyPathQuality(
    const nav_msgs::msg::Path & path, 
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief 创建双弧线路径，用于解决大角度转弯而不需要后退
   * @param start 起点位置
   * @param goal 目标位置
   * @return 生成的路径
   */
  nav_msgs::msg::Path createDualArcPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
    
  /**
   * @brief Attempt to find a path in tight spaces by incrementally reducing constraints
   * @param start Start position
   * @param goal Goal position
   * @return Path if found, empty path otherwise
   */
  nav_msgs::msg::Path findPathInTightSpaces(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
    
  /**
   * @brief Check if given goal is in a tight or difficult space requiring special handling
   * @param goal Goal position to check
   * @return True if in tight space, false otherwise
   */
  bool isInTightSpace(const geometry_msgs::msg::PoseStamped & goal);
};

} // namespace nav2_custom_planner

#endif // NAV2_CUSTOM_PLANNER_HPP_