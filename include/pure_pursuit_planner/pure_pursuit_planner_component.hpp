#ifndef PURE_PURSUIT_PLANNER__PURE_PURSUIT_PLANNER_COMPONENT_HPP_
#define PURE_PURSUIT_PLANNER__PURE_PURSUIT_PLANNER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace pure_pursuit_planner
{
class PurePursuitNode : public rclcpp::Node
{
public:
  explicit PurePursuitNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~PurePursuitNode();

private:
  // ロボットの状態を表す列挙型
  enum class RobotState {
    STOP,           // 停止状態
    START_ROTATE,   // 開始回転状態
    FOLLOW,         // 通常のパス追従状態
    GOAL_ROTATE,    // ゴールに向かって回転する状態
    GOAL            // ゴール到達状態
  };

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // パラメータ
  double lookahead_distance_;     // ルックアヘッド距離
  double min_lookahead_distance_; // 最小ルックアヘッド距離
  double max_lookahead_distance_; // 最大ルックアヘッド距離
  double linear_velocity_;        // 線形速度
  double max_angular_velocity_;   // 最大角速度
  double goal_tolerance_;         // ゴール許容誤差
  std::string robot_frame_id_;    // ロボットのフレームID
  std::string map_frame_id_;     // マップのフレームID
  double rotation_threshold_;    // 追従状態から回転状態に移行する角度閾値（ラジアン）
  double rotation_velocity_;     // その場回転時の角速度
  double max_velocity_;          // 最大速度
  double target_idx_;            // ターゲットインデックス
  size_t closest_idx_ = 0;

  // 内部状態変数
  nav_msgs::msg::Path::SharedPtr current_path_; // 現在のパス
  bool follow_path_;                            // パスを受け取ったかどうか
  bool is_goal_reached_;                        // ゴールに到達したかどうか
  RobotState current_state_;                    // 現在のロボット状態
  geometry_msgs::msg::Point target_point_;      // 現在のターゲットポイント
  
  bool costmap_received_ = false;
  nav_msgs::msg::OccupancyGrid costmap_;

  
  // コールバック関数
  void onPathReceived(const nav_msgs::msg::Path::SharedPtr msg);
  void CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void ControlLoop();
  // Pure Pursuit アルゴリズムの実装関数
  bool findTargetPoint();
  void detect_velocity();
  double calc_dinstanse(size_t index);
  double calculateAngularVelocity(const geometry_msgs::msg::Pose & target_point);

  double calculateAngularVelocity(
    const geometry_msgs::msg::Pose & current_pose,
    const geometry_msgs::msg::Point & target_point);
  geometry_msgs::msg::Pose current_pose_;
  
  // 状態マシン関連の関数
  void updateState();
  void getCurrentPose();
  void publishMarkers(const geometry_msgs::msg::Point & target_point);

  bool isStartAngleReached();
  bool isGoalAngleReached();
  std::string getCurrentMode();


  bool isGoalReached(
    const geometry_msgs::msg::Pose & current_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose);
    
 

  // ユーティリティ関数
  double calculateDistance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

  bool colisionCheck();
  bool isOnDetour_ = false;
    
};
} // namespace pure_pursuit_planner

#endif  // PURE_PURSUIT_PLANNER__PURE_PURSUIT_PLANNER_COMPONENT_HPP_

