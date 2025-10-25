#ifndef PURE_PURSUIT_PLANNER_HPP_
#define PURE_PURSUIT_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pure_pursuit_planner
{

/**
 * @brief ロボットの状態を表すenum
 */
enum class RobotState {
  STOP,           // 停止状態
  START_ROTATE,   // 開始回転状態
  FOLLOW,         // 通常のパス追従状態
  GOAL_ROTATE,    // ゴールに向かって回転する状態
  GOAL            // ゴール到達状態
};

/**
 * @brief RobotStateを文字列に変換
 */
std::string robotStateToString(RobotState state);

/**
 * @brief Pure Pursuitアルゴリズムを使用したパス追従プランナー
 */
class PurePursuitPlanner : public rclcpp::Node
{
public:
  /**
   * @brief コンストラクタ
   */
  PurePursuitPlanner();

  /**
   * @brief デストラクタ
   */
  ~PurePursuitPlanner();

private:
  /**
   * @brief パスのコールバック関数
   */
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief オドメトリのコールバック関数
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief 制御ループのタイマーコールバック
   */
  void controlLoop();

  /**
   * @brief 状態遷移の処理
   */
  void updateState();

  /**
   * @brief STOP状態の処理
   */
  void processStop();

  /**
   * @brief START_ROTATE状態の処理
   */
  void processStartRotate();

  /**
   * @brief FOLLOW状態の処理
   */
  void processFollow();

  /**
   * @brief GOAL_ROTATE状態の処理
   */
  void processGoalRotate();

  /**
   * @brief GOAL状態の処理
   */
  void processGoal();

  /**
   * @brief Pure Pursuitアルゴリズムで目標点を計算
   * @return 目標点のインデックス（見つからない場合は-1）
   */
  int findLookaheadPoint();

  /**
   * @brief 2点間の距離を計算
   */
  double calcDistance(double x1, double y1, double x2, double y2);

  /**
   * @brief 角度差を計算（-pi ~ pi）
   */
  double normalizeAngle(double angle);

  /**
   * @brief 目標角度を計算（最初の点と次の点のベクトル角度）
   */
  double calcStartTargetAngle();

  /**
   * @brief ゴールの角度を計算
   */
  double calcGoalTargetAngle();

  /**
   * @brief 速度コマンドを発行
   */
  void publishCmdVel(double linear_vel, double angular_vel);

  /**
   * @brief 現在の状態をPublish
   */
  void publishState();

  // ROS2 通信
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // パラメータ
  double lookahead_distance_;       // lookahead距離 [m]
  double max_linear_velocity_;      // 最大並進速度 [m/s]
  double max_angular_velocity_;     // 最大回転速度 [rad/s]
  double goal_tolerance_;           // ゴール到達判定の距離 [m]
  double angle_tolerance_;          // 角度一致判定の閾値 [rad]
  double rotate_angular_velocity_;  // 回転時の角速度 [rad/s]
  double control_frequency_;        // 制御周期 [Hz]

  // 状態変数
  RobotState current_state_;
  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::Pose current_pose_;
  bool has_path_;
  bool has_odom_;
  double current_yaw_;
};

}  // namespace pure_pursuit_planner

#endif  // PURE_PURSUIT_PLANNER_HPP_
