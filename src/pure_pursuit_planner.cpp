#include "pure_pursuit_planner/pure_pursuit_planner.hpp"

namespace pure_pursuit_planner
{

std::string robotStateToString(RobotState state)
{
  switch (state) {
    case RobotState::STOP:
      return "STOP";
    case RobotState::START_ROTATE:
      return "START_ROTATE";
    case RobotState::FOLLOW:
      return "FOLLOW";
    case RobotState::GOAL_ROTATE:
      return "GOAL_ROTATE";
    case RobotState::GOAL:
      return "GOAL";
    default:
      return "UNKNOWN";
  }
}

PurePursuitPlanner::PurePursuitPlanner()
: Node("pure_pursuit_planner"),
  current_state_(RobotState::STOP),
  has_path_(false),
  has_odom_(false),
  current_yaw_(0.0)
{
  // パラメータの宣言
  this->declare_parameter("lookahead_distance", 1.0);
  this->declare_parameter("max_linear_velocity", 0.5);
  this->declare_parameter("max_angular_velocity", 1.0);
  this->declare_parameter("goal_tolerance", 0.1);
  this->declare_parameter("angle_tolerance", 0.1);
  this->declare_parameter("rotate_angular_velocity", 0.5);
  this->declare_parameter("control_frequency", 10.0);

  // パラメータの取得
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
  rotate_angular_velocity_ = this->get_parameter("rotate_angular_velocity").as_double();
  control_frequency_ = this->get_parameter("control_frequency").as_double();

  // Subscriber
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", 10,
    std::bind(&PurePursuitPlanner::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&PurePursuitPlanner::odomCallback, this, std::placeholders::_1));

  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  state_pub_ = this->create_publisher<std_msgs::msg::String>("robot_state", 10);

  // Timer
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PurePursuitPlanner::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Pure Pursuit Planner initialized");
  RCLCPP_INFO(this->get_logger(), "  lookahead_distance: %.2f m", lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "  max_linear_velocity: %.2f m/s", max_linear_velocity_);
  RCLCPP_INFO(this->get_logger(), "  max_angular_velocity: %.2f rad/s", max_angular_velocity_);
  RCLCPP_INFO(this->get_logger(), "  goal_tolerance: %.2f m", goal_tolerance_);
  RCLCPP_INFO(this->get_logger(), "  angle_tolerance: %.2f rad", angle_tolerance_);
}

PurePursuitPlanner::~PurePursuitPlanner()
{
  // 停止コマンドを送信
  publishCmdVel(0.0, 0.0);
}

void PurePursuitPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path");
    return;
  }

  current_path_ = *msg;
  has_path_ = true;

  RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses", msg->poses.size());

  // 新しいパスを受信したら状態をリセット
  if (current_state_ == RobotState::GOAL || current_state_ == RobotState::STOP) {
    current_state_ = RobotState::START_ROTATE;
    RCLCPP_INFO(this->get_logger(), "State changed to START_ROTATE");
  }
}

void PurePursuitPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  current_yaw_ = tf2::getYaw(current_pose_.orientation);
  has_odom_ = true;
}

void PurePursuitPlanner::controlLoop()
{
  // データが揃っていない場合は何もしない
  if (!has_path_ || !has_odom_) {
    return;
  }

  // 状態の更新と処理
  updateState();

  // 現在の状態をPublish
  publishState();
}

void PurePursuitPlanner::updateState()
{
  switch (current_state_) {
    case RobotState::STOP:
      processStop();
      break;
    case RobotState::START_ROTATE:
      processStartRotate();
      break;
    case RobotState::FOLLOW:
      processFollow();
      break;
    case RobotState::GOAL_ROTATE:
      processGoalRotate();
      break;
    case RobotState::GOAL:
      processGoal();
      break;
  }
}

void PurePursuitPlanner::processStop()
{
  // 停止状態では何もしない
  publishCmdVel(0.0, 0.0);
}

void PurePursuitPlanner::processStartRotate()
{
  // 最初の点と次の点のベクトル角度を計算
  double target_angle = calcStartTargetAngle();

  // 現在の角度との差を計算
  double angle_diff = normalizeAngle(target_angle - current_yaw_);

  // 角度が許容範囲内であればFOLLOW状態に遷移
  if (std::abs(angle_diff) < angle_tolerance_) {
    current_state_ = RobotState::FOLLOW;
    RCLCPP_INFO(this->get_logger(), "State changed to FOLLOW");
    return;
  }

  // 回転コマンドを発行
  double angular_vel = (angle_diff > 0) ? rotate_angular_velocity_ : -rotate_angular_velocity_;

  // 角度差が小さい場合は速度を調整
  if (std::abs(angle_diff) < 0.5) {
    angular_vel *= std::abs(angle_diff) / 0.5;
  }

  publishCmdVel(0.0, angular_vel);
}

void PurePursuitPlanner::processFollow()
{
  // ゴールまでの距離を確認
  if (current_path_.poses.empty()) {
    current_state_ = RobotState::STOP;
    RCLCPP_WARN(this->get_logger(), "Path is empty, state changed to STOP");
    return;
  }

  const auto& goal_pose = current_path_.poses.back().pose;
  double distance_to_goal = calcDistance(
    current_pose_.position.x, current_pose_.position.y,
    goal_pose.position.x, goal_pose.position.y);

  // ゴール付近に到達したらGOAL_ROTATE状態に遷移
  if (distance_to_goal < goal_tolerance_) {
    current_state_ = RobotState::GOAL_ROTATE;
    RCLCPP_INFO(this->get_logger(), "State changed to GOAL_ROTATE");
    publishCmdVel(0.0, 0.0);
    return;
  }

  // Lookahead pointを探す
  int lookahead_idx = findLookaheadPoint();

  if (lookahead_idx < 0) {
    // Lookahead pointが見つからない場合は最後の点を目標とする
    lookahead_idx = current_path_.poses.size() - 1;
  }

  const auto& target_pose = current_path_.poses[lookahead_idx].pose;

  // 目標点への角度を計算
  double dx = target_pose.position.x - current_pose_.position.x;
  double dy = target_pose.position.y - current_pose_.position.y;
  double target_angle = std::atan2(dy, dx);

  // 角度差を計算
  double angle_diff = normalizeAngle(target_angle - current_yaw_);

  // Pure Pursuitの曲率計算
  double lookahead_actual = std::sqrt(dx * dx + dy * dy);
  double curvature = 2.0 * std::sin(angle_diff) / lookahead_actual;

  // 速度コマンドを計算
  double linear_vel = max_linear_velocity_;
  double angular_vel = curvature * linear_vel;

  // 角速度を制限
  angular_vel = std::max(-max_angular_velocity_,
                        std::min(max_angular_velocity_, angular_vel));

  // 角度差が大きい場合は並進速度を減速
  if (std::abs(angle_diff) > M_PI / 4) {
    linear_vel *= 0.5;
  }

  publishCmdVel(linear_vel, angular_vel);
}

void PurePursuitPlanner::processGoalRotate()
{
  // ゴールの角度を計算
  double target_angle = calcGoalTargetAngle();

  // 現在の角度との差を計算
  double angle_diff = normalizeAngle(target_angle - current_yaw_);

  // 角度が許容範囲内であればGOAL状態に遷移
  if (std::abs(angle_diff) < angle_tolerance_) {
    current_state_ = RobotState::GOAL;
    RCLCPP_INFO(this->get_logger(), "State changed to GOAL - Goal reached!");
    publishCmdVel(0.0, 0.0);
    return;
  }

  // 回転コマンドを発行
  double angular_vel = (angle_diff > 0) ? rotate_angular_velocity_ : -rotate_angular_velocity_;

  // 角度差が小さい場合は速度を調整
  if (std::abs(angle_diff) < 0.5) {
    angular_vel *= std::abs(angle_diff) / 0.5;
  }

  publishCmdVel(0.0, angular_vel);
}

void PurePursuitPlanner::processGoal()
{
  // ゴール状態では停止を維持
  publishCmdVel(0.0, 0.0);
}

int PurePursuitPlanner::findLookaheadPoint()
{
  if (current_path_.poses.empty()) {
    return -1;
  }

  int best_idx = -1;
  double best_distance = std::numeric_limits<double>::max();

  // パス上の各点について、lookahead距離に最も近い点を探す
  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    const auto& pose = current_path_.poses[i].pose;
    double distance = calcDistance(
      current_pose_.position.x, current_pose_.position.y,
      pose.position.x, pose.position.y);

    // lookahead距離以上で、最も近い点を選択
    if (distance >= lookahead_distance_) {
      if (distance < best_distance) {
        best_distance = distance;
        best_idx = i;
      }
    }
  }

  // lookahead距離以上の点が見つからない場合は、最も遠い点を選択
  if (best_idx < 0) {
    double max_distance = 0.0;
    for (size_t i = 0; i < current_path_.poses.size(); ++i) {
      const auto& pose = current_path_.poses[i].pose;
      double distance = calcDistance(
        current_pose_.position.x, current_pose_.position.y,
        pose.position.x, pose.position.y);

      if (distance > max_distance) {
        max_distance = distance;
        best_idx = i;
      }
    }
  }

  return best_idx;
}

double PurePursuitPlanner::calcDistance(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

double PurePursuitPlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double PurePursuitPlanner::calcStartTargetAngle()
{
  if (current_path_.poses.size() < 2) {
    // パスが1点しかない場合はその点の向きを使用
    return tf2::getYaw(current_path_.poses[0].pose.orientation);
  }

  // 最初の点と次の点のベクトル角度を計算
  const auto& first_pose = current_path_.poses[0].pose;
  const auto& second_pose = current_path_.poses[1].pose;

  double dx = second_pose.position.x - first_pose.position.x;
  double dy = second_pose.position.y - first_pose.position.y;

  return std::atan2(dy, dx);
}

double PurePursuitPlanner::calcGoalTargetAngle()
{
  if (current_path_.poses.empty()) {
    return 0.0;
  }

  // ゴール地点の向きを使用
  const auto& goal_pose = current_path_.poses.back().pose;
  return tf2::getYaw(goal_pose.orientation);
}

void PurePursuitPlanner::publishCmdVel(double linear_vel, double angular_vel)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_vel;
  cmd_vel.angular.z = angular_vel;
  cmd_vel_pub_->publish(cmd_vel);
}

void PurePursuitPlanner::publishState()
{
  std_msgs::msg::String state_msg;
  state_msg.data = robotStateToString(current_state_);
  state_pub_->publish(state_msg);
}

}  // namespace pure_pursuit_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pure_pursuit_planner::PurePursuitPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
