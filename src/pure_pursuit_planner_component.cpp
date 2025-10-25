#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include <angles/angles.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pure_pursuit_planner
{
PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions & node_options)
: Node("pure_pursuit_planner", node_options),
  tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
  tf_listener_(tf_buffer_)
{
  lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.5);
  min_lookahead_distance_ = declare_parameter<double>("min_lookahead_distance", 0.3);
  max_lookahead_distance_ = declare_parameter<double>("max_lookahead_distance", 1.5);
  linear_velocity_ = declare_parameter<double>("linear_velocity", 0.5);
  max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.5);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.05);
  robot_frame_id_ = declare_parameter<std::string>("robot_frame_id", "base_link");
  map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
  rotation_threshold_ = declare_parameter<double>("rotation_threshold", 0.1);
  rotation_velocity_ = declare_parameter<double>("rotation_velocity", 0.5);
  max_velocity_ = declare_parameter<double>("max_velocity", 1.0);

  // 内部状態の初期化
  follow_path_ = false;
  is_goal_reached_ = false;
  current_state_ = RobotState::STOP;
  
  twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/auto_cmd_vel", 10);
  marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 10);
  mode_publisher_ = create_publisher<std_msgs::msg::String>("~/mode", 10);
  path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(10).transient_local().reliable().keep_last(1),
     std::bind(&PurePursuitNode::onPathReceived, this, _1));
  control_timer_ = create_wall_timer(
    10ms, std::bind(&PurePursuitNode::ControlLoop, this));
}

PurePursuitNode::~PurePursuitNode()
{
  if (twist_publisher_) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    twist_publisher_->publish(stop_cmd);
  }
}

void PurePursuitNode::CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    costmap_received_ = true;
    costmap_ = *msg;
}

void PurePursuitNode::onPathReceived(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;
  follow_path_ = true;
  is_goal_reached_ = false;
  closest_idx_ = 0; 
  current_state_ = RobotState::STOP; // 新しいパスを受け取ったら状態をリセット
}

void PurePursuitNode::getCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      map_frame_id_, robot_frame_id_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Could not transform ");
    return;
  }
  current_pose_.position.x = transform.transform.translation.x;
  current_pose_.position.y = transform.transform.translation.y;
  current_pose_.position.z = transform.transform.translation.z;
  current_pose_.orientation = transform.transform.rotation;
}

void PurePursuitNode::updateState(){
  switch (current_state_) {
    case RobotState::STOP:
      if (follow_path_) current_state_ = RobotState::START_ROTATE;
      break;
    case RobotState::START_ROTATE:
      if (isStartAngleReached()) current_state_ = RobotState::FOLLOW;
      
      break;
    case RobotState::FOLLOW:
      if (isGoalReached(current_pose_, current_path_->poses.back())) current_state_ = RobotState::GOAL_ROTATE;
      break;
    case RobotState::GOAL_ROTATE:
      if (isGoalAngleReached()) current_state_ = RobotState::GOAL;
      break;
    case RobotState::GOAL:
      follow_path_ = false;
      is_goal_reached_ = true;
      current_state_ = RobotState::STOP;
      break;
  }
}

std::string PurePursuitNode::getCurrentMode()
{
  switch (current_state_) {
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

void PurePursuitNode::ControlLoop()
{
  // 1. 自己位置取得
  getCurrentPose();
  // 2. 状態更新
  colisionCheck();
  updateState();
  geometry_msgs::msg::Twist cmd;
  switch (current_state_) {
    case RobotState::STOP:
    case RobotState::GOAL:    
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      twist_publisher_->publish(cmd);
      break;
    case RobotState::START_ROTATE:
      cmd.linear.x = 0.0;
      cmd.angular.z = calculateAngularVelocity(current_path_->poses.front().pose);
      twist_publisher_->publish(cmd);
      break;
    case RobotState::FOLLOW:
      if (findTargetPoint()) {
        detect_velocity();
        publishMarkers(target_point_);
      }
      break;
    case RobotState::GOAL_ROTATE:
      cmd.linear.x = 0.0;
      cmd.angular.z = calculateAngularVelocity(current_path_->poses.back().pose);
      twist_publisher_->publish(cmd);
      break;
  }
  std_msgs::msg::String mode_msg;
  mode_msg.data = getCurrentMode();
  mode_publisher_->publish(mode_msg);
}

double PurePursuitNode::calculateAngularVelocity(const geometry_msgs::msg::Pose & target_point)
{
  double target_yaw = tf2::getYaw(target_point.orientation);
  double current_yaw = tf2::getYaw(current_pose_.orientation);
  double angle_diff = angles::shortest_angular_distance(current_yaw,target_yaw);
  double factor = std::abs(angle_diff) / M_PI_2; // 45度を基準に
  return std::min(0.1,std::max(2.0,factor * max_angular_velocity_));
}

double PurePursuitNode::calc_dinstanse(size_t target_idx)
{
  const auto& p1 = current_path_->poses[target_idx].pose.position;
  const auto& p2 = current_path_->poses[target_idx + 1].pose.position;
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double seg_len2 = dx * dx + dy * dy;
  if (seg_len2 < 1e-6) {
    return 0.0; // 同じ点の場合は距離0
  }
  double t = ((current_pose_.position.x - p1.x) * dx +
              (current_pose_.position.y - p1.y) * dy)
              / seg_len2;
  t = std::clamp(t, 0.0, 1.0);
  geometry_msgs::msg::Point proj;
  proj.x = p1.x + t * dx;
  proj.y = p1.y + t * dy;
  proj.z = p1.z + t * (p2.z - p1.z);
  double dist = std::hypot(
    current_pose_.position.x - proj.x,
    current_pose_.position.y - proj.y);
  return dist;
}


bool PurePursuitNode::findTargetPoint()
{ 
  // 現在位置に最も近いパス上のポイントを見つける
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = closest_idx_; i < current_path_->poses.size() - 1; ++i) {
    double dist = calc_dinstanse(i);
    if (dist< min_dist) {
      closest_idx_ = i;
    }
  }

  // ルックアヘッド距離に基づいてターゲットポイントを決定
  double min_proj_dist = std::numeric_limits<double>::max();
  target_idx_ = closest_idx_;
  for (size_t i = closest_idx_; i < current_path_->poses.size(); ++i) {
    double dist = std::hypot(
      current_path_->poses[i].pose.position.x - current_pose_.position.x,
      current_path_->poses[i].pose.position.y - current_pose_.position.y);
    double proj_dist =  abs(dist - lookahead_distance_);
    if (proj_dist < min_proj_dist) {
      min_proj_dist = proj_dist;
      target_idx_ = i;
    }
  }  
  target_point_ = current_path_->poses[target_idx_].pose.position;
  return true;
}

void PurePursuitNode::detect_velocity()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_velocity_ * lookahead_distance_ / max_lookahead_distance_;
  double yaw = tf2::getYaw(current_pose_.orientation);
  double dx = target_point_.x - current_pose_.position.x;
  double dy = target_point_.y - current_pose_.position.y;
  double target_yaw = std::atan2(dy, dx);
  // double local_y = -dx * sin(yaw) + dy * cos(yaw);
  // double curvature = 2.0 * local_y / (lookahead_distance_ * lookahead_distance_);
  // double angular_velocity = curvature * linea=r_velocity_;
  double remain_dist = calculateDistance(current_pose_.position, current_path_->poses.back().pose.position);
  double diff_angle;
  if (remain_dist < 0.5) {    
    double goal_yaw = tf2::getYaw(current_path_->poses.back().pose.orientation);
    diff_angle = angles::shortest_angular_distance(target_yaw, yaw) * 0.5 + 
                        angles::shortest_angular_distance(goal_yaw, yaw) * 0.5;
  }else{
    diff_angle = angles::shortest_angular_distance(target_yaw, yaw);
  }
  double angular_velocity = 2 * cmd_vel.linear.x * sin(diff_angle) / lookahead_distance_;
  cmd_vel.angular.z = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);
  twist_publisher_->publish(cmd_vel);
}

bool PurePursuitNode::isStartAngleReached()
{
  double target_yaw = tf2::getYaw(current_path_->poses.front().pose.orientation);
  double current_yaw = tf2::getYaw(current_pose_.orientation);
  double angle_diff = angles::shortest_angular_distance(target_yaw,current_yaw);
  return std::abs(angle_diff) < rotation_threshold_;
}

bool PurePursuitNode::isGoalReached(
  const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  double distance = calculateDistance(current_pose.position, goal_pose.pose.position);
  return distance < goal_tolerance_;
}

bool PurePursuitNode::isGoalAngleReached()
{
  double goal_yaw = tf2::getYaw(current_path_->poses.back().pose.orientation);
  double current_yaw = tf2::getYaw(current_pose_.orientation);
  double angle_diff = angles::shortest_angular_distance(goal_yaw, current_yaw);
  return std::abs(angle_diff) < rotation_threshold_;
}

double PurePursuitNode::calculateDistance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool PurePursuitNode::colisionCheck()
{
  if (!costmap_received_) {
    return false;
  }

  double resolution = costmap_.info.resolution;
  double origin_x = costmap_.info.origin.position.x;
  double origin_y = costmap_.info.origin.position.y;
  int width = costmap_.info.width;
  int height = costmap_.info.height;

  int robot_x_idx = static_cast<int>((current_pose_.position.x - origin_x) / resolution);
  int robot_y_idx = static_cast<int>((current_pose_.position.y - origin_y) / resolution);

  //並列化
  #pragma omp parallel for
  int radius_cells = static_cast<int>(1.0 / resolution); // 1.0mの範囲をチェック
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      int x_idx = robot_x_idx + dx;
      int y_idx = robot_y_idx + dy;
      if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height) {
        int index = y_idx * width + x_idx;
        int cost_value = costmap_.data[index];
        const int collision_threshold = 50;
        if (cost_value >= collision_threshold) {
          return true; // 衝突あり
        }
      }
    }
  }
  return false; // 衝突なし
}

void PurePursuitNode::publishMarkers(const geometry_msgs::msg::Point & target_point)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker target_marker;
  target_marker.header.frame_id = map_frame_id_;
  target_marker.header.stamp = now();
  target_marker.ns = "target_point";
  target_marker.id = 0;
  target_marker.type = visualization_msgs::msg::Marker::SPHERE;
  target_marker.action = visualization_msgs::msg::Marker::ADD;
  target_marker.pose.position = target_point;
  target_marker.pose.orientation.w = 1.0;
  target_marker.scale.x = 0.2;
  target_marker.scale.y = 0.2;
  target_marker.scale.z = 0.2;
  target_marker.color.r = 1.0;
  target_marker.color.g = 0.0;
  target_marker.color.b = 0.0;
  target_marker.color.a = 1.0;
  target_marker.lifetime = rclcpp::Duration(100ms);
  marker_array.markers.push_back(target_marker);
  marker_publisher_->publish(marker_array);
}
} // namespace pure_pursuit_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit_planner::PurePursuitNode)


