#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  steering_diff_gain_(declare_parameter<float>("steering_diff_gain", 0.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  last_steering_angle(0.0)
{
  this->declare_parameter("minimum_trj_point_size", 10);
  minimum_trj_point_size_ = this->get_parameter("minimum_trj_point_size").as_int();
  
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  need_to_stop_sub_ = create_subscription<Bool>(
    "/planning/mission_planning/need_to_stop", 1, [this](const Bool::SharedPtr msg) { need_to_stop_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{

  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closest_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());
  double target_longitudinal_vel = 0.0;
  if (
    ((closest_traj_point_idx == trajectory_->points.size() - 1) ||
    (trajectory_->points.size() <= minimum_trj_point_size_)) &&
    need_to_stop_->data) {
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -60.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
  } else {
    // get closest trajectory point from current position
    TrajectoryPoint closest_traj_point = trajectory_->points.at(closest_traj_point_idx);

    // calc longitudinal speed and acceleration
    target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_ : closest_traj_point.longitudinal_velocity_mps;
    double current_longitudinal_vel = odometry_->twist.twist.linear.x;

    cmd.longitudinal.speed = target_longitudinal_vel;
    cmd.longitudinal.acceleration =
      speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);
  }
  // calc lateral control　★ここから操舵制御
  //// calc lookahead distance
  double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
  
  //// calc center coordinate of rear wheel
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
  //// search lookahead point
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closest_traj_point_idx, trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
              lookahead_distance;
    });
  if (lookahead_point_itr == trajectory_->points.end()) {
    lookahead_point_itr = trajectory_->points.end() - 1;
  }
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  // calc steering angle for lateral control　★操舵角を求める ここにフィードバックを入れたほうが良いかも。
  // 現在の操舵角と、目標操舵角の差、および、差が大きくなっていれば小さくするために、PD制御を入れる。
  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                  tf2::getYaw(odometry_->pose.pose.orientation);
  cmd.lateral.steering_tire_angle =
    std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
// 操舵角の指示しかなく、実際のステアリングの制御はどこだ？
//  操舵角指示への追従制御を触れない場合、指示角を調整するのが良さそう
//  前回指示角と今回指示角の増減から、指示を調整する。差が大きくなっていれば、その差分を加える。
//  差が小さくなっていれば、維持すれば良いかな？
//  走行速度を考慮したゲイン調整については、速度による操舵遅れも操舵指示の大きさの変化に反映済みと考えられるので、不要と考える。
// ここから追加
  double steering_diff;

  if ((last_steering_angle >= 0 && cmd.lateral.steering_tire_angle >= 0)
   || (last_steering_angle < 0 && cmd.lateral.steering_tire_angle < 0)) { // 前回の操舵向きと同じ
    steering_diff = abs(cmd.lateral.steering_tire_angle) - abs(last_steering_angle);
    if (steering_diff > 0) { // 前回よりも指示値が大きくなった、すなわち、追従できていない。
      if (last_steering_angle >= 0) 
        cmd.lateral.steering_tire_angle += steering_diff * steering_diff_gain_;
      else
        cmd.lateral.steering_tire_angle -= steering_diff * steering_diff_gain_;
    }
  }
  last_steering_angle = cmd.lateral.steering_tire_angle;
// 追加終わり
  pub_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  return true;
}
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
