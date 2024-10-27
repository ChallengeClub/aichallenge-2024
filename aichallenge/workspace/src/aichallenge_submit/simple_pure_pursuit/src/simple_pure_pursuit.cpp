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
  lookahead_gain2_(declare_parameter<float>("lookahead_gain2", 1.0)),// 先読み用
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  steering_diff_gain_(declare_parameter<float>("steering_diff_gain", 0.5)),  // 操舵制御用
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  use_steer_angle_v_limit_(declare_parameter<bool>("use_steer_angle_v_limit", false)),
  v_limit_angle_(declare_parameter<float>("v_limit_angle", 0.20933)),  // 12degree
  v_limit_angle2_(declare_parameter<float>("v_limit_angle2", 0.20933)),  // 先読み用
  angle_limit_v_(declare_parameter<float>("angle_limit_v", 4.16667)),  // 15km/h
  angle_limit_v2_(declare_parameter<float>("angle_limit_v2", 4.16667)),  // 先読み用
  predict_time_(declare_parameter<float>("predict_time", 0.5)),  // 先読み用
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

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

  double current_longitudinal_vel = odometry_->twist.twist.linear.x; // 現在の速度
  double yaw = tf2::getYaw(odometry_->pose.pose.orientation);// 現在の車体の向き。x軸と一致する向きが0
  double predicted_x = odometry_->pose.pose.position.x + std::cos(yaw) * current_longitudinal_vel * predict_time_;
  double predicted_y = odometry_->pose.pose.position.y + std::sin(yaw) * current_longitudinal_vel * predict_time_;
  double predicted_yaw = yaw + odometry_->twist.twist.angular.z * predict_time_; // zは、車体の角速度（ラジアン/秒）
  geometry_msgs::msg::Pose predicted_pos = odometry_->pose.pose;
  predicted_pos.position.x = predicted_x;
  predicted_pos.position.y = predicted_y;

//  size_t closet_traj_point_idx =
//    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);///■これ、まずい。odometryの中身を書き換える。
  size_t closet_traj_point_idx = findNearestIndex(trajectory_->points, predicted_pos.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  if (
    (closet_traj_point_idx == trajectory_->points.size() - 1) ||
    (trajectory_->points.size() <= 2)) {
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -10.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
  } else {
    // get closest trajectory point from current position
    TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

    // calc longitudinal speed and acceleration
    double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
//    double current_longitudinal_vel = odometry_->twist.twist.linear.x;  上で宣言済み
//    下記は操舵角が決まってから設定するので下に移動
//    cmd.longitudinal.speed = target_longitudinal_vel;
//    cmd.longitudinal.acceleration =
//      speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

    // calc lateral control
    //// calc lookahead distance
    double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;

    //// calc center coordinate of rear wheel
/*  予測位置で算出するように変更
    double rear_x = odometry_->pose.pose.position.x -
                    wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
    double rear_y = odometry_->pose.pose.position.y -
                    wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
*/
    // zは誤っているが、一旦もとに戻す(10/27 20:27)
//    double rear_x = predicted_x - wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
//    double rear_y = predicted_y - wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
    double rear_x = predicted_x - wheel_base_ / 2.0 * std::cos(predicted_yaw);
    double rear_y = predicted_y - wheel_base_ / 2.0 * std::sin(predicted_yaw);

    //// search lookahead point
    auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
      [&](const TrajectoryPoint & point) {
        return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
               lookahead_distance;
      });
    if (lookahead_point_itr == trajectory_->points.end()) {
      lookahead_point_itr = trajectory_->points.end() - 1;
    }
    double lookahead_point_x = lookahead_point_itr->pose.position.x;
    double lookahead_point_y = lookahead_point_itr->pose.position.y;

    geometry_msgs::msg::PointStamped lookahead_point_msg;
    lookahead_point_msg.header.stamp = get_clock()->now();
    lookahead_point_msg.header.frame_id = "map";
    if (true) { // Original ルックアヘッド位置
      lookahead_point_msg.point.x = lookahead_point_x;
      lookahead_point_msg.point.y = lookahead_point_y;
      lookahead_point_msg.point.z = predicted_yaw;
    } else {    // デバッグ情報色々
  //    lookahead_point_msg.point.x = odometry_->pose.pose.position.x;
  //    lookahead_point_msg.point.y = odometry_->pose.pose.position.y;
      lookahead_point_msg.point.x = predicted_x;
      lookahead_point_msg.point.y = predicted_y;
  //    lookahead_point_msg.point.x = rear_x;
  //    lookahead_point_msg.point.y = rear_y;
      lookahead_point_msg.point.z = predicted_yaw;
  //    lookahead_point_msg.point.z = lookahead_distance;// 問題なさそう。
  //    lookahead_point_msg.point.z = predicted_x;  // こちらも一応連続になった。
    }
    pub_lookahead_point_->publish(lookahead_point_msg);

    // calc steering angle for lateral control
    // 以下、Original
//    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
//                   tf2::getYaw(odometry_->pose.pose.orientation);
    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) - predicted_yaw; // 車体の位置と、向きを予測
  // 操舵ブレ対策として、本来なら、車体の向きはの加速度に制限をかけるべきだが、根拠はないが、yawを現在地との間にしてみる。・・・うまく行かなかったので、コメントアウト
//    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) - (predicted_yaw + yaw) / 2; // 車体の位置と、向きを予測
    cmd.lateral.steering_tire_angle =
      steering_tire_angle_gain_ * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);

    //  上記で、simple_pure_pursuitによる操舵角を算出
    //　実際のステアリングの制御は触れるファイルとしては存在指定なさそう、すなわち、
    //  操舵角指示への追従制御を触れないので、指示角を調整するのが良さそう。
    //  前回指示角と今回指示角の増減から、指示を調整する。差が大きくなっていれば、その差分を加える。
    //  差が小さくなっていれば、もとの指示値のままとする。
    //  走行速度を考慮したゲイン調整については、速度による操舵遅れも操舵指示の大きさの変化に反映済みと考えて、不要とする。

    // ここから追加
/*
    double steering_diff;
    if ((last_steering_angle >= 0 && cmd.lateral.steering_tire_angle >= 0)
    || (last_steering_angle <  0 && cmd.lateral.steering_tire_angle <  0)) { // 前回の操舵向きと同じ
      steering_diff = abs(cmd.lateral.steering_tire_angle) - abs(last_steering_angle);
      if (steering_diff > 0) { // 前回よりも指示値が大きくなった、すなわち、追従できていない。
        if (last_steering_angle >= 0) 
          cmd.lateral.steering_tire_angle += steering_diff * steering_diff_gain_;
        else
          cmd.lateral.steering_tire_angle -= steering_diff * steering_diff_gain_;
      }
    }
*/  //  だめだ。戻しが遅い。戻しについても、微分制御を入れなければ、間に合わない。
    last_steering_angle = cmd.lateral.steering_tire_angle;
    // 追加終わり

//  操舵角による速度制限
    if (use_steer_angle_v_limit_
     && (std::fabs(cmd.lateral.steering_tire_angle) > v_limit_angle_
      || std::fabs(odometry_->twist.twist.angular.z) > v_limit_angle_)) {
      cmd.longitudinal.speed = angle_limit_v_;
    } else
      cmd.longitudinal.speed = target_longitudinal_vel;
    cmd.longitudinal.acceleration =
      speed_proportional_gain_ * (cmd.longitudinal.speed - current_longitudinal_vel); //  速度は振動しそうだ

/*
    //　操舵指令値の方で制限をかけようとしたコード。ややこしくなるのでやめ。actuation_cmd_converter.cpp側でかけた。
    // 操舵速度制限は、0.35rad/s * 30ms = 0.0105
    if (fabs(cmd.lateral.steering_tire_angle - current_steering_angle) > angle_limit_v_ ) {
      if (cmd.lateral.steering_tire_angle > current_steering_angle) {
        cmd.lateral.steering_tire_angle = current_steering_angle + angle_limit_v_;
      }
      else if (cmd.lateral.steering_tire_angle < current_steering_angle) {
        cmd.lateral.steering_tire_angle = current_steering_angle - angle_limit_v_;
      }
    }
*/
/*
    if (use_steer_angle_v_limit_) { // 実機の操舵指令値と実際の曲がり具合を調べるためのコード。余裕がなくて使わなかった。
      if (cmd.lateral.steering_tire_angle < 0
       && cmd.lateral.steering_tire_angle < -v_limit_angle_)
        cmd.lateral.steering_tire_angle = -v_limit_angle_;
      else if (cmd.lateral.steering_tire_angle >= 0
       && cmd.lateral.steering_tire_angle > v_limit_angle_)
        cmd.lateral.steering_tire_angle = v_limit_angle_;
    }
*/
  }
  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
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
