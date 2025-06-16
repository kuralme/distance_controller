#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {
public:
  DistanceController(int scene_number)
      : Node("distance_controller"), scene_number_(scene_number),
        got_odom_(false), wp_reached_(false), init_(true), paused_(false),
        target_wp_(0) {
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&DistanceController::odomCallback, this,
                  std::placeholders::_1));
    timer_ = this->create_wall_timer(
        200ms, std::bind(&DistanceController::executeCallback, this));

    SelectWaypoints();
  }

private:
  void SelectWaypoints() {
    // Waypoints [dx, dy, dphi] in robot frame
    switch (scene_number_) {
    case 1: // Simulation
      waypoints_ = {{
          {0.0, 0.0, 0.0},      // w1
          {0.475, -0.01, 0.0},  // w2
          {0.509, -1.315, 0.0}, // w3
          {0.475, -0.01, 0.0},  // w4
          {0.0, 0.0, 0.0},      // w5
      }};
      break;

    case 2: // CyberWorld
      waypoints_ = {{
          {0.0, 0.0, 0.0},   // w1
          {1.0, 1.0, 0.0},   // w2
          {1.0, 1.0, 0.0},   // w3
          {1.0, -1.0, 0.0},  // w4
          {-1.0, -1.0, 0.0}, // w5
          {0.0, 1.0, 0.0},   // w6
          {-1.0, 1.0, 0.0},  // w7
          {0.0, -1.0, 0.0},  // w8
          {1.0, -1.0, 0.0},  // w9
          {-1.0, -1.0, 0.0}, // w10
          {-1.0, 1.0, 0.0},  // w11
          {-1.0, 1.0, 0.0},  // w12
          {-1.0, -1.0, 0.0}, // w13
          {-1.0, 1.0, 0.0},  // w14
          {-1.0, -1.0, 0.0}  // w15
      }};
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto orientation = msg->pose.pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z,
                      orientation.w);
    current_pose_(0) = msg->pose.pose.position.x;
    current_pose_(1) = msg->pose.pose.position.y;
    current_pose_(2) = tf2::impl::getYaw(q);
    got_odom_ = true;
  }

  void executeCallback() {
    if (!got_odom_) {
      RCLCPP_WARN(this->get_logger(), "Odom data not received!");
      return;
    }

    // Paused state
    if (paused_) {
      rclcpp::Time now = clock_->now();
      // Check if 2 seconds have elapsed
      if ((now - pause_time_).seconds() >= 2.0) {
        paused_ = false;
      } else {
        auto msg = geometry_msgs::msg::Twist();
        twist_pub_->publish(msg);
        return;
      }
    }

    // Update target waypoint
    if (wp_reached_ || init_) {
      target_pose_ = current_pose_ + waypoints_[target_wp_];

      wp_reached_ = false;
      init_ = false;
      size_t twp = target_wp_ + 1;
      RCLCPP_INFO(this->get_logger(), "Moving to next waypoint: %ld", twp);
    }

    // Error vector
    Eigen::Vector3f error_pose = target_pose_ - current_pose_;

    // Check distance to target
    if (error_pose.norm() < 0.02) {
      auto msg = geometry_msgs::msg::Twist();
      twist_pub_->publish(msg);

      prev_error_ = {0.0, 0.0, 0.0};
      integral_error_ = {0.0, 0.0, 0.0};
      wp_reached_ = true;
      target_wp_++;

      if (target_wp_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Maze finished!");
        rclcpp::shutdown();
      } else {
        // Start the pause timer
        pause_time_ = clock_->now();
        paused_ = true;
        RCLCPP_INFO(this->get_logger(), "Stopping briefly...");
      }
      return;
    }

    // PID Controller (Proportional + Integral + Derivative)
    integral_error_ += error_pose; // Sum of errors over time
    Eigen::Vector3f V = Kp_ * error_pose + Kd_ * (error_pose - prev_error_) +
                        Ki_ * integral_error_;
    prev_error_ = error_pose; // for next iteration

    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear.x = V(0);
    cmd_vel.linear.y = V(1);
    cmd_vel.angular.z = V(2);
    twist_pub_->publish(cmd_vel);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time pause_time_;
  int scene_number_;
  bool got_odom_, wp_reached_, init_, paused_;
  size_t target_wp_;
  std::shared_ptr<rclcpp::Clock> clock_;
  Eigen::Vector3f current_pose_{0.0, 0.0, 0.0};
  Eigen::Vector3f target_pose_{0.0, 0.0, 0.0};
  Eigen::Vector3f prev_error_{0.0, 0.0, 0.0};
  Eigen::Vector3f integral_error_{0.0, 0.0, 0.0};
  std::array<Eigen::Vector3f, 15> waypoints_;

  // PID Gains
  const float Kp_ = 1.2;
  const float Ki_ = 0.1;
  const float Kd_ = 0.8;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Check if a scene number argument is provided
  int scene_number = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  auto controller_node = std::make_shared<DistanceController>(scene_number);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
