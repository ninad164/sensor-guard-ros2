#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

static double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
static double deadband(double v, double db) { return (std::fabs(v) < db) ? 0.0 : v; }

class CmdVelGuard : public rclcpp::Node {
public:
  CmdVelGuard() : Node("cmd_vel_guard")
  {
    max_lin_ = this->declare_parameter("max_linear", 0.5);
    max_ang_ = this->declare_parameter("max_angular", 1.0);
    db_lin_  = this->declare_parameter("deadband_linear", 0.02);
    db_ang_  = this->declare_parameter("deadband_angular", 0.05);

    in_topic_  = this->declare_parameter("input_topic",  std::string("/cmd_vel_raw"));
    out_topic_ = this->declare_parameter("output_topic", std::string("/cmd_vel"));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(out_topic_, 10);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      in_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::Twist out = *msg;

        out.linear.x  = clamp(deadband(out.linear.x,  db_lin_), -max_lin_, max_lin_);
        out.angular.z = clamp(deadband(out.angular.z, db_ang_), -max_ang_, max_ang_);

        pub_->publish(out);
      });

    RCLCPP_INFO(this->get_logger(), "Guarding %s -> %s", in_topic_.c_str(), out_topic_.c_str());
  }

private:
  std::string in_topic_, out_topic_;
  double max_lin_{0.5}, max_ang_{1.0}, db_lin_{0.02}, db_ang_{0.05};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelGuard>());
  rclcpp::shutdown();
  return 0;
}
