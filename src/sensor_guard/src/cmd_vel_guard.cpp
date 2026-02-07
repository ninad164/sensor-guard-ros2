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
    max_lin_ = this->declare_parameter("max_linear", 0.5);    // basic params.
    max_ang_ = this->declare_parameter("max_angular", 1.0);
    db_lin_  = this->declare_parameter("deadband_linear", 0.02);
    db_ang_  = this->declare_parameter("deadband_angular", 0.05);
    
    max_dlin_ = this->declare_parameter("max_delta_linear_per_sec", 1.0);   // rate limiters
    max_dang_ = this->declare_parameter("max_delta_angular_per_sec", 2.0);
    last_out_.linear.x = 0.0;   //new state
    last_out_.angular.z = 0.0;

    last_time_ = this->now();
    has_last_ = true;

    in_topic_  = this->declare_parameter("input_topic",  std::string("/cmd_vel_raw"));
    out_topic_ = this->declare_parameter("output_topic", std::string("/cmd_vel"));
    
    // Create publisher
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(out_topic_, 10);
    // Create subscription
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      in_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        const auto now_t = this->now();
        double dt=0.0;
        
        if (has_last_) {
          dt = (now_t - last_time_).seconds();
          if(dt<=1e-6) dt=1e-6;  // prevent undefined behavior on very fast messages
          dt = std::clamp(dt, 0.001, 0.2);
        }
        
        // RCLCPP_INFO(this->get_logger(), "dt=%.6f has_last=%d last_out=(%.3f,%.3f)",
        //     dt, has_last_ ? 1 : 0, last_out_.linear.x, last_out_.angular.z);

        geometry_msgs::msg::Twist out = *msg;
        
        out.linear.x  = clamp(deadband(out.linear.x,  db_lin_), -max_lin_, max_lin_);
        out.angular.z = clamp(deadband(out.angular.z, db_ang_), -max_ang_, max_ang_);
        
        // RCLCPP_INFO(this->get_logger(),
        //   "dt=%.3f last=(%.3f,%.3f) desired=(%.3f,%.3f) max_step=(%.3f,%.3f)",
        //   dt,
        //   last_out_.linear.x, last_out_.angular.z,
        //   out.linear.x, out.angular.z,
        //   max_dlin_ * dt, max_dang_ * dt
        // );

        if (has_last_) {
          const double max_step_lin = max_dlin_ * dt;
          const double max_step_ang = max_dang_ * dt;

          out.linear.x = clamp(
            out.linear.x,
            last_out_.linear.x - max_step_lin,
            last_out_.linear.x + max_step_lin
          );

          out.angular.z = clamp(
            out.angular.z,
            last_out_.angular.z - max_step_ang,
            last_out_.angular.z + max_step_ang
          );
        }

        pub_->publish(out);

        last_out_ = out;
        last_time_ = now_t;
        has_last_ = true;
      });

    RCLCPP_INFO(this->get_logger(), "Guarding %s -> %s", in_topic_.c_str(), out_topic_.c_str());
  }

private:
  std::string in_topic_, out_topic_;
  double max_lin_{0.5}, max_ang_{1.0}, db_lin_{0.02}, db_ang_{0.05};
  
  // Rate limits (per second)
  double max_dlin_{1.0};   // m/s^2 (rate-of-change limit)
  double max_dang_{2.0};   // rad/s^2
  rclcpp::Time last_time_;
  geometry_msgs::msg::Twist last_out_;
  bool has_last_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                       // initialize ROS 2.
  rclcpp::spin(std::make_shared<CmdVelGuard>());  // starts the event loop forever.
  rclcpp::shutdown();
  return 0;
}
