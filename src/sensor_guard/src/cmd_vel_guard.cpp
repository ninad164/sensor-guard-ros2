// L1: Stateful control (not robust)
// L2: Statistical filtering - max speed, max rate limit.
// L3: system awareness, and not just numerical filtering

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <deque>  //add remove values from both ends.
#include <numeric>  //to find avgs, variances, etc
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/string.hpp>

enum class SafetyState {
  NORMAL,
  DEGRADED,
  FAULT
};

static const char* to_string(SafetyState s) {
  switch (s) {
    case SafetyState::NORMAL:   return "NORMAL";
    case SafetyState::DEGRADED: return "DEGRADED";
    case SafetyState::FAULT:    return "FAULT";
    default: return "UNKNOWN";
  }
}

static double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
static double deadband(double v, double db) { return (std::fabs(v) < db) ? 0.0 : v; }

static double mean_of(const std::deque<double>& d) {
  if (d.empty()) return 0.0;
  const double sum = std::accumulate(d.begin(), d.end(), 0.0);
  return sum/static_cast<double>(d.size());
}

static double stddev_of(const std::deque<double>& d, double mean) {
  if (d.size() < 2) return 0.0;
  double acc = 0.0;
  for (double x : d) {
    const double e = x - mean;
    acc += e * e;
  }
  return std::sqrt(acc / static_cast<double>(d.size()));
}

class CmdVelGuard : public rclcpp::Node {
public:
  CmdVelGuard() : Node("cmd_vel_guard")
  {
    window_size_ = this->declare_parameter("window_size", 10);  // window size for L2.
    outlier_k_ = this->declare_parameter("outlier_k", 2.5);
    min_sigma_ = this->declare_parameter("min_sigma", 1e-3);

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
    
    timeout_sec_ = this->declare_parameter("timeout_sec", 0.5);
    last_input_time_ = this->now();

    // L3 publisher:
    
    safety_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/safety_state", 10
    );

    // Create publisher
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(out_topic_, 10);
    // Create subscription
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      in_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_input_time_ = this->now();
        const auto now_t = this->now();
        double dt=0.0;
        
        if (has_last_) {
          dt = (now_t - last_time_).seconds();
          if(dt<=1e-6) dt=1e-6;  // prevent undefined behavior on very fast messages
          dt = std::clamp(dt, 0.001, 0.2);
        }
        
        geometry_msgs::msg::Twist out = *msg;
        
        out.linear.x  = clamp(deadband(out.linear.x,  db_lin_), -max_lin_, max_lin_);
        out.angular.z = clamp(deadband(out.angular.z, db_ang_), -max_ang_, max_ang_);
        
        lin_hist_.push_back(out.linear.x);
        ang_hist_.push_back(out.angular.z);
        if((int)lin_hist_.size()>window_size_) lin_hist_.pop_front();
        if((int)ang_hist_.size()>window_size_) ang_hist_.pop_front();

        if((int)lin_hist_.size()>=5) {
          const double m = mean_of(lin_hist_);
          const double s = std::max(stddev_of(lin_hist_,m),min_sigma_);
          if(std::fabs(out.linear.x-m)>outlier_k_*s){
            out.linear.x = m;
          }
        }

        if ((int)ang_hist_.size()>=5) {
          const double m = mean_of(ang_hist_);
          const double s = std::max(stddev_of(ang_hist_, m), min_sigma_);
          if (std::fabs(out.angular.z - m) > outlier_k_ * s) {
            out.angular.z = m;
          }  
        }

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

    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        const auto now_t = this->now();
        const double since = (now_t - last_input_time_).seconds();

        SafetyState new_state = (since > timeout_sec_) ? SafetyState::FAULT : SafetyState::NORMAL;

        if (new_state != safety_state_) {
          safety_state_ = new_state;
          std_msgs::msg::String msg;
          msg.data = to_string(safety_state_);
          safety_pub_->publish(msg);
        }

        // If faulted, force stop output
        if (safety_state_ == SafetyState::FAULT) {
          geometry_msgs::msg::Twist stop;
          stop.linear.x = 0.0;
          stop.angular.z = 0.0;
          pub_->publish(stop);
          last_out_ = stop;  // keep state consistent
        }
      }
    );

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

  // Level 2: rate limiter
  int window_size_{10};
  std::deque<double> lin_hist_;
  std::deque<double> ang_hist_;
  double outlier_k_{2.5};
  double min_sigma_{1e-3};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

  // L3: add publisher for metrics check
  SafetyState safety_state_{SafetyState::NORMAL};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_pub_;

  double timeout_sec_{0.5};          // how long without input before FAULT
  rclcpp::Time last_input_time_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                       // initialize ROS 2.
  rclcpp::spin(std::make_shared<CmdVelGuard>());  // starts the event loop forever.
  rclcpp::shutdown();
  return 0;
}
