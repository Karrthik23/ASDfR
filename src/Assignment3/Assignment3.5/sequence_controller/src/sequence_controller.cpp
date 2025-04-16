#include <chrono>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SequenceController : public rclcpp::Node {
  public:
    SequenceController() : Node("sequence_controller"), count_(0), object_lost_counter_(0) {
        sample_time_s_ = 0.03;

        subscription_greenobject_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
            "pixel_coordinates", 10,
            std::bind(&SequenceController::update_greenobject_pos, this, _1));

        pub_xeno_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);

        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::duration<double>(sample_time_s_),
            std::bind(&SequenceController::sequence_controller, this));

        this->declare_parameter("gain", 0.2);
        this->declare_parameter("width", 360);
    }

  private:
    void sequence_controller() {
        auto gain = this->get_parameter("gain").as_double();
        auto width = this->get_parameter("width").as_int();

        auto vel_xeno = xrf2_msgs::msg::Ros2Xeno();

        if (greenobject_pos_.x == -1) {
            object_lost_counter_++;

            RCLCPP_WARN(this->get_logger(), "Green object not detected, counter: %d", object_lost_counter_);

            // If recently lost, stop to wait
            if (object_lost_counter_ <= wait_threshold_) {
                vel_xeno.left_motor_setpoint_vel = 0.0;
                vel_xeno.right_motor_setpoint_vel = 0.0;
                RCLCPP_INFO(this->get_logger(), "Waiting for object to reappear...");
            }
            // After timeout, begin slow scanning motion
            else {
                vel_xeno.left_motor_setpoint_vel = -2047 * scan_speed_;
                vel_xeno.right_motor_setpoint_vel = 2047 * scan_speed_;
                RCLCPP_INFO(this->get_logger(), "Scanning for object...");
            }
        } else {
            object_lost_counter_ = 0; // Reset counter on successful detection
            double e = gain * (greenobject_pos_.x - (width / 2));
            vel_xeno.left_motor_setpoint_vel = e;
            vel_xeno.right_motor_setpoint_vel = -e;
            RCLCPP_INFO(this->get_logger(), "Tracking object. e: %f", e);
        }

        pub_xeno_->publish(vel_xeno);
    }

    void update_greenobject_pos(const geometry_msgs::msg::Point &msg) {
        greenobject_pos_ = msg;
    }

    // Variables
    size_t count_;
    double sample_time_s_;
    geometry_msgs::msg::Point greenobject_pos_;

    int object_lost_counter_;
    const int wait_threshold_ = 30; // ~1 second at 30 Hz
    const double scan_speed_ = 0.10; // slow rotation speed

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_greenobject_pos_;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr pub_xeno_;
    rclcpp::TimerBase::SharedPtr timer_;
};
