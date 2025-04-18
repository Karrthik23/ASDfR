#ifndef SEQUENCE_CONTROLLER_HPP
#define SEQUENCE_CONTROLLER_HPP

#include <chrono>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rclcpp/rclcpp.hpp>

class SequenceController : public rclcpp::Node {
public:
  SequenceController();

private:
  // Main control loop called at a fixed interval
  void sequence_controller();

  // Callback for receiving the position of the green object
  void update_greenobject_pos(const geometry_msgs::msg::Point &msg);

  // Internals
  size_t count_;
  double sample_time_s_;
  geometry_msgs::msg::Point greenobject_pos_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_greenobject_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SEQUENCE_CONTROLLER_HPP
