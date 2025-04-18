#ifndef SEQUENCE_CONTROLLER_HPP
#define SEQUENCE_CONTROLLER_HPP

#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include "xrf2_msgs/msg/ros2_xeno.hpp"

class SequenceController : public rclcpp::Node {
public:
  SequenceController();

private:
  // Main loop: publishes motor velocities via XRF2 bridge
  void sequence_controller();

  // Updates object position based on image processing
  void update_greenobject_pos(const geometry_msgs::msg::Point &msg);

  // Internal state
  size_t count_;
  double sample_time_s_;
  geometry_msgs::msg::Point greenobject_pos_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_greenobject_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right_;
  rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr pub_xeno_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // SEQUENCE_CONTROLLER_HPP
