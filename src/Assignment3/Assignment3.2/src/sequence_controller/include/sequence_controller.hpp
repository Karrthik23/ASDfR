#ifndef SEQUENCE_CONTROLLER_HPP
#define SEQUENCE_CONTROLLER_HPP

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include "xrf2_msgs/msg/ros2_xeno.hpp"

class SequenceController : public rclcpp::Node {
public:
  SequenceController();

private:
  // Called periodically to compute and publish motor commands
  void sequence_controller();

  // Receives position of the green object in the image
  void update_greenobject_pos(const geometry_msgs::msg::Point &msg);

  // Internal data
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

#endif  // SEQUENCE_CONTROLLER_HPP
