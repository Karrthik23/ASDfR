#include "sequence_controller.hpp"

using std::placeholders::_1;

SequenceController::SequenceController()
  : Node("sequence_controller"), count_(0)
{
  // sample time in seconds
  sample_time_s_ = 0.03;

  // subscriber to pixel coordinates of the green object
  subscription_greenobject_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
    "pixel_coordinates", 10, std::bind(&SequenceController::update_greenobject_pos, this, _1));

  // publishers to motor velocity topics
  publisher_left_ = this->create_publisher<std_msgs::msg::Float64>("left_motor_setpoint_vel", 10);
  publisher_right_ = this->create_publisher<std_msgs::msg::Float64>("right_motor_setpoint_vel", 10);

  // periodic timer to trigger the control loop
  timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::duration<double>(sample_time_s_),
    std::bind(&SequenceController::sequence_controller, this));

  // parameters for control gain and camera width
  this->declare_parameter("gain", 0.2);
  this->declare_parameter("width", 360);
}

void SequenceController::sequence_controller()
{
  auto gain = this->get_parameter("gain").as_double();
  auto width = this->get_parameter("width").as_int();

  // compute error as horizontal deviation from center
  double e = gain * (greenobject_pos_.x - (width / 2));

  RCLCPP_INFO(this->get_logger(), "greenobject_pos.x: %f, e: %f", greenobject_pos_.x, e);

  // set opposite velocities for turning
  auto vel_left = std_msgs::msg::Float64();
  auto vel_right = std_msgs::msg::Float64();

  vel_left.data = e;
  vel_right.data = -e;

  publisher_left_->publish(vel_left);
  publisher_right_->publish(vel_right);
}

void SequenceController::update_greenobject_pos(const geometry_msgs::msg::Point &msg)
{
  // ignore invalid points
  if (msg.x == -1)
    return;

  greenobject_pos_.x = msg.x;
  greenobject_pos_.y = msg.y;
}

// main function remains in .cpp
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceController>());
  rclcpp::shutdown();
  return 0;
}
