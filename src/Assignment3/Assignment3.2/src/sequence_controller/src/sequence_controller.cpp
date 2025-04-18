#include "sequence_controller.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

SequenceController::SequenceController()
  : Node("sequence_controller"), count_(0)
{
  sample_time_s_ = 0.03;

  // Subscribe to the position of the green object (from image processing)
  subscription_greenobject_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
    "pixel_coordinates", 10,
    std::bind(&SequenceController::update_greenobject_pos, this, _1));

  // Publishers for direct motor velocity commands (optional, may not be used)
  publisher_left_ = this->create_publisher<std_msgs::msg::Float64>(
    "left_motor_setpoint_vel", 10);

  publisher_right_ = this->create_publisher<std_msgs::msg::Float64>(
    "right_motor_setpoint_vel", 10);

  // Publisher to Xeno bridge (main interface to FRT system)
  pub_xeno_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
    "Ros2Xeno", 10);

  // Timer to trigger the control loop periodically
  timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::duration<double>(sample_time_s_),
    std::bind(&SequenceController::sequence_controller, this));

  // Declare tunable parameters
  this->declare_parameter("gain", 0.2);
  this->declare_parameter("width", 360);
}

void SequenceController::sequence_controller()
{
  auto gain = this->get_parameter("gain").as_double();
  auto width = this->get_parameter("width").as_int();

  // Compute error based on object offset from image center
  double e = gain * (greenobject_pos_.x - (width / 2));

  RCLCPP_INFO(this->get_logger(), "greenobject_pos.x: %f, e: %f", greenobject_pos_.x, e);

  // Create and publish motor command message to FRT side
  auto vel_xeno = xrf2_msgs::msg::Ros2Xeno();
  vel_xeno.left_motor_setpoint_vel = 2047 * 0.25;
  vel_xeno.right_motor_setpoint_vel = -2047 * 0.25;

  pub_xeno_->publish(vel_xeno);
}

void SequenceController::update_greenobject_pos(const geometry_msgs::msg::Point &msg)
{
  if (msg.x == -1)
    return;

  greenobject_pos_.x = msg.x;
  greenobject_pos_.y = msg.y;
}

// Main function
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceController>());
  rclcpp::shutdown();
  return 0;
}
