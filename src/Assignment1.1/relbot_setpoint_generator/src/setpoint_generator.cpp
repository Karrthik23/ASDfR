#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp" 
#include <geometry_msgs/msg/point.hpp>

class SequenceController : public rclcpp::Node {
public:
  SequenceController() : Node("sequence_controller"), step_(0), tracking_mode_(true) {  // Default to tracking mode
    // Motor velocity publishers
    pub_left_ = this->create_publisher<example_interfaces::msg::Float64>("/input/left_motor/setpoint_vel", 10);
    pub_right_ = this->create_publisher<example_interfaces::msg::Float64>("/input/right_motor/setpoint_vel", 10);

    // Subscriber to detected object position
    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/pixel_coordinates", 10,
        std::bind(&SequenceController::position_callback, this, std::placeholders::_1));

    // Timer for manual sequence execution
    timer_ = this->create_wall_timer(std::chrono::seconds(3),
        std::bind(&SequenceController::run_sequence, this));

    // Declare ROS2 parameter for mode switching (default: true)
    this->declare_parameter("tracking_mode", true);

    RCLCPP_INFO(this->get_logger(), "Sequence Controller Initialized in Tracking Mode!");
  }

private:
  int step_;
  bool tracking_mode_;  // Mode switch

  std::vector<std::pair<double, double>> sequence_ = {
      {0.5, 0.0},  // Move forward
      {0.0, 0.5},  // Turn left
      {-0.5, 0.0}, // Move backward
      {0.0, -0.5}, // Turn right
      {0.0, 0.0}   // Stop
  };

  void position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    this->get_parameter("tracking_mode", tracking_mode_); // Check if tracking mode is enabled

    if (!tracking_mode_) return;  

    double image_center_x = 320.0;  // Assuming 640x480 image, center at x=320
    double object_x = msg->x;

    auto left_motor = example_interfaces::msg::Float64();
    auto right_motor = example_interfaces::msg::Float64();

    // Adjust motor speeds based on object position
    if (object_x < image_center_x - 50) {  // Object is to the left
      left_motor.data = 0.2;
      right_motor.data = 0.5;
    } else if (object_x > image_center_x + 50) {  // Object is to the right
      left_motor.data = 0.5;
      right_motor.data = 0.2;
    } else {  // Object is centered
      left_motor.data = 0.5;
      right_motor.data = 0.5;
    }

    pub_left_->publish(left_motor);
    pub_right_->publish(right_motor);
  }

  void run_sequence() {
    this->get_parameter("tracking_mode", tracking_mode_);

    if (tracking_mode_) return;  

    // Execute next step in manual sequence
    auto [linear, angular] = sequence_[step_];
    step_ = (step_ + 1) % sequence_.size();

    auto left_motor = example_interfaces::msg::Float64();
    auto right_motor = example_interfaces::msg::Float64();

    left_motor.data = linear - angular;
    right_motor.data = linear + angular;

    pub_left_->publish(left_motor);
    pub_right_->publish(right_motor);

    RCLCPP_INFO(this->get_logger(), "Manual Mode: Step %d - Left: %f, Right: %f", step_, left_motor.data, right_motor.data);
  }

  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_left_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_right_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceController>());
  rclcpp::shutdown();
  return 0;
}

