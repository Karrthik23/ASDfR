#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp" 
#include <geometry_msgs/msg/point.hpp>
#include <random>

class SequenceController : public rclcpp::Node {
public:
  SequenceController() : Node("sequence_controller"), tracking_mode_(true), generator_(std::random_device{}()), distribution_(-2.5, 2.5) {
    // Motor velocity publishers
    pub_left_ = this->create_publisher<example_interfaces::msg::Float64>("/input/left_motor/setpoint_vel", 10);
    pub_right_ = this->create_publisher<example_interfaces::msg::Float64>("/input/right_motor/setpoint_vel", 10);

    // Subscriber to detected object position
    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/pixel_coordinates", 10,
        std::bind(&SequenceController::position_callback, this, std::placeholders::_1));

    // Timer for manual mode random trajectory generation
    timer_ = this->create_wall_timer(std::chrono::seconds(3),
        std::bind(&SequenceController::run_sequence, this));

    // Declare ROS2 parameter for mode switching (default: true)
    this->declare_parameter("tracking_mode", true);

    RCLCPP_INFO(this->get_logger(), "Sequence Controller Initialized in Tracking Mode!");
  }

private:
  bool tracking_mode_;
  std::mt19937 generator_;
  std::uniform_real_distribution<double> distribution_;

  void position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    this->get_parameter("tracking_mode", tracking_mode_);

    if (!tracking_mode_) return;  

    double image_center_x = 320.0;  
    double object_x = msg->x;

    auto left_motor = example_interfaces::msg::Float64();
    auto right_motor = example_interfaces::msg::Float64();

    if (object_x < image_center_x - 50) {  
      left_motor.data = 0.2;
      right_motor.data = 0.5;
    } else if (object_x > image_center_x + 50) {  
      left_motor.data = 0.5;
      right_motor.data = 0.2;
    } else {  
      left_motor.data = 0.5;
      right_motor.data = 0.5;
    }

    pub_left_->publish(left_motor);
    pub_right_->publish(right_motor);
  }

  void run_sequence() {
    this->get_parameter("tracking_mode", tracking_mode_);

    if (tracking_mode_) return;  

    // Generate random values for left and right motor speeds
    auto left_motor = example_interfaces::msg::Float64();
    auto right_motor = example_interfaces::msg::Float64();

    left_motor.data = distribution_(generator_);
    right_motor.data = distribution_(generator_);

    pub_left_->publish(left_motor);
    pub_right_->publish(right_motor);

    RCLCPP_INFO(this->get_logger(), "Manual Mode: Random Trajectory - Left: %f, Right: %f", left_motor.data, right_motor.data);
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
