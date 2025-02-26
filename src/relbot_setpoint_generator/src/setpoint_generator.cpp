#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <random>

class SetpointGenerator: public rclcpp::Node{
public:
  SetpointGenerator() : Node("setpoint_generator"),
  generator_(std::random_device{}()), distribution_(-2.5, 2.5)
  {
    pub1_ = this->create_publisher<std_msgs::msg::Float64>("input/right_motor/setpoint_vel", 0.2);
    pub2_ = this->create_publisher<std_msgs::msg::Float64>("input/left_motor/setpoint_vel", 0.2);
    //timer_ = this->create_wall_timer(std::chrono::seconds time = 0.5, std::bind(&SetpointGenerator::generate_setpoints, this));
  }

private:
  void generate_setpoints(){
    auto right_setpoint = std_msgs::msg::Float64();
    auto left_setpoint = std_msgs::msg::Float64();

    right_setpoint.data = distribution_(generator_);
    left_setpoint.data = distribution_(generator_);

    RCLCPP_INFO(this->get_logger(), "Publishing setpoints:/nRight%f, /nLeft%f", right_setpoint.data, left_setpoint.data);
    pub1_->publish(right_setpoint);
    pub2_->publish(left_setpoint);
  }

  

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub2_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 generator_;
  std::uniform_real_distribution<double> distribution_;
};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointGenerator>());
  rclcpp::shutdown();
  return 0;
}
