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
    SequenceController() : Node("sequence_controller"), count_(0) {
        sample_time_s_ = 0.03;

        subscription_greenobject_pos_ =
            this->create_subscription<geometry_msgs::msg::Point>(
                "pixel_coordinates", 10,
                std::bind(&SequenceController::update_greenobject_pos, this, _1));

        publisher_left_ = this->create_publisher<std_msgs::msg::Float64>(
            "left_motor_setpoint_vel", 10);

        publisher_right_ = this->create_publisher<std_msgs::msg::Float64>(
            "right_motor_setpoint_vel", 10);

            pub_xeno_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
                "Ros2Xeno", 10);

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

        double e = gain * (greenobject_pos_.x - (width / 2));

        RCLCPP_INFO(this->get_logger(), "greenobject_pos.x: %f, e: %f", greenobject_pos_.x,
                    e);

        auto vel_xeno = xrf2_msgs::msg::Ros2Xeno();

        vel_xeno.left_motor_setpoint_vel = e;
        vel_xeno.right_motor_setpoint_vel = -e;             

        pub_xeno_ -> publish(vel_xeno);
    }

    void update_greenobject_pos(const geometry_msgs::msg::Point &msg) {
        if (msg.x == -1)
            return;

        greenobject_pos_.x = msg.x;
        greenobject_pos_.y = msg.y;
    }

    size_t count_;
    double sample_time_s_;

    geometry_msgs::msg::Point greenobject_pos_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        subscription_greenobject_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        subscription_dim_;
    // rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr sub_xeno_channel_1;
    // rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr sub_xeno_channel_2;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right_;

    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr pub_xeno_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();

    return 0;
}
