#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ImageBrightnessCalculator : public rclcpp::Node
{
public:
    ImageBrightnessCalculator() : Node("image_brightness_calculator")
    {
	// Declares threshold ROS parameter and initalisez with default value
	this->declare_parameter<double>("brightness_threshold", 97.0);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&ImageBrightnessCalculator::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/brightness_status", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

	// Creates an opencv object to store the grayscale image in
        cv::Mat gray_image;
	// Converts received image to grayscale
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
	// Calculates the mean brightness value of the grayscale image
        double brightness = cv::mean(gray_image)[0];
        RCLCPP_INFO(this->get_logger(), "Image brightness: %f", brightness);

	// Reads parameter value and writes to threshold variable
	double threshold = 0;
	this->get_parameter("brightness_threshold", threshold);

	// Initializes the datatype and publisher
	// Publishes if the image is bright enough or not based on threshold
        auto brightness_status = std_msgs::msg::Bool();
        brightness_status.data = brightness > threshold; // Threshold for the mean brightness value
        publisher_->publish(brightness_status);
    }


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageBrightnessCalculator>());
    rclcpp::shutdown();
    return 0;
}
