#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BallDetector : public rclcpp::Node
{
public:
  BallDetector() : Node("ball_detector"){
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 10, std::bind(&BallDetector::image_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/pixel_coordinates", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv_image, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      // Find the largest contour
      auto max_it = std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                      return cv::contourArea(c1) < cv::contourArea(c2);
                    });
      
      double area = cv::contourArea(*max_it);
      cv::Moments m = cv::moments(*max_it);
      double cX = m.m10 / m.m00;
      double cY = m.m01 / m.m00;

      RCLCPP_INFO(this->get_logger(), "Object at (%f, %f), Area: %f", cX, cY, area);

      auto point_msg = geometry_msgs::msg::Point();
      point_msg.x = cX;
      point_msg.y = cY;

      // Set z = -1 to indicate "too close"
      if (area > proximity_area_threshold_) {
        RCLCPP_WARN(this->get_logger(), "Object too close! Stopping robot.");
        point_msg.z = -1.0;
      } else {
        point_msg.z = 0.0; // Normal detection
      }

      publisher_->publish(point_msg);
    }
  }

  double proximity_area_threshold_ = 15000.0; // Adjust based on camera distance

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
