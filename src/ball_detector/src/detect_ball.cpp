#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
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
    cv::inRange(hsv_image, cv::Scalar(35, 100, 100), cv::Scalar(85,255,255), mask);

    cv::Moments moments = cv::moments(mask, true);
    if(moments.m00 > 0){
      double cX = moments.m10 / moments.m00;
      double cY = moments.m01 / moments.m00;

      RCLCPP_INFO(this->get_logger(), "'Center of Gravity' located at: (%f, %f)", cX, cY);

      auto point_msg = geometry_msgs::msg::Point();
      point_msg.x = cX;
      point_msg.y = cY;
      point_msg.z = 0.0;
      publisher_->publish(point_msg);
    }
    else{
      RCLCPP_WARN(this->get_logger(), "No object of the specified colour was detected");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
