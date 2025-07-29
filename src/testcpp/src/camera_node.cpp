#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      1,
      std::bind(&CameraNode::image_callback, this, std::placeholders::_1)
    );
    
    cv::namedWindow("Camera Image", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "Camera node started, waiting for images...");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert ROS image to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
      // Display the image
      cv::imshow("Camera Image", cv_ptr->image);
      
      // Check for 'q' key press to exit
      char key = cv::waitKey(1);
      if (key == 'q' || key == 27) { // 'q' or ESC key
        RCLCPP_INFO(this->get_logger(), "Exiting...");
        cv::destroyAllWindows();
        rclcpp::shutdown();
      }
      
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}