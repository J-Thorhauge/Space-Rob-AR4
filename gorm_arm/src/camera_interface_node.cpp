#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class GripperCamPub : public rclcpp::Node
{
public:
  GripperCamPub()
      : Node("camera_publisher")
  {
    // Initialize publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("gripper_cam/image_raw", 10);

    // Open camera
    cap_.open(0);
    if (!cap_.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
      rclcpp::shutdown();
      return;
    }

    // Create timer to publish frames
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), // ~30 FPS
        std::bind(&GripperCamPub::publish_frame, this));
  }

private:
  void publish_frame()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Captured empty frame.");
      return;
    }

    // Convert OpenCV image to ROS 2 message
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperCamPub>());
  rclcpp::shutdown();
  return 0;
}
