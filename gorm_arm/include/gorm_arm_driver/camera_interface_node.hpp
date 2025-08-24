#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>

class GripperCamNode : public rclcpp::Node //, public std::enable_shared_from_this<GripperCamNode>

{
public:
  GripperCamNode();

  ~GripperCamNode()
  {
    if (cap_.isOpened())
    {
      cap_.release();
    }
  }

  void init();

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  cv::VideoCapture cap_;
  cv_bridge::CvImage cv_image_;
};
