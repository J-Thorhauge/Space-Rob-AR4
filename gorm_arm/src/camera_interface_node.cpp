#include "gorm_arm_driver/camera_interface_node.hpp"

GripperCamNode::GripperCamNode()
    : Node("camera_interface_node")
{
  // Initialize camera
  cap_.open(0); // Open /dev/video0
  if (!cap_.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
    rclcpp::shutdown();
    return;
  }

  // Load camera calibration
  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, "usb_cam", "package://gorm_arm/config/camera_calib.yaml");

  image_transport::ImageTransport it(this);
  image_pub_ = it.advertise("gripper/camera/image_raw", 10);
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("gripper/camera/camera_info", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // 10 Hz
      std::bind(&GripperCamNode::timer_callback, this));
}

void GripperCamNode::timer_callback()
{
  cv::Mat frame;
  cap_ >> frame;

  if (frame.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Captured empty frame.");
    return;
  }

  auto stamp = this->get_clock()->now();
  cv_image_.header.stamp = stamp;
  cv_image_.header.frame_id = "camera_frame";
  cv_image_.encoding = "bgr8";
  cv_image_.image = frame;

  auto image_msg = cv_image_.toImageMsg();
  image_pub_.publish(image_msg);

  auto cam_info = camera_info_manager_->getCameraInfo();
  cam_info.header.stamp = stamp;
  cam_info.header.frame_id = "camera_frame";
  camera_info_pub_->publish(cam_info);
}
