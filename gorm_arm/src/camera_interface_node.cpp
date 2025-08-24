#include "gorm_arm_driver/camera_interface_node.hpp"

GripperCamNode::GripperCamNode()
    : Node("camera_interface_node")
{
  // Nothing goes here
}

void GripperCamNode::init()
{
  bool found_cam = false;

  cap_.open(0); // Open /dev/video0
  if (cap_.isOpened())
  {
    found_cam = true;
  }

  if (!found_cam)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
    rclcpp::shutdown();
    return;
  }

  // Load camera calibration
  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, "usb_cam", "package://gorm_arm/config/camera_calib.yaml");

  // Publishers
  image_transport::ImageTransport it(rclcpp::Node::shared_from_this());
  // image_pub_ = it.advertise("gripper/camera/image_raw", 10);

  // camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("gripper/camera/camera_info", 10);

  // NEW: compressed publisher
  compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "gripper/camera/compressed", 10);

  // Timer
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

  // --- RAW IMAGE ---
  cv_image_.header.stamp = stamp;
  cv_image_.header.frame_id = "camera_frame";
  cv_image_.encoding = "bgr8";
  cv_image_.image = frame;

  // auto image_msg = cv_image_.toImageMsg();
  // image_pub_.publish(image_msg);

  // --- COMPRESSED IMAGE ---  (NEW)
  sensor_msgs::msg::CompressedImage compressed_msg;
  compressed_msg.header.stamp = stamp;
  compressed_msg.header.frame_id = "camera_frame";

  std::vector<uchar> buffer;
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80}; // 0–100 (higher = better quality, bigger size)
  cv::imencode(".jpg", frame, buffer, params);

  compressed_msg.format = "jpeg";
  compressed_msg.data = buffer;

  compressed_pub_->publish(compressed_msg);

  // RCLCPP_INFO(this->get_logger(), "Captured frame: %dx%d", frame.cols, frame.rows);

  // --- CAMERA INFO ---
  // auto cam_info = camera_info_manager_->getCameraInfo();
  // cam_info.header.stamp = stamp;
  // cam_info.header.frame_id = "camera_frame";
  // camera_info_pub_->publish(cam_info);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperCamNode>();
  node->init(); // safe: shared_from_this() now works

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
