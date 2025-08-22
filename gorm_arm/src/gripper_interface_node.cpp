#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <libserial/SerialStream.h>

#include <chrono>

using namespace LibSerial;

class GripperInterfaceNode : public rclcpp::Node
{
public:
  GripperInterfaceNode() : Node("gripper_interface_node")
  {
    this->declare_parameter("gripper", "none");
    // Initialize serial port
    try
    {
      serial_port_.Open("/dev/ttyACM0");
      serial_port_.SetBaudRate(BaudRate::BAUD_9600);
      serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
      serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
      serial_port_.SetParity(Parity::PARITY_NONE);
      serial_port_.SetStopBits(StopBits::STOP_BITS_1);
    }
    catch (const OpenFailed &)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
      rclcpp::shutdown();
      return;
    }

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/servo_node/gripper_pos", 10,
        std::bind(&GripperInterfaceNode::topic_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&GripperInterfaceNode::timer_callback, this));
  }

  int pos_ = 60;
  bool first_run_ = true;

  ~GripperInterfaceNode()
  {
    if (serial_port_.IsOpen())
    {
      serial_port_.Close();
    }
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    pos_ = (msg->data + 37);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

  void timer_callback()
  {
    if (!serial_port_.IsOpen())
    {
      RCLCPP_WARN(this->get_logger(), "Serial port is not open.");
      return;
    }

    if (first_run_)
    {
      std::string gripper_param = this->get_parameter("gripper").as_string();

      RCLCPP_INFO(this->get_logger(), gripper_param.c_str());

      if (gripper_param == "none")
      {
        RCLCPP_ERROR(this->get_logger(), "No gripper chosen! (big, small).");
        rclcpp::shutdown();
        return;
      }
      serial_port_ << gripper_param;
      serial_port_.flush();
    }

    std::string message = std::to_string(pos_) + "\n";
    serial_port_ << message;
    serial_port_.flush();
    RCLCPP_INFO(this->get_logger(), "Sent to Arduino: '%s'", message.c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  SerialStream serial_port_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperInterfaceNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}