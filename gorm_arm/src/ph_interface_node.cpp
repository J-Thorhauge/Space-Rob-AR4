#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <libserial/SerialStream.h>
#include <libserial/SerialPort.h>

#include <chrono>

using namespace LibSerial;

class PHInterfaceNode : public rclcpp::Node
{
public:
  PHInterfaceNode() : Node("ph_interface_node")
  {
    // Initialize serial port
    try
    {
      serial_port_.Open("/dev/ttyACM1");
      serial_port_.SetBaudRate(BaudRate::BAUD_115200);
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

    ph_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ph_device", rclcpp::SystemDefaultsQoS());
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&PHInterfaceNode::timer_callback, this));
  }

  bool first_run_ = true;

  ~PHInterfaceNode()
  {
    if (serial_port_.IsOpen())
    {
      serial_port_.Close();
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ph_pub_;

  void timer_callback()
  {
    if (!serial_port_.IsOpen())
    {
      RCLCPP_WARN(this->get_logger(), "Serial port is not open.");
      return;
    }

    if (first_run_)
    {
      serial_port_.Write("ph\n");
      first_run_ = false;
    }

    std::string ph_data;
    serial_port_.ReadLine(ph_data, '\n', 180);
    // data.clear();
    // while (1)
    // {
    //   char c;
    //   serial_port_.ReadByte(c, 20); // timeout in milliseconds
    //   if (c == '\n')
    //   {
    //     break;
    //   }
    //   else
    //   {
    //     data += c;
    //   }
    // }
    // std::string
    auto ph_msg = std::make_unique<std_msgs::msg::Float32>();
    ph_msg->data = std::stof(ph_data);
    ph_pub_->publish(std::move(ph_msg));
  }
  rclcpp::TimerBase::SharedPtr timer_;
  SerialPort serial_port_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PHInterfaceNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}