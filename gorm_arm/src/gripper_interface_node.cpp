#include <chrono>
#include <memory>

#include <iostream>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <serial/serial.h>

using namespace std::chrono_literals;

class GripperInterfaceNode : public rclcpp::Node
{
public:
  GripperInterfaceNode() : Node("gripper_interface_node"), pos_(0)
  {
    try
    {
      serial_port_.setPort("/dev/ttyACM0");
      serial_port_.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serial_port_.setTimeout(to);
      serial_port_.open();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
      throw;
    }

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/servo_node/gripper_pos",
        10,
        std::bind(&GripperInterfaceNode::gripper_pos_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        200ms, std::bind(&GripperInterfaceNode::timer_callback, this));
  }

  int pos_;

private:
  void gripper_pos_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    pos_ = msg->data;
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Gripper pos: '%d'", pos_);

    if (serial_port_.isOpen())
    {
      try
      {
        std::string data = std::to_string(pos_) + "\n";
        serial_port_.write(data);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Serial port is not open.");
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  serial::Serial serial_port_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperInterfaceNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

// #include <chrono>
// #include <functional>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int32.hpp"
// using std::placeholders::_1;

// using namespace std::chrono_literals;

// class GripperInterfaceNode : public rclcpp::Node
// {
// public:
//   GripperInterfaceNode()
//       : Node("gripper_interface_node"), pos_(0)
//   {
//     subscription_ = this->create_subscription<std_msgs::msg::Int32>(
//         "/servo_node/gripper_pos",
//         10,
//         std::bind(&GripperInterfaceNode::gripper_pos_callback, this, _1));
//     timer_ = this->create_wall_timer(
//         200ms, std::bind(&GripperInterfaceNode::timer_callback, this));
//   }

//   int pos_;

// private:
//   void gripper_pos_callback(const std_msgs::msg::Int32::SharedPtr msg) const
//   {
//     // RCLCPP_INFO(this->get_logger(), std::to_string(msg->data).c_str());
//     pos_ = msg->data;
//   }
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

//   void timer_callback()
//   {
//     RCLCPP_INFO(this->get_logger(), std::to_string(pos_).c_str());
//   }
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<GripperInterfaceNode>());
//   rclcpp::shutdown();
//   return 0;
// }