/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <chrono>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string GRIPPER_TOPIC = "/servo_node/gripper_pos";
const std::string EEF_FRAME_ID = "ee_link";
const std::string BASE_FRAME_ID = "base_link";

enum Frame
{
  BASE = 0,
  TOOL = 1,
  TOP = 2,
  BOT = 3
};
enum Frame frame = TOP;

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  STICK_LR = 0,
  STICK_BF = 1,
  D_PAD_X = 4,
  D_PAD_Y = 5,
  STICK_TWIST = 2,
  THROTTLE = 3 //,
  // D_PAD_X = 6,
  // D_PAD_Y = 7
};
enum Button
{
  TRIGGER = 0,
  BUT_2 = 1,
  BUT_4 = 3,
  BUT_3 = 2,
  BUT_5 = 4,
  BUT_6 = 5,
  // CHANGE_VIEW = 6, // ?
  BUT_9 = 8,
  BUT_10 = 9,
  BUT_11 = 10,
  BUT_12 = 11,
  BUT_7 = 6,
  BUT_8 = 7
};

bool gripper_change = false;
int prev_gripper_pos = 0;

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS; // = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

double deadzone(double axis_value)
{
  if (axis_value < -0.1 || axis_value > 0.1)
  {
    return axis_value;
  }
  else
  {
    return 0;
  }
}

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float> &axes, const std::vector<int> &buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped> &twist,
                     std::unique_ptr<control_msgs::msg::JointJog> &joint,
                     std::unique_ptr<std_msgs::msg::Int32> &gripper)
{
  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands

  if (frame == BASE || frame == TOOL)
  {
    if (buttons[BUT_7] || buttons[BUT_8] || buttons[BUT_9] || buttons[BUT_10] || buttons[TRIGGER] || buttons[BUT_2])
    {
      // Map the D_PAD to the proximal joints
      joint->joint_names.push_back("joint_1");
      joint->velocities.push_back(buttons[BUT_7] - buttons[BUT_8]);
      joint->joint_names.push_back("joint_2");
      joint->velocities.push_back(buttons[BUT_9] - buttons[BUT_10]);

      // // Map the diamond to the distal joints
      // joint->joint_names.push_back("panda_joint7");
      // joint->velocities.push_back(buttons[B] - buttons[X]);
      // joint->joint_names.push_back("panda_joint6");
      // joint->velocities.push_back(buttons[Y] - buttons[A]);
      return false;
    }
  }
  else if (frame == TOP)
  {
    if (buttons[BUT_7] || buttons[BUT_8] || buttons[BUT_9] || buttons[BUT_10] || buttons[BUT_11] || buttons[BUT_12])
    {
      joint->joint_names.push_back("joint_4");
      joint->velocities.push_back(buttons[BUT_7] - buttons[BUT_8]);
      joint->joint_names.push_back("joint_5");
      joint->velocities.push_back(buttons[BUT_9] - buttons[BUT_10]);
      joint->joint_names.push_back("joint_6");
      joint->velocities.push_back(buttons[BUT_11] - buttons[BUT_12]);

      double gripper_angle = (axes[THROTTLE] * 30) + 30;
      gripper->data = static_cast<int>(gripper_angle);
      return false;
    }
  }
  else if (frame == BOT)
  {
    if (buttons[BUT_7] || buttons[BUT_8] || buttons[BUT_9] || buttons[BUT_10] || buttons[BUT_11] || buttons[BUT_12])
    {
      joint->joint_names.push_back("joint_1");
      joint->velocities.push_back(buttons[BUT_7] - buttons[BUT_8]);
      joint->joint_names.push_back("joint_2");
      joint->velocities.push_back(buttons[BUT_9] - buttons[BUT_10]);
      joint->joint_names.push_back("joint_3");
      joint->velocities.push_back(buttons[BUT_11] - buttons[BUT_12]);

      double gripper_angle = (axes[THROTTLE] * 30) + 30;
      gripper->data = static_cast<int>(gripper_angle);
      return false;
    }
  }

  if (frame == BASE)
  {
    twist->twist.linear.x = deadzone(axes[STICK_LR]);

    twist->twist.linear.y = -1 * (deadzone(axes[STICK_BF]));

    double z_positive = buttons[BUT_3];
    double z_negative = -1 * (buttons[BUT_4]);
    twist->twist.linear.z = z_positive + z_negative;

    twist->twist.angular.x = deadzone(axes[D_PAD_Y]);
    twist->twist.angular.y = deadzone(axes[D_PAD_X]);
    twist->twist.angular.z = deadzone(axes[STICK_TWIST]);

    double gripper_angle = (axes[THROTTLE] * 30) + 30;
    gripper->data = static_cast<int>(gripper_angle);
  }
  else if (frame == TOOL)
  {
    twist->twist.linear.x = -1 * (deadzone(axes[STICK_LR]));

    double y_positive = buttons[BUT_4];
    double y_negative = -1 * (buttons[BUT_3]);
    twist->twist.linear.y = y_positive + y_negative;

    twist->twist.linear.z = deadzone(axes[STICK_BF]);

    twist->twist.angular.x = -1 * (deadzone(axes[D_PAD_Y]));
    twist->twist.angular.y = -1 * (deadzone(axes[STICK_TWIST]));
    twist->twist.angular.z = -1 * (deadzone(axes[D_PAD_X]));

    double gripper_angle = (axes[THROTTLE] * 30) + 30;
    gripper->data = static_cast<int>(gripper_angle);
  }
  // else if (frame == TOP)
  // {

  // }
  // else if (frame == TOP)
  // {

  // }

  return true;
}

// void get_gripper_cmd(std::unique_ptr<std_msgs::msg::Int32> gripper){
//   float gripper_angle = (axes[THROTTLE]*30)+30;
//   gripper->int32.data = static_cast<int>(gripper_angle);
// }

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string &frame_name, const std::vector<int> &buttons)
{
  if (buttons[BUT_5] && frame_name == EEF_FRAME_ID)
  {
    frame_name = BASE_FRAME_ID;
    frame = BASE;
  }
  else if (buttons[BUT_6] && frame_name == BASE_FRAME_ID)
  {
    frame_name = EEF_FRAME_ID;
    frame = TOOL;
  }
  else if (buttons[BUT_11])
  {
    frame_name = BASE_FRAME_ID;
    frame = BOT;
  }
  else if (buttons[BUT_12])
  {
    frame_name = BASE_FRAME_ID;
    frame = TOP;
  }
  // int this_does = 0; // This does nothing
}

namespace moveit_servo
{
  class JoyToServoPub : public rclcpp::Node
  {
  public:
    JoyToServoPub(const rclcpp::NodeOptions &options)
        : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
    {
      // Setup pub/sub
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
          JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
          { return joyCB(msg); });

      twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
      joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
      collision_pub_ =
          this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());
      gripper_pub_ = this->create_publisher<std_msgs::msg::Int32>(GRIPPER_TOPIC, rclcpp::SystemDefaultsQoS());

      // Create a service client to start the ServoNode
      servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
      servo_start_client_->wait_for_service(std::chrono::seconds(1));
      servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

      // // Load the collision scene asynchronously
      // collision_pub_thread_ = std::thread([this]() {
      //   rclcpp::sleep_for(std::chrono::seconds(3));
      //   // Create collision object, in the way of servoing
      //   moveit_msgs::msg::CollisionObject collision_object;
      //   collision_object.header.frame_id = "panda_link0";
      //   collision_object.id = "box";

      //   shape_msgs::msg::SolidPrimitive table_1;
      //   table_1.type = table_1.BOX;
      //   table_1.dimensions = { 0.4, 0.6, 0.03 };

      //   geometry_msgs::msg::Pose table_1_pose;
      //   table_1_pose.position.x = 0.6;
      //   table_1_pose.position.y = 0.0;
      //   table_1_pose.position.z = 0.4;

      //   shape_msgs::msg::SolidPrimitive table_2;
      //   table_2.type = table_2.BOX;
      //   table_2.dimensions = { 0.6, 0.4, 0.03 };

      //   geometry_msgs::msg::Pose table_2_pose;
      //   table_2_pose.position.x = 0.0;
      //   table_2_pose.position.y = 0.5;
      //   table_2_pose.position.z = 0.25;

      //   collision_object.primitives.push_back(table_1);
      //   collision_object.primitive_poses.push_back(table_1_pose);
      //   collision_object.primitives.push_back(table_2);
      //   collision_object.primitive_poses.push_back(table_2_pose);
      //   collision_object.operation = collision_object.ADD;

      //   moveit_msgs::msg::PlanningSceneWorld psw;
      //   psw.collision_objects.push_back(collision_object);

      //   auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
      //   ps->world = psw;
      //   ps->is_diff = true;
      //   collision_pub_->publish(std::move(ps));
      // });
    }

    ~JoyToServoPub() override
    {
      if (collision_pub_thread_.joinable())
        collision_pub_thread_.join();
    }

    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
    {
      // Create the messages we might publish
      auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
      auto gripper_msg = std::make_unique<std_msgs::msg::Int32>();

      // This call updates the frame for twist commands
      updateCmdFrame(frame_to_publish_, msg->buttons);

      // Convert the joystick message to Twist or JointJog and publish
      if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg, gripper_msg))
      {
        // publish the TwistStamped
        twist_msg->header.frame_id = frame_to_publish_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
      }
      else
      {
        // publish the JointJog
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = "link_3";
        joint_pub_->publish(std::move(joint_msg));
      }
      // double gripper_angle = (axes[THROTTLE] * 30) + 30;
      // gripper_msg = static_cast<int>(gripper_angle);
      gripper_pub_->publish(std::move(gripper_msg));
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    std::string frame_to_publish_;

    std::thread collision_pub_thread_;
  }; // class JoyToServoPub

} // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)