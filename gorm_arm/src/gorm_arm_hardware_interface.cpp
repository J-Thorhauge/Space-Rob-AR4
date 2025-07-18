#include <gorm_arm_driver/gorm_arm_hardware_interface.hpp>
#include <sstream>

namespace gorm_arm_driver
{

  std::string par_serial_port = "/dev/ttyACM0";
  bool par_calibrate = true;
  std::string par_calib = "0012345";

  hardware_interface::CallbackReturn GormArmHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_INFO(logger_, "Initializing hardware interface...");

    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    init_variables();

    // init motor driver
    std::string serial_port = par_serial_port; // info_.hardware_parameters.at("serial_port");
    // std::string ar_model = info_.hardware_parameters.at("ar_model");
    std::string velocity_control_p = "True";
    // info_.hardware_parameters.at("velocity_control_enabled");
    bool velocity_control_enabled =
        velocity_control_p == "True" || velocity_control_p == "true";
    int baud_rate = 9600;
    bool success = driver_.init(serial_port, baud_rate,
                                info_.joints.size(), velocity_control_enabled);
    if (!success)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // calibrate joints if needed
    bool calibrate = par_calibrate; // info_.hardware_parameters.at("calibrate") == "True";
    if (calibrate)
    {
      // run calibration
      RCLCPP_INFO(logger_, "Running joint calibration...");
      std::string calib_sequence = par_calib; // info_.hardware_parameters.at("calib_sequence");
      if (calib_sequence.length() != 7)
      {
        RCLCPP_ERROR(logger_, "Invalid calib_sequence length: %zu. Expected: 7",
                     calib_sequence.length());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!driver_.calibrateJoints(calib_sequence))
      {
        RCLCPP_INFO(logger_, "calibration failed.");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(logger_, "calibration succeeded.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void GormArmHardwareInterface::init_variables()
  {
    // resize vectors
    int num_joints = info_.joints.size();
    actuator_pos_commands_.resize(num_joints);
    actuator_vel_commands_.resize(num_joints);
    actuator_positions_.resize(num_joints);
    actuator_velocities_.resize(num_joints);
    joint_positions_.resize(num_joints);
    joint_velocities_.resize(num_joints);
    joint_efforts_.resize(num_joints);
    joint_position_commands_.resize(num_joints);
    joint_velocity_commands_.resize(num_joints);
    joint_effort_commands_.resize(num_joints);
    joint_offsets_.resize(num_joints);
    for (int i = 0; i < num_joints; ++i)
    {
      joint_offsets_[i] =
          std::stod(info_.joints[i].parameters["position_offset"]);
    }
  }

  hardware_interface::CallbackReturn GormArmHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Activating hardware interface...");

    // // Reset Estop (if any)
    // bool success = driver_.resetEStop();
    // if (!success)
    // {
    //   RCLCPP_ERROR(logger_,
    //                "Cannot activate. Hardware E-stop state cannot be reset.");
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn GormArmHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  GormArmHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(info_.joints[i].name, "position",
                                    &joint_positions_[i]);
      state_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                    &joint_velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  GormArmHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(info_.joints[i].name, "position",
                                      &joint_position_commands_[i]);
      command_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                      &joint_velocity_commands_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::return_type GormArmHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    driver_.getJointPositions(actuator_positions_);
    driver_.getJointVelocities(actuator_velocities_);
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      // apply offsets, convert from deg to rad for moveit
      joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
      joint_velocities_[i] = degToRad(actuator_velocities_[i]);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type GormArmHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      // convert from rad to deg, apply offsets
      actuator_pos_commands_[i] =
          radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
      actuator_vel_commands_[i] = radToDeg(joint_velocity_commands_[i]);
    }
    driver_.update(actuator_pos_commands_, actuator_vel_commands_,
                   actuator_positions_, actuator_velocities_);
    // if (driver_.isEStopped())
    // {
    //   std::string logWarn =
    //       "Hardware in EStop state. To reset the EStop "
    //       "reactivate the hardware component using 'ros2 "
    //       "run annin_ar4_driver reset_estop.sh <ar_model>'.";
    //   RCLCPP_WARN(logger_, logWarn.c_str());

    //   return hardware_interface::return_type::ERROR;
    // }
    return hardware_interface::return_type::OK;
  }

} // namespace gorm_arm_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gorm_arm_driver::GormArmHardwareInterface,
                       hardware_interface::SystemInterface)
