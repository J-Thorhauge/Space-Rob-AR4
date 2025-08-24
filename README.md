# Gorm Arm ROS Driver

ROS 2 driver of the manipulator arm for GORM.
Runs in ROS 2 Humble on Ubuntu 22.04.
Tested on Raspberry Pi 5, running Docker image on ubuntu 24.04

## Overview

#### Launch files in *gorm_arm* :
- **driver.launch.py**
  - ROS interfaces for the arm, built on the ros2_control framework.
  - Manages joint offsets, limits and conversion between joint and actuator messages.
  - Handles communication with the microcontroller.

## Docker installation
Install the repository on the device that is connected to the Teensy 4.1 and Arduino Nano Every. Most likely a Raspberry Pi.
- Clone this repository:
  ```bash
  git clone -b Docker https://github.com/J-Thorhauge/Space-Rob-AR4.git
  ```
- Build the image:
  ```bash
  cd Space-Rob-AR4/
  docker build --no-cache -t gorm_arm .
  ```
- Run the container:
  ```bash
  docker run -it --user ros --network=host --ipc=host -v /dev:/dev --privileged gorm_arm:latest
  ```

<!-- ## Manual Installation (Don't do this one, I haven't checked if it works in ages)

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) for Ubuntu 22.04
- Clone this repository:
  ```bash
  git clone https://github.com/J-Thorhauge/Space-Rob-AR4.git
  ```
- Install workspace dependencies:
  ```bash
  rosdep install --from-paths . --ignore-src -r -y
  ```
- Build the workspace:
  ```bash
  colcon build
  ```
- Source the workspace:
  ```bash
  source install/setup.bash
  ```
- Enable serial port access if you haven't already done so:
  ```bash
  sudo addgroup $USER dialout
  ```
  You will need to log out and back in for changes to take effect.

### Firmware Flashing

The Teensy sketch provided in [gorm_arm_firmware](./gorm_arm_firmware/)
should be installed on the Teensy 4.1 on the manipulator.  -->

<!-- ### [Optional] Running in Docker Container

A docker container and run script has been provided that can be used to run the
robot and any GUI programs. It requires [rocker](https://github.com/osrf/rocker) to be installed. Then you can start the docker container with:

```bash
docker build -t ar4_ros_driver .

# Adjust the volume mounting and devices based on your project and hardware
rocker --ssh --x11 \
  --devices /dev/ttyUSB0 /ttyACM0 \
  --volume $(pwd):/ar4_ws/src/ar4_ros_driver -- \
  ar4_ros_driver bash
``` -->

## Usage

   - Run the docker container using:
```bash
docker run -it --network=host --ipc=host -v /dev:/dev --privileged --device=/dev/video0 gorm_arm:latest
```
   - Launch the drivers with:
```bash
ros2 launch gorm_arm driver.launch.py
```
Several arguments are available to modify the lanuch, based on what you have connected:
   - `run_gripper:=True/False` (True by default)
   - `gripper:=big/small/none` (none by default)
   - `run_camera:=True/False` (True by default)
   - `run_ph:=True/False` (False by default)

The hardware driver must always be run, in order to control the arm.

The camera feed can be viewed with:

```bash
ros2 run rqt_image_view rqt_image_view
```

Should you want to run only the camera, this can be done with:
```bash
ros2 run gorm_arm camera_interface_node
```

## Important notes

### General checklist
   - Make sure the manipulator is in a safe position before starting the homing sequence.
   - Make sure the sample gripper is connected before turning on the power in order to tare the load cell properly
   - Make sure belts are tightened.


### Usefull debugging
#### Setting the camera exposure manually
The camera may be overexposed in some lighting scenarios, being unable to auto-adjust.
In this case run the following command to turn off auto exposure:
```bash
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
```
And follow it with this command to manually set an exposure, from 1 to 5000, with 157 being default.
```bash
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=5
```
#### Ssh connection

```bash
ssh manipulator@192.168.10.219
```

#### Check device connections
Usb ports are set up for my own computer, so you may have to fiddle with the correct usb connections for a while. The Nano should be on `/dev/ttyACM0`, and the Teensy should be on `/dev/ttyACM1`. Start with connecting the nano, then the Teensy, as this "should" give them the right adresses. The camera can be connected at any time.
```bash
ls /dev/ttyACM*
```
We want to see `/dev/ttyACM0` and `/dev/ttyACM1`. 
```bash
ls /dev/video*
```
We want to see `/dev/video0`.

### Frequently used commands
#### Sampling task
```bash
ros2 launch gorm_arm driver.launch.py gripper:=big
```
#### Maintenance task
```bash
ros2 launch gorm_arm driver.launch.py gripper:=small
```
#### Science task
```bash
ros2 launch gorm_arm driver.launch.py run_gripper:=False run_ph:=True
```
#### Amputated limb
```bash
ros2 launch gorm_arm driver.launch.py run_camera:=False run_gripper:=False
```
<!-- ---

### TLDR;
Run the following in three terminals
```bash
cd ~/ar_ros_driver
source install/setup.bash
ros2 launch gorm_arm driver.launch.py
```
Wait for the robot to complete the homing procedure...
```bash
cd ~/ar_ros_driver
source install/setup.bash
ros2 launch gorm_arm serial.launch.py
```
The joystick control is now enabled. (Alternatively, replace the last command with `moveit.launch.py` for moveit drag-and-drop control.)
```bash
cd ~/ar_ros_driver
source install/setup.bash
ros2 run gorm_arm gripper_interface_node
```
The gripper controller is now enabled. -->

---
### Disclaimer
When running servo control, the motors of the manipulator become very hot over the course of a few minutes. For this reason, try to keep sessions below two or three minutes, before shutting down the servo node, to let the motors cool.

Please direct any criticisms to Carsten at [chpn21@student.aau.dk]()

<!-- # AR4 ROS Driver

ROS 2 driver of the AR4 robot arm from [Annin Robotics](https://www.anninrobotics.com).
Tested with ROS 2 Jazzy on Ubuntu 24.04. Also has branch for Humble
[here](https://github.com/ycheng517/ar4_ros_driver/tree/humble).

**Supports:**

- AR4 MK1 (Original version), MK2, MK3
- AR4 servo gripper

**Features:**

- MoveIt control
- Gazebo simulation

## Video Demo

<div align="center">

|                                        Moveit Motion Planning                                         |                                   Startup, Calibration, and Gripper Control                                   |
| :---------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------: |
| [![AR4 ROS 2 Driver Demo](http://img.youtube.com/vi/XJCrfrW7jXE/0.jpg)](https://youtu.be/XJCrfrW7jXE) | [![Startup, Calibration, Gripper](http://img.youtube.com/vi/PQtXFzqRtHM/0.jpg)](https://youtu.be/PQtXFzqRtHM) |

</div>

## Add-on Features and Capabilities

The following projects showcases additional features and capabilities built on top of this driver:

- [Hand-Eye calibration](https://github.com/ycheng517/ar4_hand_eye_calibration)
- [Teleoperation using Xbox controller](https://github.com/ycheng517/ar4_ros_driver_examples)
- [Multi-arm control](https://github.com/ycheng517/ar4_ros_driver_examples)
- [Voice controlled pick and place](https://github.com/ycheng517/tabletop-handybot)

## Overview

- **annin_ar4_description**
  - Hardware description of arm & servo gripper urdf.
- **annin_ar4_driver**
  - ROS interfaces for the arm and servo gripper drivers, built on the ros2_control framework.
  - Manages joint offsets, limits and conversion between joint and actuator messages.
  - Handles communication with the microcontrollers.
- **annin_ar4_firmware**
  - Firmware for the Teensy and Arduino Nano microcontrollers.
- **annin_ar4_moveit_config**
  - MoveIt module for motion planning.
  - Controlling the arm and servo gripper through Rviz.
- **annin_ar4_gazebo**
  - Simulation on Gazebo.

## Installation

- Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) for Ubuntu 24.04
- Clone this repository:
  ```bash
  git clone https://github.com/ycheng517/ar4_ros_driver
  ```
- Install workspace dependencies:
  ```bash
  rosdep install --from-paths . --ignore-src -r -y
  ```
- Build the workspace:
  ```bash
  colcon build
  ```
- Source the workspace:
  ```bash
  source install/setup.bash
  ```
- Enable serial port access if you haven't already done so:
  ```bash
  sudo addgroup $USER dialout
  ```
  You will need to log out and back in for changes to take effect.

### Firmware Flashing

The Teensy and Arduino Nano sketches provided in [annin_ar4_firmware](./annin_ar4_firmware/)
are compatible with the default hardware. To flash it, follow the same
procedure as specified in [AR4 Robot Setup](https://www.youtube.com/watch?v=OL6lXu8VU4s).
An extra step required is to install [Bounce2](https://github.com/thomasfredericks/Bounce2)
from the Library Manager in Arduino.

### [Optional] Running in Docker Container

A docker container and run script has been provided that can be used to run the
robot and any GUI programs. It requires [rocker](https://github.com/osrf/rocker) to be installed. Then you can start the docker container with:

```bash
docker build -t ar4_ros_driver .

# Adjust the volume mounting and devices based on your project and hardware
rocker --ssh --x11 \
  --devices /dev/ttyUSB0 /ttyACM0 \
  --volume $(pwd):/ar4_ws/src/ar4_ros_driver -- \
  ar4_ros_driver bash
```

## Usage

There are two modules that you will always need to run:

1. **Arm module** - this can be for either a real-world or simulated arm

   - For controlling the real-world arm, you will need to run the `annin_ar4_driver` module
   - For the simulated arm, you will need to run the `annin_ar4_gazebo` module
   - Either of the modules will load the necessary hardware descriptions for MoveIt

2. **MoveIt module** - the `annin_ar4_moveit_config` module provides the MoveIt interface and RViz GUI.

The various use cases of the modules and instructions to run them are described below:

---

### MoveIt Demo in RViz

If you are unfamiliar with MoveIt, it is recommended to start with this to explore planning with MoveIt in RViz. This contains neither a real-world nor a simulated arm but just a model loaded within RViz for visualisation.

The robot description, moveit interface and RViz will all be loaded in the single demo launch file

```bash
ros2 launch annin_ar4_moveit_config demo.launch.py
```

---

### Control real-world arm with MoveIt in RViz

Start the `annin_ar4_driver` module, which will load configs and the robot description:

```bash
ros2 launch annin_ar4_driver driver.launch.py calibrate:=True
```

Available Launch Arguments:

- `ar_model`: The model of the AR4. Options are `mk1`, `mk2`, or `mk3`. Defaults to `mk3`.
- `calibrate`: Whether to calibrate the robot arm (determine the absolute position
  of each joint).
- `include_gripper`: Whether to include the servo gripper. Defaults to: `include_gripper:=True`.
- `serial_port`: Serial port of the Teensy board. Defaults to: `serial_port:=/dev/ttyACM0`.
- `arduino_serial_port`: Serial port of the Arduino Nano board. Defaults to `arduino_serial_port:=/dev/ttyUSB0`.

⚠️📏 Note: Calibration is required after flashing firmware to the Teensy board, and
power cycling the robot and/or the Teensy board. It can be skipped in subsequent
runs with `calibrate:=False`.

Start MoveIt and RViz:

```bash
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

Available Launch Arguments:

- `ar_model`: The model of the AR4. Options are `mk1`, `mk2`, or `mk3`. Defaults to `mk3`.
- `include_gripper`: Whether to include the servo gripper. Defaults to:
  `include_gripper:=True`.
- `use_sim_time`: Make Moveit use simulation time. Should only be enabled when
  running with Gazebo. Defaults to: `use_sim_time:=False`.

You can now plan in RViz and control the real-world arm. Joint commands and joint states will be updated through the hardware interface.

NOTE: At any point you may interrupt the robot movement by pressing the E-Stop button
on the robot. This would abruptly stop the robot motion! To reset the E-Stop state of
the robot use the following command

```bash
ros2 run annin_ar4_driver reset_estop.sh <AR_MODEL>
```

where `<AR_MODEL>` is the model of the AR4, one of `mk1`, `mk2`, or `mk3`

---

### Control simulated arm in Gazebo with MoveIt in RViz

Start the `annin_ar4_gazebo` module, which will start the Gazebo simulator and load the robot description.

```bash
ros2 launch annin_ar4_gazebo gazebo.launch.py
```

Start Moveit and RViz:

```bash
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true include_gripper:=True
```

You can now plan in RViz and control the simulated arm.

## Tuning and Tweaks

### Tuning Joint Offsets

If for some reason your robot's joint positions appear misaligned after moving
to the home position, you can adjust the joint offsets in the
[joint_offsets/](./annin_ar4_driver/config/joint_offsets/) config folder.
Select and modify the YAML file corresponding to your AR model to fine-tune the joint positions.

### Switching to Position Control

By default this repo uses velocity-based joint trajectory control. It allows the arm to move a lot faster and the arm movement is also a lot smoother. If for any
reason you'd like to use the simpler classic position-only control mode, you can
set `velocity_control_enabled: false` in [driver.yaml](./annin_ar4_driver/config/driver.yaml). Note that you'll need to reduce velocity and acceleration scaling in order for larger motions to succeed. -->
