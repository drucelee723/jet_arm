# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

JetArm is a ROS 2 (Humble) robotic arm project targeting Ubuntu/Jetson devices. The system integrates a 5-DOF robotic arm with computer vision, AI/LLM capabilities, voice control, and various end-effectors (gripper, stepper motor sorting, chassis mobility).

## Build and Run Commands

### Building
```bash
# From workspace root (typically /home/ubuntu/ros2_ws)
colcon build --packages-select <package_name>

# Build all packages
colcon build

# Source after building
source install/setup.bash
```

### Testing
```bash
# Run Python tests for a package
colcon test --packages-select <package_name>
colcon test-result --all

# Run specific tests manually
pytest app/test/
```

### Launching the System
```bash
# Main bringup launch (starts all core nodes)
ros2 launch bringup bringup.launch.py

# Launch individual app nodes
ros2 run app calibration
ros2 run app object_tracking
ros2 run app color_tracker
ros2 run app object_sorting
ros2 run app tag_stackup
ros2 run app shape_recognition
ros2 run app waste_classification
ros2 run app finger_trace
ros2 run app lab_manager

# Launch driver nodes
ros2 run ros_robot_controller ros_robot_controller_node
ros2 run servo_controller controller_manager
ros2 run kinematics kinematics_control_node

# Launch with specific chassis type
export CHASSIS_TYPE="Slide_Rails"  # or default
ros2 launch bringup bringup.launch.py
```

## Architecture

### Package Structure

**Core Drivers** (`driver/`):
- `sdk/` - Hardware abstraction layer (Board API, buzzer, LED, servo communication)
- `ros_robot_controller/` - Main hardware controller node, interfaces with control board via USB serial
- `servo_controller/` - Servo motor control, joint trajectory controllers, action server
- `kinematics/` - Forward/inverse kinematics using modified DH parameters for 5-DOF arm
- `kinematics_msgs/` - Custom services for kinematics (SetRobotPose, SetJointValue, etc.)
- `servo_controller_msgs/` - Custom messages for servo control

**Application Layer** (`app/`):
- `lab_manager.py` - Main configuration manager with color threshold adjustment services
- `calibration.py` - Camera-to-robot calibration using AprilTag
- `object_tracking.py` - Color-based object tracking
- `object_sorting.py` - Pick and place using color detection
- `tag_stackup.py` - AprilTag-based stacking
- `shape_recognition.py` - HED (Holistically-Nested Edge Detection) based shape recognition
- `waste_classification.py` - Waste sorting with AI/LLM integration
- `finger_trace.py` - MediaPipe-based finger trajectory tracking
- `face_tracker.py` - Face tracking using MediaPipe

**Peripherals** (`peripherals/`, `chassis/`, `motor/`, `stepper/`):
- `peripherals/` - Depth camera (Orbbec), joystick control, web video server
- `chassis/` - Mobile chassis control (mecanum wheels, line following)
- `motor/` - DC motor control for conveyor belts
- `stepper/` - Stepper motor control for sorting mechanisms

**AI/LLM Integration** (`large_models/`):
- `agent_process.py` - LLM agent for reasoning and task planning
- `vocal_detect.py` - Voice command processing (Xunfei ASR)
- `tts_node.py` - Text-to-speech output

**Interfaces** (`interfaces/`, `*_msgs/`):
- Custom message/service definitions for inter-package communication
- Key services: `GetRange`, `ChangeRange`, `StashRange` for color threshold management

### Kinematics Model

The robot uses modified DH parameters for 5-DOF arm (defined in `kinematics/kinematics/transform.py`):
- Joint limits: J1[-120°,120°], J2[-180°,0°], J3[-120°,120°], J4[-200°,20°], J5[-120°,120°]
- Servo pulse range: 0-1000 (500 = center)
- Inverse kinematics accepts target position [x,y,z] in meters and pitch angle

### Config System

All calibration and configuration data is stored in `app/config/`:
- `lab_config.yaml` - Color thresholds (LAB color space), camera calibration, ROI settings
- `config.yaml` - Default color ranges for detection
- `positions.yaml` - Stored positions for pick-and-place operations
- `transform.yaml` - Coordinate transformation matrices

Color thresholds use LAB color space with min/max ranges per color (black, blue, green, red, white, yellow, tennis).

### Communication Pattern

- **Services** for one-off operations: calibration, entering/exiting apps, saving thresholds
- **Topics** for streaming: camera images (`/depth_cam/rgb/image_raw`), joint states, servo positions
- **Heartbeat mechanism** (`app/common.py::Heart`) - Services must send heartbeat every 5s or app auto-exits
- **Action servers** for trajectories: `arm_controller` follows joint trajectories with timeout

### Coordinate Systems

- Camera coordinates: RGB-D camera provides depth at 640x480 resolution
- World coordinates: Defined via calibration using AprilTag (tag ID 100 for calibration)
- Transform matrices stored in `lab_config.yaml` for camera-to-world conversion

### Key Patterns

1. **App lifecycle**: Each app node implements `enter`/`exit` services. `enter` subscribes to camera, starts processing. `exit` unsubscribes, stops heartbeat.
2. **Color detection**: Convert RGB to LAB color space, use `cv2.inRange()` with min/max thresholds, apply morphological operations (erode/dilate)
3. **Grasp planning**: `calculate_grasp_yaw.py` and `calculate_grasp_yaw_by_depth.py` compute gripper orientation based on depth data
4. **Servo control**: Use pulse units (0-1000), convert via `angle2pulse()` / `pulse2angle()` functions

## Environment Variables

- `CHASSIS_TYPE` - "Slide_Rails" or default; affects base_link length in kinematics
- `need_compile` - "True"/"False"; determines whether to use installed package paths or source paths
