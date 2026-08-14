# Autonomous Mobile Robot Navigation and Behavior System

A ROS 2-based autonomous mobile robot stack for waypoint navigation, patrol behavior, and obstacle-aware motion control. The project combines a low-level pose controller, action-oriented navigation servers, and custom ROS interfaces to control a TurtleBot3-like robot in simulation and real deployment scenarios.

```text
                    Mission
                       │
                       ▼
              ┌─────────────────┐
              │ Behavior Manager│
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │   Navigation    │
              └────────┬────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
     Localization   Planning    Obstacle
          │            │          Handling
          └────────────┼────────────┘
                       ▼
                Motion Controller
                       │
                       ▼
                  /navigation_cmd
                       │
                       ▼
                  TurtleBot3
```

## Overview

This repository is organized as a ROS 2 workspace with several packages that work together to provide:

- waypoint-based navigation using action servers
- patrol missions over repeated waypoint loops
- low-level motion control based on pose error regulation
- obstacle monitoring and avoidance support
- reusable custom interfaces for robot behavior

The current launch flow starts the TurtleBot3 Gazebo environment, the controller node, and the patrol/waypoint servers together.

## Package Architecture

### amr_controller
Responsible for converting a target pose into velocity commands for the robot.

- subscribes to robot odometry from `/odom`
- receives goal poses via `/amr_controller/goal`
- publishes velocity commands to `/navigation_cmd`
- publishes goal status to `/amr_controller/goal_reached`
- uses a polar-control strategy based on position and heading errors

Files:
- `amr_controller/controller_node.py`
- `launch/controller.launch.py`
- `config/controller.yaml`

### amr_navigation
Provides autonomous behavior services for navigation and repeated patrol motion.

- `waypoint_server.py`: action server for waypoint sequence execution
- `patrol_server.py`: wraps waypoint navigation into repeated patrol cycles
- `obstacle_avoidance.py`: obstacle handling logic for motion safety
- `launch/amr.launch.py`: full robot stack launcher
- `launch/patrol.launch.py`: navigation and patrol servers launcher
- `config/patrol.yaml`: default patrol route configuration

### amr_perception
Contains perception utilities for obstacle detection.

- `obstacle_detector.py`: publishes obstacle state information

### amr_interfaces
Defines custom ROS interfaces used across the system.

- `action/NavigateWaypoints.action`
- `action/Patrol.action`
- `msg/ObstacleStatus.msg`

## Key Functionality

### Waypoint Navigation
The `navigate_waypoints` action accepts a list of waypoints and executes them sequentially. Each waypoint is published to the low-level controller until the robot reports that the target is reached.

### Patrol Missions
The `patrol` action repeats a configured waypoint route for a specified number of cycles. This is useful for applications such as service patrol loops or automated inspection.

### Pose Control
The controller computes the polar error terms:

- $\rho$: distance to target
- $\alpha$: heading error relative to the target direction
- $\beta$: orientation difference between current heading and goal orientation

It then generates linear and angular velocity commands while respecting hardware limits from the configuration file.

## Repository Structure

```text
autonomous-mobile-robot/
├── README.md
├── amr_controller/
│   ├── amr_controller/
│   │   └── controller_node.py
│   ├── config/
│   │   └── controller.yaml
│   ├── launch/
│   │   └── controller.launch.py
│   ├── package.xml
│   ├── setup.py
│   └── test/
├── amr_navigation/
│   ├── amr_navigation/
│   │   ├── obstacle_avoidance.py
│   │   ├── patrol_server.py
│   │   └── waypoint_server.py
│   ├── config/
│   │   └── patrol.yaml
│   ├── launch/
│   │   ├── amr.launch.py
│   │   └── patrol.launch.py
│   ├── package.xml
│   ├── setup.py
│   └── test/
├── amr_perception/
│   ├── amr_perception/
│   │   └── obstacle_detector.py
│   ├── package.xml
│   ├── setup.py
│   └── test/
├── amr_interfaces/
│   ├── action/
│   │   ├── NavigateWaypoints.action
│   │   └── Patrol.action
│   ├── msg/
│   │   └── ObstacleStatus.msg
│   ├── CMakeLists.txt
│   └── package.xml
└── LICENSE
```

## Prerequisites

This project is designed for a ROS 2 environment and expects the following:

- ROS 2 (Humble recommended)
- Python 3
- `colcon` build tools
- `rosdep`
- TurtleBot3 simulation packages, such as `turtlebot3_gazebo`

Install dependencies as needed:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
rosdep update
```

Then install workspace dependencies from the ROS source tree:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build and Run

From the workspace root:

```bash
cd ~/amr_ws
colcon build --symlink-install
source install/setup.bash
```

Launch the full autonomous robot stack:

```bash
ros2 launch amr_navigation amr.launch.py
```

This launches the TurtleBot3 simulator, the controller, and the waypoint/patrol servers.

To run only the controller:

```bash
ros2 launch amr_controller controller.launch.py
```

To run only the navigation services:

```bash
ros2 launch amr_navigation patrol.launch.py
```

## Usage Examples

### 1. Send a single goal directly to the controller

```bash
ros2 topic pub --once /amr_controller/goal geometry_msgs/msg/Pose2D "{x: 1.0, y: 1.0, theta: 0.0}"
```

This message updates the controller target pose and begins motion toward the goal.

### 2. Send a waypoint navigation mission

```bash
ros2 action send_goal /navigate_waypoints amr_interfaces/action/NavigateWaypoints "{waypoints: [{x: 1.0, y: 0.0, theta: 0.0}, {x: 1.5, y: 1.5, theta: 1.57}, {x: 0.0, y: 1.5, theta: 3.14}] }"
```

### 3. Run a patrol mission

```bash
ros2 action send_goal /patrol amr_interfaces/action/Patrol "{waypoints: [{x: 1.5, y: 0.0, theta: 0.0}, {x: 1.5, y: 1.5, theta: 1.57}, {x: 0.0, y: 1.5, theta: 3.14}, {x: 0.0, y: 0.0, theta: 0.0}], patrol_cycles: 3}"
```

### 4. Monitor navigation commands

```bash
ros2 topic echo /navigation_cmd
```

### 5. Check status of goal completion

```bash
ros2 topic echo /amr_controller/goal_reached
```

## Configuration

The controller gains and tolerances are configured in:

- `amr_controller/config/controller.yaml`

The default patrol route is configured in:

- `amr_navigation/config/patrol.yaml`

These files allow tuning of:

- target position tolerance
- orientation tolerance
- proportional gains
- max linear and angular velocity
- patrol route geometry and loop count

## Notes

- The current implementation is centered around a pose-based controller and action-driven navigation, which is well suited for indoor mobile robot tasks.
- The obstacle detection and avoidance modules are present in the workspace and are intended to support safer navigation behavior when integrated into the runtime stack.
- For multi-node operation, the main launch file starts the Gazebo environment and the AMR behavior stack together.

## License

This project is licensed under the terms of the MIT license. See the repository's `LICENSE` file for details.

## Contributing

Contributions, bug reports, and feature requests are welcome. When contributing, please keep changes focused, document any new ROS interfaces, and validate behavior through launch and topic/action testing.

