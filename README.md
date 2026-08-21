# Autonomous Mobile Robot (AMR) — ROS 2

A modular Autonomous Mobile Robot (AMR) project built with **ROS 2 Jazzy**, **TurtleBot3**, and **Gazebo**.

The project progressively develops an autonomous mobile robot from a custom feedback-based motion controller into a higher-level mission system capable of waypoint navigation, patrol missions, LiDAR-based obstacle detection, reactive obstacle avoidance, command arbitration, and safety handling.

The system is designed with a modular ROS 2 architecture so that individual components can be developed, tested, and replaced independently.

---

## Project Status

### Completed

- [x] ROS 2 Jazzy environment
- [x] TurtleBot3 simulation in Gazebo
- [x] Custom Siegwart feedback controller
- [x] Position and orientation control
- [x] Configurable controller parameters
- [x] ROS 2 NavigateWaypoints Action
- [x] Multi-waypoint navigation
- [x] Waypoint feedback and results
- [x] Goal cancellation
- [x] Patrol Action
- [x] Multi-cycle patrol missions
- [x] Patrol feedback
- [x] Patrol cancellation
- [x] YAML-based patrol configuration
- [x] Modular launch system
- [x] LiDAR-based obstacle detection
- [x] Reactive obstacle avoidance
- [x] Obstacle avoidance state handling
- [x] Navigation/avoidance command arbitration
- [x] Safety and emergency obstacle handling
- [x] Manual full-system validation


# 1. Project Overview

The goal of this project is to develop a modular autonomous mobile robot using ROS 2.

Rather than immediately relying on a complete navigation framework, the project starts by implementing the fundamental robotics components independently.

The development progression is:

```text
ROS 2 Fundamentals
       ↓
Motion Controller
       ↓
Waypoint Navigation
       ↓
Patrol Missions
       ↓
LiDAR Obstacle Detection
       ↓
Reactive Obstacle Avoidance
       ↓
Command Arbitration
       ↓
Safety Handling
       ↓
Full System Integration
```

This approach provides a clear understanding of the underlying robotics architecture before moving toward higher-level navigation frameworks.

---

# 2. System Architecture

The AMR is designed as a modular ROS 2 system in which mission execution, motion control, perception, obstacle avoidance, and command arbitration are separated into independent components.

The architecture follows a hierarchical control structure:

```text
                                      ┌──────────────────────────────┐
                                      │          AMR BRINGUP         │
                                      │       amr_bringup package    │
                                      │                              │
                                      │  ros2 launch amr_bringup     │
                                      │       bringup.launch.py      │
                                      └──────────────┬───────────────┘
                                                     │
                              ┌──────────────────────┴──────────────────────┐
                              │                                             │
                              ▼                                             ▼
                 ┌─────────────────────────┐                    ┌─────────────────────────┐
                 │    amr_navigation       │                    │   mission_control_gui   │
                 │        package          │                    │         node            │
                 │                         │                    │                         │
                 │                         │                    │ User selects mission    │
                 │                         │                    │ and monitors progress   │
                 └────────────┬────────────┘                    └───────────┬─────────────┘
                              │                                             │
                              │                                             │ /patrol
                              │                                             │ Action
                              │                                             ▼
                              │                                  ┌─────────────────────────┐
                              │                                  │     Patrol Server       │
                              │                                  │                         │
                              │                                  │  High-level mission     │
                              │                                  │      execution          │
                              │                                  └───────────┬─────────────┘
                              │                                              │
                              │                                              │
                              │                                  NavigateWaypoints Action
                              │                                              │
                              │                                              ▼
                              │                                  ┌─────────────────────────┐
                              │                                  │    Waypoint Server      │
                              │                                  │                         │
                              │                                  │  Sequential waypoint    │
                              │                                  │      execution          │
                              │                                  └───────────┬─────────────┘
                              │                                              │
                              │                                              │ Goal Pose
                              │                                              ▼
                              │                                  ┌─────────────────────────┐
                              │                                  │       Controller        │
                              │                                  │                         │
                              │                                  │  Siegwart Feedback      │
                              │                                  │       Control           │
                              │                                  └───────────┬─────────────┘
                              │                                              │
                              │                                      /navigation_cmd
                              │                                              │
                              │                                              ▼
                              │                                  ┌─────────────────────────┐
                              │                                  │   Command Arbitrator    │
                              │                                  │                         │
                              │                                  │ Selects the command     │
                              │                                  │ allowed to control      │
                              │                                  │       the robot         │
                              │                                  └───────────┬─────────────┘
                              │                                              │
                              │                                           /cmd_vel
                              │                                              │
                              │                                              ▼
                              │                                  ┌─────────────────────────┐
                              │                                  │       TurtleBot3        │
                              │                                  │         Burger          │
                              │                                  │                         │
                              │                                  │     Simulated Robot     │
                              │                                  └───────────┬─────────────┘
                              │                                              │
                              │                         ┌────────────────────┴────────────────────┐
                              │                         │                                         │
                              │                       /scan                                     /odom
                              │                         │                                         │
                              │                         ▼                                         ▼
                              │              ┌─────────────────────────┐             ┌─────────────────────────┐
                              │              │   Obstacle Detector     │             │     Pose / TF2          │
                              │              │                         │             │       Feedback          │
                              │              │ Processes LaserScan     │             │                         │
                              │              │ and calculates obstacle │             │ Current robot pose      │
                              │              │ distances/status        │             │ for controller          │
                              │              └────────────┬────────────┘             └────────────┬────────────┘
                              │                           │                                     │
                              │                    /obstacle_status                             │
                              │                           │                                     │
                              │                           ▼                                     │
                              │              ┌─────────────────────────┐                        │
                              │              │   Obstacle Avoidance    │                        │
                              │              │                         │                        │
                              │              │ Reactive obstacle       │                        │
                              │              │ avoidance + safety      │                        │
                              │              └────────────┬────────────┘                        │
                              │                           │                                     │
                              │                    /avoidance_cmd                               │
                              │                           │                                     │
                              │                           └─────────────────┐                   │
                              │                                             │                   │
                              │                                             ▼                   │
                              │                                  ┌─────────────────────────┐    │
                              │                                  │   Command Arbitrator    │◄───┘
                              │                                  │                         │
                              │                                  │ Navigation command      │
                              │                                  │ vs. avoidance command   │
                              │                                  └───────────┬─────────────┘
                              │                                              │
                              │                                           /cmd_vel
                              │                                              │
                              │                                              ▼
                              │                                         TurtleBot3
                              │                                              ▲
                              └──────────────────────────────────────────────|
```

## 2.1 Top-Level Bringup

The system provides a single top-level bringup entry point:

```bash
ros2 launch amr_bringup bringup.launch.py
```

The bringup launch file is responsible for starting the major components required for a complete AMR demonstration.

Conceptually:

```text
amr_bringup
     │
     ├── amr_navigation
     │      │
     │      ├── navigation nodes
     │      ├── obstacle detection
     │      ├── obstacle avoidance
     │      └── command arbitration
     │
     └── mission_control_gui
```

This provides a reproducible system startup procedure instead of requiring the user to manually launch individual ROS 2 nodes.

---

## 2.2 Mission Layer

The mission layer contains the high-level behaviors that define what the robot should accomplish.

The main mission interface is the `Patrol` ROS 2 Action.

```text
Mission Control GUI
        │
        │ Patrol Action
        ▼
  Patrol Server
        │
        │ NavigateWaypoints Action
        ▼
 Waypoint Server
```

The Patrol Server therefore operates at a higher abstraction level than the waypoint server.

A patrol mission can contain multiple waypoints and multiple patrol cycles.

For example:

```text
Patrol Mission
      │
      ├── Cycle 1
      │     ├── Waypoint 1
      │     ├── Waypoint 2
      │     ├── Waypoint 3
      │     └── Waypoint 4
      │
      ├── Cycle 2
      │     ├── Waypoint 1
      │     ├── Waypoint 2
      │     ├── Waypoint 3
      │     └── Waypoint 4
      │
      └── ...
```

This hierarchical structure allows higher-level missions to reuse lower-level navigation capabilities.

---

## 2.3 Navigation Layer

The navigation layer is responsible for moving the robot toward individual target poses.

The `NavigateWaypoints` Action Server receives a sequence of waypoints and executes them sequentially.

```text
NavigateWaypoints
        │
        ▼
Waypoint Server
        │
        ├── Waypoint 1
        ├── Waypoint 2
        ├── Waypoint 3
        └── ...
                │
                ▼
          Motion Controller
```

The waypoint server does not directly publish the final `/cmd_vel` command.

Instead, it provides target poses to the low-level controller.

This keeps mission execution separate from velocity generation.

---

## 2.4 Motion Control Layer

The motion controller implements the custom Siegwart feedback control law.

The controller receives the desired robot pose and the current robot pose obtained from odometry/TF.

It calculates the required linear and angular velocities:

```text
Current Pose
     │
     ▼
Pose Error
     │
     ▼
Siegwart Controller
     │
     ├── Linear velocity
     └── Angular velocity
             │
             ▼
     /navigation_cmd
```

The controller publishes to:

```text
/navigation_cmd
```

rather than directly publishing to `/cmd_vel`.

This is an intentional architectural decision that prevents the navigation controller from competing with the obstacle avoidance system.

---

## 2.5 Perception Layer

The robot's LiDAR provides raw range measurements through:

```text
/scan
```

The obstacle detector converts these raw measurements into a higher-level obstacle representation.

```text
TurtleBot3 LiDAR
       │
       ▼
     /scan
       │
       ▼
Obstacle Detector
       │
       ▼
/obstacle_status
```

The obstacle status contains information such as:

```text
front_distance
left_distance
right_distance
obstacle_detected
```

This separates sensor processing from the decision-making logic of obstacle avoidance.

---

## 2.6 Reactive Obstacle Avoidance

The obstacle avoidance node consumes the processed obstacle information.

```text
/obstacle_status
        │
        ▼
Obstacle Avoidance
        │
        ├── CLEAR
        │
        └── AVOID
              │
              ▼
       /avoidance_cmd
```

When the path is clear, the avoidance system does not interfere with normal navigation.

When an obstacle is detected, it generates an avoidance command.

For critically close obstacles, safety behavior takes priority over normal navigation.

---

## 2.7 Command Arbitration

The command arbitrator is the final authority responsible for selecting the velocity command sent to the robot.

It receives two command sources:

```text
/navigation_cmd
        │
        ├──────────────┐
                       ▼
                Command Arbitrator
                       ▲
        |──────────────┤
        │
/avoidance_cmd
```

The arbitrator publishes the final command:

```text
/cmd_vel
```

Therefore:

```text
Navigation Controller ──► /navigation_cmd ─┐
                                           │
                                           ▼
                                    Command Arbitrator
                                           │
                                           ▼
                                        /cmd_vel
                                           │
                                           ▼
                                      TurtleBot3
```

During normal operation:

```text
/navigation_cmd
       ↓
Command Arbitrator
       ↓
     /cmd_vel
```

During obstacle avoidance:

```text
/avoidance_cmd
       ↓
Command Arbitrator
       ↓
     /cmd_vel
```

This architecture ensures that multiple nodes do not directly compete for control of `/cmd_vel`.

---

## 2.8 Safety Priority

Safety behavior has higher priority than normal navigation.

The command flow can therefore be viewed as:

```text
             Navigation Command
                    │
                    ▼
              ┌───────────┐
              │           │
              │ Arbitrator│
              │           │
              └─────┬─────┘
                    │
                    │
          ┌─────────┴─────────┐
          │                   │
      Normal Motion       Obstacle/Safety
          │                   │
          └─────────┬─────────┘
                    ▼
                 /cmd_vel
```

The key principle is:

> **Navigation determines where the robot should go; safety determines whether that motion is currently allowed.**

This separation makes the system easier to reason about and extend.

---

## 2.9 ROS 2 Communication Architecture

The major communication interfaces are:

| Source             | Interface                    | Destination        | Purpose                           |
| ------------------ | ---------------------------- | ------------------ | --------------------------------- |
| Mission GUI        | `/patrol` Action             | Patrol Server      | Start and monitor patrol missions |
| Patrol Server      | `/navigate_waypoints` Action | Waypoint Server    | Execute waypoint sequences        |
| Waypoint Server    | Goal pose                    | Controller         | Provide navigation target         |
| Controller         | `/navigation_cmd`            | Command Arbitrator | Normal navigation velocity        |
| TurtleBot3         | `/scan`                      | Obstacle Detector  | LiDAR measurements                |
| Obstacle Detector  | `/obstacle_status`           | Obstacle Avoidance | Processed obstacle information    |
| Obstacle Avoidance | `/avoidance_cmd`             | Command Arbitrator | Reactive/safety velocity          |
| Command Arbitrator | `/cmd_vel`                   | TurtleBot3         | Final velocity command            |
| TurtleBot3         | `/odom`                      | Navigation/Control | Robot motion feedback             |

---

## 2.10 Architectural Principles

The system follows several important robotics software architecture principles.

### Separation of Concerns

Each ROS 2 node has a clearly defined responsibility.

```text
Patrol Server
    → mission execution

Waypoint Server
    → waypoint sequencing

Controller
    → motion control

Obstacle Detector
    → sensor interpretation

Obstacle Avoidance
    → reactive behavior

Command Arbitrator
    → final command selection
```

### Hierarchical Control

Higher-level behaviors reuse lower-level capabilities:

```text
Patrol
   ↓
NavigateWaypoints
   ↓
Motion Controller
   ↓
Robot
```

### Single Command Authority

Only the command arbitrator publishes the final `/cmd_vel`.

This avoids conflicting velocity publishers.

### Safety Priority

Obstacle avoidance and safety behavior can override normal navigation.

### Modular Development

Each subsystem can be launched, tested, and debugged independently before being integrated through the top-level bringup system.

### Configuration-Driven Behavior

Controller parameters and patrol missions are configured through YAML where appropriate, reducing the need to modify source code for routine changes.

---

## 2.11 Complete System Data Flow

The complete system can be summarized as:

```text
                  MISSION
                     │
                     ▼
              Mission Control GUI
                     │
                     ▼
               Patrol Action
                     │
                     ▼
               Patrol Server
                     │
                     ▼
          NavigateWaypoints Action
                     │
                     ▼
             Waypoint Server
                     │
                     ▼
              Motion Controller
                     │
                     │ /navigation_cmd
                     ▼
              Command Arbitrator ◄──────── /avoidance_cmd
                     │                         ▲
                     │                         │
                     │                    Obstacle Avoidance
                     │                         ▲
                     │                         │
                     │                  /obstacle_status
                     │                         ▲
                     │                         │
                     │                  Obstacle Detector
                     │                         ▲
                     │                         │
                     ▼                         │
                  /cmd_vel                   /scan
                     │                         ▲
                     ▼                         │
                 TurtleBot3 ───────────────────┘
```

The architecture therefore separates the AMR into five major functional layers:

```text
┌─────────────────────────────────────────┐
│  Mission Layer                          │
│  GUI → Patrol → Waypoint Navigation     │
├─────────────────────────────────────────┤
│  Control Layer                          │
│  Siegwart Motion Controller             │
├─────────────────────────────────────────┤
│  Perception Layer                       │
│  LiDAR → Obstacle Detection             │
├─────────────────────────────────────────┤
│  Safety / Behavior Layer                │
│  Obstacle Avoidance                     │
├─────────────────────────────────────────┤
│  Actuation Layer                        │
│  Command Arbitration → /cmd_vel         │
└─────────────────────────────────────────┘
```

This modular architecture provides a foundation for future integration of SLAM, localization, Nav2, advanced perception, and real-robot deployment.


# 3. Technology Stack

| Component                 | Technology                         |
| ------------------------- | ---------------------------------- |
| Operating System          | Ubuntu 24.04 LTS                   |
| Middleware                | ROS 2 Jazzy Jalisco                |
| Robot                     | TurtleBot3 Burger                  |
| Simulator                 | Gazebo Sim via `ros_gz_sim`       |
| Programming               | Python                             |
| Robot Control             | Siegwart feedback control          |
| Communication             | ROS 2 Topics, Services and Actions |
| Configuration             | YAML                               |
| Visualization / Debugging | RViz2 / ROS 2 CLI                  |
| Version Control           | Git                                |

---

# 4. Workspace Structure

The project is organized as a ROS 2 workspace.

```text
amr_ws/
├── src/
│   └── autonomous-mobile-robot/
│       │
│       ├── amr_interfaces/
│       │   ├── action/
│       │   │   ├── NavigateWaypoints.action
│       │   │   └── Patrol.action
│       │   └── ...
│       │
│       ├── amr_controller/
│       │   ├── amr_controller/
│       │   ├── config/
│       │   │   └── controller.yaml
│       │   ├── launch/
│       │   │   └── controller.launch.py
│       │   └── ...
│       │
│       ├── amr_perception/
│       │   ├── amr_perception/
│       │   │   └── obstacle_detector.py
│       │   └── ...
│       │
│       ├── amr_navigation/
│       │   ├── amr_navigation/
│       │   │   ├── waypoint_server.py
│       │   │   ├── patrol_server.py
│       │   │   ├── obstacle_avoidance.py
│       │   │   ├── cmd_arbitrator.py
│       │   │   ├── mission_control_gui.py
│       │   │   └── tf_pose_monitor.py
│       │   ├── config/
│       │   ├── launch/
│       │   └── ...
│       │
│       ├── amr_bringup/
│       │   └── launch/bringup.launch.py
│       │
│       ├── amr_simulation/
│       │   ├── launch/simulation.launch.py
│       │   └── worlds/amr_world.sdf
│       │
│       └── ...
│
└── install/
```

---

# 5. M1 — Motion Control

The first stage implemented a custom mobile robot controller.

The controller uses the **Siegwart feedback control law** to drive the robot toward a desired pose.

A target pose is defined as:

```text
(x_goal, y_goal, theta_goal)
```

The controller calculates:

```text
rho
alpha
beta
```

where:

* `rho` represents the distance to the goal.
* `alpha` represents the angular error toward the goal.
* `beta` represents the orientation error.

The controller generates:

```text
v = linear velocity
w = angular velocity
```

subject to configured velocity limits.

---

## Orientation Handling

Position and orientation are handled separately.

When the robot is sufficiently close to the desired position:

```python
if rho < self.rho_tol:
```

the controller switches to orientation alignment.

The robot stops translating:

```python
v = 0.0
```

and rotates toward the desired orientation.

This prevents the robot from considering a waypoint complete simply because its position is close enough.

---

## Controller Configuration

Controller parameters are stored in YAML.

Example categories include:

```text
Goal position
Goal orientation
Controller gains
Position tolerance
Orientation tolerance
Maximum linear velocity
Maximum angular velocity
```

This allows controller behavior to be changed without modifying the Python source code.

---

# 6. M2 — NavigateWaypoints Action

The next stage introduced a ROS 2 Action Server for multi-waypoint navigation.

The action is:

```text
NavigateWaypoints
```

A goal contains a sequence of poses:

```text
Waypoint 1
Waypoint 2
Waypoint 3
...
```

The Waypoint Server sends each waypoint to the low-level controller.

Architecture:

```text
NavigateWaypoints Action
          │
          ▼
   Waypoint Server
          │
          ▼
      Controller
          │
          ▼
      TurtleBot3
```

---

## Feedback

The action provides feedback including:

```text
current_waypoint
distance_remaining
```

This allows a client to monitor navigation progress.

---

## Result

The action returns:

```text
success
waypoints_completed
message
```

---

## Cancellation

Navigation goals can be cancelled while the robot is moving.

When cancellation is requested:

1. The active navigation goal is cancelled.
2. The robot's current position is obtained.
3. A stop/hold command is published.
4. The action returns a cancelled result.

This ensures that cancellation does not simply terminate the action server while leaving the robot with an active navigation command.

---

# 7. M3 — Patrol Action

The waypoint navigation system was then extended into a higher-level patrol mission.

The action:

```text
Patrol
```

accepts:

```text
waypoints
patrol_cycles
```

Example:

```text
Waypoint 1
      ↓
Waypoint 2
      ↓
Waypoint 3
      ↓
Waypoint 4
      ↓
Cycle complete
      ↓
Repeat
```

The Patrol Server acts as a higher-level Action Client of the NavigateWaypoints server.

Architecture:

```text
                 Patrol Action
                      │
                      ▼
               Patrol Server
                      │
              Action Client
                      │
                      ▼
             NavigateWaypoints
                      │
                      ▼
              Waypoint Server
                      │
                      ▼
                 Controller
```

This creates a useful hierarchical action architecture.

---

## Patrol Feedback

Patrol feedback includes:

```text
current_cycle
current_waypoint
```

This allows the user to monitor the progress of a complete patrol mission.

---

## Patrol Cancellation

Patrol cancellation also propagates down to the currently active navigation goal.

```text
Patrol cancellation
        ↓
Cancel Patrol
        ↓
Cancel NavigateWaypoints
        ↓
Stop Robot
```

This was tested successfully.

---

# 8. Configuration-Driven Patrol

The patrol system supports configuration through YAML.

A patrol configuration can specify:

```text
Number of waypoints
Waypoint positions
Waypoint orientations
Number of patrol cycles
```

This allows patrol missions to be changed without modifying the navigation code.

---

# 9. Launch System

The project uses Python-based ROS 2 launch files. Components can be launched independently, while the top-level bringup includes the complete demonstration stack.

For example:

```text
bringup.launch.py
       │
       ├── amr.launch.py
       │   ├── Gazebo Sim and TurtleBot3 spawn
       │   ├── ROS-Gazebo bridge
       │   ├── Controller
       │   ├── Patrol server
       │   ├── Obstacle detector
       │   ├── Obstacle avoidance
       │   └── Command arbitrator
       │
       └── Mission control GUI
```

The simulation launch starts `ros_gz_sim` with the custom `amr_world.sdf`, spawns the TurtleBot3, publishes robot state, and runs `ros_gz_bridge`. The bridge converts the ROS 2 `/cmd_vel` `TwistStamped` message to the Gazebo `Twist` command expected by the simulator.

Start the complete stack with:

```bash
ros2 launch amr_bringup bringup.launch.py
```

This makes system startup repeatable and reduces the need to manually launch every node.

---

# 10. M4 — Obstacle-Aware Navigation

The fourth major stage introduced autonomous obstacle handling.

The robot uses its LiDAR data from:

```text
/scan
```

to detect obstacles.

The obstacle system is divided into three components:

```text
LiDAR
  ↓
Obstacle Detector
  ↓
Obstacle Status
  ↓
Obstacle Avoidance
  ↓
Avoidance Command
```

---

# 11. M4.1 — LiDAR Obstacle Detection

The obstacle detector processes `sensor_msgs/msg/LaserScan`.

The detector handles:

* Infinite measurements
* Invalid measurements
* Minimum range
* Maximum range
* Front region
* Left region
* Right region

The detector publishes:

```text
/obstacle_status
```

containing:

```text
front_distance
left_distance
right_distance
closest_obstacle_x
closest_obstacle_y
obstacle_detected
```

Example:

```yaml
front_distance: 0.44
left_distance: 0.53
right_distance: 0.53
obstacle_detected: true
```

The detector therefore provides a higher-level representation of the raw LiDAR data.

---

# 12. M4.2 — Reactive Obstacle Avoidance

A dedicated obstacle avoidance node consumes the obstacle information.

When an obstacle is detected in front of the robot, the system first uses the closest obstacle's lateral position in `base_link` to choose the opposite turn direction. If the obstacle is approximately centered, it compares left and right clearance and chooses the side with more space.

For example:

```text
Obstacle detected
       ↓
Use closest obstacle lateral position
       ↓
Compare left/right clearance if centered
       ↓
Rotate
       ↓
Monitor front distance
       ↓
Path clear
       ↓
Return to CLEAR state
```

The obstacle avoidance system was implemented as a state-based reactive behavior.

Obstacle avoidance states include:

```text
CLEAR
TURN_LEFT
TURN_RIGHT
STOP
```

The command arbitrator separately manages `FOLLOW_GOAL`, `AVOID_OBSTACLE`, `REJOIN_NAVIGATION`, and `EMERGENCY_STOP`. The robot returns to navigation when the obstacle is sufficiently clear; a front distance below `0.20 m` forces an emergency stop.

---

# 13. M4.3 — Command Arbitration

A major architectural improvement was introduced to prevent multiple nodes from directly competing for `/cmd_vel`.

The controller publishes:

```text
/navigation_cmd
```

The obstacle avoidance node publishes:

```text
/avoidance_cmd
```

The command arbitrator is the only node publishing to:

```text
/cmd_vel
```

Architecture:

```text
                     ┌──────────────────┐
/navigation_cmd ────►│                  │
                     │  Cmd Arbitrator  ├────► /cmd_vel
/avoidance_cmd ─────►│                  │
                     └──────────────────┘
```

When no obstacle is present:

```text
/navigation_cmd
       ↓
  Arbitrator
       ↓
    /cmd_vel
```

When an obstacle requires avoidance:

```text
/avoidance_cmd
       ↓
  Arbitrator
       ↓
    /cmd_vel
```

This establishes a clean separation between:

* Navigation
* Reactive safety behavior
* Final velocity command output

---

# 14. M4.4 — Safety / Emergency Behavior

The obstacle-handling architecture was extended with safety behavior for close obstacles.

The system distinguishes between normal navigation and situations where the robot needs to prioritize safety.

Conceptually:

```text
NORMAL NAVIGATION
       │
       │ obstacle
       ▼
OBSTACLE AVOIDANCE
       │
       │ clear
       ▼
NORMAL NAVIGATION
```

and for a critically close obstacle:

```text
CRITICAL OBSTACLE
       ↓
SAFETY / STOP
       ↓
Prevent unsafe motion
```

The behavior was tested as part of the obstacle-aware navigation system.

---

# 15. M4.5 — Full System Integration

The final stage of the current development phase was manual full-system validation.

The complete system was tested with:

* Waypoint navigation
* Patrol missions
* LiDAR obstacle detection
* Reactive obstacle avoidance
* Command arbitration
* Safety behavior
* Action cancellation
* Navigation recovery
* Multiple ROS 2 nodes operating together

The integrated stack was exercised manually with the scenarios listed above. The repository's package test directories currently provide copyright, flake8, and pep257 checks; they do not contain an automated Gazebo integration test.

The resulting architecture is:

```text
                         ┌─────────────────┐
                         │  Patrol Server  │
                         └────────┬────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │ Waypoint Server │
                         └────────┬────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │   Controller    │
                         └────────┬────────┘
                                  │
                         /navigation_cmd
                                  │
                                  ▼
                         ┌─────────────────┐
                         │ Cmd Arbitrator  │◄──── /avoidance_cmd
                         └────────┬────────┘             ▲
                                  │                      │
                               /cmd_vel            ┌──────────────┐
                                  │                │   Obstacle   │
                                  ▼                │   Avoidance  │
                           ┌─────────────┐         └──────▲───────┘
                           │ TurtleBot3  │                 │
                           └──────┬──────┘                 │
                                  │                        │
                                /scan                      │
                                  │                        │
                                  ▼                        │
                         ┌─────────────────┐               │
                         │ Obstacle        │───────────────┘
                         │ Detector        │
                         └─────────────────┘
```

---

# 16. ROS 2 Interfaces

## Actions

### NavigateWaypoints

Used for multi-waypoint navigation.

```text
Goal:
    waypoints

Feedback:
    current_waypoint
    distance_remaining

Result:
    success
    waypoints_completed
    message
```

### Patrol

Used for repeated waypoint patrol missions.

```text
Goal:
    waypoints
    patrol_cycles

Feedback:
    current_cycle
    current_waypoint

Result:
    success
    cycles_completed
    message
```

---

## Topics

### Navigation

```text
/amr_controller/goal       geometry_msgs/msg/Pose2D
/navigation_cmd            geometry_msgs/msg/TwistStamped
```

### Obstacle handling

```text
/scan                      sensor_msgs/msg/LaserScan
/obstacle_status           amr_interfaces/msg/ObstacleStatus
/avoidance_cmd             geometry_msgs/msg/TwistStamped
```

### Final velocity command

```text
/cmd_vel                   geometry_msgs/msg/TwistStamped
```

### Robot state

```text
/odom                      nav_msgs/msg/Odometry
```

The simulation bridge maps ROS 2 `/cmd_vel` `TwistStamped` messages to the Gazebo `Twist` interface.

---

# 17. Design Principles

Several architectural principles are intentionally used throughout the project.

### Separation of responsibilities

Each node has a specific responsibility.

```text
Controller
    → navigation control

Waypoint Server
    → waypoint mission execution

Patrol Server
    → high-level patrol mission

Obstacle Detector
    → sensor interpretation

Obstacle Avoidance
    → reactive behavior

Command Arbitrator
    → final command selection
```

### Hierarchical actions

Higher-level actions use lower-level actions.

```text
Patrol
  ↓
NavigateWaypoints
  ↓
Controller
```

This makes the system easier to extend.

### Configuration over hard-coding

Parameters such as controller gains and patrol missions are placed in configuration files wherever practical.

### Safety before navigation

Obstacle avoidance and safety behavior can override normal navigation commands.

### Test incrementally

Each subsystem was developed and tested before integrating it with the rest of the system.

---

# 18. Development Roadmap

The project is being developed in stages.

```text
M1  ROS 2 + Motion Control
        │
        ▼
M2  Waypoint Navigation
        │
        ▼
M3  Patrol Missions
        │
        ▼
M4  Obstacle-Aware Navigation
        │
        ├── M4.1 LiDAR Detection       ✓
        ├── M4.2 Obstacle Avoidance    ✓
        ├── M4.3 Command Arbitration   ✓
        ├── M4.4 Safety Behavior       ✓
        └── M4.5 Integration Testing  ✓
        │
        ▼
M5  SLAM + Localization
        │
        ▼
M6  Nav2 Integration
        │
        ▼
M7  Advanced Mission Planning
        │
        ▼
M8  Perception
        │
        ▼
M9  Real Robot Deployment
```

---

# 19. Future Development

## M5 — SLAM & Localization

The next phase will introduce map-based autonomy.

Planned components include:

* TF2 coordinate-frame validation
* 2D SLAM
* Occupancy-grid maps
* Map saving/loading
* AMCL localization
* Localization validation

Target architecture:

```text
LiDAR + Odometry
       ↓
      SLAM
       ↓
      Map
       ↓
 Localization
       ↓
   Robot Pose
```

---

## M6 — Nav2

After understanding and implementing the core navigation architecture, the project will move toward the standard ROS 2 Navigation stack.

Planned areas include:

* Nav2
* Global planning
* Local control
* Costmaps
* Recovery behaviors
* Behavior Trees
* Navigation goals
* Dynamic replanning

The custom navigation implementation will provide a useful basis for understanding the design decisions made by Nav2.

---

# 20. Running the Project

Source the ROS 2 environment:

```bash
source /opt/ros/jazzy/setup.bash
```

Source the workspace:

```bash
source ~/amr_ws/install/setup.bash
```

Set the TurtleBot3 model:

```bash
export TURTLEBOT3_MODEL=burger
```

Build the workspace:

```bash
cd ~/amr_ws
colcon build
```

Then source the workspace again:

```bash
source install/setup.bash
```

Individual nodes can be launched using:

```bash
ros2 run <package> <executable>
```

The project also provides launch files for starting multiple components together.

---

# 21. Useful ROS 2 Commands

Check nodes:

```bash
ros2 node list
```

Check topics:

```bash
ros2 topic list
```

Inspect a topic:

```bash
ros2 topic info /cmd_vel -v
```

Monitor LiDAR:

```bash
ros2 topic echo /scan
```

Monitor obstacle status:

```bash
ros2 topic echo /obstacle_status
```

Check actions:

```bash
ros2 action list
```

Inspect an action:

```bash
ros2 action info /navigate_waypoints
```

Inspect patrol:

```bash
ros2 action info /patrol
```

---

# 22. Example Patrol Mission

A patrol mission can be sent using:

```bash
ros2 action send_goal /patrol \
amr_interfaces/action/Patrol \
"{waypoints: [
{x: 0.5, y: 0.5, theta: 0.0},
{x: 1.5, y: 0.5, theta: 1.57},
{x: 1.5, y: 1.5, theta: 3.14},
{x: 0.5, y: 1.5, theta: -1.57}
], patrol_cycles: 1}" \
--feedback
```

The robot will navigate through the waypoints sequentially and report patrol feedback.

---

# 23. Project Goals

The long-term goal is to develop a complete autonomous mobile robot platform demonstrating:

* Motion control
* ROS 2 communication
* Action-based task execution
* Mission planning
* Sensor processing
* Reactive obstacle avoidance
* Safety handling
* Mapping
* Localization
* Autonomous navigation
* Perception
* Real-world deployment

The project emphasizes understanding the underlying robotics concepts rather than treating ROS 2 navigation as a black box.

---

# 24. Learning Outcomes

Through this project, the following concepts are being developed:

### ROS 2

* Nodes
* Topics
* Publishers/subscribers
* Services
* Actions
* Parameters
* YAML configuration
* Launch files
* Executors
* Callback groups
* Goal cancellation

### Robotics

* Differential-drive motion
* Odometry
* Pose control
* Feedback control
* Position/orientation control
* LiDAR processing
* Reactive obstacle avoidance
* Command arbitration
* Safety behavior

### Software Architecture

* Modular ROS 2 nodes
* Hierarchical actions
* Separation of concerns
* Configuration-driven systems
* Asynchronous task execution
* Fault/cancellation handling
* Incremental integration testing

---

# 25. Current Project Milestone

**Current milestone: Full System Integration ✓**

The robot can currently:

```text
Mission Control GUI
        ↓
Receive a patrol mission
        ↓
Execute waypoint navigation
        ↓
Control position + orientation
        ↓
Monitor LiDAR
        ↓
Detect obstacles
        ↓
Perform reactive avoidance
        ↓
Prioritize avoidance commands
        ↓
Handle safety conditions
        ↓
Resume navigation
        ↓
Complete the mission
```

---

## Author

**Prajwal Dutta**

Robotics Engineer

---
