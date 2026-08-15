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
- [x] Full system integration testing

### Planned

- [ ] TF2 validation and coordinate-frame architecture
- [ ] 2D SLAM
- [ ] Map creation and persistence
- [ ] AMCL localization
- [ ] Nav2 integration
- [ ] Advanced mission planning
- [ ] Perception extensions
- [ ] Real-robot deployment and robustness testing

---

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
       ↓
SLAM & Localization
       ↓
Nav2
````

This approach provides a clear understanding of the underlying robotics architecture before moving toward higher-level navigation frameworks.

---

# 2. System Architecture

The current system is organized into several independent ROS 2 nodes.

```text
                         ┌──────────────────────┐
                         │     Patrol Server    │
                         │    /patrol Action    │
                         └──────────┬───────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │   Waypoint Server    │
                         │ /navigate_waypoints  │
                         └──────────┬───────────┘
                                    │
                              Goal Pose
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │      Controller      │
                         │  Siegwart Feedback   │
                         └──────────┬───────────┘
                                    │
                            /navigation_cmd
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │   Command Arbitrator │
                         └──────────┬───────────┘
                                    │
                                  /cmd_vel
                                    │
                                    ▼
                             ┌────────────┐
                             │ TurtleBot3 │
                             └─────┬──────┘
                                   │
                                  /scan
                                   │
                                   ▼
                         ┌──────────────────────┐
                         │  Obstacle Detector   │
                         └──────────┬───────────┘
                                    │
                           /obstacle_status
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │  Obstacle Avoidance  │
                         └──────────┬───────────┘
                                    │
                            /avoidance_cmd
                                    │
                                    └──────────────►
```

The key design principle is **separation of responsibilities**.

The navigation controller does not directly compete with the obstacle avoidance controller for `/cmd_vel`.

Instead:

```text
Controller
    │
    └── /navigation_cmd
             │
             ▼
       Command Arbitrator
             ▲
             │
    /avoidance_cmd
             │
     Obstacle Avoidance
```

The arbitrator determines which command should ultimately be sent to the robot.

---

# 3. Technology Stack

| Component                 | Technology                         |
| ------------------------- | ---------------------------------- |
| Operating System          | Ubuntu 24.04 LTS                   |
| Middleware                | ROS 2 Jazzy Jalisco                |
| Robot                     | TurtleBot3 Burger                  |
| Simulator                 | Gazebo                             |
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
│       ├── amr_navigation/
│       │   ├── amr_navigation/
│       │   │   ├── waypoint_server.py
│       │   │   ├── patrol_server.py
│       │   │   ├── obstacle_detector.py
│       │   │   ├── obstacle_avoidance.py
│       │   │   └── cmd_arbitrator.py
│       │   ├── config/
│       │   ├── launch/
│       │   └── ...
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

The project uses Python-based ROS 2 launch files.

The launch architecture was developed incrementally.

Individual components can be launched independently, while a higher-level launch file can include multiple subsystems.

For example:

```text
AMR Launch
    │
    ├── Gazebo / TurtleBot3
    │
    ├── Controller Launch
    │
    └── Patrol Launch
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

When an obstacle is detected in front of the robot, the system selects an avoidance direction based on the available space.

For example:

```text
Obstacle detected
       ↓
Compare left/right clearance
       ↓
Choose direction
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

Typical states include:

```text
CLEAR
AVOID
```

The robot returns to the normal navigation command once the obstacle is sufficiently clear.

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

The final stage of the current development phase was full integration testing.

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

The integration test passed successfully.

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
/amr_controller/goal
/navigation_cmd
```

### Obstacle handling

```text
/scan
/obstacle_status
/avoidance_cmd
```

### Final velocity command

```text
/cmd_vel
```

### Robot state

```text
/odom
```

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

The current implementation is complete through **M4.5**.

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

**Current milestone: M4.5 — Full System Integration ✓**

The robot can currently:

```text
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

The next major milestone is:

> **M5 — SLAM & Localization**

---

## Author

**Prajwal Dutta**

Robotics Engineer

---


