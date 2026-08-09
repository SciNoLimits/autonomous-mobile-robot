# Autonomous Mobile Robot Navigation and Behavior System

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
                  /cmd_vel
                       │
                       ▼
                  TurtleBot3
```
