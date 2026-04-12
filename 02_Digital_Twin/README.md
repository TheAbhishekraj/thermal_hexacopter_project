# 🎮 02 — Digital Twin

**ROS 2 workspace, Gazebo simulation worlds, SITL/HITL configurations.**

## Stack
- **ROS 2:** Humble | **PX4:** v1.14 | **Gazebo:** Bihar Maize Farm World
- **DDS:** MicroDDS Bridge (ROS 2 ↔ PX4)
- **Docker:** UAV Master Hub Golden Image v4.0

## Key Directories

| Path | Description |
|------|-------------|
| [`../ros2_ws/`](../ros2_ws/) | Live ROS 2 colcon workspace |
| [`../simulation/`](../simulation/) | Gazebo world files & SITL scenarios |
| [`../indra_eye_project/`](../indra_eye_project/) | 5-layer autonomous mission nodes |
| [`../docker/`](../docker/) | Dockerfile & container configs |

## Launch

```bash
# Full SITL stack via tmux
./launch_tmux.sh

# Or multi-terminal launcher
./launch_multi_terminal.sh
```
