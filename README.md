# 🛡️Sensor Guard (ROS 2)

A real-time safety layer for robotic velocity commands, designed to handle noisy inputs, unstable control signals, and system-level failures.

Built in ROS 2, this node enforces safe motion constraints, anomaly detection, and fault recovery for mobile robots.

---

## 🔷 System Overview

/cmd_vel_raw  →  [Sensor Guard]  →  /cmd_vel  
                        │  
                        ├── /safety_state  
                        └── /sensor_guard/reset_fault  

---

## Core Capabilities

### 🧮 1. Signal Conditioning
- Deadband filtering to suppress noise
- Hard limits on linear and angular velocity

### 📊 2. Statistical Robustness
- Sliding-window filtering
- Outlier rejection using mean + standard deviation
- Rate limiting for smooth motion (bounded acceleration)

### 🧠 3. System Awareness
- Detects command stream failure (timeout-based)
- Publishes real-time system state:
  - NORMAL
  - DEGRADED
  - FAULT

### 📉 4. Quality Monitoring
- Tracks how often commands are corrected
- Automatically flags degraded input quality

### 🚨 5. Fault Handling (Latched Safety)
- Fault state persists until explicitly reset
- Robot is forced to stop during FAULT

Reset service:
```bash
ros2 service call /sensor_guard/reset_fault std_srvs/srv/Trigger {}
```

---

## How to Run

```bash
colcon build --symlink-install
source install/setup.bash
ros2 run sensor_guard cmd_vel_guard
```

### Test Input

```bash
ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 1.0}}"
```

### Observe Outputs

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /safety_state
```

---

## Fault Injection

Stop publishing `/cmd_vel_raw`

→ System transitions to FAULT  
→ Robot output forced to zero  

Reset:

```bash
ros2 service call /sensor_guard/reset_fault std_srvs/srv/Trigger {}
```

---

## Future Extensions

- ROS 2 diagnostics integration (`diagnostic_msgs`)  
- Visualization dashboards (Foxglove / rqt)  
- Metrics tracking (latency, correction rate)  
- Integration with Nav2 / autonomy stack  
