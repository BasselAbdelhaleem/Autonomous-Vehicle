# 🚗 Autonomous Vehicle — From Children's Ride-On to Self-Driving Robot

> A fully autonomous ground vehicle built from the ground up — hardware hacked, software architected, and driven by nothing but code.

---

## 🧠 What Is This?

What started as a toy was transformed into a fully functional self-driving platform — complete with real-time localization, path planning, and closed-loop motor control — using a layered hardware-software architecture inspired by industry-standard autonomous vehicle design.

This project demonstrates mastery across the **full robotics stack**: mechanical modification, embedded systems, low-level control theory, and high-level ROS-based autonomy.

---

## 🎥 Demo


https://github.com/user-attachments/assets/c8e445ca-634c-4c97-9aab-e79757e4ddac


---

## ✨ Key Features

- **Full hardware conversion** of a commercial ride-on car into an autonomous platform
- **Closed-loop speed & steering control** using encoder odometry and IMU heading
- **Pure Pursuit path tracking** for smooth, accurate trajectory following
- **ROS-based autonomy stack** handling navigation, localization, and path planning
- **Layered control architecture** separating high-level planning from low-level actuation
- **Real-time serial communication** between Raspberry Pi and Arduino over UART
- **Gazebo simulation environment** for algorithm validation, running in parallel with hardware development

---

## 🏗️ System Architecture

The system is split into three distinct layers, mirroring professional autonomous vehicle design:

```
┌─────────────────────────────────────────────────┐
│           HIGH-LEVEL  (Raspberry Pi)            │
│  ROS Navigation · Path Planning · Localization  │
│          Pure Pursuit Controller                │
└────────────────────┬────────────────────────────┘
                     │ UART / Serial
┌────────────────────▼────────────────────────────┐
│          LOW-LEVEL  (Arduino Uno)               │
│   Speed PID · Servo Angle · Motor Driver PWM   │
└────────────────────┬────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────┐
│            SENSING  (Arduino Nano)              │
│     IMU Heading · Encoder Angle · Feedback      │
└─────────────────────────────────────────────────┘
```

---

## 🔧 Hardware

### Platform
| Component | Details |
|---|---|
| **Base Vehicle** | Children's electric ride-on car |
| **Steering** | OEM steering wheel removed → Servo motor |
| **Drive** | Original DC motors retained, controlled via motor driver |
| **Odometry** | Rotary encoder mounted on rear axle via belt drive |
| **Heading** | IMU (Inertial Measurement Unit) |

### Electronics
| Component | Role |
|---|---|
| **Raspberry Pi** | High-level autonomy (ROS) |
| **Arduino Uno** | Low-level motor & servo control |
| **Arduino Nano** | Sensor reading (IMU + encoder) |
| **Motor Driver** | PWM voltage control to drive wheels |
| **Servo Motor** | Steering actuation |
| **Rotary Encoder** | Wheel speed & distance measurement |
| **IMU** | Yaw/heading estimation |

### Mechanical Modifications
- Steering column disassembled; servo motor coupled to the steering linkage
- Belt-and-pulley transmission added to the rear axle to interface the encoder without disrupting the drivetrain
- Electronics bay fabricated and mounted to the chassis

---

## 💻 Software Stack

### High-Level — Raspberry Pi (ROS)

The Raspberry Pi runs the full ROS navigation stack:

- **Localization**: Dead-reckoning using wheel encoder odometry fused with IMU heading data. Pose estimated by integrating velocity and angular rate over time.
- **Path Planning**: Waypoint-based global path generation.
- **Path Tracking**: **Pure Pursuit controller** — computes the required steering angle to track a look-ahead point on the desired path, producing smooth and stable trajectory following.
- **Communication**: Publishes target speed and steering commands to the Arduino Uno over serial.

```
ROS Topics:
  /odom          → Localization (encoder + IMU fusion)
  /cmd_vel       → Velocity commands to low-level controller
  /path          → Planned trajectory
  /imu/data      → Raw IMU readings
```

### Low-Level — Arduino Uno

Responsible for translating high-level commands into physical actuation:

- Receives **target speed** and **servo angle** from Raspberry Pi over UART
- Runs a **PID speed controller**: compares target speed against encoder feedback and outputs the appropriate PWM voltage to the motor driver
- Drives the **servo motor** to the commanded steering angle
- Sends sensor feedback back upstream

### Sensing Layer — Arduino Nano

Dedicated sensor node for clean, low-latency hardware reads:

- Reads **IMU** for real-time heading (yaw)
- Reads **rotary encoder** for wheel angle and velocity
- Streams data to the Arduino Uno for processing and forwarding

---

## 📐 Control Theory

### Pure Pursuit Path Tracking

The Pure Pursuit algorithm selects a **look-ahead point** on the reference path at a fixed distance ahead of the vehicle and computes the required steering angle using the geometric relationship:

```
δ = arctan(2 * L * sin(α) / ld)

Where:
  δ  = steering angle
  L  = vehicle wheelbase
  α  = angle between vehicle heading and look-ahead point
  ld = look-ahead distance
```

The look-ahead distance was tuned experimentally to balance responsiveness and stability at the vehicle's operating speed.

### Speed PID

A discrete PID controller on the Arduino Uno regulates wheel speed:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de/dt

Where:
  e(t) = target_speed − measured_speed (from encoder)
  u(t) = PWM duty cycle → motor driver
```

---

## 🖥️ Simulation — ROS + Gazebo
 
A key part of the development workflow was running the path planning software in a **Gazebo simulation environment in parallel with real hardware development**. This allowed us to iterate on the planning algorithms rapidly and validate behavior before every deployment on the physical vehicle — a practice standard in professional robotics engineering.
 
### What Was Simulated
 
Rather than simulating the full vehicle physics, we focused the simulation on what mattered most: **validating the path planning logic in a realistic spatial environment**.
 
- **Custom Gazebo World**: A purpose-built environment was designed to replicate the real-world test area, including obstacles and boundary constraints. This gave the planner a geometrically faithful space to operate in.
- **Path Planning Validation**: The Pure Pursuit controller and waypoint navigation pipeline were run inside Gazebo, allowing us to observe trajectory tracking behavior, tune look-ahead distance, and catch edge cases — all without touching the hardware.
- **Parallel Development**: Simulation and hardware work ran simultaneously. Insights from the simulator directly informed control parameter tuning on the real vehicle, and hardware limitations in turn shaped how the simulation environment was refined.
 
### Why This Matters
 
Simulation-driven development is the industry standard in autonomous systems — used by teams at Waymo, NVIDIA, and virtually every robotics company to de-risk hardware testing. Building this workflow from scratch demonstrates not just coding ability, but **engineering process maturity**.
 
### Simulation vs. Real World
 
| Aspect | Simulation (Gazebo) | Real Vehicle |
|---|---|---|
| **Environment** | Custom Gazebo world | Physical test area |
| **Path Planning** | ✅ Full pipeline | ✅ Full pipeline |
| **Localization** | Simulated ground truth | Encoder + IMU dead-reckoning |
| **Motor Control** | Not simulated | Arduino Uno PID |
| **Purpose** | Algorithm validation & tuning | End-to-end system validation |
 
---

## 📁 Repository Structure

```
Autonomous Vehicle/
├── hardware/ # 3D models of hardware modifications
├── arduino/
│   ├── nano_sensor_node/       # IMU + encoder reading (Arduino Nano)
│   └── uno_low_level_control/  # PID speed control + servo (Arduino Uno)
├── ros_ws/
│   └── build/
│   └── devel/
│   └── src/
│       └── communication/      # ROS ↔ Arduino communication
│       ├── localization/       # Odometry + IMU fusion
│       ├── navigation/         # Path planning & waypoint management
│       ├── path_planning/      # Pure Pursuit controller
├── simulation/ # ROS + Gazebo files
├── media/                      # Demo videos and images
└── README.md
```

## 🧩 Skills Demonstrated

This project touches every layer of a real autonomous system — making it directly relevant to roles in **robotics engineering**, **autonomous systems**, and **embedded control**:

| Domain | Demonstrated Skills |
|---|---|
| **Embedded Systems** | Arduino firmware, UART communication, interrupt-driven encoder reading |
| **Control Theory** | PID speed control, Pure Pursuit geometric path tracking |
| **ROS** | Node architecture, topic pub/sub, rosserial, launch files |
| **Sensor Fusion** | IMU + encoder dead-reckoning for pose estimation |
| **Mechanical Design** | Belt-drive encoder mounting, servo integration, chassis modification |
| **System Architecture** | 3-layer hierarchical control design (sensing → low-level → high-level) |
| **Simulation** | Custom Gazebo world design, parallel sim-hardware development workflow |

---

## 📬 Contact

Questions, collaborations, or just want to talk robots?

📧 bassel.abdelhaleem@gmail.com
🔗 [LinkedIn]([https://linkedin.com/in/yourprofile](https://linkedin.com/in/bassel-abdelhaleem-66a898216))

---

<p align="center">
  <i>Built with curiosity, soldered with patience, and driven by code.</i>
</p>
