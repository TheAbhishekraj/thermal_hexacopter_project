# 🚁 Autonomous Thermal-Imaging Hexacopter for Precision Agriculture
### *Democratizing AI-Driven Crop Disease Detection for Grassroots Farming*

**Lead Architect / Researcher:** Abhishek Raj  
**Institution:** School of Mechanical Engineering  
**Project Codename:** Indra-Eye  
**Status:** ✅ PhD Thesis Validated | 🚀 Startup-Ready | 📅 Sealed: February 15, 2026  

---

> *Dedicated to Shikha and Advika Raj, whose patience and unwavering support made this research possible.*

---

## 🏆 At a Glance

| Metric | Value |
|--------|-------|
| 🤖 AI Accuracy (F1-Score) | **91.9%** |
| ⚡ Inference Latency | **45 ms** |
| 📍 Flight Stability | **± 0.06 m altitude variance** |
| 💸 Platform Cost | **₹1,28,900** |
| 💰 Cost vs. Commercial Baseline (DJI M300 + H20T) | **80.2% cheaper** |
| 🗺️ Mission Level | **Level 3 Autonomous Survey** |
| ✈️ Navigation | **7-Waypoint Zig-Zag Grid** |
| 🏆 Mission Success Rate | **100%** |

---

## 🗂️ Enterprise Repository Structure

```
thermal_hexacopter_project/
├── 🧠 01_AI_Brain/            # MobileNetV2 models, thermal datasets, inferencing nodes
├── 🎮 02_Digital_Twin/        # ROS 2 workspace, Gazebo SITL/HITL configs, digital twin
├── 🚁 03_Hardware_Core/       # CAD files, BOM (₹1,28,900), wiring diagrams, Pixhawk configs
├── 📊 04_Field_Ops/           # Flight logs, Bihar maiden voyage videos, performance metrics
├── 🚀 05_Venture_Capital/     # Pitch decks, Bihar Hub-Spoke model, funding proposals
├── 🎓 06_Academic_Thesis/     # LaTeX thesis, literature review, methodology, publications
├── docs/                      # All supporting documentation and guides
├── ros2_ws/                   # Live ROS 2 workspace (src/agri_hexacopter)
├── indra_eye_project/         # Core 5-layer autonomous mission system
├── simulation/                # Gazebo world files & SITL scenarios
├── scripts/                   # Utility, launch, and automation scripts
└── 📖 README.md               # ← You are here: the master landing page
```

> **Existing directories** (`ros2_ws`, `simulation`, `indra_eye_project`, etc.) are mapped into the enterprise sectors above. The numbered folders serve as curated entry points for investors, thesis reviewers, and collaborators.

---

## 🧠 Section 1 — The System Architecture

The **Indra-Eye** system is a fully autonomous thermal-imaging hexacopter built on a Digital Twin methodology, enabling simulation-first development before real-world deployment.

### Technology Stack

| Layer | Technology |
|-------|------------|
| **Autopilot** | PX4 v1.14 on Pixhawk 4 |
| **Middleware** | ROS 2 Humble (MicroDDS Bridge) |
| **Simulation** | Gazebo (Bihar Maize Farm World) |
| **AI/ML** | MobileNetV2 (TensorFlow) on Raspberry Pi 4 (8GB) |
| **Thermal Sensor** | Seek Thermal CompactPRO |
| **Telemetry** | MAVROS / 433 MHz Radio Link |
| **Containerization** | Docker (UAV Master Hub Golden Image v4.0) |
| **Dev Pipeline** | SITL → HITL → Physical Deployment |

### The 5-Layer Autonomous Mission System

```
┌────────────────────────────────────────────────────────┐
│  Layer 5: Mission Intelligence (State Machine V5)      │
│           BOOT → PREFLIGHT → ARM → TAKEOFF →           │
│           SURVEY → OFFBOARD → LAND → DISARM            │
├────────────────────────────────────────────────────────┤
│  Layer 4: Thermal AI Node (MobileNetV2 Inference)      │
│           91.9% F1-Score | 45ms Latency                │
├────────────────────────────────────────────────────────┤
│  Layer 3: Waypoint Navigation (7-Point Zig-Zag Grid)   │
│           ENU ↔ NED Frame Translation | ±0.06m         │
├────────────────────────────────────────────────────────┤
│  Layer 2: MAVROS Bridge (ROS 2 ↔ PX4 v1.14)           │
│           MicroDDS | Health Monitoring | Failsafe       │
├────────────────────────────────────────────────────────┤
│  Layer 1: PX4 SITL / Physical Pixhawk 4                │
│           Gazebo Bihar World | Real Airframe            │
└────────────────────────────────────────────────────────┘
```

---

## 🚀 Section 2 — Startup Launchpad: Bihar Drone Operations

### The Problem

Commercial agricultural drones (DJI Matrice 300 RTK + Zenmuse H20T) cost **₹6,50,000+**, making precision agriculture economically impossible for Indian smallholder farmers. Furthermore, they run on **closed-source ecosystems**, preventing integration of regionally-tuned AI models for local crop diseases (e.g., maize blight in Bihar).

### The Solution: "The Flying Doctor"

A proprietary, custom-built autonomous hexacopter delivering:
- **Real-time thermal anomaly detection** at an AI accuracy level commercial drones cannot match for regional diseases
- **Open-source software stack** (ROS 2 + PX4) enabling localized AI retraining
- **₹1,28,900 build cost** — 80.2% cheaper than the commercial baseline

### Business Model: Hub & Spoke Operations

```
                    ┌─────────────────────┐
                    │  HQ / R&D HUB       │
                    │  Patna              │
                    │  (Data Analytics)   │
                    └──────────┬──────────┘
                               │
               ┌───────────────┼───────────────┐
               ▼               ▼               ▼
        ┌────────────┐  ┌────────────┐  ┌────────────┐
        │  Spoke 1   │  │  Spoke 2   │  │  Spoke 3   │
        │  Purnia    │  │  Darbhanga │  │  Bhagalpur │
        │  (Primary) │  │            │  │            │
        └────────────┘  └────────────┘  └────────────┘
```

**Hub (Patna/Purnia Corridor):** Central command for R&D and data science analytics, with the primary operational hub in Purnia for direct access to Bihar's agricultural belt.

### The Cooperative Model (Go-To-Market)

| Model | Cost per Farmer | Payback Period |
|-------|----------------|----------------|
| Single farmer purchase | ₹1,28,900 | ~3 years |
| **10-farmer cooperative (recommended)** | **~₹12,890** | **< 1 year** |
| 20-farmer cooperative | ~₹6,445 | < 6 months |

**Why cooperatives work:** Farmers save on fungicide costs; early disease detection prevents crop loss averaging **₹15,000–40,000/hectare/season** in Bihar.

### Service & Aftersales Network

Leveraging predictive maintenance principles (similar to commercial vehicle aftersales), we build a **localized service ecosystem**:
- Trained local technicians for T-motor part swaps and battery maintenance
- Calibration protocols for thermal sensors (Seek Thermal CompactPRO)
- Zero-downtime SLA guarantee for cooperative subscribers

### Regulatory Compliance

- **DGCA Status:** Targeting UAS Type-1 certification under India's Drone Rules 2021
- Remote Pilot Certificate (RPC) training embedded into cooperative onboarding
- Operations designed within BVLOS waiver framework for agricultural zones

---

## 💰 Section 3 — Pitch Deck: Funding Proposal

### **Slide 1: The Vision**
> *Precision Agriculture from the Sky: Democratizing AI-driven crop disease detection for grassroots farming.*

### **Slide 2: The Problem**

- 🌾 **100M+ smallholder farmers** in India with no access to precision agriculture tools (50M+ affected by crop disease annually)
- 💸 Commercial alternatives cost **₹6.5L+** — 50× a farmer's monthly income
- 🔒 Closed-source ecosystems **prevent localized AI** for regional disease variants
- 📉 Crop losses to fungal disease: **₹15,000–40,000/hectare/season** in Bihar

### **Slide 3: Our Solution**

| Feature | Specification |
|---------|--------------|
| Platform | Custom Hexacopter (F550 + Pixhawk 4) |
| Intelligence | MobileNetV2 on Raspberry Pi 4 |
| Accuracy | 91.9% F1-Score |
| Speed | 45ms inference latency |
| Connectivity | ROS 2 Humble + PX4 + MAVROS |
| Cost | ₹1,28,900 (prototype) |

### **Slide 4: Economics & Traction**

```
₹6,50,000  DJI M300 + H20T (Commercial Baseline)
    │
    │  80.2% Cost Reduction
    ▼
₹1,28,900  Indra-Eye Hexacopter (Our Platform)
    │
    │  Cooperative Model (10 Farmers)
    ▼
 ₹12,890   Per Farmer Cost → ROI < 1 Year
```

**Validated Achievements:**
- ✅ 100% autonomous mission success rate (Level 3 Survey)
- ✅ Bihar digital twin physically parity-validated
- ✅ Git Tag `v4.0-final-thesis-seal` — production-ready codebase

### **Slide 5: Funding Ask & Roadmap**

| Phase | Milestone | Funding Required |
|-------|-----------|-----------------|
| Phase 1 (Now) | Scale AI training (dual GPU workstation) | ₹5,00,000 |
| Phase 2 (6 mo) | Purnia pilot — 5 cooperatives, 50 farmers | ₹25,00,000 |
| Phase 3 (12 mo) | Bihar state rollout — 500 cooperatives | ₹1,50,00,000 |

**Target Investors:** AgriTech VCs · Startup Bihar Grants · Government of Bihar (Agricultural Dept.) · Angel Investors (Eastern India ecosystem)

### **Slide 6: The Future**

- 🤖 **Multi-agent swarm operations** for massive-scale farm coverage
- 📊 **Predictive disease analytics** — moving from detection to prediction using historical thermal data
- 🗺️ **Pan-Eastern India** fleet scaling across Bihar, Jharkhand, West Bengal, and Odisha

---

## 🎓 Section 4 — Academic Framework

### Thesis Title
**"Autonomous Thermal-Imaging Hexacopter for Precision Agriculture: A Digital Twin Approach"**

**Author:** Abhishek Raj  
**Department:** School of Mechanical Engineering  
**Status:** ✅ Validated — Ready for Final Submission

---

### Abstract

This research presents the design, simulation, and physical validation of a low-cost autonomous hexacopter integrated with a thermal imaging payload and a MobileNetV2 AI inference engine for early-stage crop disease detection. Employing a **Digital Twin methodology**, the system was developed iteratively — from Gazebo SITL simulation of Bihar farmland topology through to physical maiden voyage — achieving a **91.9% F1-Score** at **45ms latency** with a platform cost 80.2% below the commercial baseline.

---

### 1. Problem Statement

The agricultural sector lacks an economically viable, highly accurate autonomous aerial system for early-stage thermal anomaly detection in crops. Existing solutions are prohibitively expensive and operate on closed-source ecosystems, preventing localized AI integration and predictive maintenance optimization.

---

### 2. Methodology: Digital Twin Approach

```
Phase 1: SITL (Simulation-In-The-Loop)
  └─ Gazebo Bihar Maize Farm World
  └─ 7-Waypoint Autonomous Grid
  └─ PX4 SITL + ROS 2 Humble

         ↓

Phase 2: AI Integration
  └─ Thermal dataset labeling
  └─ MobileNetV2 architecture (edge-optimized)
  └─ F1-Score: 91.9% | Latency: 45ms

         ↓

Phase 3: HITL → Physical Deployment
  └─ Pixhawk 4 hardware integration
  └─ Bihar maiden voyage (Feb 15, 2026)
  └─ Flight stability: ± 0.06m
```

---

### 3. Results & Validation

| KPI | Target | Achieved |
|-----|--------|---------|
| AI F1-Score | > 90% | **91.9%** ✅ |
| Inference Latency | < 50ms | **45ms** ✅ |
| Altitude Stability | < ±0.1m | **±0.06m** ✅ |
| Mission Success Rate | 100% | **100%** ✅ |
| Platform Cost | < ₹1.5L | **₹1,28,900** ✅ |

**Evidence:** See `bihar_maiden_voyage_20260215_*.mp4` in the project root.

---

### 4. Procedure Followed

1. **Design & Kinematics** — Mathematical modeling of hexacopter dynamics (motor mixing matrix, rotor thrust calculations)
2. **Digital Twin Creation** — Gazebo world modeling of Bihar farmland, SITL waypoint validation
3. **Edge AI Training** — Thermal dataset preprocessing, MobileNetV2 fine-tuning, latency optimization for Raspberry Pi 4
4. **Physical Assembly** — F550 airframe, Pixhawk 4 integration, telemetry radio, Seek Thermal sensor mounting
5. **Field Validation** — Bihar maiden voyages; stability metrics and AI inference logged

---

### 5. Future Research Directions

- **Multi-Agent Swarm Intelligence** for parallel large-scale farm coverage
- **Predictive Yield Analytics** — processing historical thermal sequences with LSTM-based models
- **Federated Learning** — distributed AI retraining across deployed drone fleets without centralizing farmer data

---

### Key References

| # | Citation |
|---|---------|
| 1 | Howard, A. G. et al. (2017). MobileNets: Efficient Convolutional Neural Networks for Mobile Vision Applications. *arXiv:1704.04861* |
| 2 | Meier, L. et al. (2015). PX4: A Node-based Multithreaded Open Source Robotics Framework for Deeply Embedded Platforms. *ICRA* |
| 3 | Macenski, S. et al. (2022). Robot Operating System 2: Design, Architecture, and Uses in the Wild. *Science Robotics* |
| 4 | Grieves, M. (2014). Digital Twin: Manufacturing Excellence Through Virtual Factory Replication. *White Paper* |
| 5 | Government of Bihar (2023). Agricultural Statistics & Crop Loss Data. *Bihar Agriculture Department Report* |

> 📖 Full literature review: [`docs/lit_review.md`](docs/lit_review.md)

---

## 🤖 Section 5 — AI Agent System Prompt

If you are using an AI coding assistant to extend this project, paste the following into its system instructions:

```
Project Target: thermal_hexacopter_project
Agent Name: AgriHex-AI

Context & Mission:
You are the lead AI assistant for Abhishek Raj's PhD research project
originating from the School of Mechanical Engineering. Your goal is to assist in
developing, simulating, and deploying an autonomous thermal-imaging hexacopter
designed for precision agriculture.

Technical Stack & Constraints:
  - Core: ROS 2 (Humble/Iron), PX4 Autopilot v1.14, Gazebo Simulation
  - AI/ML: Python, TensorFlow/PyTorch (MobileNetV2 architecture)
    Target: >91.9% F1-score | <50ms inference latency
  - Hardware: MAVROS, Pixhawk 4, Seek Thermal CompactPRO, Raspberry Pi 4 (8GB)
  - Containerization: Docker (UAV Master Hub Golden Image v4.0)

Code Quality Standards:
  - Production-ready, highly modular, heavily commented
  - Always prioritize safety and fail-safes in flight scripts
  - Zero tolerance for hardcoded paths — use ROS 2 param files

Execution Directive:
  - Digital Twin FIRST approach: SITL testing before any physical deployment
  - All paths must align with: ros2_ws/src/agri_hexacopter/...
  - ALWAYS double-check coordinate frames: ENU (ROS 2) vs NED (PX4)
  - State machine phases: BOOT → PREFLIGHT → ARM → TAKEOFF → SURVEY → LAND → DISARM
  - Speak concisely, act as a senior mechanical & software engineer
```

---

## 📁 Quick Links

| Resource | Path |
|----------|------|
| 📜 Completion Certificate | [`COMPLETION_CERTIFICATE.md`](COMPLETION_CERTIFICATE.md) |
| 🛒 Bill of Materials | [`hardware/BOM.md`](hardware/BOM.md) |
| 🎓 Defense Presentation | [`docs/DEFENSE_PRESENTATION.md`](docs/DEFENSE_PRESENTATION.md) |
| 📚 Literature Review | [`docs/lit_review.md`](docs/lit_review.md) |
| 🗺️ PhD Master Guide | [`docs/PHD_MASTER_GUIDE.md`](docs/PHD_MASTER_GUIDE.md) |
| ✈️ Mission Cheatsheet | [`docs/MISSION_COMMAND_CHEATSHEET.md`](docs/MISSION_COMMAND_CHEATSHEET.md) |
| 📊 Audit Results | [`docs/audit_results.md`](docs/audit_results.md) |
| 🐳 Docker Config | [`docker/`](docker/) |
| 🤖 AI Model | [`ai_models/thermal_monitor.py`](ai_models/thermal_monitor.py) |
| 🌍 Simulation | [`simulation/`](simulation/) |

---

## 🛠️ Quick Start

### Run in Simulation (SITL)

```bash
# 1. Start the Docker environment
docker run -it --rm --env DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  uav_master_hub:v4.0 /bin/bash

# 2. Inside Docker - launch the full system
cd /thermal_hexacopter_project
./launch_tmux.sh

# 3. Or use the multi-terminal launcher
./launch_multi_terminal.sh
```

### Run the Bihar Mission
```bash
# Activate ROS 2 workspace
source ros2_ws/install/setup.bash

# Launch PX4 SITL + Bihar World
cd simulation && ros2 launch agri_hexacopter bihar_survey.launch.py

# In a new terminal - run the AI mission node
ros2 run agri_hexacopter mission_controller.py
```

---

## 📊 Validation Evidence

| Evidence | File |
|----------|------|
| 🎬 Maiden Voyage (Short) | `bihar_maiden_voyage_20260215_005826.mp4` |
| 🎬 Maiden Voyage (Extended) | `bihar_maiden_voyage_20260215_010259.mp4` |
| 🎬 Full Mission Recording | `bihar_maiden_voyage_20260215_104431.mp4` |
| 🏆 Completion Certificate | `COMPLETION_CERTIFICATE.md` |
| 🔖 Git Tag | `v4.0-final-thesis-seal` |

---

## 📄 License & Citation

**License:** MIT License — open for academic and non-commercial use.

**Citation (BibTeX):**
```bibtex
@phdthesis{raj2026hexacopter,
  author    = {Abhishek Raj},
  title     = {Autonomous Thermal-Imaging Hexacopter for Precision Agriculture: A Digital Twin Approach},
  school    = {School of Mechanical Engineering},
  year      = {2026},
  note      = {Platform cost: ₹1,28,900 | AI F1-Score: 91.9\% | Inference: 45ms}
}
```

---

<div align="center">

**Built with 🔥 by Abhishek Raj**  
*School of Mechanical Engineering*  
*Sealed: February 15, 2026 | Git Tag: `v4.0-final-thesis-seal`*

</div>
