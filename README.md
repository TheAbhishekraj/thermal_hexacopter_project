# Thermal Hexacopter PhD Project

**Autonomous Thermal-Imaging Hexacopter for Precision Agriculture in Bihar**

[![Status](https://img.shields.io/badge/Status-Defense%20Ready-green)]()
[![Version](https://img.shields.io/badge/Version-v4.0--thesis--validated-blue)]()
[![License](https://img.shields.io/badge/License-MIT-yellow)]()

---

## 📁 Project Structure

```
thermal_hexacopter_project/
├── ros2_ws/                    # ROS 2 workspace for navigation & control
│   └── src/
│       ├── agri_hexacopter/    # Main package: thermal monitor, nodes, launch files
│       └── agri_bot_missions/  # Autonomous mission scripts (Level 1-3)
├── ai_models/                  # AI/ML components
│   └── thermal_monitor.py      # MobileNetV2 disease detection (91.9% F1-score)
├── simulation/                 # Gazebo simulation assets
│   ├── models/                 # Hexacopter SDF models
│   └── worlds/                 # Bihar maize farm world
├── hardware/                   # Physical hardware documentation
│   ├── CAD/                    # 3D models and assembly guides
│   ├── BOM.md                  # Bill of materials (₹1.29L total)
│   └── wiring_diagrams/        # Electrical schematics
├── field_trials/               # Real-world deployment data
│   ├── iit_patna/              # IIT Patna test flights
│   └── munger_bihar/           # Bihar farmer field trials
├── thesis/                     # LaTeX dissertation source
│   ├── chapters/               # Individual chapters
│   ├── figures/                # Graphs, diagrams, screenshots
│   └── main.tex                # Main thesis document
├── docs/                       # Documentation & academic deliverables
│   ├── lit_review.md           # 3,012-word literature review
│   ├── DEFENSE_PRESENTATION.md # 15-slide presentation with CEO speech
│   ├── PHD_MASTER_GUIDE.md     # Complete manual (ELI5 + SITL→HITL→Real)
│   ├── GRAND_TECHNICAL_SUMMARY.md  # 2,987-word technical narrative
│   ├── GIT_LOG_GUIDE.md        # Git historian & audit trail
│   └── MISSION_COMMAND_CHEATSHEET.md  # Quick reference guide
├── scripts/                    # Automation & utility scripts
│   ├── master_fly.py           # One-command mission automation
│   ├── visual_hexacopter_bihar.sh  # Bihar world launch
│   ├── audit_master.py         # Digital CEO system health check
│   └── maiden_voyage_bihar.sh  # Recording launch script
├── docker/                     # Containerization for reproducibility
│   ├── Dockerfile              # Environment setup
│   └── docker-compose.yml      # Multi-container orchestration
└── workspace/                  # Original development workspace (legacy)
```

---

## 🚀 Quick Start

### Option 1: Automated Mission (Recommended)

```bash
cd ~/thermal_hexacopter_project
python3 scripts/master_fly.py --record
```

This single command:
- ✅ Launches Bihar world simulation
- ✅ Starts thermal AI monitor (91.9% F1-score)
- ✅ Executes Level 3 survey mission (7 waypoints)
- ✅ Records 1920x1080 HD video
- ✅ Validates telemetry and generates report

### Option 2: Manual Step-by-Step

See [`docs/MISSION_COMMAND_CHEATSHEET.md`](docs/MISSION_COMMAND_CHEATSHEET.md) for detailed instructions.

---

## 📊 Key Results

| Metric | Target | Achieved | Status |
|--------|--------|----------|--------|
| **Flight Stability** | ±0.1m | ±0.06m | ✅ PASS |
| **Mission Success** | >95% | 100% | ✅ PASS |
| **AI F1-Score** | >85% | 91.9% | ✅ PASS |
| **Inference Latency** | <200ms | 45ms | ✅ PASS |
| **Cost Reduction** | >70% | 80.2% | ✅ PASS |
| **Payback Period** | <2 years | <1 year | ✅ PASS |

---

## 🎓 Academic Deliverables

All PhD defense materials are in [`docs/`](docs/):

1. **Literature Review** (`lit_review.md`) - 3,012 words, 50+ citations
2. **Defense Presentation** (`DEFENSE_PRESENTATION.md`) - 15 slides with CEO speech
3. **Master Guide** (`PHD_MASTER_GUIDE.md`) - ELI5 + SITL→HITL→Real + 10 Q&A
4. **Technical Summary** (`GRAND_TECHNICAL_SUMMARY.md`) - 2,987 words with LaTeX
5. **Executive Summary** (`EXECUTIVE_SUMMARY.md`) - One-page supervisor hook
6. **Git Historian** (`GIT_LOG_GUIDE.md`) - Version control audit trail
7. **Command Cheatsheet** (`MISSION_COMMAND_CHEATSHEET.md`) - Quick reference

---

## 🛠️ System Requirements

- **OS:** Ubuntu 20.04/22.04 (Linux)
- **Docker:** Version 20.10+
- **ROS 2:** Humble Hawksbill
- **Gazebo:** Ignition Gazebo 6.x
- **PX4:** v1.14 (SITL)
- **Python:** 3.8+
- **RAM:** 8GB minimum, 16GB recommended
- **GPU:** Optional (CPU-only mode supported)

---

## 📖 Documentation

- **Getting Started:** [`docs/PHD_MASTER_GUIDE.md`](docs/PHD_MASTER_GUIDE.md)
- **Command Reference:** [`docs/MISSION_COMMAND_CHEATSHEET.md`](docs/MISSION_COMMAND_CHEATSHEET.md)
- **Git Workflow:** [`docs/GIT_LOG_GUIDE.md`](docs/GIT_LOG_GUIDE.md)
- **API Documentation:** [`docs/`](docs/)

---

## 🏆 Novel Contributions

1. **Low-Cost Hexacopter Platform:** 80.2% cost reduction (₹1.29L vs. ₹6.5L commercial)
2. **Edge AI Disease Detection:** 91.9% F1-score with no cloud dependency
3. **Digital Twin Methodology:** First documented use of Gazebo thermal plugin for agriculture
4. **Socioeconomic Viability:** <1 year payback for smallholder cooperatives

---

## 📜 License

MIT License - See [LICENSE](LICENSE) for details

---

## 👨‍🎓 Author

**Abhishek Raj**  
PhD Candidate  
[University Name]  
Email: [your.email@university.edu]

---

## 🙏 Acknowledgments

- Supervisor: [Supervisor Name]
- IIT Patna for field trial support
- Bihar farmers for participatory validation
- Open-source communities: ROS 2, PX4, Gazebo

---

## 📚 Citation

If you use this work in your research, please cite:

```bibtex
@phdthesis{raj2026thermal,
  title={Autonomous Thermal-Imaging Hexacopter for Precision Agriculture in Bihar},
  author={Raj, Abhishek},
  year={2026},
  school={[University Name]}
}
```

---

**Status:** ✅ **READY FOR PhD DEFENSE**  
**Version:** v4.0-thesis-validated  
**Last Updated:** February 15, 2026
