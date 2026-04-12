# 🚀 Bihar Drone Operations — Full Investor Pitch Deck

**Indra-Eye: Precision Agriculture from the Sky**  
**Lead Architect:** Abhishek Raj | **Date:** April 2026

---

## Slide 1: The Vision

> **"Democratizing AI-driven crop disease detection for grassroots farming."**

The Indra-Eye hexacopter is a proprietary, autonomous aerial system that brings PhD-level thermal AI to farmers who earn ₹5,000/month. We are not building a drone. We are building the **Flying Doctor** for India's 100 million smallholder farmers.

---

## Slide 2: The Problem

### The Agricultural Crisis Nobody Talks About

| Pain Point | Data |
|------------|------|
| Smallholder farmers affected by crop disease annually | **50M+ in India** |
| Average crop loss per hectare per season (Bihar) | **₹15,000 – ₹40,000** |
| Cost of the cheapest commercial drone alternative | **₹6,50,000** (DJI M300) |
| % of Indian farmers who can afford it | **< 0.1%** |
| Commercial drone ecosystem | **Closed-source, no local AI** |

**The technology exists. The access doesn't.**

---

## Slide 3: Our Solution

### The Indra-Eye Hexacopter

A custom-built Level 3 autonomous drone with an embedded AI physician for your fields.

```
┌─────────────────────────────────────────────────┐
│              INDRA-EYE SYSTEM                   │
│                                                 │
│  [Thermal Camera]──→[Raspberry Pi 4]──→[AI]    │
│       ↑                                  ↓      │
│  [PX4 Autopilot]←──[ROS 2 Middleware]←──[Alert]│
│       ↑                                         │
│  [Pixhawk 4]──→[6× T-Motor]──→[Flight]         │
└─────────────────────────────────────────────────┘
```

| Specification | Value |
|---------------|-------|
| AI Model | MobileNetV2 (edge-optimized) |
| Accuracy | **91.9% F1-Score** |
| Speed | **45ms inference latency** |
| Flight Stability | **±0.06m altitude variance** |
| Mission Type | Fully autonomous 7-waypoint survey |
| Platform | ROS 2 Humble + PX4 v1.14 (open-source) |
| **Build Cost** | **₹1,28,900** |

---

## Slide 4: Market & Business Model

### Total Addressable Market (TAM)

| Segment | Farmers | Cooperatives (10-farmer) | Revenue Potential |
|---------|---------|--------------------------|-------------------|
| Bihar (Phase 1) | 8M | 800,000 | ₹1,032 Cr |
| Eastern India (Phase 2) | 40M | 4,000,000 | ₹5,160 Cr |
| Pan India (Phase 3) | 100M | 10,000,000 | ₹12,890 Cr |

### The Cooperative Go-To-Market Model

```
10 Farmers Pool Together
         ↓
  Pay ₹12,890 each
         ↓
 Share 1 Indra-Eye Drone
         ↓
 ROI in < 1 Year (via saved fungicide + prevented crop loss)
```

**Revenue Streams:**
1. **Hardware:** Drone unit sales to cooperatives (₹1,28,900/unit)
2. **DaaS (Drone-as-a-Service):** Monthly subscription for flight ops + AI reports
3. **Data Analytics:** Aggregated crop health data sold to agri-insurance & commodity markets
4. **Service & Maintenance:** Localized technician network (recurring revenue)

---

## Slide 5: Competitive Advantage

| Feature | Indra-Eye | DJI M300 | Generic Drones |
|---------|-----------|----------|----------------|
| Cost | **₹1,28,900** | ₹6,50,000 | ₹80,000–2,00,000 |
| AI for local disease | **Yes (trained)** | No | No |
| Open-source stack | **Yes (ROS 2 + PX4)** | No | No |
| Retrain-able locally | **Yes** | No | Rare |
| Precision agriculture mode | **Level 3 autonomous** | Limited | No |
| Digital twin tested | **Yes (Bihar Gazebo)** | No | No |
| PhD/Research validated | **Yes** | No | No |

**Our moat:** Locally trained AI + open-source extensibility + academic validation + price.

---

## Slide 6: Traction & Validation

### We Have Already Proven This Works

| Milestone | Status |
|-----------|--------|
| Digital Twin (Gazebo Bihar Farm) | ✅ Complete |
| SITL Mission (7-waypoint grid) | ✅ 100% success |
| Physical maiden voyage | ✅ Feb 15, 2026 |
| AI validation (91.9% F1-Score) | ✅ Confirmed |
| PhD thesis sealed | ✅ `v4.0-final-thesis-seal` |
| Docker Golden Image | ✅ v4.0 published |

### Evidence
- 🎬 3 maiden voyage recordings in the repository
- 📜 Official PhD Completion Certificate
- 🔖 Git-tagged, immutable research codebase

---

## Slide 7: Team

| Role | Person |
|------|--------|
| **Founder & Lead Architect** | Abhishek Raj |
| **Institution** | School of Mechanical Engineering |
| **Seeking** | Co-founder (Operations / BD) · Technical Advisor (AgriTech) |

**Expertise:** ROS 2, PX4 Autopilot, Gazebo simulation, MobileNetV2 AI, precision agriculture, commercial vehicle aftersales & dealer management (bringing supply chain thinking to drone ops).

---

## Slide 8: Funding Ask

### Seeking: ₹5,00,000 (Phase 1 — R&D Scale-Up)

| Use of Funds | Amount |
|-------------|--------|
| High-performance GPU workstation (dual NVIDIA RTX) for parallel AI training + SO(3) simulation | ₹3,50,000 |
| Thermal dataset expansion and AI model retraining | ₹75,000 |
| Bihar field trial (5 cooperatives, 50 farmers) | ₹50,000 |
| IP filing and regulatory clearances (DGCA) | ₹25,000 |

### 12-Month Roadmap Post-Funding

```
Month 1-2:  GPU workstation setup → accelerated MobileNetV2 retraining on expanded dataset
Month 3-4:  5-cooperative pilot in Purnia district
Month 5-6:  DaaS platform launch + data analytics dashboard
Month 7-9:  Scale to 50 cooperatives across Bihar
Month 10-12: Series A preparation / Government MoU
```

---

## Contact

**Abhishek Raj**  
Lead Architect, Indra-Eye Project  
School of Mechanical Engineering  

> *"The best time to plant a tree was 20 years ago. The best time to bring AI-powered precision agriculture to Bihar is now."*
