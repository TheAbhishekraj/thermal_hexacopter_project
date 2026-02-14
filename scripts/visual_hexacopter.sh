#!/bin/bash
# visual_hexacopter.sh - Phase 3 Final Launch Script
WORKSPACE_DIR="/root/workspace"
PX4_DIR="/root/PX4-Autopilot"

# 1. Aggressive Cleanup
pkill -9 ruby || true; pkill -9 gz || true; pkill -9 MicroXRCEAgent || true; pkill -9 px4 || true
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# 2. Start the ROS 2 Bridge
echo "🌉 Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 &
sleep 2

# 3. Environment & Paths
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/models:$PX4_DIR/Tools/simulation/gz/models"
export PX4_HOME_LAT=25.344644
export PX4_HOME_LON=86.483958

# 4. Launch Simulation with VTOL Mixer Fallback
echo "🚀 Launching Hexacopter (Custom Model + VTOL Physics Mixer)..."
export PX4_SIM_MODEL=agri_hexacopter_drone

PX4_GZ_WORLD=default

PX4_GZ_MODEL=agri_hexacopter_drone

make -C "$PX4_DIR" px4_sitl gz_standard_vtol
