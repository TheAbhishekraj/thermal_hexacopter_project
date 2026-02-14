#!/bin/bash
# visual_hover.sh - Launch Simulation with GUI
# REQUIRES: Docker container started with X11 forwarding enabled

# 1. Setup Paths
WORKSPACE_DIR="/root/workspace"
PX4_DIR="/root/PX4-Autopilot"

# 2. Cleanup & Sync
pkill -9 ruby || true; pkill -9 gz || true; pkill -9 MicroXRCEAgent || true
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# 3. Start the Bridge
echo "🌉 Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 &
sleep 2

# 4. Environment
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/models:$PX4_DIR/Tools/simulation/gz/models"
export PX4_HOME_LAT=25.344644
export PX4_HOME_LON=86.483958
export HEADLESS=0  # FORCE GUI

# 5. Launch Simulation
echo "🚀 Launching Simulation with GUI..."
PX4_GZ_WORLD=default \
PX4_GZ_MODEL=x500 \
make -C "$PX4_DIR" px4_sitl gz_x500
