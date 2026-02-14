#!/bin/bash
# maiden_voyage_bihar.sh - The Final Recording Launch
# COO Directive: Prepare Bihar World for 1920x1080 Screen Recording

WORKSPACE_DIR="/root/workspace"
PX4_DIR="/root/PX4-Autopilot"

echo "🌾 MAIDEN VOYAGE: Bihar World Recording Setup"
echo "=============================================="

# 1. Environment Setup
export DISPLAY=${DISPLAY:-:0}
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

# 2. ROS 2 Environment
echo "📦 Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# 3. Start MicroXRCEAgent
echo "🔌 Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 3

# 4. Gazebo Resource Path
echo "🗺️  Configuring Gazebo resource paths..."
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/models:$WORKSPACE_DIR/worlds:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

# 5. PX4 Configuration
echo "🚁 Configuring PX4 for Bihar..."
export PX4_SIM_MODEL=gz_standard_vtol
export PX4_GZ_MODEL=agri_hexacopter_drone
export PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0"

# 6. Bihar GPS Coordinates (Samastipur, Bihar)
echo "📍 Setting Bihar GPS coordinates..."
export PX4_HOME_LAT=25.344644
export PX4_HOME_LON=86.483958
export PX4_HOME_ALT=50.0

# 7. Bihar World Selection
echo "🌾 Loading Bihar Maize Farm World..."
export PX4_GZ_WORLD=bihar_maize_farm

# 8. Launch Simulation
echo "🚀 Launching PX4 SITL with Bihar World..."
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║  MAIDEN VOYAGE: Smart Bird in Bihar Maize Field           ║"
echo "║                                                            ║"
echo "║  The hexacopter is now sitting on the landing pad.        ║"
echo "║  Gazebo window should show the Bihar maize farm.          ║"
echo "║                                                            ║"
echo "║  📹 RECORDING COMMAND (run in host terminal):             ║"
echo "║                                                            ║"
echo "║  ffmpeg -video_size 1920x1080 -framerate 30 \\             ║"
echo "║    -f x11grab -i :0.0 \\                                   ║"
echo "║    -c:v libx264 -preset ultrafast -crf 18 \\               ║"
echo "║    bihar_maiden_voyage_\$(date +%Y%m%d_%H%M%S).mp4         ║"
echo "║                                                            ║"
echo "║  Then say 'FLY' to trigger the survey mission!            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

cd "$PX4_DIR"
make px4_sitl gz_standard_vtol

# Cleanup
kill $AGENT_PID 2>/dev/null
