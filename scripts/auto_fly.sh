#!/bin/bash

# --- 1. SETUP ENVIRONMENT ---
source /opt/ros/humble/setup.bash

# Find and source px4_msgs (Crucial for the ModuleNotFoundError)
PX4_MSGS_PATH=$(find /root -name "local_setup.bash" | grep "px4_msgs" | head -n 1)
if [ -z "$PX4_MSGS_PATH" ]; then
    echo "⚠️ Warning: px4_msgs not found. Manual check required."
else
    source $PX4_MSGS_PATH
    echo "✅ Sourced px4_msgs from: $PX4_MSGS_PATH"
fi

# --- 2. TERMINAL 1: PHYSICS (GAZEBO) ---
echo "🚁 Opening Gazebo in 100 seconds..."
sleep 100
gnome-terminal --title="PHYSICS - PX4 SITL" -- bash -c "cd /root/PX4-Autopilot; make px4_sitl gz_x500; exec bash" &

# --- 3. TERMINAL 2: BRIDGE (DDS) ---
echo "🌉 Opening Bridge in 10 seconds..."
sleep 10
gnome-terminal --title="BRIDGE - MicroXRCE" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash" &

# --- 4. TERMINAL 3: THE BRAIN (FLIGHT) ---
echo "🧠 Opening Flight Logic in 10 seconds..."
sleep 10
gnome-terminal --title="BRAIN - ROS2 MISSION" -- bash -c "cd /root/workspace; colcon build --symlink-install --packages-select agri_hexacopter; source install/setup.bash; ros2 run agri_hexacopter real_takeoff; exec bash" &

echo "🚀 All systems triggered. Watch the new windows!"
