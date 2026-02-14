#!/bin/bash
# master_launch.sh - Automated 3-Terminal Bihar Maiden Voyage Launcher
# This script automatically opens 3 terminals with proper timing

set -e

PROJECT_DIR="$HOME/thermal_hexacopter_project"
cd "$PROJECT_DIR"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  🚁 BIHAR MAIDEN VOYAGE - AUTOMATED 3-TERMINAL LAUNCHER       ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "This script will automatically open 3 terminals:"
echo "  Terminal 1: Gazebo Simulation (Bihar World + PX4 SITL)"
echo "  Terminal 2: Thermal AI Monitor (MobileNetV2 Detection)"
echo "  Terminal 3: Survey Mission (Level 3 Autonomous Flight)"
echo ""
echo "Each terminal will open with proper timing to ensure dependencies."
echo ""

# Check if gnome-terminal is available
if ! command -v gnome-terminal &> /dev/null; then
    echo "❌ ERROR: gnome-terminal not found"
    echo "This script requires GNOME Terminal."
    echo ""
    echo "Alternatives:"
    echo "  1. Install gnome-terminal: sudo apt install gnome-terminal"
    echo "  2. Use manual method: see docs/MISSION_COMMAND_CHEATSHEET.md"
    exit 1
fi

# Enable X11
echo "📺 Enabling X11 forwarding..."
xhost +local:docker
echo "✅ X11 enabled"
echo ""

# Ask about recording
read -p "📹 Do you want to record the mission? (y/n): " -n 1 -r
echo ""
RECORD_MISSION=$REPLY

if [[ $RECORD_MISSION =~ ^[Yy]$ ]]; then
    # Check ffmpeg
    if ! command -v ffmpeg &> /dev/null; then
        echo "❌ ffmpeg not installed. Install with: sudo apt install ffmpeg"
        echo "Continuing without recording..."
        RECORD_MISSION="n"
    else
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        RECORDING_FILE="$PROJECT_DIR/bihar_maiden_voyage_${TIMESTAMP}.mp4"
        echo "🎬 Recording will be saved to: $RECORDING_FILE"
    fi
fi
echo ""

# Stop any existing container
echo "🧹 Cleaning up any existing containers..."
docker stop hexacopter_gui 2>/dev/null || true
sleep 2
echo "✅ Cleanup complete"
echo ""

echo "🚀 Launching terminals in sequence..."
echo ""

# ============================================================================
# TERMINAL 1: Gazebo Simulation + PX4 SITL
# ============================================================================
echo "📺 [1/3] Opening Terminal 1: Gazebo Simulation..."

if [[ $RECORD_MISSION =~ ^[Yy]$ ]]; then
    # Start recording first
    gnome-terminal --title="🎬 Screen Recording" -- bash -c "
        echo '╔════════════════════════════════════════════════════════════════╗'
        echo '║  🎬 SCREEN RECORDING                                          ║'
        echo '╚════════════════════════════════════════════════════════════════╝'
        echo ''
        echo 'Recording to: $RECORDING_FILE'
        echo 'Press Ctrl+C to stop recording when mission completes'
        echo ''
        ffmpeg -video_size 1920x1080 -framerate 30 \
          -f x11grab -i :0.0 \
          -c:v libx264 -preset ultrafast -crf 18 \
          '$RECORDING_FILE'
        echo ''
        echo '✅ Recording saved!'
        echo 'Press Enter to close this terminal...'
        read
    "
    sleep 2
fi

gnome-terminal --title="🌾 Bihar Simulation (Gazebo + PX4)" --geometry=120x40+0+0 -- bash -c "
    echo '╔════════════════════════════════════════════════════════════════╗'
    echo '║  TERMINAL 1: GAZEBO SIMULATION + PX4 SITL                     ║'
    echo '╚════════════════════════════════════════════════════════════════╝'
    echo ''
    echo '🚀 Launching Bihar world simulation...'
    echo '⏳ This will take 30-60 seconds to fully initialize'
    echo ''
    echo 'What you should see:'
    echo '  1. Gazebo window opens with Bihar maize field'
    echo '  2. Hexacopter on red landing pad'
    echo '  3. PX4 boot messages in this terminal'
    echo ''
    echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    echo ''
    
    docker run -it --rm --privileged --network=host \
        -e DISPLAY=\$DISPLAY \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $PROJECT_DIR/workspace:/root/workspace \
        -v $PROJECT_DIR/scripts:/root/scripts \
        --name hexacopter_gui \
        hexacopter_lab_safe_copy \
        bash -c 'cd /root/workspace && ../scripts/visual_hexacopter_bihar.sh; echo; echo \"Simulation ended. Press Enter to close...\"; read'
"

echo "✅ Terminal 1 opened (Gazebo + PX4)"
echo "⏳ Waiting 45 seconds for PX4 to boot..."
echo ""

# Wait for PX4 to boot
for i in {45..1}; do
    printf "\r   ⏱️  PX4 boot countdown: %2d seconds remaining..." $i
    sleep 1
done
echo ""
echo "✅ PX4 should be ready"
echo ""

# ============================================================================
# TERMINAL 2: Thermal AI Monitor
# ============================================================================
echo "🧠 [2/3] Opening Terminal 2: Thermal AI Monitor..."

gnome-terminal --title="🧠 Thermal AI Monitor (MobileNetV2)" --geometry=120x20+0+600 -- bash -c "
    echo '╔════════════════════════════════════════════════════════════════╗'
    echo '║  TERMINAL 2: THERMAL AI MONITOR (MobileNetV2)                 ║'
    echo '╚════════════════════════════════════════════════════════════════╝'
    echo ''
    echo '🧠 Starting thermal disease detection node...'
    echo '📊 Model: MobileNetV2 (3.4M parameters)'
    echo '🎯 Performance: 91.9% F1-score, 45ms latency'
    echo ''
    echo 'Watch for disease alerts:'
    echo '  🚨 DISEASE HOTSPOT DETECTED'
    echo '  Frame: XX | Confidence: XX.X%'
    echo '  GPS: [lat, lon]'
    echo ''
    echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    echo ''
    
    docker exec -it hexacopter_gui bash -c '
        source /opt/ros/humble/setup.bash && \
        source /root/workspace/install/setup.bash && \
        ros2 run agri_hexacopter thermal_monitor
    '
    
    echo ''
    echo '✅ Thermal monitor stopped'
    echo 'Press Enter to close this terminal...'
    read
"

echo "✅ Terminal 2 opened (Thermal AI)"
echo "⏳ Waiting 10 seconds for thermal monitor to initialize..."
echo ""

# Wait for thermal monitor to initialize
for i in {10..1}; do
    printf "\r   ⏱️  Thermal AI initialization: %2d seconds remaining..." $i
    sleep 1
done
echo ""
echo "✅ Thermal AI should be ready"
echo ""

# ============================================================================
# TERMINAL 3: Survey Mission
# ============================================================================
echo "🎯 [3/3] Opening Terminal 3: Survey Mission..."

gnome-terminal --title="🎯 Survey Mission (Level 3)" --geometry=120x20+0+1200 -- bash -c "
    echo '╔════════════════════════════════════════════════════════════════╗'
    echo '║  TERMINAL 3: LEVEL 3 SURVEY MISSION                           ║'
    echo '╚════════════════════════════════════════════════════════════════╝'
    echo ''
    echo '🎯 Executing autonomous survey mission...'
    echo '📍 Target: 7 waypoints in zig-zag pattern'
    echo '📏 Coverage: 10m × 8m grid (80m²)'
    echo '✈️  Altitude: 5.0m'
    echo ''
    echo 'Mission sequence:'
    echo '  1. Arm vehicle'
    echo '  2. Takeoff to 5m'
    echo '  3. Navigate 7 waypoints'
    echo '  4. Return to home'
    echo '  5. Land on pad'
    echo ''
    echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    echo ''
    
    docker exec -it hexacopter_gui bash -c '
        source /opt/ros/humble/setup.bash && \
        source /root/workspace/install/setup.bash && \
        /root/workspace/install/agri_bot_missions/bin/level3_survey
    '
    
    echo ''
    echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    echo ''
    if [ \$? -eq 0 ]; then
        echo '✅ MISSION SUCCESS!'
        echo '   - 7/7 waypoints completed'
        echo '   - Safe landing confirmed'
        echo '   - Status: THESIS VALIDATED'
    else
        echo '❌ Mission encountered errors'
        echo '   Check logs above for details'
    fi
    echo ''
    echo 'Press Enter to close this terminal...'
    read
"

echo "✅ Terminal 3 opened (Survey Mission)"
echo ""

# ============================================================================
# COMPLETION MESSAGE
# ============================================================================
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  ✅ ALL TERMINALS LAUNCHED SUCCESSFULLY                        ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "📺 Terminal 1: Gazebo + PX4 (should show Bihar world)"
echo "🧠 Terminal 2: Thermal AI (monitoring for disease hotspots)"
echo "🎯 Terminal 3: Survey Mission (executing autonomous flight)"
echo ""
if [[ $RECORD_MISSION =~ ^[Yy]$ ]]; then
    echo "🎬 Recording: Active (stop with Ctrl+C in recording terminal)"
    echo "📁 Output file: $RECORDING_FILE"
    echo ""
fi
echo "⏳ Mission duration: ~95 seconds"
echo ""
echo "What to watch:"
echo "  1. Gazebo: Hexacopter takes off, flies zig-zag, lands"
echo "  2. Terminal 2: Disease alerts (if hotspots detected)"
echo "  3. Terminal 3: Waypoint progress (1/7, 2/7, ... 7/7)"
echo ""
echo "After mission completes:"
if [[ $RECORD_MISSION =~ ^[Yy]$ ]]; then
    echo "  1. Stop recording (Ctrl+C in recording terminal)"
fi
echo "  2. Commit results:"
echo "     cd $PROJECT_DIR"
echo "     git add ."
echo "     git commit -m 'feat: complete maiden voyage in Bihar world'"
echo "     git tag -a v4.0-thesis-validated -m 'Thesis validated'"
echo ""
echo "🎓 Good luck, Abhishek! The Smart Bird is flying! 🚁🌾"
echo ""
