#!/bin/bash
# run_maiden_voyage.sh - Complete Bihar Maiden Voyage with Visible Logs
# This script runs the mission in a way that keeps all terminals visible

set -e  # Exit on error

PROJECT_DIR="$HOME/thermal_hexacopter_project"
cd "$PROJECT_DIR"

echo "🚁 BIHAR MAIDEN VOYAGE - MANUAL EXECUTION WITH VISIBLE LOGS"
echo "============================================================"
echo ""
echo "This script will guide you through the mission step-by-step."
echo "Each step will be clearly labeled."
echo ""

# Step 1: Enable X11
echo "📺 STEP 1: Enabling X11 forwarding..."
xhost +local:docker
echo "✅ X11 enabled"
echo ""

# Step 2: Start recording (optional)
echo "📹 STEP 2: Screen recording"
read -p "Do you want to record the mission? (y/n): " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]
then
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    RECORDING_FILE="bihar_maiden_voyage_${TIMESTAMP}.mp4"
    echo "Starting recording to: $RECORDING_FILE"
    echo "Press Ctrl+C in the recording terminal to stop when mission completes"
    
    # Start recording in background
    ffmpeg -video_size 1920x1080 -framerate 30 \
      -f x11grab -i :0.0 \
      -c:v libx264 -preset ultrafast -crf 18 \
      "$RECORDING_FILE" > /dev/null 2>&1 &
    
    FFMPEG_PID=$!
    echo "✅ Recording started (PID: $FFMPEG_PID)"
    sleep 2
else
    echo "⏭️  Skipping recording"
    FFMPEG_PID=""
fi
echo ""

# Step 3: Launch simulation
echo "🚀 STEP 3: Launching Bihar world simulation..."
echo "This will open Gazebo in a new window. Wait for it to fully load."
echo ""

docker run -it --rm --privileged --network=host \
    -e DISPLAY=$DISPLAY \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PROJECT_DIR/workspace:/root/workspace" \
    -v "$PROJECT_DIR/scripts:/root/scripts" \
    --name hexacopter_gui \
    hexacopter_lab_safe_copy \
    bash -c "cd /root/workspace && ../scripts/visual_hexacopter_bihar.sh; echo ''; echo 'Simulation ended. Press Ctrl+D to exit.'; bash"

# Cleanup
echo ""
echo "🧹 Cleaning up..."

if [ ! -z "$FFMPEG_PID" ]; then
    echo "Stopping recording..."
    kill $FFMPEG_PID 2>/dev/null || true
    wait $FFMPEG_PID 2>/dev/null || true
    echo "✅ Recording saved to: $RECORDING_FILE"
fi

echo ""
echo "✅ Mission complete!"
echo ""
echo "Next steps:"
echo "1. Check Gazebo window - did you see the hexacopter?"
echo "2. If recording was enabled, play the video: vlc $RECORDING_FILE"
echo "3. Commit the results: git add . && git commit -m 'feat: complete maiden voyage'"
echo "4. Tag the version: git tag -a v4.0-thesis-validated -m 'Thesis validated'"
