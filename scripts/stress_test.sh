#!/bin/bash

# stress_test.sh
# Executes the "Stress Test" commands for physics verification.

# Detect Workspace Root
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "📍 Running Stress Test in: $WORKSPACE_DIR"

echo ""
echo "🔍 Pillar 1: The Physics Pillar (Center of Gravity)"
echo "Checking mass and inertia in model.sdf..."

MODEL_SDF="$WORKSPACE_DIR/models/agri_hexacopter_drone/model.sdf"

if [ -f "$MODEL_SDF" ]; then
    # Action: grep -C 5 "mass" ...
    grep -C 5 "mass" "$MODEL_SDF" | head -n 10
else
    echo "❌ Error: model.sdf not found at $MODEL_SDF"
fi