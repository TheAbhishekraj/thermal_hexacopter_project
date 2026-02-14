#!/bin/bash

# final_check.sh
# Executes the "Check of Truth" commands for final verification.

# Detect Workspace Root
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "📍 Running Final Check in: $WORKSPACE_DIR"

echo ""
echo "🔍 1. Model Identity (<name> tag):"
# Check: grep "<name>" /root/workspace/models/agri_hexacopter_drone/model.config
grep "<name>" "$WORKSPACE_DIR/models/agri_hexacopter_drone/model.config"

echo ""
echo "🔍 2. Ghost Extensions (World files):"
# Check: ls /root/workspace/src/agri_hexacopter/worlds/
ls "$WORKSPACE_DIR/src/agri_hexacopter/worlds/"

echo ""
echo "🔍 3. Model Skeleton (base_link check):"
# Check: grep "base_link" /root/workspace/models/agri_hexacopter_drone/model.sdf | head -n 5
grep "base_link" "$WORKSPACE_DIR/models/agri_hexacopter_drone/model.sdf" | head -n 5