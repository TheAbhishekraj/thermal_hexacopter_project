#!/bin/bash

# diagnostic_audit.sh
# Runs the "Auditor" checks for the Thermal Hexacopter project.

# Detect Workspace Root
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "📍 Running Diagnostic Audit in: $WORKSPACE_DIR"
echo "---------------------------------------------------"

# Q1: The "Identity" Check
echo "🔍 Q1: Checking model.config identity..."
MODEL_CONFIG="$WORKSPACE_DIR/models/agri_hexacopter_drone/model.config"
if [ -f "$MODEL_CONFIG" ]; then
    echo "File found."
    echo "Content of <name>:"
    grep "<name>" "$MODEL_CONFIG"
else
    echo "❌ Error: model.config not found at $MODEL_CONFIG"
fi
echo ""

# Q2: The "Ghost Extension" Check
echo "🔍 Q2: Checking for .sdf.sdf ghost extensions..."
WORLDS_DIR="$WORKSPACE_DIR/src/agri_hexacopter/worlds"
if [ -d "$WORLDS_DIR" ]; then
    echo "Files in worlds directory:"
    ls "$WORLDS_DIR"
    if ls "$WORLDS_DIR"/*.sdf.sdf 1> /dev/null 2>&1; then
        echo "⚠️  WARNING: Found .sdf.sdf files!"
    fi
else
    echo "❌ Error: Worlds directory not found at $WORLDS_DIR"
fi
echo ""

# Q3: The "Joint" Check
echo "🔍 Q3: Checking model.sdf for parenting errors..."
MODEL_SDF="$WORKSPACE_DIR/models/agri_hexacopter_drone/model.sdf"
if [ -f "$MODEL_SDF" ]; then
    if grep -q "x500_heavy::base_link" "$MODEL_SDF"; then
        echo "❌ FAIL: Found 'x500_heavy::base_link'. Joints are parented to the wrong model!"
        grep -n "x500_heavy::base_link" "$MODEL_SDF"
    else
        echo "✅ PASS: No 'x500_heavy::base_link' found."
    fi
else
    echo "❌ Error: model.sdf not found at $MODEL_SDF"
fi
echo ""

# Q4: The "Dependency" Check
echo "🔍 Q4: Checking package.xml for px4_msgs..."
PACKAGE_XML="$WORKSPACE_DIR/src/agri_hexacopter/package.xml"
if [ -f "$PACKAGE_XML" ]; then
    if grep -q "px4_msgs" "$PACKAGE_XML"; then
        echo "✅ PASS: Found 'px4_msgs' dependency."
    else
        echo "❌ FAIL: 'px4_msgs' dependency missing in package.xml."
    fi
else
    echo "❌ Error: package.xml not found at $PACKAGE_XML"
fi
echo "---------------------------------------------------"