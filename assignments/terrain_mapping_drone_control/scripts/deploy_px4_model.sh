#!/bin/bash

# Exit on any error
set -e

# Check if PX4-Autopilot exists in home directory
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    echo "Error: PX4-Autopilot directory not found in home directory"
    echo "Please make sure you have PX4-Autopilot cloned in your home directory"
    exit 1
fi

# Define paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
PX4_DIR="$HOME/PX4-Autopilot"

# Check if our model files exist
if [ ! -d "$PACKAGE_DIR/models/px4_models" ]; then
    echo "Error: PX4 model files not found in package"
    echo "Please run setup_px4_model.sh first if you're developing"
    exit 1
fi


# Copy files to PX4-Autopilot
echo "Deploying model files to PX4-Autopilot..."

# Copy airframe files
echo "Copying airframe files..."
cp "$PACKAGE_DIR/models/px4_models/airframes/4021_gz_x500_depth_mono" \
   "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"
cp "$PACKAGE_DIR/models/px4_models/airframes/4022_gz_x500_oaklite" \
   "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"

# Copy CMakeLists.txt if it exists
if [ -f "$PACKAGE_DIR/models/px4_models/airframes/CMakeLists.txt" ]; then
    cp "$PACKAGE_DIR/models/px4_models/airframes/CMakeLists.txt" \
       "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"
fi

# Copy Gazebo models
echo "Copying Gazebo models..."

# Copy x500_depth_mono model
if [ -d "$PACKAGE_DIR/models/px4_models/gz_models/x500_depth_mono" ]; then
    cp -r "$PACKAGE_DIR/models/px4_models/gz_models/x500_depth_mono" \
          "$PX4_DIR/Tools/simulation/gz/models/"
else
    echo "Warning: x500_depth_mono model not found in package"
fi

# Copy x500_oaklite model
if [ -d "$PACKAGE_DIR/models/px4_models/gz_models/x500_oaklite" ]; then
    cp -r "$PACKAGE_DIR/models/px4_models/gz_models/x500_oaklite" \
          "$PX4_DIR/Tools/simulation/gz/models/"
else
    echo "Warning: x500_oaklite model not found in package"
fi

echo "Deployment complete! Model files have been copied to PX4-Autopilot"
echo "Please rebuild PX4-Autopilot for changes to take effect" 