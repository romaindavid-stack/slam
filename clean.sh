#!/bin/bash


echo "--- Cleaning Workspace ---"

# 1. Main ROS 2 folders
rm -rf build/ install/ log/

# 2. SDK internal build
rm -rf src/Livox-SDK2/build/

# 3. Driver internal artifacts (from the official build.sh)
rm -rf src/livox_ros_driver2/build/
rm -rf src/livox_ros_driver2/install/

echo "✅ All build artifacts removed. Workspace is clean."