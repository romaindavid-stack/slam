#!/bin/bash
# (Run as ./setup.sh)

# --- 1. Submodule Sync ---
echo "--- Syncing Submodules ---"
git submodule update --init --recursive

# --- 2. Build & Install SDK2 (System-wide) ---
echo "--- Building Livox-SDK2 ---"
cd src/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd ../../../  # Back to ~/slam

# --- 3. Build Driver ---

echo "--- 3. Building livox_ros_driver2 ---"
cd src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
cd ../../

echo "--- 4. Building Fast-LIO & Configs ---"
source install/setup.bash
colcon build --symlink-install --packages-select FAST_LIO my_configs
