#!/bin/bash
# (Run as ./setup.sh)

# Exit immediately if a command FAILS (returns non-zero)
set -e

# --- 0. Set the Root Directory ---
ROOT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
cd "$ROOT_DIR"

# --- 1. System Permissions & Udev ---
echo "--- 1. Setting up Hardware Permissions ---"
if getent group dialout | grep -q "\b$USER\b"; then
    echo "✅ User $USER is already in the dialout group."
else
    sudo usermod -a -G dialout $USER
    echo "⚠️  NOTE: You must RESTART for hardware permissions to take effect!"
fi

UDEV_FILE="/etc/udev/rules.d/50-ardusimple.rules"
if [ ! -f "$UDEV_FILE" ]; then
    echo "Creating Udev rule for ArduSimple GPS..."
    echo 'KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK="tty_Ardusimple", GROUP="dialout", MODE="0666"' | sudo tee $UDEV_FILE
    sudo udevadm control --reload-rules && sudo udevadm trigger
fi

# --- 2. Submodule Sync ---
echo "--- 2. Syncing Submodules (SLAM + GPS) ---"
git submodule update --init --recursive

# --- 3. Build & Install SDK2 ---
echo "--- 3. Building Livox-SDK2 ---"
cd "$ROOT_DIR/src/Livox-SDK2"
mkdir -p build && cd build
cmake .. 1> /dev/null
make -j$(nproc) 1> /dev/null
sudo make install 1> /dev/null
cd "$ROOT_DIR"

# --- 4. Build Driver ---
echo "--- 4. Building livox_ros_driver2 ---"
cd "$ROOT_DIR/src/livox_ros_driver2"
source /opt/ros/humble/setup.bash
./build.sh humble 1> /dev/null
cd "$ROOT_DIR"

# --- 5. Install Dependencies (Python & ROS) ---
echo "--- 5. Installing Dependencies (Pandas & ROS) ---"
# Install Pandas for Keithley DMM analysis
pip install pandas --quiet

# TRY/EXCEPT for rosdep: 
# We run these with '|| true' so that even if the system database has 
# local errors (like the missing debian.yaml), the script continues.
echo "Updating rosdep database (non-fatal)..."
rosdep update 1> /dev/null 2>&1 || echo "⚠️  Warning: rosdep update had some issues, continuing anyway."

echo "Installing ROS dependencies..."
rosdep install --from-paths src --ignore-src -r -y --skip-keys="ament_python" 1> /dev/null 2>&1 || echo "⚠️  Warning: Some rosdeps failed to resolve, but we will try building anyway."

# --- 6. Final Workspace Build ---
echo "--- 6. Building All Packages (SLAM, GPS, DMM) ---"
source /opt/ros/humble/setup.bash

# Stage 1: Problematic ublox packages (No-Symlink)
# These must be built as real directories to avoid symlink conflicts
echo "Stage 1: Building ublox packages (Stage 1/2)..."
colcon build --packages-select ublox_msgs ublox_serialization ublox_gps --event-handlers console_cohesion+

# Stage 2: Everything else (With Symlink)
# Including the new keithley_dmm package
echo "Stage 2: Building SLAM, Driver, DMM, and Configs (Stage 2/2)..."
colcon build --symlink-install \
    --packages-select FAST_LIO my_configs ntrip_client keithley_dmm \
    --event-handlers console_cohesion+

echo ""
echo "********************************************************"
echo "🎉 SUCCESS: All packages (including Keithley DMM) built!"
echo "********************************************************"
echo "To start the system, run:"

echo "if the project is not sourced:"
echo "source install/setup.bash"
echo "then"
echo "ros2 launch my_configs fast_lio_deploy.launch.py"
echo "********************************************************"

source install/setup.bash 