#!/bin/bash
# (Run as ./setup.sh)

# --- 0. Set the Root Directory (The "1-Click" Secret) ---
# This finds the folder where THIS script is sitting
ROOT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
cd "$ROOT_DIR"

# --- 1. System Permissions & Udev ---
echo "--- 1. Setting up Hardware Permissions ---"
if getent group dialout | grep -q "\b$USER\b"; then
    echo "✅ User $USER is already in the dialout group."
else
    sudo usermod -a -G dialout $USER
    echo "⚠️  NOTE: You must RESTART for GPS permissions to take effect!"
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
cmake .. && make -j$(nproc)
sudo make install

# --- 4. Build Driver ---
echo "--- 4. Building livox_ros_driver2 ---"
cd "$ROOT_DIR/src/livox_ros_driver2"
source /opt/ros/humble/setup.bash
./build.sh humble

# --- 5. Install GPS & Workspace Dependencies ---
echo "--- 5. Installing GPS/ROS Dependencies ---"
cd "$ROOT_DIR"
rosdep update
rosdep install --from-paths src --ignore-src -r -y


# --- 6. Final Workspace Build ---
echo "--- 6. Building All Packages (Fast-LIO, GPS, Configs) ---"
source /opt/ros/humble/setup.bash

# FIX: First, surgically build the problematic ublox packages without symlinks 
# to bypass the "Is a directory" conflict.
echo "Running specialized build for ublox packages..."
colcon build --packages-select ublox_msgs ublox_serialization ublox_gps

# Now build the rest of the workspace normally.
# This ensures Fast-LIO and your custom configs are linked correctly.
echo "Finishing full workspace build..."
colcon build --symlink-install --packages-select FAST_LIO my_configs

source install/setup.bash