#!/bin/bash
# Setup script for X5 ROS Driver
# This installs udev rules so you don't need sudo to access the camera

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RULES_FILE="$SCRIPT_DIR/99-insta360.rules"

if [ ! -f "$RULES_FILE" ]; then
    echo "Error: 99-insta360.rules not found in $SCRIPT_DIR"
    exit 1
fi

echo "Installing udev rules for Insta360 cameras..."
sudo cp "$RULES_FILE" /etc/udev/rules.d/

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Adding user to plugdev group..."
sudo usermod -a -G plugdev $USER

echo ""
echo "========================================="
echo "Setup complete!"
echo "========================================="
echo ""
echo "Please:"
echo "1. Reconnect your Insta360 camera"
echo "2. Log out and back in (for group changes)"
echo ""
echo "Then you can run the driver without sudo:"
echo "  roslaunch x5_ros_driver x5_driver.launch"
echo ""
