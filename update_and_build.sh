#!/bin/bash

set -e  # Exit on error
cd "$(dirname "$0")"  # Ensure script is run from repo root

echo "🔄 Fetching latest Git changes..."
git fetch
git pull

echo "🔧 Building ROS 2 workspace..."
cd ros_ws
colcon build

echo "✅ Sourcing workspace environment..."
source install/setup.bash

echo "✅ Done: workspace updated and ready."

