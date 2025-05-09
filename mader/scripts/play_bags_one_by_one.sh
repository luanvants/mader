#!/bin/bash

# Directory containing bag files (edit if needed)
sleep 5
BAG_DIR="/home/nvhung/Desktop/bags"

# Check if directory exists
if [ ! -d "$BAG_DIR" ]; then
  echo "Directory $BAG_DIR does not exist!"
  exit 1
fi

# Biến đếm
COUNT=1

# Loop over each .bag file in the directory
for BAG_FILE in "$BAG_DIR"/*.bag; do
  echo "=========================="
  echo "Playing: $BAG_FILE"
  echo "=========================="

  # Play the bag file
  read -p "Press Enter to continue to next bag..." # Optional: wait for user to press Enter before continuing
  rosbag play "$BAG_FILE" /SQ02s/semantic_mader/sem_actual_traj:=actual_traj_$COUNT /SQ01s/mader/actual_traj:=actual_traj_$COUNT

  # Tăng biến đếm
  COUNT=$((COUNT + 1))

done
