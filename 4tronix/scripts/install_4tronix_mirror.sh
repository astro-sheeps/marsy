#!/bin/bash

# M.A.R.S. Rover software installer
# Marsy / AstroSheeps mirror version
#
# New Marsy repository structure:
#   4tronix/original/             - original 4tronix rover files
#   external/4tronix-rover-simulator/ - simulator code
#   marsy_core/, marsy_backends/, missions/ - Marsy project code
#
# This installer downloads only the original 4tronix rover files
# needed on the Raspberry Pi / real rover.
#
# Source mirror:
# https://github.com/astro-sheeps/marsy/tree/main/4tronix/original

set -e

BASE_URL="https://raw.githubusercontent.com/astro-sheeps/marsy/main/4tronix/original"
TARGET_DIR="$HOME/marsrover"

FILES=(
  "rover.py"
  "ledTest.py"
  "motorTest.py"
  "pca9685.py"
  "servoTest.py"
  "sonarTest.py"
  "keypad.py"
  "driveRover.py"
  "calibrateServos.py"
)

echo "Creating marsrover folder"
mkdir -p "$TARGET_DIR"
cd "$TARGET_DIR"

echo "Copying M.A.R.S. Rover files from AstroSheeps mirror"
echo "Source: $BASE_URL"
echo "Target: $TARGET_DIR"

for file in "${FILES[@]}"; do
  echo "  - $file"
  wget -q "$BASE_URL/$file" -O "$file"
done

echo "M.A.R.S. Rover files copied"
