#!/usr/bin/env bash
# Record an Odin1 sequence and convert it into a TUM RGB-D style dataset.
#
#   ./collect_tum.sh /home/liyh/odin1/data/seq01
#
# Press Ctrl-C to stop recording; conversion starts automatically.
# The raw rosbag is kept in <seq>/bag for re-processing with different options.
set -eo pipefail

SEQ="${1:-}"
if [ -z "$SEQ" ]; then
  echo "usage: $0 <output_sequence_dir>"
  exit 1
fi

source /opt/ros/humble/setup.bash
source /home/liyh/odin1/catkin_ws/install/setup.bash

if ! ros2 topic list 2>/dev/null | grep -qx "/odin1/cloud_raw"; then
  echo "[error] /odin1/cloud_raw not found - start the driver first:"
  echo "        /home/liyh/odin1/tools/odin_driver.sh"
  exit 1
fi

mkdir -p "$SEQ"
if [ -e "$SEQ/bag" ]; then
  echo "[error] $SEQ/bag already exists"
  exit 1
fi

echo "[info] recording to $SEQ/bag  (Ctrl-C to stop)"
ros2 bag record -o "$SEQ/bag" \
  /odin1/image/compressed \
  /odin1/cloud_raw \
  /odin1/odometry \
  /odin1/imu \
  /tf /tf_static &
BAG_PID=$!
trap 'kill -INT $BAG_PID 2>/dev/null || true' INT TERM
wait $BAG_PID || true

echo "[info] recording finished, converting to TUM format"
python3 /home/liyh/odin1/tools/bag2tum.py "$SEQ/bag" "$SEQ" "${@:2}"
