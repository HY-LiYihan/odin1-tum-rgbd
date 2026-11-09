#!/usr/bin/env bash
# Record an Odin1 sequence and convert it into a TUM RGB-D style dataset.
#
#   ./tools/collect_tum.sh data/seq01
#
# Press Ctrl-C to stop recording; conversion starts automatically.
# The raw rosbag is kept in <seq>/bag for re-processing with different options.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

SEQ="${1:-}"
if [ -z "$SEQ" ]; then
  echo "usage: $0 <output_sequence_dir>"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$ROOT/catkin_ws/install/setup.bash"

if ! ros2 topic list 2>/dev/null | grep -qx "/odin1/cloud_raw"; then
  echo "[error] /odin1/cloud_raw not found - start the driver first:"
  echo "        $ROOT/tools/odin_driver.sh"
  exit 1
fi

mkdir -p "$SEQ"
if [ -e "$SEQ/bag" ]; then
  echo "[error] $SEQ/bag already exists"
  exit 1
fi

# fail fast if the device never started streaming (e.g. "Open device failed")
if ! timeout 5 ros2 topic echo /odin1/odometry --once >/dev/null 2>&1; then
  echo "[error] no odometry data - check the driver log and restart it if it"
  echo "        reported 'Open device failed'"
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
python3 "$ROOT/tools/bag2tum.py" "$SEQ/bag" "$SEQ" "${@:2}"
