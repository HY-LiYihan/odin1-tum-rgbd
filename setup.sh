#!/usr/bin/env bash
# One-shot setup: dependencies, udev rule, driver source, config, build.
set -eo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
DRIVER_DIR="$ROOT/catkin_ws/src/odin_ros_driver"
DRIVER_URL="https://github.com/manifoldsdk/odin_ros_driver.git"

echo "[1/5] installing build dependencies"
sudo apt-get update -qq
sudo apt-get install -y build-essential cmake git \
  libusb-1.0-0-dev libyaml-cpp-dev libeigen3-dev libssl-dev \
  libopencv-dev libpcl-dev

echo "[2/5] writing udev rule"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-odin-usb.rules >/dev/null
sudo udevadm control --reload
sudo udevadm trigger

echo "[3/5] checking odin_ros_driver submodule"
mkdir -p "$ROOT/catkin_ws/src"
if [ -d "$DRIVER_DIR/.git" ] || [ -n "$(ls -A "$DRIVER_DIR" 2>/dev/null)" ]; then
  echo "  already present ($(git -C "$DRIVER_DIR" describe --tags 2>/dev/null || echo unknown))"
elif git clone --depth 1 "$DRIVER_URL" "$DRIVER_DIR"; then
  echo "  cloned via git"
else
  echo "  git clone failed, falling back to tarball"
  tmp="$(mktemp -d)"
  curl -sL --max-time 300 -o "$tmp/driver.tar.gz" \
    "https://codeload.github.com/manifoldsdk/odin_ros_driver/tar.gz/refs/heads/main"
  tar -xzf "$tmp/driver.tar.gz" -C "$tmp"
  mv "$tmp/odin_ros_driver-main" "$DRIVER_DIR"
  rm -rf "$tmp"
fi

echo "[4/5] building ROS2 workspace"
chmod +x "$DRIVER_DIR"/script/*.sh
# Odin's generated ROS2 interfaces are memory-hungry; with many cores plus other
# workloads (e.g. local LLM servers) an unbounded -j can OOM the compiler.
export MAKEFLAGS="-j${BUILD_JOBS:-4}"
"$DRIVER_DIR/script/build_ros2.sh"
# the build script swaps in package_ros2.xml; undo it so the submodule stays clean
git -C "$DRIVER_DIR" checkout -- package.xml 2>/dev/null || true

# Enable undistorted RGB + the depth demo. Both nodes read
# <package source dir>/config/control_command.yaml via COLCON_PREFIX_PATH, so the
# patch has to go into the submodule (it will show up as one modified file).
CONF="$DRIVER_DIR/config/control_command.yaml"
sed -i 's/^  sendrgbundistort: 0/  sendrgbundistort: 1/' "$CONF"
sed -i 's/^  senddepth: 0/  senddepth: 1/' "$CONF"
grep -E "^  (sendrgbundistort|senddepth):" "$CONF"

echo "[5/5] done"

echo
echo "done. next:"
echo "  tools/odin_driver.sh                       # start the driver"
echo "  tools/collect_tum.sh data/seq01            # record + convert"
