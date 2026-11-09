#!/usr/bin/env python3
"""Convert an Odin1 ROS2 bag into a TUM RGB-D like dataset.

Output layout (TUM RGB-D convention):
    rgb/<timestamp>.png        undistorted RGB  (pinhole, BGR->RGB PNG)
    depth/<timestamp>.png      16-bit depth PNG (value = depth[m] * depth_scale, 0 = invalid)
    rgb.txt                    "timestamp rgb/<timestamp>.png"
    depth.txt                  "timestamp depth/<timestamp>.png"
    groundtruth.txt            "timestamp tx ty tz qx qy qz qw"  (camera pose in odom frame)
    associated.txt             rgb/depth pairs matched by timestamp
    camera.txt                 fx fy cx cy d0..d4 (+ depth_scale notes)
    extrinsics.yaml            T_camera_lidar / T_imu_lidar / T_imu_camera
    calib.yaml                 copy of the device calibration used

Usage:
    python3 bag2tum.py <bag_dir> <out_dir> [options]

Notes:
- Odin1 has no dense depth sensor. Depth is obtained by projecting the LiDAR
  cloud (lidar frame) into the undistorted RGB camera frame, then filling holes
  with a small dilation. 0 = no measurement.
- The driver publishes device-internal time (seconds since boot), so every
  timestamp is shifted by one constant offset derived from the bag reception
  time, giving wall-clock-like timestamps without breaking inter-sensor sync.
"""

import argparse
import math
import os
import shutil
import sys

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def load_calib(path):
    with open(path, "r") as f:
        cfg = yaml.safe_load(f)
    cam = cfg["cam_0"]
    Tcl = np.array(cfg["Tcl_0"], dtype=np.float64).reshape(4, 4)
    # Fixed factory extrinsic: P_imu = T_il * P_lidar
    T_il = np.array([[1.0, 0.0, 0.0, -0.026630],
                     [0.0, 1.0, 0.0,  0.034470],
                     [0.0, 0.0, 1.0,  0.021740],
                     [0.0, 0.0, 0.0,  1.0]])
    T_ic = T_il @ np.linalg.inv(Tcl)  # P_imu = T_ic * P_cam
    return {
        "width": int(cam["image_width"]),
        "height": int(cam["image_height"]),
        "fx": float(cam["A11"]),
        "fy": float(cam["A22"]),
        "cx": float(cam["u0"]),
        "cy": float(cam["v0"]),
        "skew": float(cam.get("A12", 0.0)),
        "k": [float(cam.get("k%d" % i, 0.0)) for i in range(2, 8)],
        "T_camera_lidar": Tcl,
        "T_imu_lidar": T_il,
        "T_imu_camera": T_ic,
    }


def build_undistort_maps(calib):
    """Replicate PolynomialCamera (equidistant + k2..k7) -> ideal pinhole."""
    w, h = calib["width"], calib["height"]
    fx, fy, cx, cy, skew = calib["fx"], calib["fy"], calib["cx"], calib["cy"], calib["skew"]
    k2, k3, k4, k5, k6, k7 = calib["k"]

    u_out, v_out = np.meshgrid(np.arange(w, dtype=np.float64),
                               np.arange(h, dtype=np.float64))
    x = (u_out - cx) / fx
    y = (v_out - cy) / fy
    x = x - y * skew / fx          # remove skew, same as the driver

    r = np.sqrt(x * x + y * y)
    theta = np.arctan(r)
    thetad = (theta + k2 * theta ** 2 + k3 * theta ** 3 + k4 * theta ** 4 +
              k5 * theta ** 5 + k6 * theta ** 6 + k7 * theta ** 7)
    scaling = np.where(r > 1e-8, thetad / np.maximum(r, 1e-12), 1.0)
    xd = x * scaling
    yd = y * scaling

    map_x = (xd * fx + yd * skew + cx).astype(np.float32)
    map_y = (yd * fy + cy).astype(np.float32)
    return map_x, map_y


def cloud_to_xyz(msg):
    """sensor_msgs/PointCloud2 -> (N,3) float32 array."""
    names = {f.name: f for f in msg.fields}
    if not all(n in names for n in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    step = msg.point_step
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = len(buf) // step
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    xyz = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(("x", "y", "z")):
        off = names[name].offset
        xyz[:, i] = np.ndarray(shape=(n,), dtype="<f4", buffer=buf,
                               offset=off, strides=(step,))
    return xyz


def project_depth(points_lidar, calib, T, fill_iters, min_depth, max_depth):
    w, h = calib["width"], calib["height"]
    if points_lidar.shape[0] == 0:
        return np.zeros((h, w), dtype=np.float32)

    R = T[:3, :3]
    t = T[:3, 3]
    pc = points_lidar.astype(np.float64) @ R.T + t
    z = pc[:, 2]
    valid = (z > min_depth) & (z < max_depth)
    pc = pc[valid]
    z = z[valid]
    if z.size == 0:
        return np.zeros((h, w), dtype=np.float32)

    u = np.rint(calib["fx"] * pc[:, 0] / z + calib["cx"]).astype(np.int32)
    v = np.rint(calib["fy"] * pc[:, 1] / z + calib["cy"]).astype(np.int32)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    if not np.any(inside):
        return np.zeros((h, w), dtype=np.float32)

    idx = v[inside] * w + u[inside]
    zz = z[inside].astype(np.float32)
    depth = np.full(h * w, np.inf, dtype=np.float32)
    np.minimum.at(depth, idx, zz)      # nearest point wins (z-buffer)
    depth[np.isinf(depth)] = 0.0
    depth = depth.reshape(h, w)

    for _ in range(fill_iters):
        padded = cv2.copyMakeBorder(depth, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
        filled = np.zeros_like(depth)
        for dy in (0, 1, 2):
            for dx in (0, 1, 2):
                if dx == 1 and dy == 1:
                    continue
                nb = padded[dy:dy + h, dx:dx + w]
                filled = np.where(filled > 0, filled, nb)
        depth = np.where(depth > 0, depth, filled)
    return depth


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def mat_to_tum(T):
    t = T[:3, 3]
    R = T[:3, :3]
    qw = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if qw < 1e-8:
        qx = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
        qy = (R[0, 1] + R[1, 0]) / (4 * qx)
        qz = (R[0, 2] + R[2, 0]) / (4 * qx)
        qw = (R[1, 2] - R[2, 1]) / (4 * qx)
    else:
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return [float(t[0]), float(t[1]), float(t[2]), qx, qy, qz, qw]


def associate(rgb_list, depth_list, max_dt):
    """Greedy timestamp association (TUM style)."""
    out = []
    used = set()
    for ts, name in rgb_list:
        best = None
        best_dt = max_dt
        for i, (dts, dname) in enumerate(depth_list):
            if i in used:
                continue
            dt = abs(dts - ts)
            if dt < best_dt:
                best_dt = dt
                best = i
        if best is not None:
            used.add(best)
            out.append((ts, name, depth_list[best][0], depth_list[best][1]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="input bag directory (or .db3/mcap file)")
    ap.add_argument("out", help="output dataset directory")
    ap.add_argument("--calib", default=os.path.expanduser("~/.ros/odin_ros_driver/calib.yaml"))
    ap.add_argument("--max-dt", type=float, default=0.02, help="max rgb/depth time diff (s)")
    ap.add_argument("--depth-scale", type=float, default=1000.0,
                    help="depth PNG factor, depth[m] = value / scale (default 1000 = mm)")
    ap.add_argument("--min-depth", type=float, default=0.2)
    ap.add_argument("--max-depth", type=float, default=65.0)
    ap.add_argument("--fill-iters", type=int, default=1,
                    help="hole-filling iterations (3x3 dilation of sparse depth)")
    ap.add_argument("--no-undistort", action="store_true",
                    help="keep raw fisheye RGB (then camera.txt intrinsics are invalid)")
    args = ap.parse_args()

    if not os.path.isfile(args.calib):
        sys.exit("calib.yaml not found: %s (start the driver once to fetch it)" % args.calib)
    calib = load_calib(args.calib)

    for sub in ("rgb", "depth"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    map_x, map_y = (None, None) if args.no_undistort else build_undistort_maps(calib)

    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {name: get_message(t) for name, t in topics.items()}

    rgb_topic = next((t for t in ("/odin1/image", "/odin1/image/compressed") if t in topics), None)
    if rgb_topic is None:
        sys.exit("no RGB topic found in bag (expected /odin1/image or /odin1/image/compressed)")
    if "/odin1/cloud_raw" not in topics:
        sys.exit("no /odin1/cloud_raw topic found in bag")
    if "/odin1/odometry" not in topics:
        sys.exit("no /odin1/odometry topic found in bag")

    rgb_entries, depth_entries, gt_entries = [], [], []
    stamps_raw = []      # (device_stamp, bag_recv_stamp) for offset estimation
    n_rgb = n_depth = n_gt = 0

    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        if topic not in (rgb_topic, "/odin1/cloud_raw", "/odin1/odometry"):
            continue
        msg = deserialize_message(data, msg_cls[topic])
        ts = stamp_to_sec(msg.header.stamp)
        recv = recv_ns * 1e-9

        if topic == rgb_topic:
            if topic.endswith("compressed"):
                img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            else:
                buf = np.frombuffer(bytes(msg.data), np.uint8)
                if msg.encoding == "bgr8":
                    img = buf.reshape(msg.height, msg.width, 3).copy()
                elif msg.encoding == "rgb8":
                    img = cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
                elif msg.encoding == "mono8":
                    img = cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
                else:
                    continue
            if img is None:
                continue
            if map_x is not None and (img.shape[1], img.shape[0]) == (calib["width"], calib["height"]):
                img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
            name = "rgb/%.6f.png" % ts
            cv2.imwrite(os.path.join(args.out, name), img)
            rgb_entries.append((ts, name))
            stamps_raw.append((ts, recv))
            n_rgb += 1

        elif topic == "/odin1/cloud_raw":
            xyz = cloud_to_xyz(msg)
            depth = project_depth(xyz, calib, calib["T_camera_lidar"],
                                  args.fill_iters, args.min_depth, args.max_depth)
            depth_u16 = np.clip(np.rint(depth * args.depth_scale), 0, 65535).astype(np.uint16)
            name = "depth/%.6f.png" % ts
            cv2.imwrite(os.path.join(args.out, name), depth_u16)
            depth_entries.append((ts, name))
            stamps_raw.append((ts, recv))
            n_depth += 1

        else:  # odometry
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            T_w_imu = np.eye(4)
            x, y, z, w = q.x, q.y, q.z, q.w
            T_w_imu[:3, :3] = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            T_w_imu[:3, 3] = [p.x, p.y, p.z]
            T_w_cam = T_w_imu @ calib["T_imu_camera"]
            gt_entries.append((ts, mat_to_tum(T_w_cam)))
            stamps_raw.append((ts, recv))
            n_gt += 1

    if not stamps_raw:
        sys.exit("bag contains no usable messages")

    # device time -> host time offset (constant, median over all frames)
    offset = float(np.median([recv - ts for ts, recv in stamps_raw]))

    def write_index(path, entries, header):
        with open(os.path.join(args.out, path), "w") as f:
            f.write(header)
            for ts, name in entries:
                f.write("%.6f %s\n" % (ts + offset, name))

    write_index("rgb.txt", rgb_entries, "# timestamp filename\n")
    write_index("depth.txt", depth_entries, "# timestamp filename\n")

    with open(os.path.join(args.out, "groundtruth.txt"), "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw (camera pose in odom frame)\n")
        for ts, v in gt_entries:
            f.write("%.6f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" %
                    (ts + offset, v[0], v[1], v[2], v[3], v[4], v[5], v[6]))

    pairs = associate(rgb_entries, depth_entries, args.max_dt)
    with open(os.path.join(args.out, "associated.txt"), "w") as f:
        f.write("# rgb_timestamp rgb_file depth_timestamp depth_file\n")
        for rts, rname, dts, dname in pairs:
            f.write("%.6f %s %.6f %s\n" %
                    (rts + offset, rname, dts + offset, dname))

    c = calib
    with open(os.path.join(args.out, "camera.txt"), "w") as f:
        f.write("# Odin1 RGB camera, undistorted pinhole (OpenCV model)\n")
        f.write("# width height\n%d %d\n" % (c["width"], c["height"]))
        f.write("# fx fy cx cy d0 d1 d2 d3 d4\n")
        f.write("%.8f %.8f %.8f %.8f 0 0 0 0 0\n" % (c["fx"], c["fy"], c["cx"], c["cy"]))
        f.write("# depth_scale: %g   (depth[m] = png_value / %g, 0 = no measurement)\n"
                % (args.depth_scale, args.depth_scale))
        f.write("# depth source: /odin1/cloud_raw projected into camera, fill_iters=%d\n"
                % args.fill_iters)
        f.write("# rgb undistorted: %s\n" % ("no" if args.no_undistort else "yes"))

    with open(os.path.join(args.out, "extrinsics.yaml"), "w") as f:
        yaml.safe_dump({
            "T_camera_lidar": c["T_camera_lidar"].tolist(),
            "T_imu_lidar": c["T_imu_lidar"].tolist(),
            "T_imu_camera": c["T_imu_camera"].tolist(),
            "note": "P_cam = T_camera_lidar * P_lidar ; P_imu = T_imu_lidar * P_lidar",
            "groundtruth_frame": "camera optical frame (x right, y down, z forward)",
            "time_offset_device_to_host": offset,
        }, f, sort_keys=False)

    shutil.copyfile(args.calib, os.path.join(args.out, "calib.yaml"))

    print("dataset: %s" % os.path.abspath(args.out))
    print("  rgb frames     : %d" % n_rgb)
    print("  depth frames   : %d" % n_depth)
    print("  gt poses       : %d" % n_gt)
    print("  associated     : %d (max_dt=%.3fs)" % (len(pairs), args.max_dt))
    print("  time offset    : %.6f s (device -> host)" % offset)


if __name__ == "__main__":
    main()
