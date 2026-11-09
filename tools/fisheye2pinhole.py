#!/usr/bin/env python3
"""Convert Odin1 fisheye (polynomial equidistant) images to a pinhole camera.

Two input modes:
  1) image / directory of images (raw fisheye jpg/png):
        python3 fisheye2pinhole.py raw.jpg -o out/
        python3 fisheye2pinhole.py raw_dir/ -o out/
  2) a recorded bag (uses /odin1/image/compressed or /odin1/image):
        python3 fisheye2pinhole.py --bag seq01/bag -o out/

The output uses an ideal pinhole model with zero distortion:
    u = fx * X/Z + cx,  v = fy * Y/Z + cy
Default output intrinsics come from calib.yaml (same as the driver, ~95 deg HFOV).
Use --fov-scale < 1.0 to shrink the FOV (less edge stretching), or set
--fx/--fy/--width/--height explicitly.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np
import yaml


def load_calib(path):
    with open(path, "r") as f:
        cam = yaml.safe_load(f)["cam_0"]
    return {
        "width": int(cam["image_width"]),
        "height": int(cam["image_height"]),
        "fx": float(cam["A11"]),
        "fy": float(cam["A22"]),
        "cx": float(cam["u0"]),
        "cy": float(cam["v0"]),
        "skew": float(cam.get("A12", 0.0)),
        "k": [float(cam.get("k%d" % i, 0.0)) for i in range(2, 8)],
    }


def build_map(calib, fx, fy, cx, cy, width, height):
    """output pinhole pixel -> source fisheye pixel (cv2.remap coordinates)."""
    k2, k3, k4, k5, k6, k7 = calib["k"]
    u, v = np.meshgrid(np.arange(width, dtype=np.float64),
                       np.arange(height, dtype=np.float64))
    x = (u - cx) / fx
    y = (v - cy) / fy
    x = x - y * calib["skew"] / fx     # cancel the source affine skew

    r = np.sqrt(x * x + y * y)
    theta = np.arctan(r)
    thetad = (theta + k2 * theta ** 2 + k3 * theta ** 3 + k4 * theta ** 4 +
              k5 * theta ** 5 + k6 * theta ** 6 + k7 * theta ** 7)
    scaling = np.where(r > 1e-8, thetad / np.maximum(r, 1e-12), 1.0)

    map_x = (x * scaling * calib["fx"] + calib["cx"]).astype(np.float32)
    map_y = (y * scaling * calib["fy"] + calib["cy"]).astype(np.float32)
    return map_x, map_y


def convert(img, map_x, map_y):
    return cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", help="fisheye image file or directory")
    ap.add_argument("--bag", help="rosbag2 directory to read raw images from")
    ap.add_argument("-o", "--out", default="pinhole", help="output directory")
    ap.add_argument("--calib", default=os.path.expanduser("~/.ros/odin_ros_driver/calib.yaml"))
    ap.add_argument("--fx", type=float, help="output fx (default: A11)")
    ap.add_argument("--fy", type=float, help="output fy (default: A22)")
    ap.add_argument("--cx", type=float, help="output cx (default: u0)")
    ap.add_argument("--cy", type=float, help="output cy (default: v0)")
    ap.add_argument("--width", type=int, help="output width (default: source width)")
    ap.add_argument("--height", type=int, help="output height (default: source height)")
    ap.add_argument("--fov-scale", type=float, default=1.0,
                    help="multiply fx/fy by this (0.8 = narrower FOV, 1.2 = wider)")
    args = ap.parse_args()

    if not os.path.isfile(args.calib):
        sys.exit("calib.yaml not found: %s" % args.calib)
    calib = load_calib(args.calib)

    w = args.width or calib["width"]
    h = args.height or calib["height"]
    fx = (args.fx if args.fx else calib["fx"]) * args.fov_scale
    fy = (args.fy if args.fy else calib["fy"]) * args.fov_scale
    cx = args.cx if args.cx is not None else calib["cx"] * w / calib["width"]
    cy = args.cy if args.cy is not None else calib["cy"] * h / calib["height"]
    map_x, map_y = build_map(calib, fx, fy, cx, cy, w, h)
    os.makedirs(args.out, exist_ok=True)
    print("pinhole model: %dx%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f" % (w, h, fx, fy, cx, cy))

    n = 0
    if args.bag:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        import rosbag2_py

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
        topic = next((t for t in ("/odin1/image/compressed", "/odin1/image") if t in topics), None)
        if topic is None:
            sys.exit("no /odin1/image[.compressed] topic in bag")
        cls = get_message(topics[topic])
        while reader.has_next():
            name, data, _ = reader.read_next()
            if name != topic:
                continue
            msg = deserialize_message(data, cls)
            if name.endswith("compressed"):
                img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            else:
                buf = np.frombuffer(bytes(msg.data), np.uint8)
                img = buf.reshape(msg.height, msg.width, 3).copy()
            if img is None:
                continue
            if img.shape[1] != calib["width"] or img.shape[0] != calib["height"]:
                print("[warn] unexpected source size %dx%d, skipped" %
                      (img.shape[1], img.shape[0]))
                continue
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            out = convert(img, map_x, map_y)
            cv2.imwrite(os.path.join(args.out, "%.6f.png" % ts), out)
            n += 1
    else:
        if not args.input:
            sys.exit("give an input image/directory or --bag")
        files = ([args.input] if os.path.isfile(args.input)
                 else sorted(sum((glob.glob(os.path.join(args.input, e))
                                  for e in ("*.jpg", "*.jpeg", "*.png", "*.bmp")), [])))
        if not files:
            sys.exit("no images found in %s" % args.input)
        for f in files:
            img = cv2.imread(f)
            if img is None:
                print("[warn] cannot read %s" % f)
                continue
            out = convert(img, map_x, map_y)
            dst = os.path.join(args.out, os.path.splitext(os.path.basename(f))[0] + ".png")
            cv2.imwrite(dst, out)
            n += 1

    print("converted %d image(s) -> %s" % (n, os.path.abspath(args.out)))


if __name__ == "__main__":
    main()
