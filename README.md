# odin1-tum-rgbd

在 Ubuntu 22.04 + ROS 2 Humble 下把 **ManifoldTech Odin1**（LiDAR + RGB + IMU 一体化设备）采集的数据转换成 **TUM RGB-D 格式**（RGB + depth + 相机内外参 + 位姿真值）的工具集。

官方驱动：[manifoldsdk/odin_ros_driver](https://github.com/manifoldsdk/odin_ros_driver)（Apache-2.0，本仓库不包含其源码，由 `setup.sh` 自动拉取）。

## 目录结构

```
.
├── setup.sh                     # 依赖安装、udev 规则、拉取驱动、改配置、编译
├── tools/
│   ├── odin_driver.sh           # 启动驱动（RGB / LiDAR / IMU / 里程计 / 深度演示）
│   ├── collect_tum.sh           # 录 bag，Ctrl-C 后自动转 TUM 数据集
│   ├── bag2tum.py               # bag -> TUM 格式转换（去畸变、点云投深度、位姿）
│   └── fisheye2pinhole.py       # 鱼眼图 -> 针孔图（可从 bag 或图片目录批量转）
├── catkin_ws/                      # ROS2 工作空间（build/install/log 被忽略）
│   └── src/odin_ros_driver/        # 官方驱动（git submodule, v0.14.4）
└── data/                           # 采集结果（git 忽略）
```

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble（ROS 1 Noetic 亦可，但本仓库脚本按 Humble 编写）
- Python 3：`opencv-python`、`numpy`、`PyYAML`，以及 `rosbag2_py` / `rclpy`（随 ROS 安装）
- Odin1 通过 **USB 3.0** 连接主机，并单独接 12–24 V 电源
- 可选：`ros-humble-*` 相关包（`cv_bridge`、`image_transport`、`pcl_conversions`、`message_filters`、`tf2_ros`、`rviz2`）

## 快速开始

```bash
git clone --recurse-submodules <this-repo> && cd odin1-tum-rgbd
./setup.sh
```

已经 clone 过但没带 submodule 时：

```bash
git submodule update --init --recursive
```

submodule 默认走 SSH（`git@github.com:manifoldsdk/odin_ros_driver.git`）。若机器上没有配置 GitHub SSH key，可改成 HTTPS：

```bash
git config submodule.catkin_ws/src/odin_ros_driver.url https://github.com/manifoldsdk/odin_ros_driver.git
git submodule update --init --recursive
```

`setup.sh` 会依次完成：

1. 安装编译依赖（OpenCV / PCL / yaml-cpp / libusb / Eigen / OpenSSL）
2. 写入 `/etc/udev/rules.d/99-odin-usb.rules`（`2207:0019`，允许普通用户访问）
3. 检查 `catkin_ws/src/odin_ros_driver` submodule，缺失时回退到 git clone / tarball
4. 编译工作空间（`colcon build`），并修改 `catkin_ws/src/odin_ros_driver/config/control_command.yaml`，
   打开 `sendrgbundistort` 与 `senddepth`。驱动通过 `COLCON_PREFIX_PATH` 直接读取**源码目录**里的配置，
   所以完成后 submodule 会显示 1 个已修改文件（`config/control_command.yaml`），属预期行为

然后：

```bash
# 终端 1：启动驱动（同时打开 RViz）
tools/odin_driver.sh

# 终端 2：采集，Ctrl-C 结束并自动转换
tools/collect_tum.sh data/seq01
```

## 输出格式

```
seq01/
├── rgb/<timestamp>.png        去畸变后的针孔图（1600x1296, bgr8 -> RGB PNG）
├── depth/<timestamp>.png      16 bit 深度，值 = 米 * depth_scale（默认 1000 即毫米），0 = 无效
├── rgb.txt                    "timestamp rgb/<timestamp>.png"
├── depth.txt                  "timestamp depth/<timestamp>.png"
├── groundtruth.txt            "timestamp tx ty tz qx qy qz qw"（相机在 odom 系下的位姿）
├── associated.txt             RGB / depth 按时间戳关联（默认阈值 20 ms）
├── camera.txt                 内参 fx fy cx cy + 畸变（去畸变后全 0）+ depth_scale
├── extrinsics.yaml            T_camera_lidar / T_imu_lidar / T_imu_camera
├── calib.yaml                 本次使用的设备出厂标定（逐台不同）
└── bag/                       原始 rosbag2（image/compressed, cloud_raw, odometry, imu, tf）
```

- **内参**：针孔模型 `u = fx*X/Z + cx`，`v = fy*Y/Z + cy`，畸变系数为 0；数值来自设备标定 `calib.yaml`。
- **外参**：`P_cam = T_camera_lidar * P_lidar`，`P_imu = T_imu_lidar * P_lidar`，其中 `T_imu_lidar` 为固定出厂值。
- **位姿**：`groundtruth.txt` 给的是相机光学系（x 右、y 下、z 前）在 odom 系下的位姿，换到 IMU 系需左乘 `T_imu_camera`。
- **时间戳**：驱动发布设备内部时间，转换时按 bag 接收时间估计一个常量偏移，统一换算到主机时间轴，保持各传感器之间的相对时序不变。

原始 bag 保留下来，可用不同参数重新转换：

```bash
python3 tools/bag2tum.py data/seq01/bag data/seq01_v2 --fill-iters 2 --depth-scale 5000
```

`bag2tum.py` 参数：

| 参数 | 说明 |
| --- | --- |
| `--calib` | 标定文件，默认 `~/.ros/odin_ros_driver/calib.yaml` |
| `--max-dt` | RGB/depth 关联最大时间差，默认 `0.02` s |
| `--depth-scale` | 深度 PNG 换算系数，默认 `1000`（毫米）；若下游脚本硬编码 TUM 的 5000 可改 |
| `--min-depth` / `--max-depth` | 有效深度范围，默认 0.2 / 65 m |
| `--fill-iters` | 稀疏深度空洞填充迭代次数，默认 1 |
| `--no-undistort` | 保留原始鱼眼 RGB（此时 `camera.txt` 内参无效） |

## 鱼眼转针孔

数据集里的 `rgb/` 已经是针孔图。若要自定义视场或分辨率：

```bash
source /opt/ros/humble/setup.bash

python3 tools/fisheye2pinhole.py --bag data/seq01/bag -o out/          # 从 bag 批量转
python3 tools/fisheye2pinhole.py raw.jpg -o out/                       # 单张
python3 tools/fisheye2pinhole.py raw_dir/ -o out/                      # 目录
python3 tools/fisheye2pinhole.py --bag data/seq01/bag -o out/ --fov-scale 0.7
python3 tools/fisheye2pinhole.py --bag data/seq01/bag -o out/ --width 1280 --height 1024 --fx 640 --fy 640
```

Odin1 的 RGB 使用等距投影 + k2..k7 全阶多项式畸变模型（与 OpenCV `cv2.fisheye` 参数化不同），脚本按驱动同款模型实现：

```
r = norm(x, y),  theta = atan(r)
theta_d = theta + k2*theta^2 + k3*theta^3 + ... + k7*theta^7
u_src = x * (theta_d/r) * A11 + A12*y + u0
v_src = y * (theta_d/r) * A22 + v0
```

注意：改动输出内参或分辨率后，`depth/` 的投影网格（1600x1296、fx=730）不再匹配，需要同步重新投影或只使用 RGB。

## 数据特性与限制

1. **深度不是稠密传感器数据**。Odin1 没有 RGB-D 相机，深度由 LiDAR 点云按外参投影到相机像素得到，室内单帧约 8% 像素有值（几何精确、但稀疏），`--fill-iters` 只是插值。
2. **帧率 10 Hz**。`rgb_fps`/`dtof_fps = 145`（14.5 Hz）在 V0.13.x 固件上会导致设备初始化超时，本仓库统一使用 10 Hz。
3. **数据量**：原始 bag 约 13 MB/s（10 分钟约 8 GB），建议采集时预留磁盘。
4. **图像方向**由设备实际安装姿态决定；若设备倒置，图像也是倒的（不要软件翻转，否则内参/外参都需要跟着改）。
5. 采集前确认设备已独立供电；未上电时 USB 虽能枚举，但驱动会报 `Open device failed`。

## 常见问题

- **日志出现 `Open device failed` / `recv control cmd failed or timed out`**：设备刚上电或 USB 重新枚举时驱动会放弃初始化，重新运行 `tools/odin_driver.sh` 即可。`collect_tum.sh` 记录前会先探测 `/odin1/odometry`，没有数据会直接报错退出。
- **编译时 g++ 段错误 / 被 killed**：并行编译生成的大量 ROS2 接口会吃满内存，`setup.sh` 默认 `-j4`；内存更紧张时用 `BUILD_JOBS=2 ./setup.sh`。
- **`ros2 bag record` 录到 0 帧**：说明驱动没在推流（同上，重启驱动）。

## 致谢

驱动与设备标定来自 [ManifoldTech Odin SDK](https://github.com/manifoldsdk/odin_ros_driver)（Apache-2.0）。本仓库仅包含采集与格式转换脚本。
