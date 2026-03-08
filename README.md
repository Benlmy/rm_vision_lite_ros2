# RoboMaster Visual Perception System (Lite & Agile)

## 基于 ROS2 (Jazzy) 的轻量化、高鲁棒性机甲大师单目视觉与 Topic 通信系统

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.12-yellow)
![YOLOv8](https://img.shields.io/badge/YOLOv8-ONNX_Runtime-orange)
![OpenCV](https://img.shields.io/badge/OpenCV-4.8.1-green)

## 🌟 项目简介

本项目旨在为 RoboMaster/RoboTac 机器人提供一套高实时性、高稳定性的视觉感知解决方案。考虑到边缘计算设备算力受限，以及复杂的 ROS2 多节点异步通信极易导致“画面掉帧”与“视觉撕裂”，本项目放弃了臃肿的 C++ 多节点离线渲染队列方案，创新性地采用了 **“Python 极简双节点同步架构 + ONNX Runtime CPU 加速”**。

[cite_start]系统能极其稳定地输出原图、传统视觉、神经网络“三联屏”对比画面，并以约 30Hz 的频率将目标装甲板 2D 坐标通过 ROS2 Topic 统一发布，供下游模块使用 [cite: 14]。

## 🏗️ 系统架构设计

系统包含两个核心 ROS2 节点，通过极简的发布-订阅模式实现 100% 的帧同步：

1. **`video_pub.py` (图像源节点)**
   - **职责**: 稳定读取本地视频帧，转换为 `sensor_msgs/Image`，模拟物理摄像头向 ROS2 域内广播。
2. **`vision_sub.py` (感知与渲染中枢)**
   - **职责**: 订阅图像流，**在单次回调内同步执行**传统视觉预处理与 ONNX 神经网络推理。
   - **输出**: 渲染绝不掉帧的三联屏可视化窗口，并提取最优装甲板坐标，以 `geometry_msgs/Point` 发布至 `/armor_target_center` Topic。

## ⚙️ 核心算法演进

### 1. 神经网络 (YOLOv8 -> ONNX)

- 使用 RoboMaster 开源数据集对 YOLOv8s 进行针对性训练，mAP50 达到 99%。
- **工程优化**: 为摆脱庞大的 PyTorch 训练框架依赖，彻底解决算力受限导致的卡顿，将 `best.pt` 导出为静态计算图 `best.onnx`。利用 ONNX Runtime 进行推理，实现了在纯 CPU 环境下的高帧率实时检测。

### 2. 传统视觉 (高鲁棒性调优)

- **抗光照干扰**: 放弃传统的灰度高光提取，采用 RM 经典的**“通道相减法 (Channel Subtraction)”**精准提取红蓝特征。
- **几何约束收紧**: 为解决车轮反光导致的误检，引入严格的灯条匹配机制（限制高度差 < 40%，严格对齐 Y 轴中心，限制长宽比）。

## 🛠️ 环境部署与避坑指南

### 环境依赖

- Ubuntu 24.04 (WSL/Native) + ROS 2 Jazzy
- `pip3 install onnxruntime ultralytics opencv-python==4.8.1.78`

### 避坑记录 (Troubleshooting)

1. **环境锁死问题 (PEP 668)**: 最新 Ubuntu 限制全局 pip 安装。采用 `--break-system-packages` 用户级安装策略，完美兼顾 ROS2 底层通信库与上层推理库。
2. **`cv_bridge` 与 NumPy 2.x 的段错误冲突 (Core Dumped)**: YOLO 依赖的最新 NumPy 2.x 会导致以 C++ 编译的 ROS2 `cv_bridge` 底层内存崩溃。**解决方案**: 强制将 NumPy 降级为 1.26.4，并同步将 OpenCV 降级至 4.8.1，完美解决包依赖死锁。

## 🚀 运行方法

```bash
# 终端 1：启动图像流
source /opt/ros/jazzy/setup.bash
python3 src/video_pub.py

# 终端 2：启动感知中枢
source /opt/ros/jazzy/setup.bash
python3 src/vision_sub.py

# 终端 3：监听输出结果
ros2 topic echo /armor_target_center
