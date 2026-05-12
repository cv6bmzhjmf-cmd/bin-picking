# 未来视控 — 论文调研与技术方案总索引

> 妙妙负责：资料收集 + 论文推理 | Claude Code 负责：编程实现

## 已完成的调研文档

| # | 文档 | 核心内容 | 状态 |
|---|------|---------|:---:|
| 01 | [标定方案](01-calibration.md) | 自标定方法、漂移检测、标定向导 | ✅ |
| 02 | [双目视觉](02-stereo-vision.md) | SGBM→RAFT-Stereo、低纹理策略、点云处理 | ✅ |
| 03 | [位姿估计](03-pose-estimation.md) | PPF→PVN3D→FoundationPose、zero-shot方案 | ✅ |
| 04 | [抓取控制](04-grasping-control.md) | 无序抓取、运动规划、多品牌抽象、具身智能 | ✅ |
| 05 | [系统架构](05-system-architecture.md) | 技术栈、流程编辑器、全平台PWA、开发里程碑 | ✅ |
| 06 | [新型相机&衍生课题](06-novel-camera-and-topics.md) | 6种全新相机构成方式、8个衍生课题、15+功能 | ✅ |

## 关键技术决策摘要

```
标定：   棋盘格基准标定 → 在线漂移检测 → 自监督学习标定
立体匹配：SGBM (MVP) → RAFT-Stereo (v1) → CREStereo+TensorRT (v2)
位姿估计：PPF+ICP (MVP) → FFB6D (v1) → FoundationPose zero-shot (v2)
抓取：   规则策略 (MVP) → GraspNet学习 (v1) → VLA具身智能 (v2)
前端：   React+Three.js+ReactFlow → PWA全平台
后端：   FastAPI+WebSocket+Celery → Docker Compose部署
创新储备：滑轨可变基线 | 结构光散斑 | 事后标定 | 数字孪生 | 零标定DL
```

## 给 Claude Code 的第一批任务

**MVP Sprint 1 — 立即可开始：**

```
任务清单 (src/ 下创建对应模块)：

1. src/vision/camera_capture.py
   - 类 CameraCapture(camera_id, config)
   - 方法: open(), grab_frame(), close()
   - 支持 USB/RTSP 相机
   - 双相机同步采集

2. src/calibration/stereo_calibrator.py
   - 类 StereoCalibrator(board_size, square_size)
   - 方法: detect_corners(), calibrate(), save/load()
   - 封装 OpenCV stereoCalibrate + stereoRectify

3. src/vision/stereo_matcher.py
   - 类 StereoSGBMMatcher(config)
   - 方法: match(left, right) → disparity_map
   - LR-Check + 散斑滤波 + WLS滤波

4. src/vision/pointcloud.py
   - 函数: disparity_to_pointcloud(disparity, Q)
   - 类 PointCloudProcessor (降采样/滤波/平面分割)
   - 输出: numpy array (N, 3)

5. src/backend/main.py
   - FastAPI app 骨架
   - 路由: /api/v1/vision/*, /api/v1/calib/*
   - WebSocket: /ws/camera/{id}
   - CORS 配置

6. src/frontend/
   - Vite + React + TypeScript 初始化
   - 基础布局 + 相机预览组件
   - API client 封装

参考文档请先通读 docs/research/ 下的五份文档再动手编程。
遇到算法/技术细节不确定的地方，找我（妙妙）确认。
```

## 项目文件结构 (当前)

```
D:\pianqian\
├── README.md                           # 项目总览
├── docs/
│   ├── architecture.md                 # 技术架构设计
│   └── research/
│       ├── 01-calibration.md           # 标定方案
│       ├── 02-stereo-vision.md         # 双目视觉
│       ├── 03-pose-estimation.md       # 6D位姿估计
│       ├── 04-grasping-control.md      # 抓取控制
│       ├── 05-system-architecture.md   # 系统架构
│       └── INDEX.md                    # 本文件
├── src/
│   ├── vision/         # (待开发)
│   ├── calibration/    # (待开发)
│   ├── control/        # (待开发)
│   ├── frontend/       # (待开发)
│   └── backend/        # (待开发)
├── data/               # (测试数据)
└── tests/              # (测试用例)
```
