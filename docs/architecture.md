# 未来视控 — 技术架构设计 v0.1

## 1. 系统分层

### 1.1 前端层 (Frontend)
**选型：Web + PWA**

理由：
- 全平台（Windows/Mac/Linux/Android/iOS）零成本覆盖
- PWA 可离线运行，接近原生体验
- 工业界趋势：Siemens Industrial Edge、ABB Ability 均采用 Web
- 技术栈推荐：React + Three.js (3D 可视化) + AntV (数据图表)

关键功能模块：
- **流程编排器 (Flow Editor)**：拖拽式节点编排，类似 Node-RED 但针对视觉-控制场景简化
- **3D 场景预览**：Three.js 实时渲染双目点云 + 机械臂姿态
- **标定向导**：Step-by-step 引导式标定流程
- **控制面板**：实时 Jog 操作、抓取参数调节
- **监控仪表盘**：精度指标、运行日志、异常告警

### 1.2 后端层 (Backend)
**选型：Python FastAPI + WebSocket + gRPC**

理由：
- Python 与 OpenCV/深度学习生态无缝对接
- FastAPI 异步高性能，WebSocket 支持实时推送
- gRPC 用于内部微服务高性能通信

核心服务：
- **API Gateway**：RESTful + WebSocket 统一入口
- **任务编排引擎**：解析前端流程定义，调度各引擎执行
- **实时数据通道**：相机流推送、机器人状态广播
- **硬件驱动层**：统一抽象，支持多品牌机器人 (UR, KUKA, 汇川, 埃斯顿...)

### 1.3 视觉引擎 (Vision Engine)
核心流水线：
```
[左相机] ──→ [校正] ──→ [立体匹配] ──→ [视差图] ──→ [点云]
[右相机] ──→ [校正] ──→                                  ↓
                                                    [目标检测]
                                                        ↓
                                                  [6D 位姿估计]
                                                        ↓
                                                  [抓取点计算]
```

关键算法模块：
- **立体匹配**：SGBM → RAFT-Stereo → 自监督微调
- **目标检测/分割**：YOLO → SAM → 工业工件定制模型
- **6D 位姿估计**：PVN3D / FFB6D → 针对工业零件的领域适配
- **点云处理**：降采样、离群点去除、平面分割、聚类

### 1.4 标定引擎 (Calibration Engine)
- **双目标定**：传统棋盘格 + 无标定板自标定（研究方向）
- **手眼标定**：Eye-in-hand / Eye-to-hand 两种模式
- **多相机注册**：多视角外参标定
- **在线校正**：运行时漂移检测与补偿

### 1.5 控制引擎 (Control Engine)
- **运动学**：正/逆运动学求解（支持多构型）
- **轨迹规划**：RRT-Connect, TrajOpt
- **抓取规划**：力控/位控混合策略
- **碰撞检测**：基于 FCL/Bullet 的实时碰撞避免

### 1.6 硬件抽象层 (HAL)
- **相机抽象**：支持 USB/ GigE / IP Camera
- **机器人抽象**：统一控制接口（位置/速度/力矩模式）
- **夹爪抽象**：电动/气动夹爪通用控制

## 2. 数据流

```
[2D Camera L] ──→ [Frame Grabber] ──→ [Stereo Rectifier]
[2D Camera R] ──→ [Frame Grabber] ──→ [Stereo Rectifier]
                                            ↓
                                     [Stereo Matcher]
                                            ↓
                              ┌───── [Disparity Map]
                              ↓              ↓
                         [Point Cloud]  [6D Pose Est.]
                              ↓              ↓
                         [Grasp Planner] ←───┘
                              ↓
                         [Robot Controller]
                              ↓
                         [Robot Arm + Gripper]
```

## 3. 关键技术指标

| 指标 | 目标 | 备注 |
|------|------|------|
| 立体匹配精度 | < 1 pixel (sub-pixel) | 亚像素精匹配 |
| 位姿估计误差 | 平移 < 2mm, 旋转 < 2° | 工业级要求 |
| 标定重投影误差 | < 0.3 pixel | 张正友标定法基准 |
| 系统延迟 | < 200ms (端到端) | 抓取→规划→执行 |
| 最小工件 | 10mm × 10mm | 小零件抓取能力 |

## 4. 技术风险

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 低成本相机精度不足 | 高 | 亚像素算法 + 多帧融合 + 深度学习增强 |
| 无标定板标定可靠性 | 高 | 结合场景约束 + 在线优化 |
| 动态抓取实时性 | 中 | GPU加速 + 预测性规划 |
| 跨平台兼容性 | 中 | Web标准 + 渐进式增强 |
