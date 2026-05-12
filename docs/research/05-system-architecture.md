# 方向五：系统架构 & 全平台部署方案

> 核心目标：一个浏览器链接 → 任何设备 → 任意工业机械臂 → 零门槛操作

## 1. 整体技术栈

```
┌─────────────────────────────────────────────────┐
│                   前端                            │
│  React 18 + TypeScript + Vite                    │
│  UI: Ant Design / shadcn-ui                      │
│  3D: Three.js / React-Three-Fiber                │
│  流程编排: React Flow                             │
│  图表: ECharts                                   │
│  状态: Zustand                                   │
├─────────────────────────────────────────────────┤
│                   后端                            │
│  Python 3.11+ + FastAPI                          │
│  WebSocket: FastAPI WebSocket                    │
│  任务队列: Celery + Redis                         │
│  实时通信: Socket.IO (WebSocket fallback)        │
│  ORM: SQLAlchemy + SQLite/PostgreSQL             │
├──────────────┬──────────────┬───────────────────┤
│  视觉服务      │  标定服务      │  控制服务          │
│  OpenCV       │  OpenCV       │  numpy + scipy    │
│  ONNX Runtime │  Open3D       │  roboticstoolbox  │
│  PyTorch      │  SciPy        │  pybullet         │
│  Open3D       │  NumPy        │  RTDE/gRPC        │
└──────────────┴──────────────┴───────────────────┘
```

## 2. 前端架构

### 2.1 核心页面

```
未来视控
├── 🏠 首页/Dashboard
│   ├── 系统状态总览 (相机连接、机器人状态)
│   ├── 运行统计数据
│   └── 快速启动向导
│
├── 🎯 流程编辑器 (核心创新)
│   ├── 拖拽式节点编排
│   ├── 节点库：
│   │   ├── 视觉节点：采集、校正、匹配、检测
│   │   ├── 控制节点：移动、抓取、释放、等待
│   │   ├── 逻辑节点：条件判断、循环、变量
│   │   └── IO节点：输入检测、输出控制
│   ├── 实时预览面板
│   └── 一键部署运行
│
├── 📷 标定向导
│   ├── Step-by-step 引导流程
│   ├── 实时角点检测可视化
│   ├── 精度评估报告
│   └── 配置保存/加载
│
├── 👁️ 视觉监控
│   ├── 双目实时画面
│   ├── 视差图/点云 3D 可视化
│   ├── 检测结果叠加
│   └── 历史回放
│
├── 🦾 机器人控制
│   ├── 手动 Jog 控制 (方向键)
│   ├── 坐标系切换 (基座/工具/工件)
│   ├── 位姿数值输入
│   ├── IO 监控
│   └── 急停按钮
│
└── ⚙️ 设置
    ├── 相机参数配置
    ├── 机器人连接配置
    ├── 流程模板管理
    └── 日志/诊断
```

### 2.2 流程编辑器技术方案

**为什么用流程编辑器？**
> "初次接触的人也可以一秒变身行业专家完成整个流程的编写"

传统的机器人编程需要：
- 学习专用编程语言 (URScript, KRL, RAPID...)
- 理解坐标系变换
- 调试周期长

**流程编辑器核心设计：**

```typescript
// 流程节点定义
interface FlowNode {
  id: string;
  type: 'vision_capture' | 'stereo_match' | 'object_detect' 
      | 'pose_estimate' | 'move_robot' | 'grasp' | 'release'
      | 'move_home' | 'wait' | 'if' | 'loop' | 'io_out' | 'io_in';
  position: { x: number; y: number };
  data: Record<string, any>;  // 节点参数
}

interface FlowEdge {
  id: string;
  source: string;  // 源节点ID
  target: string;  // 目标节点ID
  sourceHandle: 'success' | 'failure' | 'output';
}

// 流程执行引擎（后端）
class FlowEngine:
    def execute(self, flow_definition: dict):
        nodes = flow_definition['nodes']
        edges = flow_definition['edges']
        
        # 拓扑排序 → 确定执行顺序
        execution_order = topological_sort(nodes, edges)
        
        # 逐个执行节点
        context = {}  # 节点间数据传递
        for node_id in execution_order:
            node = get_node(node_id)
            result = node.execute(context)
            context[node_id] = result
            yield node_id, result  # 流式回传前端
```

### 2.3 3D 可视化方案

```typescript
// 基于 React-Three-Fiber 的工业场景渲染
function Scene3D({ pointCloud, robotJoints, detections }) {
  return (
    <Canvas camera={{ position: [2, 2, 2], fov: 50 }}>
      <ambientLight intensity={0.5} />
      <directionalLight position={[5, 5, 5]} />
      
      {/* 点云渲染 */}
      <PointCloudMesh points={pointCloud} 
                       size={0.002} 
                       colorMap="z_height" />
      
      {/* 机器人模型 */}
      <RobotModel jointAngles={robotJoints} 
                  urdf="/models/ur5e.urdf" />
      
      {/* 检测框 */}
      {detections.map(det => (
        <PoseBBox pose={det.pose} 
                  size={det.size} 
                  color={det.classColor} />
      ))}
      
      {/* 抓取候选 */}
      {graspCandidates.map(g => (
        <GraspVisualizer grasp={g} />
      ))}
      
      <OrbitControls />
      <GridHelper args={[2, 20]} />
    </Canvas>
  );
}
```

## 3. 后端架构

### 3.1 服务划分

```
后端微服务 (单进程/多进程可切换)

┌─────────────────────────────────────┐
│          API Gateway (FastAPI)       │
│  /api/v1/flow/*   流程管理          │
│  /api/v1/vision/* 视觉服务          │
│  /api/v1/calib/*  标定服务          │
│  /api/v1/robot/*  机器人控制        │
│  /ws/stream       实时数据通道      │
└─────────────────────────────────────┘
         │
    ┌────┼────┬────────┐
    ▼    ▼    ▼        ▼
 [Vision] [Calib] [Control] [Data]
```

### 3.2 实时通信方案

```python
# WebSocket 实时数据推送架构
class RealTimeHub:
    """
    统一管理所有实时数据通道
    
    通道：
    - /ws/camera/left    左相机视频流
    - /ws/camera/right   右相机视频流  
    - /ws/pointcloud     实时点云流
    - /ws/robot/state    机器人状态
    - /ws/flow/status    流程执行状态
    """
    
    async def stream_camera(self, websocket, camera_id):
        """MJPEG 兼容 + WebSocket 视频流"""
        cap = cv2.VideoCapture(camera_id)
        while True:
            ret, frame = cap.read()
            if not ret: break
            
            # JPEG 编码
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            await websocket.send_bytes(jpeg.tobytes())
            await asyncio.sleep(1/30)  # 30 FPS
    
    async def stream_pointcloud(self, websocket):
        """点云流（解码后为 Float32Array）"""
        while True:
            pcd = get_latest_pointcloud()
            # 压缩为二进制传输
            data = compress_pointcloud(pcd)
            await websocket.send_bytes(data)
```

### 3.3 数据库设计

```sql
-- 核心表结构
CREATE TABLE calibration_configs (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100),
    camera_matrix_L TEXT,  -- JSON
    camera_matrix_R TEXT,  -- JSON
    dist_coeffs_L TEXT,    -- JSON
    dist_coeffs_R TEXT,    -- JSON
    R TEXT,  -- 旋转矩阵 JSON
    T TEXT,  -- 平移向量 JSON
    Q TEXT,  -- 重投影矩阵 JSON
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE robot_configs (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100),
    brand VARCHAR(50),     -- 'UR', 'KUKA', 'ESTUN', etc.
    ip_address VARCHAR(45),
    urdf_path TEXT,
    joint_limits TEXT,     -- JSON
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE flow_templates (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100),
    description TEXT,
    flow_data TEXT,        -- JSON (nodes + edges)
    version INTEGER DEFAULT 1,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE task_history (
    id SERIAL PRIMARY KEY,
    flow_id INTEGER REFERENCES flow_templates(id),
    status VARCHAR(20),    -- 'running', 'success', 'failed'
    result TEXT,           -- JSON
    started_at TIMESTAMP,
    finished_at TIMESTAMP
);
```

## 4. 全平台策略

### 4.1 PWA 方案

```json
// manifest.json
{
  "name": "未来视控",
  "short_name": "FVControl",
  "start_url": "/",
  "display": "standalone",
  "background_color": "#0a0a1a",
  "theme_color": "#1a73e8",
  "icons": [...],
  "screenshots": [...]
}
```

PWA 优势：
- ✅ 安装到桌面/主屏幕
- ✅ 离线缓存流程模板
- ✅ 推送通知（任务完成/异常告警）
- ✅ 全平台：Windows/Mac/Linux/Android/iOS

### 4.2 移动端适配

```
关键交互适配：
├── 机器人 Jog 控制 → 触屏手势 (滑动=XY, 双指=Z)
├── 流程编辑器 → 响应式节点布局，大屏编辑/小屏监控
├── 3D 视图 → 触摸旋转/缩放
├── 标定向导 → 竖直布局，单步一屏
└── 急停按钮 → 大红色按钮，页面始终可见
```

### 4.3 部署方案

```
开发环境：      生产环境：
               ┌──────────────────────────┐
localhost:5173 │ 工控机 (IPC)              │
localhost:8000 │ ├── Docker Compose        │
               │ ├── Nginx (反向代理)      │
               │ ├── FastAPI (后端)        │
               │ ├── Redis (任务队列)      │
               │ └── GPU (视觉推理)        │
               │                          │
               │ 访问：http://ipc-ip:8080 │
               │ 手机/平板同局域网即可操作   │
               └──────────────────────────┘
```

## 5. 开发优先级 & 里程碑

### Milestone 1: 核心视觉验证 (Week 1-2)
```
✅ 相机采集 + 双目标定
✅ SGBM 立体匹配 + 点云生成
✅ 基本 Web 预览界面
```

### Milestone 2: 机器人控制 (Week 3-4)
```
✅ 机器人通信接口 (UR首选)
✅ 手动 Jog 控制面板
✅ 点云 → 抓取点 → 执行 闭环
```

### Milestone 3: 流程编辑器 (Week 5-6)
```
✅ React Flow 集成
✅ 节点系统 + 执行引擎
✅ 流程保存/加载/运行
```

### Milestone 4: 智能升级 (Week 7-8)
```
✅ 深度学习立体匹配集成
✅ 6D 位姿估计集成
✅ 无序抓取策略
```

### Milestone 5: 全平台打磨 (Week 9-10)
```
✅ PWA 离线支持
✅ 移动端适配
✅ 性能优化 + 测试
```

## 6. Claude Code 开发任务

### 第一批任务（可并行）

| 任务 | 模块 | 优先级 |
|------|------|:---:|
| 相机采集模块 (CameraCapture) | src/vision/ | P0 |
| 双目标定 (StereoCalibrator) | src/calibration/ | P0 |
| SGBM立体匹配 (StereoMatcher) | src/vision/ | P0 |
| 点云生成+处理 (PointCloudProcessor) | src/vision/ | P0 |
| FastAPI 骨架 (路由+WebSocket) | src/backend/ | P0 |
| React 前端骨架 + 相机预览 | src/frontend/ | P1 |

### 第二批任务

| 任务 | 模块 | 优先级 |
|------|------|:---:|
| 机器人接口 (URInterface) | src/control/ | P1 |
| 抓取点计算 (GraspPlanner) | src/control/ | P1 |
| 流程编辑器 (FlowEditor) | src/frontend/ | P1 |
| 3D 可视化 (Scene3D) | src/frontend/ | P1 |
| RAFT-Stereo ONNX 推理 | src/vision/ | P2 |
| PPF + ICP 位姿估计 | src/vision/ | P2 |
