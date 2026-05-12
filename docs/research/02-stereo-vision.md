# 方向二：低成本双目立体视觉系统

> 核心挑战：用两个低成本 2D 相机达到工业级 3D 重建精度

## 1. 技术全景

```
双目视觉流水线：
[左图] ──── [去畸变] ────┐
                          ├── [极线校正] ── [立体匹配] ── [视差图] ── [点云]
[右图] ──── [去畸变] ────┘
```

### 关键指标

| 指标 | 传统方法(SGBM) | 深度学习方法 | 目标 |
|------|:---:|:---:|:---:|
| 匹配精度 | EPE 1.5-3px | EPE 0.3-0.8px | EPE < 1px |
| 速度 | 30-60ms | 50-200ms (GPU) | < 100ms |
| 低纹理表现 | ❌ 差 | ✅ 较好 | ✅ |
| 内存占用 | 低 | 高(GPU) | 可控 |

## 2. 立体匹配算法选型

### 2.1 传统方法：SGBM（Semi-Global Block Matching）

**优点：** CPU 友好，工业验证充分，无需 GPU
**缺点：** 低纹理区域、遮挡边界表现差

```python
# OpenCV SGBM 最优参数配置（工业场景）
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=128,      # 根据基线调整
    blockSize=7,             # 3-11，工业零件建议7-9
    P1=8 * 3 * blockSize**2, # 平滑惩罚
    P2=32 * 3 * blockSize**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,      # 提高可减少误匹配
    speckleWindowSize=100,   # 散斑滤波窗口
    speckleRange=2,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
```

**工业增强策略：**
1. **多基线融合**：双相机 + 结构光投射随机纹理图案（低成本替代方案）
2. **亚像素精化**：对视差图做抛物线拟合 → 亚像素精度
3. **置信度滤波**：LR-Check + 唯一性比率 + 散斑滤波三级过滤

### 2.2 深度学习方法（生产级推荐 ⭐）

#### 方案一：RAFT-Stereo
- **核心思想：** 在多尺度相关体上迭代优化光流场 → 拓展到立体匹配
- **优势：** 泛化能力强（合成数据训练，真实场景可用），低纹理区域鲁棒
- **推理速度：** ~200ms (RTX 3060)
- **开源：** ✅ (https://github.com/princeton-vl/RAFT-Stereo)

```python
# RAFT-Stereo 关键流程伪代码
class RAFTStereo(nn.Module):
    def __init__(self):
        self.fnet = FeatureEncoder()      # 特征提取
        self.cnet = ContextEncoder()      # 上下文编码
        self.update_block = UpdateBlock() # GRU 迭代更新
        
    def forward(self, imgL, imgR):
        fmaps = self.fnet(imgL, imgR)     # 1/4 分辨率特征
        ctx = self.cnet(imgL)
        
        # 构建多尺度相关体
        corr_pyramid = build_corr_pyramid(fmaps[0], fmaps[1])
        
        # 迭代优化
        disp = torch.zeros_like(imgL[:, 0])
        for i in range(ITERATIONS):  # 通常 22-32 次
            corr = corr_lookup(corr_pyramid, disp)
            delta = self.update_block(ctx, corr, disp)
            disp = disp + delta
        
        return upsample(disp)  # 恢复到原图分辨率
```

#### 方案二：CREStereo（实时优化版）
- **优势：** 速度更快 (~50ms)，专门针对真实场景优化
- **适合场景：** 工业实时应用
- **开源：** ✅

#### 方案三：IGEV-Stereo（精度优先）
- **核心：** 结合几何编码体和迭代更新
- **精度：** SOTA 级别，KITTI 排行榜前列
- **代价：** 较慢

### 2.3 立体匹配精度提升策略（低成本相机专项）

低成本相机面临的问题：
- 传感器噪声大
- 镜头畸变严重且非均匀
- 一致性差（两个相机不完全同步）

**解决策略：**

```
┌── 预处理层 ───────────────────────┐
│ 1. 暗角校正 (Vignetting Correction)│
│ 2. 自适应直方图均衡 (CLAHE)         │
│ 3. 双边滤波去噪 (保留边缘)          │
│ 4. 局部色调映射 (两个相机色彩一致)   │
└──────────────────────────────────┘
           ↓
┌── 匹配增强层 ─────────────────────┐
│ 1. Census 变换 (光照不敏感)         │
│ 2. 多尺度匹配 (粗→细)              │
│ 3. 引导滤波 (边缘保持上采样)        │
│ 4. 遮挡检测 + 孔洞填充             │
└──────────────────────────────────┘
           ↓
┌── 后处理层 ───────────────────────┐
│ 1. 亚像素精化 (抛物线拟合)          │
│ 2. 置信度图 → 加权点云             │
│ 3. 时间域滤波 (多帧融合)            │
│ 4. 离群点去除 (Statistical filter) │
└──────────────────────────────────┘
```

### 2.4 低纹理工件策略（工业核心问题）

金属零件、塑料件的共通问题：表面纹理少，立体匹配失败率高。

**解决方案矩阵：**

| 方案 | 原理 | 成本 | 效果 |
|------|------|:---:|:---:|
| 主动投射激光散斑 | 低成本激光器投射随机纹理 | 低 | ★★★★ |
| LED 条纹投影 | 投影仪投射编码条纹 | 中 | ★★★★★ |
| 深度学习语义先验 | 用检测框约束视差搜索范围 | 零 | ★★★ |
| 多曝光融合 | HDR 捕捉阴影/高光细节 | 零 | ★★★ |
| 偏振成像 | 去除金属表面镜面反射 | 中 | ★★★★ |

**推荐组合（低成本）：**
```
主动散斑投影 + 深度学习语义约束 + CLAHE预处理
```

## 3. 三维重建与点云处理

### 3.1 视差→点云公式

```python
def disparity_to_pointcloud(disparity, Q_matrix):
    """
    Q: 重投影矩阵 (4×4)，由 stereoRectify 输出
    点云: (X, Y, Z) = Q @ [u, v, disparity, 1]^T / W
    """
    points_3d = cv2.reprojectImageTo3D(disparity, Q_matrix)
    return points_3d  # shape: (H, W, 3)
```

### 3.2 点云后处理流水线

```python
def process_pointcloud(points_3d, confidence=None):
    """工业级点云处理"""
    # 1. 剔除无效点 (Z < 0 或 Z > 远平面)
    mask = (points_3d[..., 2] > 0) & (points_3d[..., 2] < MAX_Z)
    
    # 2. 置信度加权滤波
    if confidence is not None:
        mask &= (confidence > CONF_THRESHOLD)
    
    # 3. Statistical Outlier Removal
    cloud = points_3d[mask].reshape(-1, 3)
    cloud = statistical_filter(cloud, nb_neighbors=20, std_ratio=2.0)
    
    # 4. 降采样 (Voxel Grid)
    cloud = voxel_downsample(cloud, voxel_size=0.001)  # 1mm
    
    # 5. 工作平面分割 (RANSAC 平面拟合)
    plane_model, inliers = ransac_plane(cloud, distance_threshold=0.002)
    objects = cloud[~inliers]  # 非平面点 = 工件
    
    return objects, plane_model
```

## 4. 硬件方案推荐

### 低成本双目方案
```
相机选型：
├── USB 工业相机 (推荐)
│   - 型号参考：海康 MV-CA050-20UC (500万像素, USB3.0)
│   - 分辨率：2448×2048
│   - 帧率：24fps
│   - 成本：约 ¥800/个
│
├── 镜头
│   - 焦距：6-12mm 手动变焦
│   - 接口：C-Mount
│   - 成本：约 ¥200/个
│
└── 基线距离
    - 工作距离 0.5-1.5m：基线 100-300mm
    - 公式：Z = f * B / d
    - 精度：ΔZ ≈ Z² / (f * B) * Δd
```

### 精度估算
```
配置：f=8mm, B=200mm, pixel_size=3.45μm, Z=1m
视差精度 Δd = 0.5 pixel (亚像素)
→ ΔZ = 1000²/(8*200) * 0.5*0.00345
→ ΔZ ≈ 1.08 mm  ✅ 满足 <2mm 目标
```

## 5. 技术选型决策

| 阶段 | 方法 | 理由 |
|------|------|------|
| MVP v0.1 | SGBM + 后处理 | 快速验证，无GPU依赖 |
| v0.5 | RAFT-Stereo FP16 | 精度大幅提升，GPU加速 |
| v1.0 | CREStereo + TensorRT | 实时推理优化 |
| v2.0 | 自蒸馏小模型 | 端侧部署（Jetson等） |
