# 方向三：无序工件 6D 位姿估计

> 核心：从点云/RGB-D 中估计工件 3D 位置 + 3D 旋转 = 6DoF 位姿

## 1. 问题定义

```
输入：双目点云 / RGB-D 图像 + 工件 CAD 模型（可选）
输出：(R ∈ SO(3), t ∈ ℝ³) → 变换矩阵 T ∈ SE(3)

抓取点 = T_pose @ grasp_point_model
```

### 工业场景特殊挑战
- 工件堆叠、互相遮挡
- 无纹理（金属反光、塑料同色）
- 尺寸范围大（10mm~500mm）
- 需要 < 2mm, < 2° 的精度

## 2. 技术路线

### 路线 A：基于模板匹配（传统）

**代表方法：** LINEMOD, PPF (Point Pair Feature)

```python
# OpenCV PPF 6D 位姿估计
detector = cv2.ppf_match_3d.PPF3DDetector(step=0.01, angle_step=12)
detector.trainModel(pcd_model)  # CAD → 点云
results = detector.match(pcd_scene, relative_pose_th=0.05)

# ICP 精化
icp = cv2.ppf_match_3d.ICP(iterations=100)
pose_refined = icp.registerModelToScene(pcd_model, pcd_scene, results[0].pose)
```

**优点：** 无需训练，CAD模型直接使用
**缺点：** 对堆叠/遮挡场景效果差，速度慢

### 路线 B：基于深度学习的 6D 位姿估计

#### 方法一：PVN3D — 3D关键点投票
```
核心思路：
1. 从RGB-D图像中检测目标的2D关键点
2. 每个像素投票给3D关键点位置
3. 用最小二乘拟合从3D关键点恢复6D位姿

Pipeline：
RGB-D → [PointNet++ 主干] → 逐点特征
    ├→ 语义分割 (Semantic Head)
    ├→ 3D关键点偏移 (Offset Head)  
    └→ 中心点投票 (Center Head)
              ↓
    聚类 + 投票 → 3D关键点 → SVD求解位姿
```

#### 方法二：FFB6D — 全流双向融合
```
核心思路：
1. RGB特征 + 点云特征双向增强
2. 稠密对应 → 通过PnP求解位姿
3. 比PVN3D更快，更适合实时场景

性能：YCB-Video数据集 AUC > 95%
推理速度：~30ms (RTX 3090)
```

#### 方法三：FoundationPose ⭐（推荐统一方案）
```
核心创新：
1. 统一模型支持 model-based 和 model-free
   - Model-based: 输入CAD模型
   - Model-free: 输入少量参考图像（无需CAD！）
   
2. 神经隐式表示 (NeRF-like)
   - 从参考图像合成新视角
   - 下游位姿估计模块不变

3. 大语言模型辅助合成训练
   - LLM生成纹理描述 → 增强合成数据多样性

4. Transformer架构 + 对比学习
   - 强泛化能力
   - 对novel object零样本支持

优点：✅ 新工件无需重新训练（零样本）
      ✅ 无需精确CAD模型
      ✅ SOTA精度
缺点：⏳ 推理速度相对慢
```

### 路线 C：工业实战混合方案 ⭐

```
阶段一：粗定位（Rough）
├── YOLOv8 2D检测 → ROI裁剪
├── 2D检测框 → 约束3D搜索范围
└── 粗位姿估计（PPF / 全局描述子匹配）

阶段二：精定位（Fine）  
├── ICP 点云配准（粗位姿初始化）
├── 边缘引导的ICP（针对低纹理）
└── 多假设验证（生成多个候选，选最佳匹配）

阶段三：跟踪（Tracking）
├── 连续帧ICP跟踪（实时更新位姿）
├── 粒子滤波（处理动态物体）
└── 位姿图优化（回环检测）
```

## 3. 实测实现方案（MVP推荐）

```python
class IndustrialPoseEstimator:
    """
    工业6D位姿估计器 - 适用于已知CAD模型的工件
    """
    def __init__(self, cad_model_path):
        self.cad_pcd = load_cad_as_pointcloud(cad_model_path)
        self.cad_features = compute_fpfh(self.cad_pcd)
        self.detector = YOLO('yolov8s.pt')  # 2D 检测
        
    def estimate_pose(self, scene_pcd, scene_rgb):
        # Step 1: 2D 检测定位 ROI
        detections = self.detector(scene_rgb)
        
        for det in detections:
            roi_pts = crop_pointcloud(scene_pcd, det.bbox)
            
            # Step 2: 全局配准 (RANSAC + FPFH)
            global_pose = ransac_global_registration(
                roi_pts, self.cad_pcd, self.cad_features
            )
            
            # Step 3: ICP 精配准
            refined_pose = point_to_plane_icp(
                roi_pts, self.cad_pcd, 
                init=global_pose
            )
            
            # Step 4: 质量评估
            fitness, inlier_rmse = evaluate_registration(
                roi_pts, self.cad_pcd, refined_pose
            )
            
            if fitness > 0.5 and inlier_rmse < 0.003:
                return refined_pose, fitness
                
        return None, 0  # 未找到有效位姿
```

## 4. 零样本/少样本方案（新时代方向）

### 新工件快速部署策略

```
┌── 有CAD模型 ──────────────────────┐
│ → 虚拟渲染 (BlenderProc/NVISII)    │
│ → 自动生成合成训练数据             │
│ → 微调/零样本位姿估计              │
└────────────────────────────────────┘

┌── 无CAD模型 ──────────────────────┐
│ → 拍摄工件多视角图像 (20-50张)     │
│ → 结构光/ToF 采集部分点云          │
│ → NeRF/3DGS 重建粗糙模型           │
│ → FoundationPose 输入参考图像      │
└────────────────────────────────────┘
```

## 5. 精度评估指标

| 指标 | 计算方式 | 目标值 |
|------|---------|--------|
| ADD(-S) | 平均点距离 | < 2mm |
| ADD-S (对称物体) | 最近点距离 | < 2mm |
| 平移误差 | ‖t_pred - t_gt‖ | < 2mm |
| 旋转误差 | arccos((tr(R_pred^T R_gt)-1)/2) | < 2° |

## 6. 技术决策

| 阶段 | 方法 | 场景 |
|------|------|------|
| MVP | PPF + ICP | 工件少、场景简单 |
| v1.0 | FFB6D / PVN3D | 中等复杂度 |
| v2.0 | FoundationPose | 多工件、零样本 |
| 终极 | 端到端抓取-位姿联合学习 | 具身智能 |
