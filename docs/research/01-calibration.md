# 方向一：无标定板的双目相机自标定方法

> 研究目标：消除传统棋盘格标定的繁琐流程，实现快速部署

## 1. 问题背景

传统双目相机标定（张正友法）需要：
- 打印高精度棋盘格/圆点标定板
- 在不同位姿下拍摄 ≥10 组图像对
- 手动移动标定板，对操作人员要求高
- 工厂部署时每次换线/碰撞后需重新标定

**核心痛点：** 标定过程繁琐、依赖人工操作、无法部署即用。

## 2. 技术路线对比

### 路线 A：基于场景特征的自标定（传统方法）

**原理：** 利用场景中的自然特征点（角点、边缘、纹理）替代标定板，通过多视图几何约束求解相机内参和外参。

**代表方法：**
| 方法 | 核心思想 | 内参 | 外参 |
|------|---------|------|------|
| Kruppa 方程 | 绝对二次曲线的对偶投影 | ✅ | ❌ |
| Mendonça-Cipolla | 本质矩阵约束 | ✅ | ❌ |
| Pollefeys 分层自标定 | 射影→仿射→度量逐级恢复 | ✅ | ✅ |
| 基于消失点 | 利用平行线消失点恢复内参 | ✅ | ❌ |

**可行性分析：**
- ✅ 无需标定板，理论上任意场景即可
- ✅ 鲁棒性差——工业场景纹理少、光照变化大
- ✅ 精度通常低于标定板方法（重投影误差 0.5-1.0px vs 0.1-0.3px）

### 路线 B：基于深度学习的标定（前沿方向）

**原理：** 用神经网络直接从图像预测相机参数，隐式学习几何约束。

**代表方法：**
| 方法 | 特点 | 输入 | 输出 |
|------|------|------|------|
| DeepCalib (Lopez+, 2019) | 单图预测焦距 | 单图 | f |
| DeepFocal (Workman+, 2015) | 从场景分类预测焦距 | 单图 | f |
| Perceptual Calibration (Liao+, 2023) | 从多帧视频端到端标定 | 视频序列 | K + E |
| **自监督在线标定** | 用立体匹配损失反向传播优化标定参数 | 双目图对 | K + [R|t] |

**核心思路（自监督）：**
```
1. 初始化相机参数（粗略估计）
2. 用参数对双目图像做校正（rectify）
3. 计算立体匹配损失（photometric + smoothness）
4. 梯度反向传播，更新相机参数
5. 迭代至收敛
```

**伪代码：**
```python
class SelfCalibNet(nn.Module):
    def __init__(self):
        self.k_net = KPredictor()  # 预测内参 K
        self.e_net = EPredictor()  # 预测外参 E
        
    def forward(self, img_L, img_R):
        K_L, K_R = self.k_net(img_L), self.k_net(img_R)
        E = self.e_net(torch.cat([img_L, img_R], dim=1))
        
        # 校正双目图像
        rect_L, rect_R = stereo_rectify(img_L, img_R, K_L, K_R, E)
        
        # 计算立体匹配损失
        loss = photometric_loss(rect_L, rect_R) + smoothness_loss(disparity)
        
        return K_L, K_R, E, loss
```

### 路线 C：混合方案（推荐 ⭐）

**思路：** 出厂时用标定板做一次精密标定，部署后通过在线优化维持精度。

```
┌── 出厂标定（离线） ──────────────────────────────┐
│ 标定板 → 张正友法 → 精确 K, [R|t] (基准)        │
└────────────────────────────────────────────────┘
                         ↓
┌── 部署标定（在线） ──────────────────────────────┐
│ 场景特征点 → 本质矩阵约束 → 修正外参 [R|t]       │
│ 若温度/振动漂移 → 在线校正 → 更新标定参数         │
└────────────────────────────────────────────────┘
```

**关键算法——在线外参校正：**
```
1. 从双目图像对提取 ORB/SIFT 特征点
2. 使用 KNN + Ratio Test + RANSAC 匹配
3. 计算本质矩阵 E
4. 从 E 中恢复 R, t（已知内参 K）
5. 用滑窗中值滤波平滑估计值
6. 若漂移量超过阈值 → 更新外参
```

## 3. 技术决策

### 3.1 标定策略分层

| 场景 | 策略 | 精度 |
|------|------|------|
| 初次部署 | 棋盘格标定（一次到位） | ★★★★★ |
| 快速换线 | 已知内参 + 场景自标定外参 | ★★★★ |
| 在线运行 | 特征点匹配 + 滑窗校正 | ★★★ |
| 碰撞/振动后 | 自动检测漂移 → 触发重标定 | ★★★★ |

### 3.2 推荐方案

**第一阶段（MVP）：**
- 标准棋盘格双目标定（张正友法 + Bouguet 立体校正）
- 提供标定向导 UI，降低操作门槛
- 标定结果保存为配置文件，一次标定反复使用

**第二阶段（智能化）：**
- 实现在线漂移检测（基于极线约束残差）
- 场景特征点自动外参校正
- 多帧融合提高鲁棒性

**第三阶段（终极目标）：**
- 自监督深度学习在线标定
- 仅需在场景中随意放置任意物体即可完成标定
- 对标定板零依赖

## 4. 实现要点

### 4.1 标定向导 UI 设计
```
Step 1: 相机选择 ──→ Step 2: 标定板规格 ──→ Step 3: 图像采集
（自动检测相机）     （支持自定义/预设）     （实时检测棋盘格角点）
                                                 ↓
Step 5: 精度验证 ←── Step 4: 自动标定
（重投影误差可视化） （OpenCV stereoCalibrate）
```

### 4.2 漂移检测机制
```python
def detect_calibration_drift(img_L, img_R, K, baseline_params, threshold=0.5):
    """检测标定是否漂移"""
    # 提取特征点并匹配
    kp_L, des_L = sift.detectAndCompute(img_L, None)
    kp_R, des_R = sift.detectAndCompute(img_R, None)
    matches = bf.knnMatch(des_L, des_R, k=2)
    good = [m for m,n in matches if m.distance < 0.7*n.distance]
    
    # 计算极线约束误差
    pts_L = np.float32([kp_L[m.queryIdx].pt for m in good])
    pts_R = np.float32([kp_R[m.trainIdx].pt for m in good])
    
    E, mask = cv2.findEssentialMat(pts_L, pts_R, K)
    _, R, t, mask = cv2.recoverPose(E, pts_L, pts_R, K)
    
    # 比较与基准外参的差异
    drift = compute_pose_difference(R, t, R_baseline, t_baseline)
    return drift > threshold, drift
```

## 5. 参考文献方向

- Zhang, Z. (2000). "A Flexible New Technique for Camera Calibration." IEEE TPAMI. — 经典张氏标定
- Faugeras, O. et al. (1992). "Camera Self-Calibration: Theory and Experiments." ECCV. — 自标定理论基础
- Pollefeys, M. et al. (1999). "Self-Calibration and Metric Reconstruction." IJCV. — 分层自标定
- Hartley, R. & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision*. — 多视图几何圣经
- Long, L. et al. (2021). "Learning to Calibrate: Deep Camera Calibration." — 深度学习标定
- OpenCV: `cv2.stereoCalibrate()`, `cv2.stereoRectify()`, `cv2.findEssentialMat()`
