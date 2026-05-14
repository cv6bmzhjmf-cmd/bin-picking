# 未来视控 (Future Vision Control)

> 低成本双目视觉 + 工业机器人控制 — Bin-Picking 无序抓取系统

## 项目目标

基于双目立体视觉 + UR5 机械臂，实现工业机器人无序抓取（bin picking）完整管线：

```
Stereo Camera → Disparity → Point Cloud → 6D Pose → Sort → Collision → Reach → IK → Grasp
     ✅             ✅           ✅           ✅       ✅        ✅        ✅      ✅     ✅
```

## 技术架构

```
┌─────────────────────────────────────────────────────────────┐
│                      Gazebo Simulation                       │
│  ┌──────────┐  ┌───────────┐  ┌──────────────┐              │
│  │  Stereo   │  │  料箱+物体 │  │  UR5 (rviz)  │              │
│  │  Camera   │  │  (6 objects)│  │             │              │
│  └─────┬─────┘  └───────────┘  └──────────────┘              │
└────────┼────────────────────────────────────────────────────┘
         │
    ┌────▼────┐    ┌──────────┐    ┌──────────────┐
    │ Stereo  │───▶│   PCD +  │───▶│   Grasp      │
    │ Matcher │    │ 6D Pose  │    │   Planner    │
    │(SGBM+WLS)   │(Open3D)  │    │ (ikpy IK)    │
    └─────────┘    └──────────┘    └──────┬───────┘
                                          │
                              ┌───────────▼───────────┐
                              │  /joint_states        │
                              │  /gripper_cmd         │
                              │  /grasp_target        │
                              │  /grasp_marker ×3     │
                              └───────────────────────┘
```

## 快速开始

### 环境要求

- Ubuntu 22.04 + ROS2 Humble
- Python 3.10+
- Gazebo Classic 11

### 安装依赖

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-ur-description
pip3 install numpy scipy open3d opencv-python ikpy
```

### 运行仿真

```bash
cd ~/ros2_ws
colcon build --packages-select bin_picking_sim
source install/setup.bash
ros2 launch bin_picking_sim sim.launch.py
```

### 运行离线测试

```bash
# 生成 URDF
source /opt/ros/humble/setup.bash
xacro /opt/ros/humble/share/ur_description/urdf/ur.urdf.xacro name:=ur5 ur_type:=ur5 \
  | sed 's|package://ur_description|/opt/ros/humble/share/ur_description|g' > /tmp/ur5.urdf
python3 tests/test_pipeline.py
```

## 抓取管线

1. **立体匹配** — SGBM + WLS 滤波，60mm 基线，1280×720@1Hz
2. **点云生成** — 手动 3D 计算（Z = fx·B/d）
3. **物体分割** — RANSAC 平面分割 + DBSCAN 聚类
4. **6D 位姿** — PCA 主方向估计，可选 ICP 精化
5. **多物体排序** — 按距料箱中心距离排序，近优先
6. **碰撞检测** — 抓取点在料箱边界框内
7. **可达性检查** — 目标距 UR5 底座 < 0.85m
8. **逆运动学** — ikpy 朝向约束 IK，回退 position-only
9. **夹爪状态机** — 接近(开爪) ↔ 抓取(闭爪)

## ROS2 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/stereo/disparity_raw` | Image | 视差图 |
| `/stereo/object_poses` | PoseArray | 物体 6D 位姿 |
| `/stereo/objects_cloud` | PointCloud2 | 物体点云 |
| `/grasp_target` | PoseStamped | 抓取目标（world 帧） |
| `/joint_states` | JointState | 关节角（抓取/接近交替） |
| `/gripper_cmd` | Float64 | 1=开, 0=闭 |
| `/grasp_marker` | Marker | 蓝(预抓取), 红(抓取), 绿(末端) |

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `approach_height` | 0.05m | 抓取点高于物体中心 |
| `pre_grasp_height` | 0.10m | 预抓取点高于抓取点 |
| `bin_size_x/y/z` | 0.4/0.3/0.15m | 料箱尺寸 |
| `bin_z_tolerance` | 0.35m | z 轴容差 |
| `ur5_max_reach` | 0.85m | UR5 最大工作半径 |

## 已知限制

- WSL2 不支持 ros2_control，需实体 Ubuntu 工控机做 Gazebo 闭环
- z 轴坐标变换存在 ~0.3m 系统偏移（`bin_z_tolerance` 容差过渡）
- RViz OGRE 在 WSL2 无法加载 COLLADA mesh

## 技术栈

Gazebo Classic 11 · ROS2 Humble · OpenCV · Open3D · ikpy · NumPy · SciPy

## License

MIT
