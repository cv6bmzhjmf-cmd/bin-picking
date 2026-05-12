# 方向四：机器人抓取规划与控制系统

> 目标：从视觉输出到机器人执行的完整闭环

## 1. 系统架构

```
视觉层            规划层              执行层
┌──────────┐    ┌──────────┐      ┌──────────┐
│ 位姿估计  │───→│ 抓取规划  │─────→│ 运动控制  │
│ 点云处理  │    │ 碰撞检测  │      │ 轨迹跟踪  │
│ 目标检测  │    │ 轨迹优化  │      │ 力控混合  │
└──────────┘    └──────────┘      └──────────┘
    ↑                                  │
    └──────── 传感器反馈 ──────────────┘
```

## 2. 抓取策略

### 2.1 抓取类型

| 类型 | 适用场景 | 策略 |
|------|---------|------|
| 平行抓取 | 规则工件 | 两点夹持，力控 |
| 真空吸附 | 平面工件 | 负压检测 + 重心估计 |
| 包络抓取 | 异形工件 | 多指灵巧手 |
| 无序抓取 | 堆叠场景 | 按高程逐层抓取 |

### 2.2 无序抓取策略（核心场景）

```
┌── 场景分析 ────────────────────────────────────┐
│ 1. 获取工作区点云                               │
│ 2. RANSAC 分割工作平面（传送带/料框底面）        │
│ 3. 剩余点云 → DBSCAN 聚类 → 识别独立工件        │
│ 4. 按 Z 高度排序 → 从顶层开始抓取               │
└────────────────────────────────────────────────┘
                     ↓
┌── 抓取点计算 ──────────────────────────────────┐
│ 1. 对每个聚类计算3D边界框                        │
│ 2. 基于工件形状/对称性推理抓取位姿               │
│ 3. 评估抓取质量（接触面积、重心偏移、碰撞）      │
│ 4. 按质量分数排序抓取候选                       │
└────────────────────────────────────────────────┘
                     ↓
┌── 抓取执行 ─────────────────────────────────────┐
│ 1. 选择最高分抓取候选                           │
│ 2. 路径规划（接近→抓取→提起→放置）              │
│ 3. 力控夹持确认                                 │
│ 4. 失败 → 重试下一个候选                        │
└────────────────────────────────────────────────┘
```

### 2.3 基于学习的抓取检测

**方法一：GraspNet-1Billion 范式**
```
输入：点云（从双目重建）
输出：抓取候选 (grasp_center, grasp_R, grasp_width, score)

关键：抓取表示为矩形框的6DoF位姿
- 抓取中心点
- 抓取方向（接近轴 + 夹爪开合轴）
- 开合宽度
- 质量分数
```

**方法二：AnyGrasp 范式**
```
创新：泛化到任意物体的抓取检测
- 大规模合成数据训练
- 并行抓口检测 → 速度更快
- 支持多种夹爪构型
- 开源 ✅
```

**方法三：接触抓取（Contact-GraspNet）**
```
思路：不直接预测抓取框，而是预测"应该接触的点"
- 更符合人类抓取直觉
- 对异形物体更鲁棒
```

## 3. 机器人运动控制

### 3.1 运动学

```python
class RobotKinematics:
    def __init__(self, urdf_path):
        self.chain = load_robot_chain(urdf_path)  # 从URDF加载
        
    def forward_kinematics(self, joint_angles):
        """正运动学：关节角 → 末端位姿"""
        return self.chain.forward_kinematics(joint_angles)
    
    def inverse_kinematics(self, target_pose, seed=None):
        """逆运动学：末端位姿 → 关节角 (数值解)"""
        ik = pybullet.calculateInverseKinematics(
            robot_id, end_effector_link_index, 
            target_pose[:3], target_pose[3:],
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            joint_ranges=joint_ranges,
            rest_poses=seed
        )
        return ik
    
    def check_collision(self, joint_angles, scene):
        """碰撞检测"""
        set_joint_positions(joint_angles)
        return pybullet.getContactPoints() != []
```

### 3.2 轨迹规划

```python
def plan_pick_place_path(grasp_pose, place_pose, scene):
    """
    抓取-放置轨迹规划
    
    阶段：
    1. HOME → PRE-GRASP (物体上方50mm)
    2. PRE-GRASP → GRASP (直线下降)
    3. 闭合夹爪 (force control)
    4. GRASP → POST-GRASP (直线提起)
    5. POST-GRASP → PRE-PLACE (避障路径)
    6. PRE-PLACE → PLACE (直线下降)
    7. 释放工件
    8. PLACE → HOME (返回)
    """
    
    # RRT-Connect 全局路径规划
    path = rrt_connect(
        start=current_joints,
        goal=ik_solve(grasp_pose),
        collision_fn=check_collision
    )
    
    # 轨迹平滑 (Minimum Snap / B-Spline)
    smooth_traj = trajectory_optimization(path)
    
    # 时间参数化 (TOPRA)
    timed_traj = time_parametrization(smooth_traj, vel_limits, acc_limits)
    
    return timed_traj
```

### 3.3 力控混合策略

```
抓取力控策略：
┌── 接近阶段 ── 位置控制 ──────────────────┐
│ 快速移动到预抓取位置                       │
└──────────────────────────────────────────┘
┌── 抓取阶段 ── 力控 ──────────────────────┐
│ 夹爪闭合 + 力/力矩阈值检测                 │
│ 检测到力突变 → 停止闭合 → 抓到             │
│ 无突变超时 → 抓取失败 → 重试               │
└──────────────────────────────────────────┘
┌── 验证阶段 ── 力监控 ────────────────────┐
│ 提起过程中持续监控夹持力                   │
│ 力突然下降 → 滑落检测 → 紧急停止           │
└──────────────────────────────────────────┘
```

## 4. 多机器人品牌统一抽象

```python
class RobotInterface(ABC):
    """统一机器人控制接口"""
    
    @abstractmethod
    def connect(self, ip: str, port: int): ...
    
    @abstractmethod
    def move_j(self, joints: List[float], speed: float): ...
    
    @abstractmethod
    def move_l(self, pose: List[float], speed: float): ...
    
    @abstractmethod
    def get_pose(self) -> List[float]: ...
    
    @abstractmethod
    def set_digital_out(self, pin: int, value: bool): ...
    
    @abstractmethod
    def get_digital_in(self, pin: int) -> bool: ...

class URInterface(RobotInterface):
    """优傲机器人驱动"""
    def connect(self, ip, port=30002):
        self.sock = socket.create_connection((ip, port))
        self.rtde = RTDE(ip)  # 实时数据交换
        
    def move_l(self, pose, speed):
        # URScript: movel(p[x,y,z,rx,ry,rz], v, a)
        script = f"movel(p{pose}, a=0.5, v={speed})"
        self.send_script(script)

class ESTUNInterface(RobotInterface):
    """埃斯顿机器人驱动"""
    # 类似的统一接口实现
    ...
```

## 5. 具身智能方向研究

### 5.1 视觉-语言-动作模型 (VLA)
```
范式：自然语言指令 → 视觉理解 → 动作生成

应用：RT-2 (Google), Octo, OpenVLA
- "把红色工件放到左边料框"
- 模型理解场景 → 定位红色工件 → 生成抓取动作
```

### 5.2 未来视控具身智能路线图
```
Phase 1: 规则抓取 (当前)
├── 模板匹配 + 预定义抓取策略
└── 固定工作流程

Phase 2: 自适应抓取
├── 学习不同工件的最佳抓取方式
└── 少量新工件快速适配

Phase 3: 具身智能
├── 自然语言指令理解
├── 未知物体自主探索抓取
├── 从失败中学习改进策略
└── 多任务泛化
```

## 6. 仿真验证环境

推荐工具链：
| 工具 | 用途 |
|------|------|
| PyBullet / MuJoCo | 物理仿真 + 抓取验证 |
| Isaac Sim (NVIDIA) | 高保真渲染 + RL训练 |
| Gazebo + ROS2 | 完整机器人仿真 |
| BlenderProc | 合成数据生成 |

**MVP 仿真方案：**
```python
# PyBullet 快速验证
import pybullet as p
p.connect(p.GUI)
robot = p.loadURDF("robot.urdf")
objects = load_scene_objects("scene.sdf")

# 执行抓取
grasp_pose = estimate_grasp(point_cloud)
execute_grasp(robot, grasp_pose)

# 验证
success = check_grasp_success(robot, object_id)
```
