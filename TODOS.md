# TODOS — bin-picking 项目待办

## P0 (阻塞性 / 下一步)

- [x] **z-offset 补偿下沉** — 补偿已从 grasp_planner.cam_z_offset 移到 pose_estimator.z_correction (commit 7f4eb30, 2026-05-14)
- [ ] **z-offset 根因诊断** — 需在 Gazebo 运行时收集数据：pose_estimator 的 raw_z vs corrected_z vs stereo_matcher 中心区 Z，确认 0.33m 偏差是否恒定、来源是否为聚类点选择偏差。
  - 入口: `src/vision/pose_estimator.py:estimate_pose` — debug 日志已添加 raw_z / corrected_z
  - 入口: `src/vision/stereo_matcher.py:process` — 中心区 Z 统计
  - 前提: 需 Gazebo 运行环境（WSL2 或实体机）

- [ ] **Gazebo 关节控制闭环** — 当前 grasp_planner 发布 `/joint_states` 话题但 Gazebo 中 UR5 不会实际移动。WSL2 不支持 ros2_control。备选方案：`ros2 service call /gazebo/set_model_configuration` 直接设置关节位姿（原生 Gazebo 接口）。
  - 入口: `src/vision/grasp_planner.py:_execute_phase`
  - 前提: 实体 Ubuntu 工控机 或 在 WSL2 中验证 `/gazebo/set_model_configuration` 可用性
  - 难度: 需要配合 Gazebo 物理仿真调试

## P1 (重要)

- [ ] **夹爪物理仿真** — gripper 状态机已经实现了 开(1.0)/闭(0.0) 逻辑，但 Gazebo 中没有夹爪物理模型。需要在 URDF/SDF 中添加 gripper joint 并配合关节控制使用。
  - 入口: `src/vision/grasp_planner.py:_execute_phase`

- [ ] **SGBM 参数可配置化** — `stereo_matcher.py` 中 minDisparity、numDisparities、blockSize 等硬编码。应声明为 ROS 参数。
  - 入口: `src/vision/stereo_matcher.py:32-38`

- [ ] **pose_estimator ICP 真实化** — 当前 ICP 用自身点云扰动作为 source/target（玩具实现）。应改为对 CAD 模型采样点云做 source，实际点云做 target。
  - 入口: `src/vision/pose_estimator.py:166-189`

## P2 (改善)

- [ ] **pytest 迁移** — 测试框架从全局变量 PASS/FAIL 手动计数迁移到 pytest fixtures + assert。不阻塞功能。
  - 入口: `tests/` 下所有测试文件

- [ ] **CI/CD** — 添加 GitHub Actions 自动跑离线测试（test_geometry + test_stereo_matcher + test_pipeline 离线部分）
  - 前提: GitHub Actions Ubuntu runner 上 pip install numpy scipy + 生成 URDF

- [ ] **实体工控机部署** — 将整套管线从 WSL2 迁移到实体 Ubuntu 22.04 工控机，真正连接 UR5 和双目相机
  - 依赖: Gazebo 关节控制、z-offset 根除、夹爪物理

## 已完成 (Phase 5)

- [x] 拆分 grasp_planner.process() 巨方法 (2026-05-14)
- [x] 提取共享 geometry_utils.py (2026-05-14)
- [x] 修复 stereo_matcher 亚像素精度损失 (2026-05-14)
- [x] 删除 grasp_planner 未使用的 object_cloud (2026-05-14)
- [x] 测试覆盖率 4% → 25%+ (2026-05-14)
