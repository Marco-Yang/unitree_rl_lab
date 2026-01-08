# Unitree RL Lab 任务结构总结

## 当前任务列表

### 1. Unitree-Go2-Velocity（原始速度任务）
- **配置文件**: `velocity_env_cfg.py`
- **Command**: `mdp.UnifiedVelocityCommandCfg`
- **目标**: 跟踪线速度(x,y)和角速度(yaw)指令
- **观测维度**: 45 (base_ang_vel:3 + projected_gravity:3 + velocity_commands:3 + joint_pos_rel:12 + joint_vel_rel:12 + last_action:12)

### 2. Unitree-Go2-VelocityPose（新增统一任务）✅
- **配置文件**: `velocity_pose_env_cfg.py`
- **Command**: `mdp.UnifiedPoseVelocityCommandCfg` 
- **目标**: 同时跟踪速度和姿态指令
  - 线速度指令 (x, y)
  - 角速度指令 (yaw)
  - 姿态指令 (roll, pitch)
  - 高度指令 (height)
- **观测维度**: 50 (base_ang_vel:3 + projected_gravity:3 + unified_commands:7 + joint_pos_rel:12 + joint_vel_rel:12 + last_action:12)
- **奖励函数**: 保留 Velocity 任务的所有权重，新增姿态跟踪奖励

---

## Command 文件使用状态

### 当前使用中 ✅
**文件**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/commands/unified_pose_velocity_command.py`

- **类名**: `UnifiedPoseVelocityCommand` / `UnifiedPoseVelocityCommandCfg`
- **功能**: 统一的速度+姿态指令管理器
- **指令维度**: 7D
  - `lin_vel_x, lin_vel_y` (2D) - 线速度
  - `ang_vel_z` (1D) - 角速度
  - `roll, pitch` (2D) - 姿态角
  - `height` (1D) - 高度
- **使用任务**: `Unitree-Go2-VelocityPose`

**特性**:
```python
# 坐标系转换
self.pose_command_b[:, :2] = quat_apply_inverse(  # ✅ 已修复警告
    base_quat_w, 
    torch.cat([self.pose_command_w[:, :2], zeros], dim=1)
)[:, :2]

# 指令范围（课程学习支持）
ranges = Ranges(
    lin_vel_x=(-1.0, 1.0),
    lin_vel_y=(-1.0, 1.0),
    ang_vel_z=(-1.0, 1.0),
    roll=(0.0, 0.0),      # Stage 1: 固定
    pitch=(0.0, 0.0),     # Stage 1: 固定
    height=(0.35, 0.35),  # Stage 1: 固定
)
```

---

### 废弃文件 ⚠️
**文件**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/commands/pose_command.py`

- **类名**: `PoseCommand` / `PoseCommandCfg`
- **功能**: 仅支持姿态指令（roll, pitch, height）
- **指令维度**: 3D
- **使用任务**: 无（已被 VelocityPose 替代）
- **状态**: ⚠️ **可以安全删除**

**原因**:
1. 功能被 `UnifiedPoseVelocityCommand` 完全覆盖
2. 没有任何任务配置文件引用它
3. 仅在 `__init__.py` 中导出但未实际使用

---

## 奖励函数权重对比

### Velocity 任务原始权重
```python
track_lin_vel_xy = 1.5
track_ang_vel_z = 0.75
lin_vel_z_l2 = -2.0
ang_vel_xy_l2 = -0.05
flat_orientation_l2 = -2.5
joint_vel = -0.001
joint_acc = -2.5e-7
joint_torques = -2e-4
action_rate = -0.1
energy = -2e-5
feet_air_time = 0.1
air_time_variance = -1.0
```

### VelocityPose 任务权重（继承+扩展）
```python
# 继承自 Velocity 任务
track_lin_vel_xy_exp = 1.5      ✅
track_ang_vel_z_exp = 0.75      ✅
base_linear_velocity = -2.0     ✅
base_angular_velocity = -0.05   ✅
flat_orientation_l2 = -2.5      ✅
joint_vel = -0.001              ✅
joint_acc = -2.5e-7             ✅
joint_torques = -2e-4           ✅
action_rate = -0.1              ✅
energy = -2e-5                  ✅
feet_air_time = 0.1             ✅
air_time_variance = -1.0        ✅

# 新增姿态控制
track_roll = 2.0                🆕
track_pitch = 2.0               🆕
track_height = 3.0              🆕
```

---

## 推荐操作

### ✅ 可以安全删除的文件
```bash
rm source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/commands/pose_command.py
```

### ⚠️ 需要同时修改
删除后需要更新 `__init__.py`:
```python
# 文件: source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/commands/__init__.py

# 删除这一行：
from .pose_command import PoseCommand, PoseCommandCfg  # ❌ 删除

# 保留这一行：
from .unified_pose_velocity_command import (  # ✅ 保留
    UnifiedPoseVelocityCommand,
    UnifiedPoseVelocityCommandCfg,
)
```

---

## 训练建议

### Stage 1: 固定姿态（当前配置）
```bash
./unitree_rl_lab.sh -t --task Unitree-Go2-VelocityPose --num_envs 4096 --max_iterations 1000
```
- 姿态指令固定: roll=0, pitch=0, height=0.35
- 速度指令变化: lin_vel_x/y ∈ [-1, 1], ang_vel_z ∈ [-1, 1]
- 目标: 学习在保持稳定姿态的同时进行运动

### Stage 2: 变化姿态（课程学习）
修改 `velocity_pose_env_cfg.py`:
```python
ranges=mdp.UnifiedPoseVelocityCommandCfg.Ranges(
    lin_vel_x=(-1.0, 1.0),
    lin_vel_y=(-1.0, 1.0),
    ang_vel_z=(-1.0, 1.0),
    roll=(-0.1, 0.1),      # ±5.7°
    pitch=(-0.1, 0.1),     # ±5.7°
    height=(0.30, 0.40),   # ±5cm
)
```
继续训练从 Stage 1 checkpoint 开始。

---

## 观测空间对比

### Velocity 任务 (45D)
```
base_ang_vel        (3D)
projected_gravity   (3D)
velocity_commands   (3D)  ← lin_vel_x, lin_vel_y, ang_vel_z
joint_pos_rel      (12D)
joint_vel_rel      (12D)
last_action        (12D)
```

### VelocityPose 任务 (50D)
```
base_ang_vel        (3D)
projected_gravity   (3D)
unified_commands    (7D)  ← lin_vel_x, lin_vel_y, ang_vel_z, roll, pitch, height
joint_pos_rel      (12D)
joint_vel_rel      (12D)
last_action        (12D)
```

**增加**: 5D (新增 roll, pitch, height, lin_vel_z 指令)

---

## 总结

✅ **当前运行状态**: 
- 任务 `Unitree-Go2-VelocityPose` 正常运行
- 使用 `UnifiedPoseVelocityCommand` 作为指令管理器
- 奖励权重完全继承自 Velocity 任务

⚠️ **待清理**: 
- `pose_command.py` 已废弃，可安全删除
- 删除后需更新 `__init__.py` 的导入语句

🎯 **设计优势**:
- 统一的指令接口，支持速度+姿态联合控制
- 保持与原始 Velocity 任务的权重一致性
- 支持课程学习，逐步扩展指令范围
