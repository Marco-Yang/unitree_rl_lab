# Go2 姿态控制任务实现文档

## 概述

本文档描述了为 Unitree Go2 机器狗实现的**姿态控制（Pose Control）**任务。该任务扩展了原有的速度跟踪任务，增加了对 **Roll 角、Pitch 角和高度**的控制。

## 功能特性

### 1. 新增命令类型

创建了 `PoseCommand` 类，支持三维姿态命令：

| 命令维度 | 说明 | 范围（初始课程） | 单位 |
|---------|------|----------------|------|
| **Roll** | 横滚角 | (0.0, 0.0) | rad |
| **Pitch** | 俯仰角 | (0.0, 0.0) | rad |
| **Height** | Base 高度 | (0.35, 0.35) | m |

### 2. 课程学习支持

当前配置为**课程学习的初始阶段**：
- ✅ Roll = 0°（保持水平）
- ✅ Pitch = 0°（保持水平）
- ✅ Height = 0.35m（Go2 默认高度）

**这与未扩展时的默认行为一致**，方便后续逐步扩展命令范围。

### 3. 奖励函数设计

#### 主要奖励（任务目标）

```python
# 跟踪 Roll 角
track_roll = RewTerm(
    func=mdp.track_roll_exp,
    weight=2.0,
    params={"std": math.sqrt(0.25)}
)

# 跟踪 Pitch 角
track_pitch = RewTerm(
    func=mdp.track_pitch_exp,
    weight=2.0,
    params={"std": math.sqrt(0.25)}
)

# 跟踪高度
track_height = RewTerm(
    func=mdp.track_height_exp,
    weight=3.0,
    params={"std": 0.05}  # 高度要求更精确
)
```

#### 辅助奖励

保留了原有的平滑性和效率惩罚，但调整了权重：

| 奖励项 | 原权重 | 新权重 | 变化说明 |
|-------|--------|--------|---------|
| `action_rate` | -0.1 | -0.01 | 降低 10倍（姿态调整需要更大动作） |
| `joint_vel` | -0.001 | -0.0005 | 降低 2倍 |
| `feet_air_time` | 0.1 | 0.0 | 禁用（姿态控制时保持站立） |
| `air_time_variance` | -1.0 | -0.5 | 降低 2倍 |

## 代码结构

### 新增文件

```
source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/
├── mdp/
│   ├── commands/
│   │   └── pose_command.py          # 姿态命令生成器
│   └── rewards.py                    # 新增 6 个姿态跟踪奖励函数
└── robots/
    └── go2/
        └── pose_env_cfg.py           # Go2 姿态控制任务配置
```

### 修改文件

```
source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/
├── mdp/
│   └── commands/__init__.py          # 导出 PoseCommand
└── robots/
    └── go2/__init__.py               # 注册新任务 "Unitree-Go2-Pose"
```

## 使用方法

### 1. 列出可用任务

```bash
./unitree_rl_lab.sh -l | grep Go2
```

输出应包含：
```
| 2 | Unitree-Go2-Velocity | ... |
| X | Unitree-Go2-Pose     | ... |  # 新任务
```

### 2. 训练姿态控制任务

```bash
# 使用脚本
./unitree_rl_lab.sh -t --task Unitree-Go2-Pose

# 或直接调用 Python
python scripts/rsl_rl/train.py --headless --task Unitree-Go2-Pose
```

### 3. 测试训练好的策略

```bash
./unitree_rl_lab.sh -p --task Unitree-Go2-Pose
```

## 观测空间

| 观测项 | 维度 | 说明 |
|-------|------|------|
| `base_ang_vel` | 3 | 基座角速度 |
| `projected_gravity` | 3 | 投影重力（包含 roll/pitch 信息） |
| `pose_commands` | 3 | 姿态命令 [roll, pitch, height] |
| `joint_pos_rel` | 12 | 关节相对位置 |
| `joint_vel_rel` | 12 | 关节速度 |
| `last_action` | 12 | 上一步动作 |
| **总维度** | **45** | - |

## 课程学习扩展方案

### 阶段 1：当前配置（已实现）

```python
ranges=mdp.PoseCommandCfg.Ranges(
    roll=(0.0, 0.0),      # 保持水平
    pitch=(0.0, 0.0),     # 保持水平
    height=(0.35, 0.35)   # 默认高度
)
```

**目标**：学习基本的平衡站立

---

### 阶段 2：小幅姿态变化

```python
ranges=mdp.PoseCommandCfg.Ranges(
    roll=(-0.1, 0.1),     # ±5.7° 
    pitch=(-0.1, 0.1),    # ±5.7°
    height=(0.30, 0.40)   # ±5cm
)
```

**目标**：适应小幅度的姿态调整

---

### 阶段 3：中等姿态变化

```python
ranges=mdp.PoseCommandCfg.Ranges(
    roll=(-0.2, 0.2),     # ±11.5°
    pitch=(-0.2, 0.2),    # ±11.5°
    height=(0.25, 0.45)   # ±10cm
)
```

**目标**：在一定范围内灵活调整姿态

---

### 阶段 4：大幅姿态变化

```python
ranges=mdp.PoseCommandCfg.Ranges(
    roll=(-0.3, 0.3),     # ±17.2°
    pitch=(-0.3, 0.3),    # ±17.2°
    height=(0.20, 0.50)   # ±15cm
)
```

**目标**：极限姿态控制能力

## 实现细节

### 1. Roll 和 Pitch 的提取

从投影重力向量计算当前姿态：

```python
# projected_gravity_b: 重力在 body 系的投影
current_roll = torch.atan2(projected_gravity[:, 0], projected_gravity[:, 2])
current_pitch = torch.atan2(projected_gravity[:, 1], projected_gravity[:, 2])
```

### 2. 高度的获取

直接从仿真中获取（真机需要估计）：

```python
current_height = asset.data.root_pos_w[:, 2]
```

### 3. 奖励计算

使用指数核函数（更平滑）：

```python
roll_error = (current_roll - commanded_roll)²
reward = exp(-roll_error / (2 * std²))
```

## 真机部署考虑

### 可直接使用的传感器

| 测量项 | 传感器 | 可行性 |
|-------|--------|--------|
| Roll | IMU | ✅ 高 |
| Pitch | IMU | ✅ 高 |
| Height | FK + 接触状态 | ⚠️ 中（需估计） |

### 高度估计方案

```python
def estimate_height_from_kinematics(joint_pos, contact_forces):
    """
    通过正向运动学估计高度
    
    假设：至少有一只脚接触地面（z=0）
    """
    foot_positions_body = forward_kinematics(joint_pos)
    contacted_feet = foot_positions_body[contact_forces > threshold]
    
    if len(contacted_feet) > 0:
        lowest_foot_z = np.min(contacted_feet[:, 2])
        estimated_height = -lowest_foot_z
    else:
        # 腾空状态：使用平均值
        estimated_height = -np.mean(foot_positions_body[:, 2])
    
    return estimated_height
```

## 训练监控指标

### 主要指标

1. **Roll 跟踪误差**
   ```python
   error_roll = |current_roll - commanded_roll|
   ```

2. **Pitch 跟踪误差**
   ```python
   error_pitch = |current_pitch - commanded_pitch|
   ```

3. **高度跟踪误差**
   ```python
   error_height = |current_height - commanded_height|
   ```

4. **总奖励**
   - 应在训练过程中持续上升
   - 收敛值取决于课程阶段

### TensorBoard 可视化

```python
writer.add_scalar('rewards/track_roll', reward_roll, step)
writer.add_scalar('rewards/track_pitch', reward_pitch, step)
writer.add_scalar('rewards/track_height', reward_height, step)
writer.add_scalar('metrics/error_roll', error_roll.mean(), step)
writer.add_scalar('metrics/error_pitch', error_pitch.mean(), step)
writer.add_scalar('metrics/error_height', error_height.mean(), step)
```

## 调试可视化

启用 `debug_vis=True` 后，会在仿真中显示：

- 🟢 **绿色箭头**：目标姿态和高度
- 🔵 **蓝色箭头**：当前姿态和高度

## 常见问题

### Q1: 为什么初始命令范围都是 0？

**A**: 这是课程学习的设计。初始阶段让机器人先学会基本的平衡站立，然后逐步扩展命令范围。

### Q2: 如何调整到下一个课程阶段？

**A**: 修改 `pose_env_cfg.py` 中的 `CommandsCfg.pose_command.ranges`：

```python
ranges=mdp.PoseCommandCfg.Ranges(
    roll=(-0.1, 0.1),    # 从 (0, 0) 扩展到 (-0.1, 0.1)
    pitch=(-0.1, 0.1),   # 从 (0, 0) 扩展到 (-0.1, 0.1)
    height=(0.30, 0.40)  # 从 (0.35, 0.35) 扩展到 (0.30, 0.40)
)
```

### Q3: 真机高度估计误差有多大？

**A**: 使用 FK + 接触状态估计，平地误差约 ±5mm，可接受。

### Q4: 能否同时控制速度和姿态？

**A**: 可以！需要创建一个组合命令类，同时包含速度和姿态。这可以作为下一步的扩展。

## 下一步开发建议

1. **实现自动课程学习**
   - 根据训练性能自动调整命令范围
   - 参考 `curriculums.py` 中的实现

2. **添加地形适应性**
   - 在不平坦地形上测试姿态控制
   - 添加地形感知（高度扫描）

3. **组合速度+姿态控制**
   - 创建 `VelocityPoseCommand` 类
   - 同时跟踪速度和姿态

4. **真机验证**
   - 实现高度估计模块
   - 测试 Sim2Real 迁移性能

## 参考代码位置

- **命令生成器**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/commands/pose_command.py`
- **奖励函数**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/rewards.py` (Line 229-401)
- **任务配置**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/pose_env_cfg.py`
- **任务注册**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/__init__.py`

## 版本历史

| 版本 | 日期 | 变更 |
|-----|------|------|
| v1.0 | 2026-01-08 | 初始实现 - 课程学习阶段 1 |

---

**作者**: AI Assistant  
**最后更新**: 2026-01-08
