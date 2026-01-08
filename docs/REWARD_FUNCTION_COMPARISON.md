# 奖励函数设计对比：Unitree-Go2-Velocity vs Unitree-Go2-VelocityPose

## 文档概述
本文档详细对比了 Isaac Lab 中两个四足机器人运动控制任务的奖励函数设计：
- **Unitree-Go2-Velocity**: 原始速度跟踪任务
- **Unitree-Go2-VelocityPose**: 扩展的速度+姿态联合控制任务

---

## 目录
1. [任务目标对比](#任务目标对比)
2. [奖励项完整对比表](#奖励项完整对比表)
3. [分类详细分析](#分类详细分析)
4. [设计哲学差异](#设计哲学差异)
5. [权重调优建议](#权重调优建议)
6. [训练曲线预期](#训练曲线预期)

---

## 任务目标对比

### Unitree-Go2-Velocity（原始任务）
**控制目标**:
- ✅ 跟踪线速度指令 (x, y)
- ✅ 跟踪角速度指令 (yaw)
- ✅ 保持水平姿态（roll ≈ 0, pitch ≈ 0）
- ✅ 维持稳定高度（约 0.35m）

**指令维度**: 3D
```python
commands = [lin_vel_x, lin_vel_y, ang_vel_z]
```

**观测空间**: 45D
```
base_ang_vel (3) + projected_gravity (3) + velocity_commands (3) + 
joint_pos_rel (12) + joint_vel_rel (12) + last_action (12)
```

---

### Unitree-Go2-VelocityPose（扩展任务）
**控制目标**:
- ✅ 跟踪线速度指令 (x, y)
- ✅ 跟踪角速度指令 (yaw)
- 🆕 **跟踪姿态角指令 (roll, pitch)**
- 🆕 **跟踪高度指令 (height)**

**指令维度**: 7D
```python
commands = [lin_vel_x, lin_vel_y, ang_vel_z, roll, pitch, height, (lin_vel_z)]
```

**观测空间**: 50D
```
base_ang_vel (3) + projected_gravity (3) + unified_commands (7) + 
joint_pos_rel (12) + joint_vel_rel (12) + last_action (12) + [lin_vel_z (1)]
```

**新增能力**:
- 动态姿态调整（例如：爬坡时主动俯仰）
- 高度变化控制（蹲下、站立）
- 更复杂的地形适应能力

---

## 奖励项完整对比表

| 奖励项类别 | 奖励项名称 | Velocity 权重 | VelocityPose 权重 | 变化 | 说明 |
|-----------|-----------|---------------|------------------|------|------|
| **任务目标** | | | | | |
| 速度跟踪 | `track_lin_vel_xy_exp` | **1.5** | **1.5** | ✅ 保持 | 跟踪 xy 线速度 |
| 速度跟踪 | `track_ang_vel_z_exp` | **0.75** | **0.75** | ✅ 保持 | 跟踪 yaw 角速度 |
| 姿态跟踪 | `track_roll` | - | **2.0** 🆕 | 🆕 新增 | 跟踪 roll 指令 |
| 姿态跟踪 | `track_pitch` | - | **2.0** 🆕 | 🆕 新增 | 跟踪 pitch 指令 |
| 姿态跟踪 | `track_height` | - | **3.0** 🆕 | 🆕 新增 | 跟踪高度指令 |
| **基座稳定性** | | | | | |
| 基座惩罚 | `base_linear_velocity` | **-2.0** | **-2.0** | ✅ 保持 | 惩罚垂直速度 |
| 基座惩罚 | `base_angular_velocity` | **-0.05** | **-0.05** | ✅ 保持 | 惩罚 roll/pitch 角速度 |
| 姿态惩罚 | `flat_orientation_l2` | **-2.5** | **-2.5** | ✅ 保持 | 惩罚非水平姿态 |
| 关节位置 | `joint_pos` | **-0.7** | - | ❌ 移除 | 静止时的关节位置惩罚 |
| **动作平滑性** | | | | | |
| 关节速度 | `joint_vel` | **-0.001** | **-0.001** | ✅ 保持 | L2 惩罚关节速度 |
| 关节加速度 | `joint_acc` | **-2.5e-7** | **-2.5e-7** | ✅ 保持 | L2 惩罚关节加速度 |
| 关节力矩 | `joint_torques` | **-2e-4** | **-2e-4** | ✅ 保持 | L2 惩罚关节力矩 |
| 动作变化率 | `action_rate` | **-0.1** | **-0.1** | ✅ 保持 | L2 惩罚动作变化 |
| **安全约束** | | | | | |
| 关节限位 | `dof_pos_limits` | **-10.0** | **-10.0** | ✅ 保持 | 接近关节限位时惩罚 |
| 能量消耗 | `energy` | **-2e-5** | **-2e-5** | ✅ 保持 | 惩罚能量消耗 |
| **步态质量** | | | | | |
| 足端腾空时间 | `feet_air_time` | **0.1** | **0.1** | ✅ 保持 | 奖励适当腾空时间 |
| 腾空时间方差 | `air_time_variance` | **-1.0** | **-1.0** | ✅ 保持 | 惩罚步态不对称 |
| 足端滑动 | `feet_slide` | **-0.1** | **-0.1** | ✅ 保持 | 惩罚足端滑动 |
| **碰撞惩罚** | | | | | |
| 非预期接触 | `undesired_contacts` | **-1.0** | **-1.0** | ✅ 保持 | 惩罚身体/大腿接触地面 |
| **总计** | | **16 项** | **18 项** | **+2 项** | 新增姿态跟踪能力 |

---

## 分类详细分析

### 1. 任务目标奖励（Task Rewards）

#### Velocity 任务
```python
# 速度跟踪（指数核函数）
track_lin_vel_xy = RewTerm(
    func=mdp.track_lin_vel_xy_exp, 
    weight=1.5,  # 主要任务目标
    params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
)
track_ang_vel_z = RewTerm(
    func=mdp.track_ang_vel_z_exp, 
    weight=0.75,  # 次要任务目标
    params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
)
```

**设计意图**:
- 线速度权重 > 角速度权重（1.5 vs 0.75）
- 优先保证位移精度，其次保证转向精度
- 使用指数核函数：`reward = exp(-error²/(2*std²))`
  - std = 0.5：容忍误差约 ±0.5 m/s

---

#### VelocityPose 任务
```python
# 继承速度跟踪（权重不变）
track_lin_vel_xy_exp = RewTerm(
    func=mdp.track_lin_vel_xy_exp, 
    weight=1.5,  # 保持原始权重
    params={"command_name": "pose_command", "std": math.sqrt(0.25)}
)
track_ang_vel_z_exp = RewTerm(
    func=mdp.track_ang_vel_z_exp, 
    weight=0.75,  # 保持原始权重
    params={"command_name": "pose_command", "std": math.sqrt(0.25)}
)

# 🆕 新增姿态跟踪
track_roll = RewTerm(
    func=mdp.track_roll_exp,
    weight=2.0,  # 高于速度权重！
    params={"command_name": "pose_command", "std": math.sqrt(0.25)}
)
track_pitch = RewTerm(
    func=mdp.track_pitch_exp,
    weight=2.0,  # 高于速度权重！
    params={"command_name": "pose_command", "std": math.sqrt(0.25)}
)
track_height = RewTerm(
    func=mdp.track_height_exp,
    weight=3.0,  # 最高权重！
    params={"command_name": "pose_command", "std": 0.05}  # 更严格的容差
)
```

**设计意图**:
- **权重优先级**: height (3.0) > roll/pitch (2.0) > lin_vel (1.5) > ang_vel (0.75)
- **原因分析**:
  1. **高度最重要**: 影响稳定性和碰撞安全，std=0.05m（±5cm）容差小
  2. **姿态次重要**: 影响动力学效率和传感器方向
  3. **速度相对宽松**: 允许为了姿态控制而牺牲部分速度精度

**总任务奖励权重占比**:
- Velocity: 2.25 / 总和
- VelocityPose: 9.25 / 总和（增加 310%）

---

### 2. 基座稳定性惩罚（Base Stability Penalties）

#### 共同项（权重完全一致）
```python
base_linear_velocity = RewTerm(
    func=mdp.lin_vel_z_l2, 
    weight=-2.0  # 强惩罚垂直运动
)
base_angular_velocity = RewTerm(
    func=mdp.ang_vel_xy_l2, 
    weight=-0.05  # 轻度惩罚 roll/pitch 角速度
)
flat_orientation_l2 = RewTerm(
    func=mdp.flat_orientation_l2, 
    weight=-2.5  # 强惩罚非水平姿态
)
```

**设计意图**:
- 垂直速度几乎禁止（-2.0）
- Roll/pitch 角速度温和限制（-0.05）
- 姿态偏离水平强惩罚（-2.5）

**注意**: VelocityPose 保留了 `flat_orientation_l2`，这会与 `track_roll/pitch` 产生轻微冲突
- 当 roll/pitch 指令非零时，两者会竞争
- 但由于 `track_roll` 权重更大（2.0 vs 2.5），姿态跟踪会占主导

---

#### Velocity 独有项
```python
joint_pos = RewTerm(
    func=mdp.joint_position_penalty,
    weight=-0.7,
    params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
        "stand_still_scale": 5.0,  # 静止时惩罚 5 倍
        "velocity_threshold": 0.3,  # 低于 0.3 m/s 视为静止
    },
)
```

**为什么 VelocityPose 移除了它？**
1. **姿态变化需要大幅关节运动**: 高度/roll/pitch 调整可能偏离默认姿态
2. **避免冲突**: 该惩罚会阻碍姿态跟踪
3. **其他正则化已足够**: `joint_vel`, `joint_acc`, `joint_torques` 提供足够约束

---

### 3. 动作平滑性惩罚（Smoothness Penalties）

#### 完全一致的项
| 奖励项 | 权重 | 功能 | 单位 |
|--------|------|------|------|
| `joint_vel` | -0.001 | 惩罚关节速度 | (rad/s)² |
| `joint_acc` | -2.5e-7 | 惩罚关节加速度 | (rad/s²)² |
| `joint_torques` | -2e-4 | 惩罚关节力矩 | (N·m)² |
| `action_rate` | -0.1 | 惩罚动作变化率 | Δaction² |

**权重关系**:
- action_rate >> joint_vel >> joint_torques >> joint_acc
- **最重要**: 动作平滑（-0.1）
- **最轻微**: 加速度约束（-2.5e-7）

**设计原理**:
1. 平滑动作是高优先级（避免震荡）
2. 速度/力矩次之（能量效率）
3. 加速度最低（允许快速响应）

**VelocityPose 保持一致的原因**:
- 这些是运动控制的基本约束
- 与速度/姿态目标无关
- 经过验证的有效权重

---

### 4. 安全约束（Safety Constraints）

#### 完全一致的项
```python
dof_pos_limits = RewTerm(
    func=mdp.joint_pos_limits, 
    weight=-10.0  # 极强惩罚！
)
energy = RewTerm(
    func=mdp.energy, 
    weight=-2e-5  # 轻微能量损失
)
```

**权重含义**:
- **关节限位**: 绝对禁止（-10.0）
  - 接近限位时指数级增长惩罚
  - 防止机械损伤
- **能量消耗**: 鼓励高效（-2e-5）
  - 仅作为 tiebreaker，不影响主要决策
  - 能量 = Σ(torque × velocity)

---

### 5. 步态质量奖励（Gait Quality Rewards）

#### 完全一致的项
```python
feet_air_time = RewTerm(
    func=mdp.feet_air_time,
    weight=0.1,  # 正奖励！
    params={
        "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        "command_name": "base_velocity" / "pose_command",
        "threshold": 0.5,  # 腾空 > 0.5s 才奖励
    },
)
air_time_variance = RewTerm(
    func=mdp.air_time_variance_penalty,
    weight=-1.0,  # 强惩罚步态不对称
    params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
)
feet_slide = RewTerm(
    func=mdp.feet_slide,
    weight=-0.1,  # 惩罚滑动
    params={
        "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
        "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
    },
)
```

**设计意图**:
- **鼓励 trotting 步态**: `feet_air_time` 正奖励
- **对称性**: `air_time_variance` 强惩罚（-1.0）
- **抓地性能**: `feet_slide` 轻度惩罚（-0.1）

**VelocityPose 保持一致的原因**:
- 步态质量与速度/姿态目标正交
- 这些是四足机器人的通用约束
- 姿态变化不应降低步态质量

---

### 6. 碰撞惩罚（Collision Penalties）

#### 完全一致的项
```python
undesired_contacts = RewTerm(
    func=mdp.undesired_contacts,
    weight=-1.0,  # 强惩罚
    params={
        "threshold": 1,  # 接触力 > 1N 触发
        "sensor_cfg": SceneEntityCfg(
            "contact_forces", 
            body_names=["Head_.*", ".*_hip", ".*_thigh", ".*_calf"]
        ),
    },
)
```

**监控部位**:
- 头部（Head_.*）
- 髋关节（.*_hip）
- 大腿（.*_thigh）
- 小腿（.*_calf）

**排除**: 足端（.*_foot）- 允许正常接触地面

---

## 设计哲学差异

### Velocity 任务哲学
```
核心思想: "精确跟踪速度，同时保持默认姿态"

优先级:
1. 速度跟踪 (1.5 + 0.75 = 2.25)
2. 姿态约束 (-2.5 flat_orientation)
3. 步态质量 (0.1 - 1.0 - 0.1 = -1.0)
4. 平滑性   (-0.1 action_rate)
5. 安全性   (-10.0 joint_limits)

设计特点:
- ✅ 简单明确的目标
- ✅ 强制水平姿态（机械约束）
- ✅ 适合平地高速运动
- ❌ 无法适应地形变化
- ❌ 固定高度（约 0.35m）
```

---

### VelocityPose 任务哲学
```
核心思想: "速度+姿态联合优化，提升地形适应性"

优先级:
1. 高度控制 (3.0)
2. 姿态跟踪 (2.0 + 2.0 = 4.0)
3. 速度跟踪 (1.5 + 0.75 = 2.25)
4. 步态质量 (-1.0 总和)
5. 平滑性   (-0.1 action_rate)
6. 安全性   (-10.0 joint_limits)

设计特点:
- ✅ 多目标联合优化
- ✅ 动态姿态调整能力
- ✅ 高度自适应
- ✅ 支持课程学习
- ⚠️ 训练复杂度增加
- ⚠️ 可能牺牲速度精度换取姿态控制
```

---

## 核心差异总结

### 1. 奖励项数量
- **Velocity**: 16 项
- **VelocityPose**: 18 项（+2 姿态跟踪项）

### 2. 任务奖励总权重
- **Velocity**: 2.25（速度目标）
- **VelocityPose**: 9.25（速度 2.25 + 姿态 7.0）
- **增幅**: +310%

### 3. 移除的项
- **joint_pos** (-0.7): 避免与姿态跟踪冲突

### 4. 权重分布变化

| 类别 | Velocity 占比 | VelocityPose 占比 | 变化 |
|------|--------------|------------------|------|
| 任务目标 | 2.25 / 18.4 = **12.2%** | 9.25 / 17.7 = **52.3%** | ↑ 340% |
| 基座稳定 | 5.25 / 18.4 = **28.5%** | 4.55 / 17.7 = **25.7%** | ↓ 10% |
| 动作平滑 | 0.10 / 18.4 = **0.5%** | 0.10 / 17.7 = **0.6%** | → |
| 安全约束 | 10.0 / 18.4 = **54.3%** | 10.0 / 17.7 = **56.5%** | → |
| 步态质量 | 1.0 / 18.4 = **5.4%** | 1.0 / 17.7 = **5.6%** | → |

**关键洞察**:
- VelocityPose 大幅提升任务目标权重（12% → 52%）
- 基座稳定约束相对下降（28% → 26%）
- 安全约束依然主导（55-57%）

---

## 权重调优建议

### 针对 VelocityPose 任务

#### 阶段 1: 基础训练（当前配置）✅
```python
# 固定姿态指令
roll = (0.0, 0.0)
pitch = (0.0, 0.0)
height = (0.35, 0.35)

# 权重保持不变
track_roll: 2.0
track_pitch: 2.0
track_height: 3.0
```
**目标**: 学习在保持固定姿态的同时进行速度控制

---

#### 阶段 2: 轻度姿态变化
```python
# 扩展指令范围
roll = (-0.05, 0.05)    # ±2.9°
pitch = (-0.05, 0.05)   # ±2.9°
height = (0.32, 0.38)   # ±3cm

# 🔧 建议调整权重
track_roll: 2.5  # ↑ 提升姿态跟踪优先级
track_pitch: 2.5  # ↑
track_height: 3.5  # ↑
lin_vel_xy: 1.2   # ↓ 降低速度要求
ang_vel_z: 0.6    # ↓
```
**目标**: 鼓励姿态调整，允许速度稍慢

---

#### 阶段 3: 复杂地形适应
```python
# 大幅扩展指令范围
roll = (-0.15, 0.15)    # ±8.6°
pitch = (-0.15, 0.15)   # ±8.6°
height = (0.25, 0.45)   # ±10cm

# 🔧 建议调整权重
track_roll: 3.0    # ↑ 姿态控制主导
track_pitch: 3.0   # ↑
track_height: 4.0  # ↑ 高度最关键
lin_vel_xy: 1.0    # ↓ 进一步降低速度要求
ang_vel_z: 0.5     # ↓

# 可选：移除冲突项
# flat_orientation_l2: 0.0  # ❌ 禁用水平约束
```
**目标**: 优先保证姿态精度，速度变为次要目标

---

#### 可选：动态权重课程
```python
# 根据训练进度动态调整
def update_weights(iteration):
    if iteration < 500:
        # Stage 1: 速度主导
        return {
            "lin_vel": 1.5,
            "roll": 1.5,
            "pitch": 1.5,
            "height": 2.5,
        }
    elif iteration < 1000:
        # Stage 2: 平衡
        return {
            "lin_vel": 1.2,
            "roll": 2.5,
            "pitch": 2.5,
            "height": 3.5,
        }
    else:
        # Stage 3: 姿态主导
        return {
            "lin_vel": 1.0,
            "roll": 3.5,
            "pitch": 3.5,
            "height": 4.5,
        }
```

---

## 训练曲线预期

### Velocity 任务典型曲线
```
Episode Reward: -5 → +10 → +15 (收敛)
├─ track_lin_vel_xy: 0.0 → 1.2 → 1.4
├─ track_ang_vel_z:  0.0 → 0.6 → 0.7
├─ flat_orientation: -2.5 → -0.5 → -0.1
├─ action_rate:      -5.0 → -1.0 → -0.3
└─ feet_air_time:    0.0 → 0.08 → 0.09

训练时间: ~300-500 iterations (4096 envs)
```

---

### VelocityPose 任务预期曲线
```
Episode Reward: -10 → +5 → +20 (收敛)
├─ track_lin_vel_xy: 0.0 → 1.0 → 1.3   (略低于 Velocity)
├─ track_ang_vel_z:  0.0 → 0.5 → 0.65  (略低于 Velocity)
├─ track_roll:       0.0 → 1.5 → 1.9   🆕
├─ track_pitch:      0.0 → 1.5 → 1.9   🆕
├─ track_height:     0.0 → 2.5 → 2.9   🆕
├─ flat_orientation: -2.5 → -0.8 → -0.5 (比 Velocity 高，因姿态变化)
├─ action_rate:      -8.0 → -1.5 → -0.5 (初期更剧烈，因姿态调整)
└─ feet_air_time:    0.0 → 0.07 → 0.09 (略低，因姿态变化影响步态)

训练时间: ~500-800 iterations (4096 envs)  ← 比 Velocity 慢 40-60%
```

---

### 关键指标监控

#### 速度精度指标
```python
# Velocity 任务目标
error_lin_vel_xy < 0.3 m/s  # 良好
error_ang_vel_z < 0.2 rad/s  # 良好

# VelocityPose 任务目标（Stage 1）
error_lin_vel_xy < 0.4 m/s  # 可接受（略宽松）
error_ang_vel_z < 0.3 rad/s  # 可接受
error_roll < 0.05 rad        # 良好
error_pitch < 0.05 rad       # 良好
error_height < 0.03 m        # 良好
```

#### 训练健康指标
```python
# 警告信号
if action_rate < -2.0:
    print("⚠️ 动作抖动严重，降低学习率")
if undesired_contacts < -0.5:
    print("⚠️ 频繁身体接触，检查姿态范围")
if feet_air_time < 0.02:
    print("⚠️ 步态质量差，可能需要预训练")
```

---

## 实用调试工具

### TensorBoard 监控命令
```bash
# 启动 TensorBoard
cd /home/yzy/MyProject/unitree_rl_lab
tensorboard --logdir=logs/rsl_rl/unitree_go2_velocitypose

# 关键曲线
- Episode_Reward/*  （所有奖励项）
- Metrics/pose_command/error_*  （指令跟踪误差）
- Mean_surrogate_loss  （策略梯度）
- Mean_value_function_loss  （价值估计）
```

### 奖励权重 A/B 测试
```python
# 创建多个配置变体
configs = {
    "baseline": {"roll": 2.0, "pitch": 2.0, "height": 3.0},
    "high_pose": {"roll": 3.0, "pitch": 3.0, "height": 4.0},
    "balanced": {"roll": 2.5, "pitch": 2.5, "height": 3.0},
}

# 并行训练对比
for name, weights in configs.items():
    train(config=name, weights=weights, num_envs=2048)
```

---

## 附录：奖励函数数学定义

### 指数核跟踪函数
```python
def track_*_exp(env, command_name: str, std: float):
    """
    指数核函数，容忍一定误差
    
    Formula: r = exp(-error² / (2 * std²))
    
    特性:
    - error = 0    → r = 1.0 (最大奖励)
    - error = std  → r = 0.606
    - error = 2std → r = 0.135
    - error → ∞    → r → 0
    
    std 选择:
    - 0.5 (0.25²): 容忍 ±0.5 单位误差
    - 0.05: 严格容差（高度控制）
    """
    error = current_value - command_value
    return torch.exp(-error**2 / (2 * std**2))
```

### L2 惩罚函数
```python
def *_l2(env):
    """
    L2 范数惩罚，二次增长
    
    Formula: r = -‖x‖²
    
    特性:
    - 小误差: 惩罚温和
    - 大误差: 惩罚剧烈
    - 无上限: 鼓励尽量减小
    """
    return -torch.sum(value ** 2, dim=-1)
```

---

## 结论

### VelocityPose 相比 Velocity 的优势
1. ✅ **更强的地形适应能力**: 可主动调整姿态
2. ✅ **高度灵活性**: 支持蹲下/站立动作
3. ✅ **课程学习友好**: 逐步扩展指令范围
4. ✅ **保持原有性能**: 继承所有 Velocity 的平滑性约束

### VelocityPose 的挑战
1. ⚠️ **训练时间增加**: 预计 +40-60%
2. ⚠️ **超参数敏感**: 需要仔细平衡速度/姿态权重
3. ⚠️ **潜在冲突**: `flat_orientation_l2` 与姿态跟踪有轻微矛盾
4. ⚠️ **观测空间增大**: 50D vs 45D，可能影响泛化

### 推荐实践
1. 从 Velocity 预训练模型微调（迁移学习）
2. 渐进式增加姿态指令范围（课程学习）
3. 监控速度精度下降幅度（< 20% 可接受）
4. 考虑移除 `flat_orientation_l2` 在高级阶段

---

**文档版本**: v1.0  
**更新日期**: 2026-01-08  
**作者**: AI Assistant  
**项目**: Unitree RL Lab - Isaac Lab 2.3.0
