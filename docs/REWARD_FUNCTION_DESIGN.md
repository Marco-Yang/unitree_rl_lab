# Unitree Go2 速度跟踪任务奖励函数设计文档

## 📋 目录
- [概述](#概述)
- [任务目标](#任务目标)
- [奖励函数架构](#奖励函数架构)
- [详细奖励项说明](#详细奖励项说明)
- [真机部署考虑](#真机部署考虑)
- [参数调优建议](#参数调优建议)

---

## 概述

本文档详细说明了 Unitree Go2 机器狗在速度跟踪任务（Velocity Tracking Task）中的奖励函数设计。该任务的目标是让机器狗根据速度命令进行平滑、高效且稳定的四足运动。

**环境配置文件**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/velocity_env_cfg.py`  
**奖励函数实现**: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/rewards.py`

---

## 任务目标

### 主要目标
- ✅ **速度跟踪**: 准确跟踪给定的线速度（x, y）和角速度（z）命令
- ✅ **稳定运动**: 保持机身姿态稳定，避免翻倒
- ✅ **能量效率**: 最小化能量消耗和不必要的动作
- ✅ **自然步态**: 产生类似真实四足动物的步态模式

### 次要目标
- 避免关节限位
- 减少足端打滑
- 防止非足端部位接触地面
- 保持平滑的动作输出

---

## 奖励函数架构

奖励函数采用**加权线性组合**的方式，由多个子奖励项构成：

```
Total Reward = Σ (weight_i × reward_term_i)
```

总共包含 **18 个奖励项**，分为 5 大类：

| 类别 | 奖励项数量 | 主要作用 |
|------|-----------|---------|
| **任务奖励** | 2 | 速度跟踪 |
| **基础惩罚** | 8 | 平滑性和效率 |
| **机器人姿态** | 2 | 姿态稳定性 |
| **足端奖励** | 4 | 步态和接触控制 |
| **其他惩罚** | 1 | 非期望接触 |

---

## 详细奖励项说明

### 🎯 1. 任务奖励（Task Rewards）

#### 1.1 线速度跟踪 (`track_lin_vel_xy`)

**权重**: `1.5` ✨ **最高权重**

**函数**: `mdp.track_lin_vel_xy_exp`

**目的**: 奖励机器人在 xy 平面上的速度接近命令速度

**数学表达式**:
```python
error = ||v_commanded_xy - v_actual_xy||²
reward = exp(-error / (2 * std²))
```
其中 `std = sqrt(0.25) = 0.5`

**特点**:
- 使用高斯核函数（指数函数），误差越小奖励越高
- 在速度命令为 0 时也有效（站立不动）

**真机可行性**: ✅ 高（通过 IMU 和状态估计器可获取）

---

#### 1.2 角速度跟踪 (`track_ang_vel_z`)

**权重**: `0.75`

**函数**: `mdp.track_ang_vel_z_exp`

**目的**: 奖励机器人的偏航角速度接近命令角速度

**数学表达式**:
```python
error = (ω_commanded_z - ω_actual_z)²
reward = exp(-error / (2 * std²))
```
其中 `std = sqrt(0.25) = 0.5`

**真机可行性**: ✅ 高（IMU 直接提供）

---

### ⚙️ 2. 基础惩罚（Base Penalties）

#### 2.1 垂直线速度惩罚 (`base_linear_velocity`)

**权重**: `-2.0` ⚠️ **强惩罚**

**函数**: `mdp.lin_vel_z_l2`

**目的**: 惩罚 z 轴方向的速度（防止跳跃或快速下落）

**数学表达式**:
```python
penalty = v_z²
```

**原因**: 四足动物在行走时基座应保持相对稳定的高度

**真机可行性**: ✅ 高

---

#### 2.2 基座角速度惩罚 (`base_angular_velocity`)

**权重**: `-0.05`

**函数**: `mdp.ang_vel_xy_l2`

**目的**: 惩罚 roll 和 pitch 方向的角速度（保持姿态稳定）

**数学表达式**:
```python
penalty = ω_roll² + ω_pitch²
```

**注意**: 仅惩罚 xy 平面角速度，不惩罚 yaw（偏航需要控制）

**真机可行性**: ✅ 高

---

#### 2.3 关节速度惩罚 (`joint_vel`)

**权重**: `-0.001`

**函数**: `mdp.joint_vel_l2`

**目的**: 惩罚过大的关节速度

**数学表达式**:
```python
penalty = Σ q̇ᵢ²
```

**原因**: 减少高频抖动，保护硬件

**真机可行性**: ✅ 高

---

#### 2.4 关节加速度惩罚 (`joint_acc`)

**权重**: `-2.5e-7` （非常小）

**函数**: `mdp.joint_acc_l2`

**目的**: 惩罚关节加速度，促进平滑运动

**数学表达式**:
```python
penalty = Σ q̈ᵢ²
```

**真机可行性**: ⚠️ 中等（需要数值微分或卡尔曼滤波）

---

#### 2.5 关节力矩惩罚 (`joint_torques`)

**权重**: `-2e-4`

**函数**: `mdp.joint_torques_l2`

**目的**: 惩罚过大的电机输出力矩

**数学表达式**:
```python
penalty = Σ τᵢ²
```

**原因**: 减少能量消耗和电机磨损

**真机可行性**: ✅ 高（电机反馈）

---

#### 2.6 动作变化率惩罚 (`action_rate`)

**权重**: `-0.1` ⚠️ **较强惩罚**

**函数**: `mdp.action_rate_l2`

**目的**: 惩罚相邻时间步动作的剧烈变化

**数学表达式**:
```python
penalty = ||aₜ - aₜ₋₁||²
```

**原因**: 促进策略输出平滑，避免高频震荡

**真机可行性**: ✅ 高

---

#### 2.7 关节限位惩罚 (`dof_pos_limits`)

**权重**: `-10.0` 🚨 **极强惩罚**

**函数**: `mdp.joint_pos_limits`

**目的**: 防止关节超出物理限位

**特点**:
- 在接近限位时给予强烈惩罚
- 保护硬件安全

**真机可行性**: ✅ 高

---

#### 2.8 能量消耗惩罚 (`energy`)

**权重**: `-2e-5`

**函数**: `mdp.energy`

**目的**: 惩罚能量消耗

**数学表达式**:
```python
penalty = Σ |q̇ᵢ| × |τᵢ|
```

**原因**: 
- 机械功率 = 速度 × 力矩
- 鼓励能量高效的运动模式

**真机可行性**: ✅ 高

---

### 🤖 3. 机器人姿态奖励（Robot Orientation Rewards）

#### 3.1 水平姿态惩罚 (`flat_orientation_l2`)

**权重**: `-2.5` ⚠️ **强惩罚**

**函数**: `mdp.flat_orientation_l2`

**目的**: 惩罚机身倾斜，保持水平姿态

**数学表达式**:
```python
# 投影重力在 body 系的 z 分量应接近 -1（向下）
penalty = 1 - |projected_gravity_z|
```

**原因**: 四足机器人在行走时应保持基座接近水平

**真机可行性**: ✅ 高（IMU）

---

#### 3.2 关节位置惩罚 (`joint_pos`)

**权重**: `-0.7`

**函数**: `mdp.joint_position_penalty`

**目的**: 惩罚关节偏离默认位置

**特殊逻辑**:
```python
if velocity_cmd > 0 or body_velocity > 0.3:
    penalty = ||q - q_default||
else:  # 站立时
    penalty = 5.0 × ||q - q_default||  # 加强 5 倍
```

**原因**: 
- 运动时：允许一定偏离
- 静止时：强制回到默认姿态

**真机可行性**: ✅ 高

---

### 🦶 4. 足端奖励（Feet Rewards）

#### 4.1 腾空时间奖励 (`feet_air_time`)

**权重**: `0.1` ✨ **正向奖励**

**函数**: `mdp.feet_air_time`

**目的**: 奖励足端在空中停留合理时间（促进 trotting 步态）

**参数**:
- `threshold`: 0.5 秒（只有腾空时间 > 0.5s 才给奖励）

**原因**: 避免拖脚行走，鼓励清晰的 swing-stance 相位

**真机可行性**: ✅ 高（足端力传感器）

---

#### 4.2 腾空时间方差惩罚 (`air_time_variance`)

**权重**: `-1.0` ⚠️ **强惩罚**

**函数**: `mdp.air_time_variance_penalty`

**目的**: 惩罚各足腾空时间的不一致性

**数学表达式**:
```python
penalty = Var(air_time_foot1, air_time_foot2, ...) + 
          Var(contact_time_foot1, contact_time_foot2, ...)
```

**原因**: 促进对称且规律的步态

**真机可行性**: ✅ 高

---

#### 4.3 足端滑动惩罚 (`feet_slide`)

**权重**: `-0.1`

**函数**: `mdp.feet_slide`

**目的**: 惩罚足端在接触地面时滑动

**数学表达式**:
```python
if is_contact:
    penalty = ||v_foot_xy||  # 足端水平速度
```

**原因**: 
- 打滑会导致能量浪费
- 影响运动稳定性

**真机可行性**: ⚠️ 中等（需要 FK + 接触状态）

---

#### 4.4 足端碰撞惩罚 (`undesired_contacts`)

**权重**: `-1.0` ⚠️ **强惩罚**

**函数**: `mdp.undesired_contacts`

**目的**: 惩罚非足端部位接触地面

**监测部位**:
- 头部（`Head_.*`）
- 髋关节（`.*_hip`）
- 大腿（`.*_thigh`）
- 小腿（`.*_calf`）

**原因**: 这些部位接触意味着姿态失败或即将摔倒

**真机可行性**: ⚠️ 中等（需要额外接触传感器或基于力矩异常判断）

---

### ❌ 5. 被注释掉的奖励项

#### 5.1 足端接触力惩罚 (`feet_contact_forces`)

```python
# feet_contact_forces = RewTerm(
#     func=mdp.contact_forces,
#     weight=-0.02,
#     params={"threshold": 100.0, ...},
# )
```

**为何禁用**: 可能会导致机器人不敢用力踩地，影响稳定性

---

## 📊 奖励权重分布

### 权重绝对值排序（Top 10）

| 排名 | 奖励项 | 权重 | 类型 | 重要性 |
|-----|-------|------|------|--------|
| 1 | `dof_pos_limits` | -10.0 | 惩罚 | 🚨 安全限制 |
| 2 | `flat_orientation_l2` | -2.5 | 惩罚 | 姿态稳定 |
| 3 | `base_linear_velocity` | -2.0 | 惩罚 | 防止跳跃 |
| 4 | `track_lin_vel_xy` | +1.5 | 奖励 | ✨ 主任务 |
| 5 | `air_time_variance` | -1.0 | 惩罚 | 步态对称 |
| 6 | `undesired_contacts` | -1.0 | 惩罚 | 防止摔倒 |
| 7 | `track_ang_vel_z` | +0.75 | 奖励 | 主任务 |
| 8 | `joint_pos` | -0.7 | 惩罚 | 姿态控制 |
| 9 | `feet_slide` | -0.1 | 惩罚 | 防打滑 |
| 10 | `action_rate` | -0.1 | 惩罚 | 平滑性 |

### 权重比例分析

```
正向奖励: 1.5 + 0.75 + 0.1 = 2.35  (10.4%)
负向惩罚: 20.25 (89.6%)
```

**设计哲学**: 
- ✅ **稀疏正向奖励** - 只在完成任务时给予
- ⚠️ **密集负向惩罚** - 持续约束不良行为
- 🎯 **重安全轻效率** - 首要保证不摔倒

---

## 🚀 真机部署考虑

### 可直接使用的奖励（无需修改）

| 奖励项 | 传感器需求 | 备注 |
|-------|----------|------|
| `track_lin_vel_xy` | IMU + 状态估计 | ✅ |
| `track_ang_vel_z` | IMU | ✅ |
| `base_linear_velocity` | IMU | ✅ |
| `base_angular_velocity` | IMU | ✅ |
| `joint_vel` | 关节编码器 | ✅ |
| `joint_torques` | 电机反馈 | ✅ |
| `action_rate` | 控制器记录 | ✅ |
| `dof_pos_limits` | 关节编码器 | ✅ |
| `energy` | 编码器 + 电机 | ✅ |
| `flat_orientation_l2` | IMU | ✅ |
| `joint_pos` | 关节编码器 | ✅ |
| `feet_air_time` | 足端力传感器 | ✅ |
| `air_time_variance` | 足端力传感器 | ✅ |

### 需要修改的奖励

| 奖励项 | 挑战 | 解决方案 |
|-------|------|---------|
| `joint_acc` | 需要数值微分 | 卡尔曼滤波器平滑估计 |
| `feet_slide` | 需要足端速度 | FK + IMU 融合 |
| `undesired_contacts` | 需要额外传感器 | 基于异常力矩检测或移除 |

### 建议的真机奖励配置

```python
# 真机部署时的奖励配置建议
class RealRobotRewardsCfg:
    # 保留所有可直接测量的奖励
    # 降低或移除难以测量的奖励权重
    
    # 可选：移除 undesired_contacts
    # undesired_contacts = RewTerm(..., weight=0.0)  # 禁用
    
    # 可选：降低 feet_slide 权重
    # feet_slide = RewTerm(..., weight=-0.01)  # 从 -0.1 降到 -0.01
```

---

## 🎛️ 参数调优建议

### 1. 提高速度跟踪精度

```python
track_lin_vel_xy = RewTerm(
    func=mdp.track_lin_vel_xy_exp,
    weight=2.0,  # 从 1.5 增加到 2.0
    params={"std": 0.3}  # 缩小标准差，要求更精确
)
```

### 2. 增强稳定性

```python
flat_orientation_l2 = RewTerm(
    func=mdp.flat_orientation_l2,
    weight=-5.0,  # 从 -2.5 增加到 -5.0
)
```

### 3. 鼓励更自然的步态

```python
feet_air_time = RewTerm(
    func=mdp.feet_air_time,
    weight=0.5,  # 从 0.1 增加到 0.5
    params={"threshold": 0.3}  # 降低阈值
)
```

### 4. 减少能量消耗

```python
energy = RewTerm(
    func=mdp.energy,
    weight=-1e-4,  # 从 -2e-5 增加到 -1e-4
)
```

---

## 📈 训练监控指标

在训练过程中应重点监控以下指标：

### 主要指标
1. **速度跟踪误差** - `track_lin_vel_xy` 和 `track_ang_vel_z`
2. **总奖励值** - 应持续上升
3. **成功率** - 不触发 `bad_orientation` 终止的比例

### 次要指标
4. **平均能量消耗** - `energy` 项
5. **关节限位触发率** - `dof_pos_limits` 惩罚频率
6. **非期望接触率** - `undesired_contacts` 惩罚频率

### TensorBoard 可视化

```python
# 在训练脚本中记录
writer.add_scalar('rewards/track_lin_vel_xy', rew_lin, step)
writer.add_scalar('rewards/total', total_reward, step)
writer.add_scalar('metrics/energy', energy_consumed, step)
```

---

## 📚 参考文献

1. **Learning to Walk in Minutes Using Massively Parallel Deep RL** (Rudin et al., 2022)
2. **Isaac Lab Documentation**: https://isaac-sim.github.io/IsaacLab
3. **Unitree Robotics Official Repo**: https://github.com/unitreerobotics

---

## 🔄 版本历史

| 版本 | 日期 | 变更说明 |
|-----|------|---------|
| v1.0 | 2026-01-08 | 初始版本 - 基于 Go2 velocity tracking 任务 |

---

## 📝 附录

### A. 完整奖励配置代码

```python
@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    
    # Task rewards
    track_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_exp, 
        weight=1.5, 
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z = RewTerm(
        func=mdp.track_ang_vel_z_exp, 
        weight=0.75, 
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    
    # Base penalties
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2e-4)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.1)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-10.0)
    energy = RewTerm(func=mdp.energy, weight=-2e-5)
    
    # Robot orientation
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)
    joint_pos = RewTerm(
        func=mdp.joint_position_penalty,
        weight=-0.7,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.3,
        },
    )
    
    # Feet rewards
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.1,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        },
    )
    air_time_variance = RewTerm(
        func=mdp.air_time_variance_penalty,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )
    
    # Other penalties
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1,
        params={
            "threshold": 1,
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", 
                body_names=["Head_.*", ".*_hip", ".*_thigh", ".*_calf"]
            ),
        },
    )
```

### B. 观测空间配置

```python
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # 基座角速度（IMU）
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, 
            scale=0.2, 
            noise=Unoise(n_min=-0.2, n_max=0.2)
        )
        # 投影重力（IMU）
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity, 
            noise=Unoise(n_min=-0.05, n_max=0.05)
        )
        # 速度命令
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, 
            params={"command_name": "base_velocity"}
        )
        # 关节位置（相对于默认值）
        joint_pos_rel = ObsTerm(
            func=mdp.joint_pos_rel, 
            noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        # 关节速度
        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel, 
            scale=0.05, 
            noise=Unoise(n_min=-1.5, n_max=1.5)
        )
        # 上一次动作
        last_action = ObsTerm(func=mdp.last_action)
        
        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
    
    policy: PolicyCfg = PolicyCfg()
```

### C. 终止条件配置

```python
@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    
    # 超时终止（正常）
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    
    # 基座接触地面（异常）
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), 
            "threshold": 1.0
        },
    )
    
    # 姿态过度倾斜（异常）
    bad_orientation = DoneTerm(
        func=mdp.bad_orientation, 
        params={"limit_angle": 0.8}  # 约 45 度
    )
```

---

## 📧 联系方式

如有问题或建议，请通过以下方式联系：

- **项目仓库**: https://github.com/unitreerobotics/unitree_rl_lab
- **Discord**: https://discord.gg/ZwcVwxv5rq

---

**文档生成时间**: 2026-01-08  
**适用版本**: Isaac Lab 2.3.0 / Isaac Sim 5.1.0  
**机器人型号**: Unitree Go2
