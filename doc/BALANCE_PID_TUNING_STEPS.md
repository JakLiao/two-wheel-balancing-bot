# 两轮平衡车 · 直立环 PID 调参详细步骤

> 综合整理：慧文调研 × 小朵硬件审计 × balance.c 代码分析
> 版本：v1.0 · 日期：2026-05-22
> 平台：野火小智 STM32F103C8T6 + JGA25-370 12V 减速比 20.4:1 + MPU6050 + TB6612FNG

---

## 一、调参前硬件状态确认

**必须全部通过才能上车调 PID。** 打开串口助手（115200 8N1），观察每 500ms 的打印：

| 检查项 | 期望现象 | 如不符合 |
|--------|---------|---------|
| `Pitch` | 静止时 ≈ 0° ± 2° | MPU6050 共地或零点漂移 |
| `Gx/Gy/Gz` | 静止时 ≈ 0 °/s | 需重新校准 |
| `Ax/Ay/Az` | Az ≈ 16384，其他轴 ≈ 0 | I2C 通信异常 |
| 编码器 RPM | 静止时 ≈ 0 | 编码器信号跳变 |
| 电池电压 | > 11V | 电量不足会导致 PID 异常 |

---

## 二、直立环 PID 核心概念

### 当前 balance.c 默认参数（待整定）

```c
// APP/balance/app_balance.c

#define BALANCE_KP          50.0f   // 比例（越大越"硬"，太大会抖）
#define BALANCE_KI           0.0f   // 积分（平衡车不需要！）
#define BALANCE_KD          20.0f    // 微分（越大越"阻尼"，抑制振荡）

#define BALANCE_OUT_MAX      100     // 直立环输出限幅（PWM 最大值）
#define BALANCE_OUT_MIN     -100

#define SPEED_OUT_MAX       30.0f   // 速度环输出限幅（期望倾角，度）
#define SPEED_OUT_MIN      -30.0f

#define SPEED_KP             1.0f   // 速度环参数
#define SPEED_KI             0.01f
#define SPEED_KD             0.0f

#define SPEED_INTEGRAL_MAX   50.0f
#define SPEED_INTEGRAL_MIN  -50.0f
```

### 直立环控制周期

| 环 | 控制周期 | dt 参数 | 频率 |
|----|---------|---------|------|
| 直立环 | 5ms | `dt = 0.005f` | 200Hz |
| 速度环 | 10ms | `dt = 0.01f` | 100Hz |

### 为什么直立环不用 I（积分）？

平衡车的目标是"不倒"，不是"停在精确角度"。积分项有三个弊端：
- **允许稳态误差存在，反而更安全**——车身有轻微前倾角才能产生补偿力矩维持运动
- **积分累加恢复慢**——一旦超调，积分项消退需要时间，更难纠正
- **积分消除误差会让车身持续前倾**——维持运动状态而非真正"直立"

### P 和 D 的物理意义

- **P（比例）**：倾角越大，补偿力越大 → "棍子歪得多，手就要推得猛"
- **D（微分）**：倾角变化率的反向阻尼 → "棍子正在快速倒，提前刹车"

---

## 三、直立环调参步骤（5步）

### Step 1：确认 P 极性

先把 `BALANCE_KD = 0`，`BALANCE_KI = 0`，`BALANCE_KP` 从 0 开始。

```
BALANCE_KP = 0      → 用手轻推，车完全无阻力 → 继续加
BALANCE_KP = 10     → 有轻微阻力感
BALANCE_KP = 30     → 车体能短时间立住，推一下能回正一点
BALANCE_KP = 50     → 车体立得较稳，推一下能弹回
```

> ⚠️ **极性判断**：如果车体往一边倒且越来越快（而不是来回摆动），说明 P 极性反了。需要把 `BALANCE_KP` 改为负值，或调换电机 AIN1/AIN2 接线。

### Step 2：找 Kp 临界点

继续增大 Kp，观察出现**低频抖动**（类似"点头"，频率约 0.5~2Hz）：

```
BALANCE_KP = 80     → 开始出现低频抖动（临界点附近）
BALANCE_KP = 100    → 低频抖动明显，车体来回摆动
BALANCE_KP = 120    → 剧烈振荡，可能失控
```

**记录临界 Kp 值**（例如这里是 80~100），然后取 **60%~70%** 作为正式 Kp：

```
工程经验值 = 临界Kp × 0.6
例如：临界Kp = 80 → BALANCE_KP = 48（取 50）
```

> 本项目默认 Kp = 50，说明之前已做过初步评估，50 大致在合理范围。

### Step 3：加 Kd 消振

固定 Kp，逐步加大 Kd，抑制 Step 2 中的低频抖动：

```
BALANCE_KD = 5      → 低频抖动略微减小
BALANCE_KD = 15     → 推一下，车体能快速回正，抖动消失
BALANCE_KD = 30     → 车身变得"僵硬"，响应变慢
BALANCE_KD = 20     → 良好平衡点 ← 参考默认值
```

**判断标准：**
- 推一下，车体能快速回正，无明显来回振荡 → Kd 合适
- 高频颤抖出现（啸叫）→ Kd 太大，退回上一步

### Step 4：最终参数（参考值）

```
BALANCE_KP = 50     // （临界值 × 0.6，或从默认值微调）
BALANCE_KI = 0.0    // 不需要
BALANCE_KD = 20     // 视振荡情况调整（范围 5~80）
```

**调好的标志：** 用手轻推车体，车体有阻力但不会立刻倒，能自己回正。

### Step 5：安全保护确认

```c
// balance.c 中已实现，调参前确认这些值存在：
if (pitch >  45.0f || pitch < -45.0f) {
    Motor_Stop();   // 倾角超过 45° 自动停电机
    PID_Reset(&balance_pid);
    PID_Reset(&speed_pid);
    balance_state = BALANCE_IDLE;
    return;
}
```

---

## 四、速度环调参（直立环调好后进行）

**前置条件：直立环调好，车体能稳定立住**

```c
// 当前 balance.c 默认值
SPEED_KP = 1.0f
SPEED_KI = 0.01f   // 积分消除静差
SPEED_KD = 0.0f

// 速度环控制周期：10ms（100Hz）
// 输出限幅：期望倾角 ±30°
```

**调参步骤：**

```
SPEED_KP = 0.5     → 手推车，车速有轻微回正趋势
SPEED_KP = 1.0     → 遥控前进/后退，车速跟踪较好 ← 默认值
SPEED_KP = 2.0     → 速度响应快但松手后有明显回弹振荡

SPEED_KI = 0.01    → 稳态误差减小 ← 默认值
SPEED_KI = 0.05    → 松手后速度快速归零
SPEED_KI = 0.1     → 出现振荡 → 退回
```

---

## 五、串口调参界面实现（免重烧录）

每次改参数重烧录太麻烦，建议实现蓝牙调参：

**协议（文本，方便调试）：**

```
SET_BALANCE_KP=60\r\n     → 设置直立环 Kp
SET_BALANCE_KD=25\r\n     → 设置直立环 Kd
GET_STATUS\r\n            → 查询所有当前参数
DUMP\r\n                  → 打印所有参数值
```

**实现思路（在 balance.c 中添加）：**

```c
// 在主循环或串口接收中解析指令
void Balance_Parse_Command(char *cmd) {
    if (strncmp(cmd, "SET_BALANCE_KP=", 15) == 0) {
        float val = atof(cmd + 15);
        if (val >= 0 && val <= 200) {
            balance_pid.kp = val;
            printf("[PID] BALANCE_KP=%.1f\r\n", val);
        }
    } else if (strncmp(cmd, "SET_BALANCE_KD=", 15) == 0) {
        float val = atof(cmd + 15);
        if (val >= 0 && val <= 200) {
            balance_pid.kd = val;
            printf("[PID] BALANCE_KD=%.1f\r\n", val);
        }
    } else if (strcmp(cmd, "DUMP\r\n") == 0 || strcmp(cmd, "DUMP") == 0) {
        printf("[PID] BALANCE Kp=%.1f Ki=%.2f Kd=%.1f\r\n",
               balance_pid.kp, balance_pid.ki, balance_pid.kd);
        printf("[PID] SPEED   Kp=%.2f Ki=%.3f Kd=%.2f\r\n",
               speed_pid.kp, speed_pid.ki, speed_pid.kd);
    }
}
```

---

## 六、本项目硬件关键参数（供调参参考）

| 参数 | 值 | 说明 |
|------|-----|------|
| 系统时钟 | 72 MHz | — |
| PWM 频率 | 10kHz | ARR=99, PSC=72 |
| PWM 量程 | 0~100 | 直立环输出 ±100 |
| 直立环周期 | 5ms（200Hz） | `dt = 0.005f` |
| 速度环周期 | 10ms（100Hz） | `dt = 0.01f` |
| MPU6050 采样率 | 100Hz | SMPLRT_DIV=9 |
| MPU6050 低通带宽 | 44Hz | DLPF_CFG=3 |
| 编码器分辨率 | 896 CPR/圈 | 224 × 4 倍频 |
| 电机减速比 | 20.4:1 | — |
| 倾角保护阈值 | ±45° | 已实现，balance.c |
| 平衡角度限幅 | ±30° | 速度环输出限幅 |
| 电机驱动 | TB6612FNG | 扩展板直插 |
| 姿态传感器 | MPU6050 | I2C2（PB10/PB11）|

---

## 七、常见问题排查

| 现象 | 原因 | 解决 |
|------|------|------|
| P 从 0 开始加但车一直往一边倒 | P 极性反了 | 改为负值或调换 AIN1/AIN2 |
| Kp 很小就开始抖 | MPU6050 共地不良 / 电机干扰 | 检查 GND 连线，加滤波电容 |
| 低频抖动（来回摆） | Kp 太大 | ↓Kp 或乘 0.6 |
| 高频颤抖（啸叫） | Kd 太大 | ↓Kd |
| 推一下才倒，无阻尼 | Kp 太小 | ↑Kp |
| 直立环稳了但速度环一加就抖 | 速度环加太急 | 降低 SPEED_KP，从 0 开始重新调 |
| 编码器数值乱跳 | 电机电源干扰 / 编码器线太长 | 编码器信号线换短，加 100Ω 电阻 |
| 车倒了电机还在转 | 倾角保护阈值太大 | 缩小 BALANCE_ANGLE_MAX（< ±45°）|
| 手机指令和实际方向相反 | 电机接线或方向定义反了 | 调换 AIN1/AIN2 或修改代码 |

---

## 八、调参安全 Checklist

- [x] 倾角 > 45° 自动停电机（代码已实现）
- [ ] 上车前用手扶住，做好扶手准备
- [ ] 准备软垫做倒地缓冲
- [ ] 先低电压测试，确认正常再上全压（12V）
- [ ] 调参时保持 `printf` 打印开着，随时观察 `Pitch` 数值
- [ ] 准备一根绳子绑在车体上，防止车跑出去摔坏

---

## 九、调参记录表

> 每次上车调参后记录参数和现象

| 日期 | Kp | Ki | Kd | Speed_Kp | Speed_Ki | 现象描述 |
|------|----|----|----|---------|---------|---------|
| | | | | | | |
| | | | | | | |
| | | | | | | |

---

## 十、相关文档

| 文档 | 说明 |
|------|------|
| `doc/HARDWARE_SPEC.md` | 硬件规格书（引脚分配、电源、接线）|
| `doc/PID_TUNING_GUIDE.md` | PID 调参核心思路 |
| `doc/PID_TUNING_PRACTICAL.md` | PID 实用整定指南 |
| `APP/balance/app_balance.c` | 直立环核心代码 |
| `APP/pid/app_pid.c` | 通用离散 PID 控制器 |

---

_文档版本：v1.0，2026-05-22_
