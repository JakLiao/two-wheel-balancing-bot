# 两轮平衡车 · STM32F103C8T6

> 基于野火小智扩展板 + PlatformIO + 双环 PID
> 目标：学习 PID 调试

---

## 项目状态

```
✅ 工程框架搭建完成
✅ 引脚分配表（pin_map.h）
✅ 电机驱动（TB6612 + TIM1 PWM）
✅ 编码器驱动（TIM2 正交解码）
✅ MPU6050 驱动（I2C + 互补滤波）
✅ 蓝牙指令解析（HC-05）
✅ 直立环 + 速度环 PID 控制
✅ PID 整定实战指南
⏳ 待实际烧录验证
```

---

## 硬件清单

| 模块 | 型号 | 数量 |
|------|------|------|
| 主控板 | 野火小智 + STM32F103C8T6 | 1 |
| 电机驱动 | TB6612FNG | 1 |
| 电机 | SK11S-48:1 编码器电机 | 2 |
| 姿态传感器 | MPU6050 | 1 |
| 蓝牙模块 | HC-05 | 1 |
| 电池 | 7.4V 2S 锂电池 | 1 |

---

## 快速开始

### 1. 安装 PlatformIO

```bash
# VS Code 安装 PlatformIO 插件
# 或者命令行安装：
pip install platformio
pio pkg install
```

### 2. 连接硬件

按 `doc/HARDWARE_SPEC.md` 接线，**共地是重中之重**。

### 3. 编译

```bash
cd projects/two-wheel-balancing-bot
pio run
```

### 4. 烧录

```bash
pio run --target upload
# 或
pio run --target upload --upload-port /dev/ttyACM0
```

### 5. 调参

按 `doc/PID_TUNING_GUIDE.md` 操作，**先直立环，后速度环**。

---

## 引脚占用一览

| 外设 | 引脚 | 功能 |
|------|------|------|
| TB6612 PWMA | PA8 | 左轮 PWM（TIM1_CH1） |
| TB6612 PWMB | PA9 | 右轮 PWM（TIM1_CH2） |
| TB6612 AIN1/BIN1 | PB5/PB8 | 方向控制 |
| TB6612 AIN2/BIN2 | PB4/PB9 | 方向控制 |
| TB6612 STBY | PB0 | 使能 |
| 左编码器 | PA0/PA1 | TIM2_CH1/CH2 |
| 右编码器 | PA2/PA3 | TIM2_CH3/CH4 |
| MPU6050 SDA/SCL | PB7/PB6 | I2C1 |
| HC-05 TX/RX | PB11/PB10 | USART3 |

---

## 控制架构

```
手机蓝牙遥控
     ↓
速度环 PID ──→ 期望倾角
     ↓
直立环 PID ──→ PWM 电机
     ↓
MPU6050 反馈（角度 + 角速度）
```

---

## 下一步

- [ ] 用串口监视器验证 MPU6050 数据正常
- [ ] 用示波器验证 TIM1 PWM 输出正常
- [ ] 按调参指南先调直立环（车体能立住）
- [ ] 再加速度环（遥控前进/后退）
- [ ] 加上转向控制（差速）

---

_项目创建：2026-05-12_
