# 两轮平衡车 · STM32F103C8T6

> 主控：**野火双 USB STM32F103C8T6** · 电机：**JGA25-370** ×2 · 驱动：**TB6612FNG**  
> **推荐开发：Keil MDK + STM32CubeMX**（第一版姿态：**互补滤波**，不用卡尔曼）  
> 仓库内 `platformio.ini` 可作为可选参考，与课程要求不一致时以 `doc/KEIL_BUILD.md` 为准。

完整说明见 **[doc/DOCUMENT_INDEX.md](doc/DOCUMENT_INDEX.md)**。

---

## 项目状态

```
✅ 工程框架与驱动源码骨架
✅ 引脚分配表（pin_map.h，须与 Cube 一致）
✅ 电机驱动（TB6612 + TIM1 PWM）
✅ 编码器驱动（演进目标：TIM2 + TIM4 双路正交）
✅ MPU6050（I2C + 互补滤波）
✅ 蓝牙指令解析（HC-05）
✅ 直立环 + 速度环 PID 骨架
✅ 文档：硬件规格、系统设计、Keil 建工程、PID 整定
⏳ 待板级烧录与双 TIM 编码器工程对齐
```

---

## 硬件清单

| 模块 | 型号 | 数量 |
|------|------|------|
| 主控板 | 野火双 USB STM32F103C8T6 | 1 |
| 电机驱动 | TB6612FNG | 1 |
| 电机 | JGA25-370 霍尔编码器 | 2 |
| 姿态传感器 | MPU6050 | 1 |
| 蓝牙模块 | HC-05 | 1 |
| 超声波（可选） | HC-SR04 | 1 |
| 电池 | 7.4V 2S 锂电池 | 1 |

---

## 快速开始（Keil）

1. 阅读 [doc/HARDWARE_SPEC.md](doc/HARDWARE_SPEC.md) 完成接线与共地。  
2. 按 [doc/KEIL_BUILD.md](doc/KEIL_BUILD.md) 用 STM32CubeMX 生成 MDK 工程并合并本仓库 `src/`、`include/`。  
3. 按 [doc/PID_TUNING_GUIDE.md](doc/PID_TUNING_GUIDE.md) 整定：**先直立环，后速度环**。

---

## 引脚占用一览（推荐方案，详见硬件规格书）

| 外设 | 引脚 | 功能 |
|------|------|------|
| TB6612 PWMA | PA8 | 左轮 PWM（TIM1_CH1） |
| TB6612 PWMB | PA9 | 右轮 PWM（TIM1_CH2） |
| TB6612 AIN1/BIN1 | PB5/PB8 | 方向控制 |
| TB6612 AIN2/BIN2 | PB4/PB9 | 方向控制 |
| TB6612 STBY | PB0 | 使能 |
| 左编码器 | PA0/PA1 | TIM2 编码器模式 CH1/CH2 |
| 右编码器 | PB6/PB7（推荐） | TIM4 编码器模式 CH1/CH2；与 I2C 需 Remap 协调 |
| MPU6050 SDA/SCL | PB7/PB6 或 PB9/PB8 | I2C1（视 Remap） |
| HC-05 | PB10/PB11 | USART3（交叉接 TXD/RXD） |
| HC-SR04（可选） | PA15 / PB3 | Trig / Echo；须 SWD 仅 Serial Wire |

---

## 控制架构

```
手机蓝牙遥控（期望速度）
     ↓
编码器测速 ──→ 速度环 PID ──→ 期望倾角 / 倾角偏置
     ↓
MPU6050（互补滤波俯仰角）──→ 直立环 PID ──→ PWM → 电机
```

---

## 下一步

- [ ] 用串口监视器验证 MPU6050 数据正常
- [ ] 用示波器验证 TIM1 PWM 输出正常
- [ ] 按调参指南先调直立环（车体能立住）
- [ ] 再加速度环（遥控前进/后退）
- [ ] 加上转向控制（差速）

---

_更新：2026-05-14_
