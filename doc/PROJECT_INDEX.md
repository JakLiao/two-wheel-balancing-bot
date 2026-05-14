# 两轮平衡车 PID 学习项目

> 野火双 USB STM32F103C8T6 · TB6612FNG · JGA25-370 · MPU6050 · HC-05 ·（可选 HC-SR04）  
> **正式教学/比赛路径：Keil MDK + STM32CubeMX**（姿态第一版：**互补滤波**，不用卡尔曼、不用 DMP）

**完整文档目录**见 [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md)。

---

## 项目结构（源码 + 文档）

```
two-wheel-balancing-bot/
├── doc/
│   ├── DOCUMENT_INDEX.md      ← 文档阅读顺序（总索引）
│   ├── HARDWARE_SPEC.md      硬件规格、引脚、电源、线序冲突分析
│   ├── SYSTEM_DESIGN.md      控制架构、互补滤波、双编码器 TIM 说明
│   ├── KEIL_BUILD.md         Keil + CubeMX 建工程与烧录
│   ├── PID_TUNING_GUIDE.md   双环整定
│   └── PROJECT_INDEX.md      本文件
├── include/
│   └── pin_map.h             引脚宏（须与 Cube 配置一致）
├── src/
│   ├── app/
│   │   ├── main.c            入口
│   │   ├── balance.c/h       直立环 + 速度环
│   │   └── bluetooth.c/h     HC-05 指令（若引脚与 Cube 一致）
│   ├── drivers/
│   │   ├── mpu6050.c/h       I2C + 互补滤波
│   │   ├── encoder.c/h       编码器（目标：TIM2 + TIM4 双路正交）
│   │   ├── motor.c/h         TB6612 + TIM1 PWM
│   │   └── ...
│   └── middleware/
│       └── pid.c/h
├── platformio.ini            可选：仅作参考/CI，与 Keil 二选一为主工具链
└── test/
```

---

## 开发环境（已拍板）

| 项目 | 选择 |
|------|------|
| 主工具链 | **Keil MDK-ARM** + STM32CubeMX 生成 HAL 工程 |
| 姿态算法 | **一阶互补滤波**（见 `SYSTEM_DESIGN.md`、`mpu6050.c`） |
| 滤波升级 | 第二版可换卡尔曼；第一版不引入 |
| 编码器 | **两路正交解码 = 两个定时器的 CH1+CH2**（如 TIM2 + TIM4）；勿用单 TIM 的 CH3+CH4 冒充第二路 |
| PID 整定顺序 | **先直立环，后速度环** |

---

## 学习里程碑

- [x] **M0**：GPIO 测试 — 电机能转（逻辑待板级验证）
- [ ] **M1**：双编码器独立 TIM — 左右速度方向正确
- [ ] **M2**：MPU6050 + 互补滤波 — 俯仰角稳定可读
- [ ] **M3**：直立环 PID — 车体能立住
- [ ] **M4**：速度环 PID — 手推回中、遥控跟速
- [ ] **M5**：蓝牙遥控 — 前进/后退/转向
- [ ] **M6**：HC-SR04 避障或停车（不参与直立主环）

---

## 待你本地确认

| 项 | 说明 |
|----|------|
| HC-05 接哪组 USART | 推荐 USART3：`PB10`/`PB11`，与 `pin_map.h` 一致；以 Cube 为准 |
| TB6612 线序与丝印 | 将你提供的线序与模块 **PWMA/AINx/…** 用万用表核对后再写死到 `pin_map.h` |
| JGA25-370 每转脉冲数 | 写入 `encoder.c` 或配置头文件，供速度环标定 |

---
_更新：2026-05-14_
