# 项目文档索引（完整阅读顺序）

面向：**野火双 USB STM32F103C8T6 + TB6612FNG + JGA25-370×2 + MPU6050 + HC-05（+ 可选 HC-SR04）**，**Keil MDK** 开发，姿态**第一版互补滤波**。

| 顺序 | 文档 | 内容 |
|------|------|------|
| 1 | [PROJECT_INDEX.md](PROJECT_INDEX.md) | 仓库结构、里程碑、待确认项 |
| 2 | [HARDWARE_SPEC.md](HARDWARE_SPEC.md) | BOM、引脚、电源、共地、你提供的线序与冲突分析 |
| 3 | [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) | 控制架构、互补滤波公式、双编码器定时器约束、任务频率 |
| 4 | [KEIL_BUILD.md](KEIL_BUILD.md) | CubeMX + Keil 工程、SWD/JTAG、与 `src/` 源码衔接 |
| 5 | [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md) | 双环 PID 实操整定 |

---

_创建：2026-05-14_
