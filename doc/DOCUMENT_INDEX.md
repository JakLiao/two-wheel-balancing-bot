# 项目文档索引（完整阅读顺序）

面向：**野火双 USB STM32F103C8T6 + TB6612FNG + JGA25-370×2 + MPU6050 + HC-05**，**Keil MDK** 开发，姿态**互补滤波**。

| 顺序 | 文档 | 内容 |
|------|------|------|
| 1 | [HARDWARE_SPEC.md](HARDWARE_SPEC.md) | BOM、引脚、电源、共地、线序与冲突分析 |
| 2 | [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) | 控制架构、互补滤波公式、双编码器定时器约束、任务频率 |
| 3 | [KEIL_BUILD.md](KEIL_BUILD.md) | Keil 工程配置、SWD 烧录 |
| 4 | [INCLUDE_DEPENDENCY.md](INCLUDE_DEPENDENCY.md) | 所有 `.c` / `.h` 的 include 依赖关系、extern 句柄声明表 |
| 5 | **[PID_GUIDE.md](PID_GUIDE.md)** | **PID 完整指南：物理推导 + Z-N 法 + 量程换算 + 实战整定 + 判断标准（v3.0 合并版）** |

---

_更新：2026-05-28_
