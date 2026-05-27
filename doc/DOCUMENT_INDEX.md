# 项目文档索引（完整阅读顺序）

面向：**野火双 USB STM32F103C8T6 + TB6612FNG + JGA25-370×2 + MPU6050 + HC-05**，**Keil MDK** 开发，姿态**互补滤波**。

| 顺序 | 文档 | 内容 |
|------|------|------|
| 1 | [PROJECT_INDEX.md](PROJECT_INDEX.md) | 仓库结构、里程碑、待确认项 |
| 2 | [HARDWARE_SPEC.md](HARDWARE_SPEC.md) | BOM、引脚、电源、共地、线序与冲突分析 |
| 3 | [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) | 控制架构、互补滤波公式、双编码器定时器约束、任务频率 |
| 4 | [KEIL_BUILD.md](KEIL_BUILD.md) | Keil 工程配置、SWD 烧录 |
| 5 | [INCLUDE_DEPENDENCY.md](INCLUDE_DEPENDENCY.md) | 所有 `.c` / `.h` 的 include 依赖关系、extern 句柄声明表 |
| 6 | [PID_TUNING_PRACTICAL.md](PID_TUNING_PRACTICAL.md) | 双环 PID 实战整定步骤、参数参考值 |
| 7 | **[PID_PHYSICAL_CALCULATION.md](PID_PHYSICAL_CALCULATION.md)** | **PID 参数物理推导、Z-N 法计算、Kp/Kd/Ki 过大过小判断标准（2026-05-27 新增）** |

---

_更新：2026-05-27_
