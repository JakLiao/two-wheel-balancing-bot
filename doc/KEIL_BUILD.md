# 使用 Keil MDK 开发本工程（推荐教学路径）

> 说明：本仓库历史上存在 PlatformIO 示例配置；**按课程/比赛要求使用 Keil 时，以本文与 STM32CubeMX 生成工程为准**。  
> 芯片包：STM32F1xx_DFP（Keil Pack Installer 中安装）。

---

## 一、推荐工作流

1. **STM32CubeMX**（与芯片包版本匹配）新建工程：MCU 选 `STM32F103C8Tx`。  
2. 在 Cube 中配置时钟、GPIO、**TIM1 PWM**（电机）、**TIM2 + TIM4 编码器模式**（双轮）、**I2C1**（MPU6050，若与编码器引脚冲突则启用 **I2C1 Remap** 到 PB8/PB9）、**USART2**（HC-05，PA2/PA3）、SysTick。  
3. 生成代码时选择 **MDK-ARM V5** 工具链。  
4. 用 Keil 打开生成的 `.uvprojx`，将本仓库 `src/`、`include/` 中业务源文件加入工程（或保持 Cube 生成结构，把驱动与 `balance.c` 等拷贝到 `User` 目录并在工程中 Add Files）。  
5. 在 `main.c` 初始化流程中调用：`MPU6050_Init`、`Encoder_Init`、`Motor_Init`、PID 初始化、`Balance_Init` 等（与现逻辑一致即可）。

---

## 二、与 HAL 相关的注意点

- **全局句柄**：`hi2c1`、`htim1`、`htim2`、`htim4`、`huart*` 等名称需与驱动代码中 `extern` 声明一致。  
- **系统时钟**：F103 常见 72 MHz HSE + PLL；确保 `SystemClock_Config` 与板载晶振一致（野火 C8 板多为 8 MHz 外部晶振，以实物为准）。  
- **浮点**：Cortex-M3 **无 FPU**，PID 与滤波用 `float` 即可，注意编译器优化等级与 `-ffast-math` 等非必要选项勿随意改动以免影响调试可重复性。

---

## 三、HC-SR04 与 SWD 占用 PA15

若超声波 **Trig 使用 PA15**：默认复位后 PA15 为 **JTDI**（JTAG 功能）。需在 Cube 里将调试接口设为 **Serial Wire**，释放 PA15/PB3/PB4 等引脚为 GPIO（仅保留 SWDIO、SWCLK）。  
否则 Trig 无波形或电平异常。

---

## 四、工程内文件与 Cube 生成代码的衔接

本仓库 `include/pin_map.h` 为**逻辑宏**，便于阅读；**真实引脚以 CubeMX 配置为准**。  
合并方式二选一：

- **A**：修改 Cube 引脚使与 `pin_map.h` 完全一致，驱动几乎不改。  
- **B**：保留你的硬件接线，改 `pin_map.h` 与各 `*_Init` 中的 GPIO 绑定，与 Cube 一致。

---

## 五、编译与烧录

- **编译**：Keil 菜单 Project → Build Target（F7）。  
- **烧录器**：ST-Link / CMSIS-DAP 等；在 **Options for Target → Debug / Utilities** 中选择对应驱动。  
- **串口打印**：USART1（PA9/PA10）接 CH340 用于调试；HC-05 蓝牙接 **USART2**（PA2/PA3），两者完全独立。

---

## 六、无 PlatformIO 时的目录建议

```
two-wheel-balancing-bot/
├── doc/                    ← 文档
├── MDK-ARM/                ← Cube 生成的 Keil 工程（本地生成，可不提交或提交团队约定）
├── Core/                   ← Cube 生成的 Src/Inc（同上）
├── include/                ← 应用层头文件
└── src/                    ← 应用与驱动源码
```

将 `src`、`include` 加入 Keil 的 **Include Path**（Options → C/C++ → Include Paths）。

---

_Last updated: 2026-05-27_
