# 两轮平衡车项目 · 经验总结

> 创建：2026-05-12
> 平台：野火小智 STM32F103C8T6 + PlatformIO + WSL Ubuntu

---

## 一、PlatformIO 项目配置

### 1.1 board ID 正确写法
```
genericSTM32F103C8Tx  ❌（PlatformIO 不认识）
bluepill_f103c8       ✅（STM32F103C8T6 蓝丸通用 ID）
bluepill_f103c8_128k  ✅（128KB Flash 版本）
```

### 1.2 build_flags 常见错误
```ini
# ❌ 错误：build_src_flags 不是有效选项
build_src_flags = -I include -I src/app

# ✅ 正确：PlatformIO 自动扫描 src/ 和 include/，手动加路径用 -I
build_flags =
    -I include
    -I src/drivers
```

### 1.3 F_CPU 不要重复定义
```ini
# ❌ 错误：board 板级配置里已经定义了 F_CPU，手动加会 warning
build_flags =
    -D F_CPU=72000000

# ✅ 正确：board 自带 F_CPU，不写即可
build_flags =
    -D HSE_VALUE=8000000
    -D USE_HAL_DRIVER
```

### 1.4 外设句柄的 extern 声明
PlatformIO + STM32Cube HAL 架构下，所有外设句柄（htim1/2、hi2c1、huart3）
都在 `stm32_init.c` 中定义，其他 .c 文件使用前必须加 `extern` 声明：

```c
// main.h — 集中声明，所有文件引用
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
```

### 1.5 HAL_TIM_Encoder_Init 参数
```c
// ❌ 错误：参数不足
HAL_TIM_Encoder_Init(&htim2);

// ✅ 正确：必须传入 TIM_Encoder_InitTypeDef 结构体
TIM_Encoder_InitTypeDef sEncoderConfig = {0};
sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;
sEncoderConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;
sEncoderConfig.IC1Selection      = TIM_ICSELECTION_DIRECTTI;
// ... IC2 同理
HAL_TIM_Encoder_Init(&htim2, &sEncoderConfig);
```

### 1.6 头文件缺失
```c
// 数学函数（atan2f）
#include <math.h>

// 字符串函数（strlen）
#include <string.h>
```

---

## 二、WSL + PlatformIO 环境

### 2.1 WSL 里的 PlatformIO 路径
```
~/.platformio/penv/bin/pio
```
需要加入 PATH 或完整路径调用：
```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"
pio run
```

### 2.2 WSL 里运行 PIO Home（有时超时）
WSL 里 PIO Home 首次启动慢，建议关闭，用纯 CLI：
```json
// VS Code settings.json
"platformio-ide.disablePIOHome": true
```

### 2.3 代码在 WSL，VS Code 在 Windows
项目路径：
```
WSL:  /home/xiaoduo/.openclaw/workspace-coding/projects/two-wheel-balancing-bot
Windows: \\wsl$\Ubuntu\home\xiaoduo\.openclaw\workspace-coding\projects\two-wheel-balancing-bot
```
**不要在 Windows 环境下用 pio run**，build 输出路径会冲突。

---

## 三、野火 DAP + WSL USB 直通

### 3.1 工具链
- Windows：usbipd-win（已通过 winget 安装）
- WSL：内置 Linux USB 支持

### 3.2 busid 查询方法
拔掉 DAP → `lsusb` → 记录
插上 DAP → `lsusb` → 新出现的就是 DAP
本机 DAP 信息：`0416:5021 Winbond FireDAP CMSIS-DAP`，busid = `5-1`

### 3.3 绑定命令（每次重启后都要运行）
```powershell
# Windows PowerShell（管理员）
usbipd bind --busid 5-1
usbipd attach --busid 5-1 --wsl
```

### 3.4 开机自动脚本
```
D:\Software\scripts\attach-dap.ps1
```
右键 → 用 PowerShell 运行，每次开机后执行一次。

### 3.5 DAP vs CH340 串口
| 方式 | 协议 | 跳线帽 | 推荐度 |
|------|------|--------|--------|
| 野火 DAP | CMSIS-DAP / SWD | 不需要 | ✅ 推荐 |
| CH340 串口 | UART Bootloader | 需要 BOOT0 | 仅无 DAP 时用 |

---

## 六、WSL pio 命令冲突

**问题：** 系统自带 `/usr/bin/pio`（v4.3.4）与 `~/.platformio/penv/bin/pio`（v6.1.19）冲突

**修复：** `~/.local/bin/pio` 包装脚本（`~/.local/bin` 在 PATH 中优先于 `/usr/bin`）

**验证：**
```bash
which pio     # /home/xiaoduo/.local/bin/pio
pio --version # PlatformIO Core, version 6.1.19
```

---

## 四、Git 提交记录

| 提交 | 内容 |
|------|------|
| `0432662` | Initial commit：工程骨架（22 files） |
| `8a641b6` | fix：board ID 改为 bluepill_f103c8 |
| `306412b` | fix：添加 src/drivers 等 include 路径 |
| `cf9b56b` | fix：编译错误全修复（7 files） |

---

## 五、Flash 占用评估

| 项目 | 限制 | 当前固件 | 余量 |
|------|------|---------|------|
| Flash | 64 KB | 12 KB | ✅ 52 KB（18%） |

---

_最后更新：2026-05-12_
