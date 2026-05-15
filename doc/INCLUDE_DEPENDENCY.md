# INCLUDE_DEPENDENCY.md — 文件引用关系

> 本文件记录所有 `.c` / `.h` 文件之间的 `#include` 依赖链。
> 规则：**C 文件只 include 自己的 .h，.h 负责带上它需要的所有类型声明。**
> 最后更新：2026-05-15

---

## 一、规则

| 场景 | 处理方式 |
|------|---------|
| 需要 HAL 类型句柄（`TIM_HandleTypeDef` 等） | 在 `.h` 中 include `main.h`（经 `stm32f1xx_hal.h` 链） |
| 只用 `stdint.h` 类型（`uint32_t` 等） | `.h` 中只 include `<stdint.h>` |
| 需要 pin 宏（`HEARTBEAT_LED_PIN` 等） | 在 `.c` 中单独 include `pin_map.h` |
| `extern` 句柄声明 | **全部放在 `main.h`**，不分散 |
| `bsp_i2c2.h/c` | SPL 旧库，**不在 Keil 编译列表**，已废弃 |

---

## 二、include 依赖图

```
main.h
└── stm32f1xx_hal.h
    ├── stm32f1xx_hal_tim.h
    │   └── stm32f1xx_hal_tim_ex.h     ← TIMEx 回调声明
    └── stm32f1xx_hal_def.h
        └── stm32f1xx_hal.h (递归)

pin_map.h
└── stm32f1xx_hal.h                    ← 提供 GPIO_TypeDef / GPIO_Pin_xx 等
```

### 各 BSP .h 依赖链

```
bsp_bluetooth.h  → main.h              ← 需要 UART_HandleTypeDef / Error_Handler
bsp_mpu6050.h    → main.h              ← 需要 I2C_HandleTypeDef / hi2c2
bsp_usart.h      → main.h              ← 需要 UART_HandleTypeDef / huart1
bsp_dwt.h        → main.h              ← 需要 HAL_Init / SystemCoreClock
bsp_led.h        → main.h              ← 需要 Error_Handler / GPIO 相关类型
bsp_encoder.h    → stm32f1xx_hal.h     ← 需要 TIM_HandleTypeDef / extern htim2/htim4
```

### 各 .c 文件依赖链

```
main.c              → main.h + pin_map.h + app_motor.h + bsp_encoder.h
                                 + bsp_mpu6050.h + app_balance.h
                                 + bsp_bluetooth.h + app_pid.h

printf_redirect.c   → stdio.h + main.h

stm32_init.c        → main.h

stm32f1xx_it.c      → main.h + stm32f1xx_it.h + bsp_mpu6050.h

bsp_bluetooth.c     → bsp_bluetooth.h + string.h
                         └── bsp_bluetooth.h → main.h

bsp_mpu6050.c       → bsp_mpu6050.h + math.h
                         └── bsp_mpu6050.h → main.h

bsp_encoder.c       → bsp_encoder.h
                         └── bsp_encoder.h → stm32f1xx_hal.h

bsp_usart.c         → bsp_usart.h
                         └── bsp_usart.h → main.h

bsp_led.c           → bsp_led.h
                         └── bsp_led.h → main.h

bsp_dwt.c           → bsp_dwt.h
                         └── bsp_dwt.h → main.h

bsp_i2c2.c          → bsp_i2c2.h        ← ⚠️ SPL 旧库，不在编译列表，已废弃

test_uart_monitor.c → main.h + app_motor.h + bsp_encoder.h
                                   + bsp_mpu6050.h + pin_map.h

app_balance.c       → app_balance.h + bsp_mpu6050.h + bsp_encoder.h
                                   + app_motor.h + bsp_bluetooth.h
                                   + app_pid.h + main.h

app_motor.c         → app_motor.h + pin_map.h

app_pid.c           → app_pid.h
```

---

## 三、extern 句柄声明（统一在 main.h）

| 句柄 | 类型 | 定义位置 | main.h 中的声明 |
|------|------|---------|--------------|
| `htim3` | `TIM_HandleTypeDef` | `stm32_init.c` | `extern TIM_HandleTypeDef htim3;` |
| `htim2` | `TIM_HandleTypeDef` | `stm32_init.c` | `extern TIM_HandleTypeDef htim2;` |
| `htim4` | `TIM_HandleTypeDef` | `stm32_init.c` | `extern TIM_HandleTypeDef htim4;` |
| `hi2c2` | `I2C_HandleTypeDef` | `stm32_init.c` | `extern I2C_HandleTypeDef hi2c2;` |
| `huart1` | `UART_HandleTypeDef` | `stm32_init.c` | `extern UART_HandleTypeDef huart1;` |

**不要在其他任何 .h 或 .c 中重复声明以上句柄。**

---

## 四、避免循环引用的原则

1. **禁止 `main.c` include `bsp_xxx.h` 链回头再 include `main.h`** → 已通过在 `.h` 中使用 `main.h` 解决（单向依赖）
2. **禁止在 `.c` 中直接 include `stm32f1xx_hal.h`** → 所有 HAL 类型统一经 `main.h` 间接引入
3. **`extern` 不需要知道类型细节**，因此 `bsp_encoder.h` 可以直接 `include "stm32f1xx_hal.h"` 而不产生循环（HAL 库是无环的）
4. **`pin_map.h` 只含宏定义**，不引用任何其他项目头文件，安全

---

## 五、文件→分组映射（Keil）

| Keil 分组 | 包含文件 |
|-----------|---------|
| STARTUP | `startup_stm32f103xb.s` |
| CMSIS | `system_stm32f1xx.c` |
| STM32F1xx_HAL_Driver | `stm32f1xx_hal*.c`（全部） |
| USER | `main.c` `stm32_init.c` `stm32f1xx_it.c` `stm32f1xx_hal_conf.h`<br>`pin_map.h` `printf_redirect.c`<br>`bsp_led.c` `bsp_dwt.c` `bsp_usart.c`<br>`bsp_encoder.c` `bsp_mpu6050.c` `bsp_bluetooth.c` |
| APP | `app_balance.c` `app_motor.c` `app_pid.c` |

---

_Last updated: 2026-05-15_
