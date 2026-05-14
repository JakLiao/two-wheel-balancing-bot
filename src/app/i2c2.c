/**
 * @file i2c2.c
 * @brief I2C2 寄存器级实现（裸机，无 HAL）
 * PB10=SCL, PB11=SDA, 400kHz Fast Mode
 *
 * 供 src/drivers/mpu6050.c 底层调用
 */

#include "i2c2.h"

/* ==================== 初始化 ==================== */

void I2C2_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(I2C2_APB_CLOCK, ENABLE);
    RCC_APB2PeriphClockCmd(I2C2_GPIO_CLOCK | RCC_APB2Periph_AFIO, ENABLE);

    // 2. PB10/PB11 复用开漏输出（I2C 必须开漏）
    GPIO_InitStructure.GPIO_Pin   = I2C2_SCL_PIN | I2C2_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStructure);

    // 3. 配置 I2C2
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;      // 占空比 1:2
    I2C_InitStructure.I2C_OwnAddress1          = 0x00;                // 主设备，无需从地址
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;       // 使能 ACK
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed         = I2C2_SPEED;          // 400kHz

    I2C_Cmd(I2C2_BASE, DISABLE);
    I2C_Init(I2C2_BASE, &I2C_InitStructure);
    I2C_Cmd(I2C2_BASE, ENABLE);
}

/* ==================== 内部工具 ==================== */

static uint8_t I2C2_WaitEvent(uint32_t flag)
{
    uint32_t timeout = I2C2_TIMEOUT;
    while (!I2C_CheckEvent(I2C2_BASE, flag)) {
        timeout--;
        if (timeout == 0) return 1;  // 超时
    }
    return 0;
}

/* ==================== 底层操作 ==================== */

void I2C2_Start(void)
{
    I2C_GenerateSTART(I2C2_BASE, ENABLE);
    I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);
}

void I2C2_Stop(void)
{
    I2C_GenerateSTOP(I2C2_BASE, ENABLE);
}

uint8_t I2C2_SendByte(uint8_t data)
{
    I2C_SendData(I2C2_BASE, data);
    return I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);
}

uint8_t I2C2_RecvByte(uint8_t ack)
{
    if (ack) I2C_AcknowledgeConfig(I2C2_BASE, ENABLE);
    else     I2C_AcknowledgeConfig(I2C2_BASE, DISABLE);

    I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);
    return I2C_ReceiveData(I2C2_BASE);
}

/* ==================== 寄存器读写 ==================== */

void I2C2_WriteReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // START
    I2C2_Start();

    // 发送从地址 + Write (bit0=0)
    I2C_SendData(I2C2_BASE, (addr << 1) | 0);
    I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    // 发送寄存器地址
    I2C_SendData(I2C2_BASE, reg);
    I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);

    // 发送数据
    while (len--) {
        I2C_SendData(I2C2_BASE, *buf++);
        I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);
    }

    I2C2_Stop();
}

void I2C2_WriteRegByte(uint8_t addr, uint8_t reg, uint8_t value)
{
    I2C2_WriteReg(addr, reg, &value, 1);
}

void I2C2_ReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // 发送从地址 + Write，进入 transmitter 模式发送寄存器地址
    I2C2_Start();
    I2C_SendData(I2C2_BASE, (addr << 1) | 0);
    I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    I2C_SendData(I2C2_BASE, reg);
    I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);

    // 切换到 Receiver 模式，发送 Repeated START
    I2C2_Start();
    I2C_SendData(I2C2_BASE, (addr << 1) | 1);
    I2C2_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    // 读取数据
    while (len) {
        if (len == 1) {
            *buf++ = I2C2_RecvByte(0);   // 最后一个字节 NACK
        } else {
            *buf++ = I2C2_RecvByte(1);   // 中间字节 ACK
        }
        len--;
    }

    I2C2_Stop();
}

uint8_t I2C2_ReadRegByte(uint8_t addr, uint8_t reg)
{
    uint8_t val;
    I2C2_ReadReg(addr, reg, &val, 1);
    return val;
}
