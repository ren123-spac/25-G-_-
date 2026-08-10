#ifndef _DRIVE_AD9959_H_
#define _DRIVE_AD9959_H_

#include "main.h"
#include "stdint.h"

// ============================================================
// AD9959 寄存器地址定义
// ============================================================
#define CSR_ADD     0x00    // 通道选择/使能寄存器
#define FR1_ADD     0x01    // 功能寄存器1
#define FR2_ADD     0x02    // 功能寄存器2
#define CFR_ADD     0x03    // 通信/功能寄存器
#define CFTW0_ADD   0x04    // 频率调谐字0
#define CPOW0_ADD   0x05    // 相位偏移字0
#define ACR_ADD     0x06    // 幅度控制寄存器
#define LSRR_ADD    0x07    // 线性扫频上升速率
#define RDW_ADD     0x08    // 扫频下降字
#define FDW_ADD     0x09    // 扫频频率字

// ============================================================
// 引脚映射（按照你的硬件连接）
// ============================================================
// ---- 软件 SPI 引脚 ----
#define AD9959_CS_PORT      GPIOE
#define AD9959_CS_PIN       GPIO_PIN_9      // PE9

#define AD9959_SCLK_PORT    GPIOJ
#define AD9959_SCLK_PIN     GPIO_PIN_6      // PJ6

#define AD9959_SDIO_PORT    GPIOJ
#define AD9959_SDIO_PIN     GPIO_PIN_9      // PJ9 (SD0)

// ---- 控制引脚 ----
#define AD9959_PDC_PORT     GPIOD
#define AD9959_PDC_PIN      GPIO_PIN_14     // PD14, 低电平正常工作

#define AD9959_RESET_PORT   GPIOD
#define AD9959_RESET_PIN    GPIO_PIN_0      // PD0

#define AD9959_UPDATE_PORT  GPIOE
#define AD9959_UPDATE_PIN   GPIO_PIN_7      // PE7

// ---- Profile 选择引脚 (P0~P3) ----
#define AD9959_P0_PORT      GPIOD
#define AD9959_P0_PIN       GPIO_PIN_15     // PD15

#define AD9959_P1_PORT      GPIOD
#define AD9959_P1_PIN       GPIO_PIN_1      // PD1

#define AD9959_P2_PORT      GPIOE
#define AD9959_P2_PIN       GPIO_PIN_8      // PE8

#define AD9959_P3_PORT      GPIOE
#define AD9959_P3_PIN       GPIO_PIN_10     // PE10

// ---- 其他SDIO引脚（备用，1线模式需拉低） ----
#define AD9959_SD1_PORT     GPIOK
#define AD9959_SD1_PIN      GPIO_PIN_2      // PK2

#define AD9959_SD2_PORT     GPIOJ
#define AD9959_SD2_PIN      GPIO_PIN_11     // PJ11

#define AD9959_SD3_PORT     GPIOK
#define AD9959_SD3_PIN      GPIO_PIN_1      // PK1

// ============================================================
// 用户API函数
// ============================================================
void AD9959_Init(void);
void AD9959_WriteData(uint8_t RegisterAddress, uint8_t NumberofRegisters, 
                      uint8_t *RegisterData, uint8_t temp);
void AD9959_IO_Update(void);
void AD9959_Reset(void);
void AD9959_SetFrequency(uint8_t Channel, uint32_t Freq);
void AD9959_SetAmplitude(uint8_t Channel, uint16_t Ampli);
void AD9959_SetPhase(uint8_t Channel, uint16_t Phase);
void AD9959_SyncPhase(void);
void AD9959_EnableChannel(uint8_t Channel);
void AD9959_DisableChannel(uint8_t Channel);

// ============================================================
// 通道宏定义
// ============================================================
#define CH0  0
#define CH1  1
#define CH2  2
#define CH3  3
#define CH_ALL 4

#endif
