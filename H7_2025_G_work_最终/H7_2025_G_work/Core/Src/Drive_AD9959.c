#include "Drive_AD9959.h"
#include <stdio.h>

// ============================================================
// 内部寄存器缓存
// ============================================================
static uint8_t g_CSR_DATA[1] = {0xF0};      // 默认全部通道使能
static uint8_t g_CFTW_DATA[4] = {0x00, 0x00, 0x00, 0x00};
static uint8_t g_CPOW_DATA[2] = {0x00, 0x00};
static uint8_t g_ACR_DATA[3] = {0x00, 0x13, 0xFF};  // 默认最大幅度

// ============================================================
// 延时函数（只用于提供保守的软件 SPI 时序，不用于精确定时）
// ============================================================
static void delay_us(uint32_t us)
{
    us *= 480;
    while(us--) {
        __NOP();
    }
}

// ============================================================
// AD9959 复位 (E5)
// ============================================================
void AD9959_Reset(void)
{
    HAL_GPIO_WritePin(AD9959_RESET_PORT, AD9959_RESET_PIN, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(AD9959_RESET_PORT, AD9959_RESET_PIN, GPIO_PIN_SET);
    delay_us(50);
    HAL_GPIO_WritePin(AD9959_RESET_PORT, AD9959_RESET_PIN, GPIO_PIN_RESET);
    delay_us(100);
}

// ============================================================
// IO_UPDATE 脉冲 (E2)
// ============================================================
void AD9959_IO_Update(void)
{
    HAL_GPIO_WritePin(AD9959_UPDATE_PORT, AD9959_UPDATE_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(AD9959_UPDATE_PORT, AD9959_UPDATE_PIN, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(AD9959_UPDATE_PORT, AD9959_UPDATE_PIN, GPIO_PIN_RESET);
    delay_us(2);
}

// ============================================================
// SPI 写数据（核心函数）
// ============================================================
void AD9959_WriteData(uint8_t RegisterAddress, uint8_t NumberofRegisters, 
                      uint8_t *RegisterData, uint8_t temp)
{
    uint8_t ControlValue = RegisterAddress & 0x7F;  // 最高位0=写
    uint8_t ValueToWrite;
    uint8_t i, j;
    
    // ---- 1. 拉低CS (A5) ----
    HAL_GPIO_WritePin(AD9959_CS_PORT, AD9959_CS_PIN, GPIO_PIN_RESET);
    delay_us(2);
    
    // ---- 2. 发送指令字节（8位地址） ----
    for(i = 0; i < 8; i++)
    {
        // SCLK (D7) 拉低
        HAL_GPIO_WritePin(AD9959_SCLK_PORT, AD9959_SCLK_PIN, GPIO_PIN_RESET);
        delay_us(1);
        
        // 设置 SDIO (G10) 数据
        if(ControlValue & 0x80)
            HAL_GPIO_WritePin(AD9959_SDIO_PORT, AD9959_SDIO_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(AD9959_SDIO_PORT, AD9959_SDIO_PIN, GPIO_PIN_RESET);
        
        delay_us(1);  // 数据建立时间
        
        // SCLK 上升沿锁存
        HAL_GPIO_WritePin(AD9959_SCLK_PORT, AD9959_SCLK_PIN, GPIO_PIN_SET);
        delay_us(2);
        
        ControlValue <<= 1;
    }
    
    // ---- 3. 发送数据字节 ----
    for(i = 0; i < NumberofRegisters; i++)
    {
        ValueToWrite = RegisterData[i];
        
        for(j = 0; j < 8; j++)
        {
            HAL_GPIO_WritePin(AD9959_SCLK_PORT, AD9959_SCLK_PIN, GPIO_PIN_RESET);
            delay_us(1);
            
            if(ValueToWrite & 0x80)
                HAL_GPIO_WritePin(AD9959_SDIO_PORT, AD9959_SDIO_PIN, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(AD9959_SDIO_PORT, AD9959_SDIO_PIN, GPIO_PIN_RESET);
            
            delay_us(1);
            
            HAL_GPIO_WritePin(AD9959_SCLK_PORT, AD9959_SCLK_PIN, GPIO_PIN_SET);
            delay_us(2);
            
            ValueToWrite <<= 1;
        }
    }
    
    // ---- 4. SCLK 回到低电平 ----
    HAL_GPIO_WritePin(AD9959_SCLK_PORT, AD9959_SCLK_PIN, GPIO_PIN_RESET);
    delay_us(1);
    
    // ---- 5. 拉高CS (A5) ----
    HAL_GPIO_WritePin(AD9959_CS_PORT, AD9959_CS_PIN, GPIO_PIN_SET);
    delay_us(2);
    
    // ---- 6. 如果需要，触发IO_UPDATE ----
    if(temp == 1) {
        AD9959_IO_Update();
    }
}

// ============================================================
// 通道使能
// ============================================================
void AD9959_EnableChannel(uint8_t Channel)
{
    switch(Channel)
    {
        case CH0: g_CSR_DATA[0] = 0x10; break;
        case CH1: g_CSR_DATA[0] = 0x20; break;
        case CH2: g_CSR_DATA[0] = 0x40; break;
        case CH3: g_CSR_DATA[0] = 0x80; break;
        case CH_ALL: g_CSR_DATA[0] = 0xF0; break;
        default: return;
    }
    AD9959_WriteData(CSR_ADD, 1, g_CSR_DATA, 1);
}

// ============================================================
// 通道禁用
// ============================================================
void AD9959_DisableChannel(uint8_t Channel)
{
    g_CSR_DATA[0] = 0x00;
    AD9959_WriteData(CSR_ADD, 1, g_CSR_DATA, 1);
}

// ============================================================
// 设置频率
// ============================================================
void AD9959_SetFrequency(uint8_t Channel, uint32_t Freq)
{
    uint32_t Temp;
    
    if(Freq > 500000000) Freq = 500000000;
    
    // FTW = Freq * (2^32 / 500MHz)
    Temp = (uint32_t)((double)Freq * 8.589934592);
    printf("SetFreq CH%d=%uHz\r\n", Channel, (unsigned int)Freq);
    g_CFTW_DATA[3] = (uint8_t)Temp;
    g_CFTW_DATA[2] = (uint8_t)(Temp >> 8);
    g_CFTW_DATA[1] = (uint8_t)(Temp >> 16);
    g_CFTW_DATA[0] = (uint8_t)(Temp >> 24);
    
    // 先选中通道，再写频率
    switch(Channel)
    {
        case CH0: g_CSR_DATA[0] = 0x10; break;
        case CH1: g_CSR_DATA[0] = 0x20; break;
        case CH2: g_CSR_DATA[0] = 0x40; break;
        case CH3: g_CSR_DATA[0] = 0x80; break;
        case CH_ALL: g_CSR_DATA[0] = 0xF0; break;
        default: return;
    }
    
    AD9959_WriteData(CSR_ADD, 1, g_CSR_DATA, 0);
    AD9959_WriteData(CFTW0_ADD, 4, g_CFTW_DATA, 1);
}

// ============================================================
// 设置幅度 (0~576mV)
// ============================================================
void AD9959_SetAmplitude(uint8_t Channel, uint16_t Ampli)
{
    uint32_t A_temp;
    
    if(Ampli > 576) Ampli = 576;
    
    // DAC幅度控制：10位数据
    A_temp = (uint32_t)(((float)Ampli / 576.0f) * 1023.0f);
    A_temp |= 0x1000;  // 使能幅度控制
    g_ACR_DATA[2] = (uint8_t)A_temp;
    g_ACR_DATA[1] = (uint8_t)((A_temp >> 8) | 0x10);
    g_ACR_DATA[0] = 0x00;
    
    switch(Channel)
    {
        case CH0: g_CSR_DATA[0] = 0x10; break;
        case CH1: g_CSR_DATA[0] = 0x20; break;
        case CH2: g_CSR_DATA[0] = 0x40; break;
        case CH3: g_CSR_DATA[0] = 0x80; break;
        default: return;
    }
    
    AD9959_WriteData(CSR_ADD, 1, g_CSR_DATA, 0);
    AD9959_WriteData(ACR_ADD, 3, g_ACR_DATA, 1);
}

// ============================================================
// 设置相位 (0~360度)
// ============================================================
void AD9959_SetPhase(uint8_t Channel, uint16_t Phase)
{
    uint16_t P_temp;
    
    if(Phase > 360) Phase = 360;
    
    // 相位转换：Phase * (2^14 / 360)
    P_temp = (uint16_t)((float)Phase * 45.511111f);
    g_CPOW_DATA[1] = (uint8_t)P_temp;
    g_CPOW_DATA[0] = (uint8_t)(P_temp >> 8);
    
    switch(Channel)
    {
        case CH0: g_CSR_DATA[0] = 0x10; break;
        case CH1: g_CSR_DATA[0] = 0x20; break;
        case CH2: g_CSR_DATA[0] = 0x40; break;
        case CH3: g_CSR_DATA[0] = 0x80; break;
        case CH_ALL: g_CSR_DATA[0] = 0xF0; break;
        default: return;
    }
    
    AD9959_WriteData(CSR_ADD, 1, g_CSR_DATA, 0);
    AD9959_WriteData(CPOW0_ADD, 2, g_CPOW_DATA, 1);
}

// ============================================================
// 同步所有通道相位（FR2寄存器）
// ============================================================
void AD9959_SyncPhase(void)
{
    uint8_t FR2_CLEAR[2] = {0x10, 0x00};
    uint8_t FR2_NORMAL[2] = {0x00, 0x00};
    
    // 清除相位累加器
    AD9959_WriteData(FR2_ADD, 2, FR2_CLEAR, 1);
    delay_us(10);
    
    // 恢复正常模式
    AD9959_WriteData(FR2_ADD, 2, FR2_NORMAL, 1);
    delay_us(10);
}

// ============================================================
// GPIO 初始化
// ============================================================
static void AD9959_GPIO_Init(void)
{
    // PDC 低电平：退出掉电状态
    HAL_GPIO_WritePin(AD9959_PDC_PORT, AD9959_PDC_PIN, GPIO_PIN_RESET);

    // CS 默认高
    HAL_GPIO_WritePin(AD9959_CS_PORT, AD9959_CS_PIN, GPIO_PIN_SET);
    
    // SCLK 默认低
    HAL_GPIO_WritePin(AD9959_SCLK_PORT, AD9959_SCLK_PIN, GPIO_PIN_RESET);
    
    // SD0 默认低
    HAL_GPIO_WritePin(AD9959_SDIO_PORT, AD9959_SDIO_PIN, GPIO_PIN_RESET);
    
    // RESET 默认低
    HAL_GPIO_WritePin(AD9959_RESET_PORT, AD9959_RESET_PIN, GPIO_PIN_RESET);
    
    // IO_UPDATE 默认低
    HAL_GPIO_WritePin(AD9959_UPDATE_PORT, AD9959_UPDATE_PIN, GPIO_PIN_RESET);
    
    // P0~P3 是 Profile 选择脚；单音模式固定选择 Profile 0
    HAL_GPIO_WritePin(AD9959_P0_PORT, AD9959_P0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_P1_PORT, AD9959_P1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_P2_PORT, AD9959_P2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_P3_PORT, AD9959_P3_PIN, GPIO_PIN_RESET);
    
    // ---- SD1/SD2/SD3 全部拉低（1线模式） ----
    HAL_GPIO_WritePin(AD9959_SD1_PORT, AD9959_SD1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_SD2_PORT, AD9959_SD2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_SD3_PORT, AD9959_SD3_PIN, GPIO_PIN_RESET);
}

// ============================================================
// AD9959 完整初始化
// ============================================================
void AD9959_Init(void)
  
{
	  printf("AD9959_Init start...\r\n");
    __disable_irq();
    
    // ---- 1. GPIO 初始化 ----
    AD9959_GPIO_Init();
    printf("  GPIO ok\r\n");
    printf("  Software SPI: SCLK=PJ6, SD0=PJ9\r\n");
    delay_us(10000);
    
    // ---- 2. 硬件复位 ----
    AD9959_Reset();
        printf("  Reset ok\r\n"); delay_us(50000);
    
    // ---- 3. 配置 FR1 (PLL倍频) ----
    // 0xD3 = 0b11010011 → 500MHz PLL输出
    uint8_t fr1_data[3] = {0xD3, 0x00, 0x00};
    AD9959_WriteData(FR1_ADD, 3, fr1_data, 1);
    printf("  FR1 ok\r\n");
    delay_us(10000);
    
    // ---- 4. 配置 CFR (关闭4线模式) ----
    uint8_t cfr_data[3] = {0x00, 0x03, 0x00};
    AD9959_WriteData(CFR_ADD, 3, cfr_data, 1);
    printf("  CFR ok\r\n");
    delay_us(10000);
    // ---- 5. 初始化为 1kHz / 幅值关闭，等待上层选择题目模式 ----
    AD9959_SetFrequency(CH0, 1000);
    printf("  Freq ok\r\n");  
    AD9959_SetAmplitude(CH0, 0);
    printf("  Amp ok\r\n");
    
    // ---- 6. 同步相位 ----
    AD9959_SyncPhase();
    printf("  Sync ok\r\n");
    
    // 保持 CSR 只选择 CH0，避免后续通道寄存器写入影响其他通道
    AD9959_EnableChannel(CH0);
    
    __enable_irq();
    printf("AD9959_Init done.\r\n");
}
