# CMSIS-DSP FFT 双频检测 — 逻辑分析

## 一、为什么选 CMSIS-DSP 而非手写

| | 手写 Radix-2 | CMSIS-DSP arm_rfft_fast_f32 |
|------|-------------|------------------------------|
| 代码量 | ~50 行 | ~15 行 |
| 速度 | ~0.5ms (C 编译器优化后) | ~0.05ms (SIMD 汇编优化) |
| 精度 | 单精度 | 单精度 |
| 依赖 | 仅 math.h | arm_math.h + 库链接 |
| 维护 | 自己维护蝶形运算 | ST 官方维护 |

对 2048 点 FFT，CMSIS-DSP 比手写快约 10 倍。H 题要求 20 秒内完成，FFT 耗时占比完全可以忽略。

## 二、arm_rfft_fast_f32 工作流程

```
输入: float32_t in[2048]   — ADC 原始值转 float，去直流
输出: float32_t out[2048]  — 打包格式

打包格式 (rfft 特有)：
  out[0]   = re[0]    (DC, 实数)
  out[1]   = re[N/2]  (Nyquist, 实数)
  out[2]   = re[1]
  out[3]   = im[1]
  out[4]   = re[2]
  out[5]   = im[2]
  ...
  out[2k]     = re[k]    (k = 1..N/2-1)
  out[2k+1]   = im[k]
```

幅值 = re² + im²。跳过 bin 0（DC），从 bin 5 开始扫（避开 DC 附近低频噪声），找两个最大值。

## 三、频率换算

采样率 Fs 由 DWT 周期计数器实测：

```
Fs = N × CPU_CLK / (CYCCNT_end - CYCCNT_start)
   = 2048 × 480000000 / cycles

频率 f = bin_index × Fs / N
       = bin_index × Fs / 2048
```

bin 5 ~ 1023 覆盖的频率范围（Fs=1.25M~3.27M 时）：

| Fs | bin 5 频率 | bin 1023 频率 |
|------|-----------|--------------|
| 1.25 MHz | 3.0 kHz | 624 kHz |
| 3.27 MHz | 8.0 kHz | 1635 kHz |

H 题信号 20kHz~100kHz 完全在范围内。

## 四、CMSIS-DSP 使能步骤

**CubeMX 方式（推荐）：**
1. `Pinout & Configuration` → `Software Packs` → `Select Components`
2. 找到 `CMSIS Pack` → `DSP` → 勾选 `Library`（不是 Source）
3. GENERATE CODE — 自动添加 include 路径和链接库

**Keil 手动方式：**
1. `Options → C/C++ → Define` 加 `ARM_MATH_CM7`
2. `Options → C/C++ → Include Paths` 加 `CMSIS/DSP/Include`
3. `Options → Linker` 添加 `arm_cortexM7lfdp_math.lib`
4. 如报 `__builtin_arm_qadd8` 错误，Misc Controls 加 `-Wno-macro-redefined`

## 五、缓冲区规划

| 变量 | 类型 | 大小 | 说明 |
|------|------|------|------|
| adc_buf | uint16_t[2048] | 4KB | DMA 目标，SRAM1 |
| fft_in | float32_t[2048] | 8KB | FFT 输入 |
| fft_out | float32_t[2048] | 8KB | FFT 输出 |
| 合计 | | 20KB | H743 有 1MB，完全够 |

fft_in/out 用 static 声明，只分配一次，不反复 malloc。

## 六、完整数据流

```
串口按 '1'
  │
  ├→ HAL_ADC_Stop_DMA           // 确保 ADC 空闲
  ├→ t0 = DWT->CYCCNT           // 记录起始时刻
  ├→ HAL_ADC_Start_DMA(buf,2048) // 启动 DMA 采集
  │
  ├→ TIM2 TRGO 触发 ADC → DMA 搬 2048 点 → ConvCpltCallback
  │                                                │
  │                                          SCB_InvalidateDCache
  │                                          adc_done = 1
  │
  ├→ while(!adc_done) 等待
  ├→ cycles = CYCCNT - t0        // 采集耗时
  ├→ Fs = 2048 × 480M / cycles   // 实际采样率
  │
  ├→ 加载 adc_buf → fft_in       // uint16 → float32
  ├→ 去直流 (减均值)
  ├→ arm_rfft_fast_f32(fft_in, fft_out)  // FFT
  │
  ├→ 遍历 bin 5..1023
  │    mag = out[2k]² + out[2k+1]²
  │    找最大的两个峰 → pk1, pk2
  │
  ├→ if(pk1 > pk2) swap          // 低频在前
  ├→ f1 = pk1 × Fs / 2048
  ├→ f2 = pk2 × Fs / 2048
  │
  ├→ printf("FS=%u,FA=%uHz,FB=%uHz")
  ├→ printf 2048 行原始数据 + ---END---
  └→ LED 翻转，等待下一次按 1
```

## 七、与现有零交叉方案的差异

| | 零交叉 | CMSIS-DSP FFT |
|------|--------|---------------|
| 单音调 | 准 | 准 |
| 双音调混合 | 不工作 | 正确分离两个峰 |
| 三角波 | 谐波干扰 | 可识别基频+谐波 |
| 计算量 | 极小 | ~0.05ms |
| 适合 H 题 | ❌ 只能单音 | ✅ 覆盖全部要求 |
