# Embedded_Getting_Start

从STM32到Freertos再到Linux，从基础学习到项目开发

- STM32 HAL库 -> 寄存器
- 基于STM32的 Freertos
- Linux应用 -> 驱动

# STM32

ST中文社区网https://www.stmcu.org.cn/

ST官网https://www.st.com/

项目立创开源硬件平台https://oshwhub.com/

## 一、简介

### 1.命名

ARM11之后改用Cortex命名，分为三个系列

- Cortex-A系列：面向基于虚拟内存的操作系统和用户应用（智能手机、平板电脑和机顶盒等）；
- Cortex-R系列：用作实时系统；
- Cortex-M系列：用作微控制器，STM32单片机采用Cortex-内核。

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306092238624.png" style="zoom: 50%;" />

### 2.最小系统电路

STM32单片机芯片、电源电路、复位电路、时钟电路、启动方式设置电路、下载调试端、

### 3.ST-Link下载调试器

一端为USB接口（与电脑连接）。另一端如果采用SWD方式与STM32单片机连接，只需要使用其中4针（3.3V、GND、SWCLK、SWDIO）。

**ST-Link-STM32接线：**

​	3.3V-VDD

​	GND-VSS

​	SWCLK-SWCLK

​	SWDIO-SWDIO

### 4.STM32F103ZET6硬件系统

ARM Cortex-M3内核，存储器容量412KB Flash和64KB SRAM， 有144个引脚，其中112个IO引脚（PA0\~PA15、PB0\~PB15、PC0\~PC15、PD0\~PD15、PE0\~PE15、PF0\~PF15、PG0\~PG15）,其他为电源、复位、时钟、启动方式、后备供电、参考电压引脚。

### 5.STM32存储器的地址分配图

![img](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306121116927.png)

存储器由程序存储器、数据存储器和寄存器组成。

存储器映射：分配地址，访问这些存储器

STM32可管理4GB的存储空间，地址编号采用8位十六进制数，地址编号范围为0x0000 0000 \~ 0xFFFF FFFF（$2^{32}B = 4GB,4*8$），分成4个块，每个块512MB，用作不同的功能。

### 6.



# Freertos

官网https://www.freertos.org/

Tips：

FreeRTOS 不使用 C99 标准引入或其之后出现的任何 C 语言功能或语法。但使用 `stdint.h` 头文件的情况除外。`FreeRTOS/Source/include` 目录包含一个名为 `stdint.readme` 的文件。可将该文件重命名为 `stdint.h`，以提供构建 FreeRTOS 所需的最低 stdint 类型定义，但前提是用户的编译器本身无此类型定义。

## 项目

### 音乐播放器



# Linux

## 项目

### 视频播放器

mplayer移植

