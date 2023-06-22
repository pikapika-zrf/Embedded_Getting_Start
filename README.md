# Embedded_Getting_Started

从STM32到Freertos再到Linux，从基础学习到项目开发

- STM32 HAL库 -> 寄存器
- 基于STM32的 Freertos
- Linux应用 -> 驱动

# STM32

ST中文社区网https://www.stmcu.org.cn/

ST官网https://www.st.com/

项目立创开源硬件平台https://oshwhub.com/

来源[正点原子b站视频](https://www.bilibili.com/video/BV1bv4y1R7dp?p=4&spm_id_from=pageDriver&vd_source=f8705e057957dafd33c98d31eb54e3cd)与开发板资料

## 一、简介

### 1.1 命名

ARM11之后改用Cortex命名，分为三个系列

- Cortex-A系列：面向基于虚拟内存的操作系统和用户应用（智能手机、平板电脑和机顶盒等）；
- Cortex-R系列：用作实时系统；
- Cortex-M系列：用作微控制器，STM32单片机采用Cortex-内核。

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306092238624.png" style="zoom: 50%;" />

Tips：

1. STM32选型，由高到低，由大到下。H7->F4->F1(代码优化)
2. 数据手册获取:[ST官网](https://www.st.com/)和[ST中文社区网](https://www.stmcu.org.cn/)

### 1.2 STM32F103ZET6硬件系统

1. ARM Cortex-M3内核，主频72MHz，存储器容量412KB Flash和64KB SRAM。

2. 工作电压：2.0~3.6V。最大电流150mA

3. 有144个引脚，其中112个IO引脚（PA0\~PA15、PB0\~PB15、PC0\~PC15、PD0\~PD15、PE0\~PE15、PF0\~PF15、PG0\~PG15）。

   IO引脚接入电压范围：CMOS端口：3.3V（-0.3V\~3.6V）。兼容5V端口：5V（-0.3V\~5.5V）

   单个IO引脚最大电流：25mA

4. 其他为电源、复位、时钟、启动方式、后备供电、参考电压引脚。

5. 封装LQFP144

### 1.3 最小系统

保证MCU正常工作的最小电路组成单元。

STM32芯片、电源电路、晶振电路、复位电路、BOOT启动电路、下载调试端电路。

![image-20230614113013604](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306141130747.png)

### 1.4 单片机程序烧录方式

- ISP（In-System Programming）
  在系统编程，使用引导程序（Bootloader）加上外围**UART/SPI等接口**进行烧录。
- ICP （In-circuit programmer）
  在电路编程，使用**SWD/JTAG接口**。
- IAP（In-Application Programming）
  指MCU可以在系统中获取新代码并对自己重新编程，即**用程序来改变程序**。

> 系统存储器（起始位置0x1FFF 000）,STM32在出厂时，在这个区域内部预置了一段BootLoader， 也就是我们常说的ISP程序 ，这是一块ROM，出厂后无法修改。
>
> 通常，我们编写的代码，是放到**主存储器**的起始位置（0x0800 0000）开始运行的，烧录程序时，直接将程序烧录到这里即可。

**启动方式选择，参考手册2.4节**

![image-20230614144553455](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306141445542.png)

- BOOT0=0, BOOT1任意

在主存储区运行程序，BOOT0引脚拉低即可。

- BOOT0=1,BOOT1=0

用于串口方式烧录。单片机从系统存储器启动，运行BootLoader程序，接收串口发来的程序，并将其写入主存储器。程序烧录完成后，再将BOOT0引脚拉低，主存储器处运行刚烧录的代码。

- BOOT0=1,BOOT1=1

从内存中启动，内置SRAM，既然是SRAM，自然也就没有程序存储的能力了，一般用于程序调试。

### 1.5 下载接口

方式：JTAG、SWD、串口。建议使用SWD，占用IO少并且可以调试。

![image-20230614110346102](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306141107599.png)

1. ST-Link下载调试器

一端为USB接口（与电脑连接）。另一端如采用SWD方式与STM32单片机连接，只需要使用其中4针（3.3V、GND、SWCLK、SWDIO）。

**ST-Link与STM32接线：**

​	3.3V-VDD

​	GND-VSS

​	SWCLK-SWCLK

​	SWDIO-SWDIO

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306141544675.png" alt="image-20230614154426547" style="zoom:50%;" />

![image-20230614154458449](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306141544520.png)

### 1.6 JTAG/SWD调试

Cortex-M内核含有硬件调试模块，该模块可在取指(指令断点)或访问数据(数据断点)时停止。内核停止时，可以查询内核的内部状态和系统的外部状态。完成查询后，可恢复程序执行。

见参考手册29.4

![image-20230614153216071](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306141542414.png)

F1系列可以用复用重映射和调试I/O配置寄存器(AFIO_MAPR)寄存器(见8.4.2节)来选择引脚功能。  

F4/F6/H7默认SWJ-DP引脚为复用功能

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306141602179.png" alt="image-20230614160251013" style="zoom: 33%;" />

## 二、STM32系统框架

### 2.1 F1系统架构

1. 4个主动单元与4个被动单元。见参考手册2.1

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306142248831.png" alt="image-20230614224836680" style="zoom:33%;" />

见数据手册（3 引脚定义附近）

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306142252801.png" alt="image-20230614225129156" style="zoom:50%;" />

2. 总线时钟频率：

AHB：72MHz(Max)

APB1：36MHz(Max)

APB2：72MHz(Max)

### 2.2 存储器映射

存储器（FLASH,SRAM）本身没有地址信息，对存储器分配地址的过程称为存储器映射。

![image-20230615194651882](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306151946756.png)

存储器由程序存储器、数据存储器和寄存器组成。

存储器映射：分配地址，访问这些存储器

STM32可管理4GB的存储空间，地址编号采用8位十六进制数，地址编号范围为0x0000 0000 \~ 0xFFFF FFFF（$2^{32}B = 4GB,4*8$），分成4个块，每个块512MB，用作不同的功能。

### 2.3 寄存器映射

- 单片机内部特殊的内存，实现对单片机各功能的控制。寄存器就是单片机内部的控制结构。
- 寄存器是特殊的存储器，寄存器映射就是给寄存器地址命名的过程。

#### 1.直接操作寄存器地址：

```C
*(unsigned int *)(0x4001080C) = 0xFFFF;
```

#### 2.定义一个名字再操作：

```C
#define GPIOA_ODR *(unsigned int *)(0x4001080C)
GPIOA_ODR = 0xFFFF;
```

#### 3.为方便编写代码及使用，将寄存器地址分为三个部分：

1. 总线基地址（BUS_BASE_ADDR）
2. 外设基于总线基地址的偏移量（PERIPH_OFFSET）
3. 寄存器相对外设基地址的偏移量（REG_OFFSET）

寄存器地址 = BUS_BASE_ADDR + PERIPH_OFFSET + REG_OFFSET

#### 4.使用结构体，方便的完成寄存器映射

```C
typedef struct
{
    __IO uint32_t CRL;
    __IO uint32_t CRH;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t BRR;
    __IO uint32_t LCKR;
}GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
GPIOA->ODR = 0xFFFF;
```

stm32f103xe.h主要组成部分。

### 2.4 HAL库

- CMSIS：微控制器软件接口标准

- 用户层（用户代码）->中间层（CMSIS）->硬件库

- ST为开发STM32提供三种库:

1. 标准外设库
2. HAL库（硬件抽象层）
3. LL库

直接操作寄存器。执行效率高、时间成本高。

标准库。停止维护。

HAL库。全系列兼容，ST主推的库，兼容性、易移植性、效率低。

LL库。全系列兼容、与HAL库捆绑发布、轻量级、效率高、不匹配部分复杂外设。

**Tips：**HAL库回调函数通常被**_weak修饰**（弱函数），允许用户重新定义该函数。

**如何使用HAL库**

1. CMSIS核心层

   device和core中的11个文件

   `stm32f1xx.h`，`stm32f103xe.h`，`system_stm32f1xx.h`，`system_stm32f1xx.c`，`startup_stm32f1xx.s`，`core_cm3`，`cmsis_armcc.h`，`cmsis_armclang.h`，`cmsis_compiler.h`，`cmsis_version.h`，`mpu_armv7.h`

2. HAL库

   `sm32f1xx_hal.c`,`stm32f1xx_hal.h`,`stm32hxx_hal_def.h`,

   `stm32f1xx_hal_ppp.c`,`stm32f1xx_hal_ppp.h`,

   `stm32f1xx_hal_ppp_ex.c`,`stm32f1xx_hal_ppp_ex.h`,`stm32f1xx_II_ppp.c`,`stm32f1xx_II_ppp.h`

3. 用户程序文件

   `main.c`,`stm32f1xx_it.c`,`stm32f1xx_it.h`,`stm32f1xx_hal_conf.h`
   不是必须：`main.h`,`stm32f1xx_hal_msp.c`

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306192203548.png" alt="image-20230619220342802" style="zoom:33%;" />

HAL库的用户配置文件`stm32f1xx_hal_conf.h`

1. 裁剪HAL库外设驱动源码

   注释掉不需要的宏定义

2. 设置外部低速和高速晶振

### 2.5 MAP文件

分析各个.c文件占用FLASH和RAM的大小，方便优化代码

### 2.6 启动模式/启动过程

1. 从地址0x0000 0000取堆栈指针MSP的初始值，该值是栈顶地址
2. 从地址0x0000 0000取出程序计数器指针PC，该值是复位向量

系统复位后，SYSCLK的第4个上升沿，BOOT引脚的值被锁存

**启动过程**

复位->获取MSP值->获取PC值->Reset_Handler->启动文件startup_stm32xxx.s->main函数

**启动文件：**

初始化MSP、初始化PC、设置堆栈大小、初始化中断向量表、调用初始化函数、调用__main

## 三、STM32开发

### 3.1 时钟树

**配置函数：**

1. 时钟源、锁相环：HAL_RCC_OscConfig()

2. 系统时间、总线：HAL_RCC_CLockConfig()

3. 使能外设时钟（宏）：__Hal_RCC_PPP_CLK_ENABLE()

4. 扩展外设时钟（RTC/ADC/USB）：HAL_RCCEx_PeriphCLKConfig()

**系统时钟配置：**

1. 配置HSE_VALUE，设置HAL库外部晶振频率，stm32xxxx_hal_conf.h
2. 调用SystemInit()函数（可选），启动文件中调用，system_stm32xxxx.c定义
3. 选择时钟源，配置PLL，HAL_RCC_OscConfig()函数设置
4. 选择系统时钟源，配置总线分频器，HAL_RCC_CLockConfig()函数设置
5. 配置扩展外设时钟（可选），HAL_RCCEx_PeriphCLKConfig()函数设置

**SysTick**

系统滴答定时器，包含在M3/4/7内核里，核心是一个24位的递减计数器。

**delay**

**printf**

### 3.2 GPIO

**特点**

1. 快速翻转，每次翻转最快只需要两个时钟周期
2. 每个IO口都可以做中断
3. 支持8中工作模式

**电气特性**

1. 工作电压范围。2V<=VDD<=3.6V

2. 识别电压范围。CMOS端口：3.3V。-0.3V<=$V_{IL}$<=1.164V。1.833V<=$V_{IH}$<=3.6V

   TTL端口（数据手册中I/O电平标记为FT）：5V。

3. 输出电流。单个IO，最大25mA。

**8种工作模式**

**寄存器**

见参考手册 8.2

![image-20230622133039158](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306221330221.png)























Tips：

1. 

```c
//给位6置一
temp |= 1<<6
//给位6置0
temp &= ~(1<<6);
//给位6翻转
temp ^= 1<<6
```

2. 使用do{...}while(0)构造宏定义









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

