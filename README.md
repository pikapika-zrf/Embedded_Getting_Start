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

### 1.1 STM32系列芯片命名

ARM11之后改用Cortex命名，分为三个系列

- Cortex-A系列：面向基于虚拟内存的操作系统和用户应用（智能手机、平板电脑和机顶盒等）；
- Cortex-R系列：用作实时系统；
- Cortex-M系列：用作微控制器，STM32单片机采用Cortex-内核。

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306092238624.png" style="zoom: 50%;" />

tips：

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

1. STM32芯片
2. 电源电路。VSS/VDD等。**参考手册4.1**
3. 复位电路。NRST。**参考手册6.1.2**
4. 晶振电路。OSC_IN/OSC_OUT等。**参考手册7.2.1、7.2.4**
5. BOOT启动电路。BOOT0/BOOT1。**参考手册2.4**
6. 下载调试端电路。推荐（SW接口）SWDIO/SWCLK。**参考手册29.3**

### 1.4 程序烧录

- ISP（In-System Programming）
  在系统编程，使用引导程序（Bootloader）加上外围**UART/SPI等接口**进行烧录。
- ICP （In-circuit programmer）
  在电路编程，使用**SWD/JTAG接口**。
- IAP（In-Application Programming）
  指MCU可以在系统中获取新代码并对自己重新编程，即**用程序来改变程序**。

> 系统存储器（起始位置0x1FFF 000）,STM32在出厂时，在这个区域内部预置了一段BootLoader， 也就是我们常说的ISP程序 ，这是一块ROM，出厂后无法修改。
>
> 通常，我们编写的代码，是放到**主存储器**的起始位置（0x0800 0000）开始运行的，烧录程序时，直接将程序烧录到这里即可。

### 1.5 下载接口

SWD方式与STM32单片机连接，**ST-Link与STM32接线：**

​	3.3V-VDD / GND-VSS / SWCLK-SWCLK / SWDIO-SWDIO

MDK工程中魔术棒Debug选项卡选择调试器并设置settings。

​								Utilities选项卡选择use Debug Driver。

tips：

1. 结束仿真报错解决方法：
   1. 仿真结束前将所有断点清除
   2. 将工程路径改浅，或全改为英文路径

## 二、STM32系统框架

### 2.1 F1系统架构

1. 4个主动单元与4个被动单元。**见参考手册2.1。**

   ​	D-bus S-bus DMA1 DMA2

   ​	SRAM FLASH FSMC AHB\APB

   模块框图**见数据手册**（3 引脚定义附近）

2. 总线时钟频率：

   AHB：72MHz(Max)

   APB1：36MHz(Max)	APB2：72MHz(Max)

### 2.2 存储器映射

存储器（FLASH,SRAM）本身没有地址信息，对存储器分配地址的过程称为存储器映射。**见数据手册4。**

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

STM32单片机编程主要有寄存器方式和库函数（固件库）方式。

寄存器方式：直接操作单片机内部硬件的寄存器。生成的程序代码量少，要求对硬件和相关寄存器很熟悉，开发难度大，维护调试比较繁琐。

库函数方式：通过调取固件库中不同功能的函数，让函数来操作单片机内部的寄存器。不要求用户很熟悉单片机内部硬件，开发难度小，维护调试比较容易，但生成的程序代码量大。

[ST官网](https://www.st.com/)搜索STM32Cube下载固件库。



**tips：**

1. HAL库回调函数通常被**_weak修饰**（弱函数），允许用户重新定义该函数。
2. LL库。全系列兼容、与HAL库捆绑发布、轻量级、效率高、不匹配部分复杂外设。
3. CMSIS：微控制器软件接口标准。用户层（用户代码）->中间层（CMSIS）->硬件库

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

HAL库的用户配置文件`stm32f1xx_hal_conf.h`

1. 裁剪HAL库外设驱动源码

   注释掉不需要的宏定义

2. 设置外部低速和高速晶振

重要文件说明

| 文件名                                       | 功能描述           | 具体说明                                                     |
| -------------------------------------------- | ------------------ | ------------------------------------------------------------ |
| core_cm3.h                                   | 内核及设备文件     | 访问内核及其设备（NVIC、Systics等），以及CPU寄存器和内核外设的函数。<br />在Drivers\CMSIS\Core\Include下。 |
| system_stm32f1xx.h<br />system_stm32f1xx.c   | MCU专用系统文件    | 包含时钟的相关函数。有SystemInit()函数声明，该函数在系统启动时会调用，用来设置整个系统和总线的时钟。<br />在Drivers\CMSIS\Device\ST\STM32F1xx\Include下。 |
| stm32f103xe.h                                | 微控制器专用头文件 | 外设寄存器的定义（寄存器基地址等）、位定义、中断向量表、存储空间的地址映射。<br />在Drivers\CMSIS\Device\ST\STM32F1xx\Include下。 |
| startup_stm32f103xe.s                        | 编译器启动代码     | 1.初始化堆栈指针SP。2.初始化程序计数器指针PC。3.设置堆栈大小。4.设置中断向量表入口地址。5.配置外部SRAM为数据存储器。6.调用SystemInit()函数配置STM32的系统时钟。7.设置C库的分支入口“__main”调用执行main函数。<br />在Drivers\CMSIS\Device\ST\STM32F1xx\Source\Templates\arm下。 |
| stm32f1xx_hal_conf.h                         | 固件库配置文件     | 通过更改包含的外设头文件来选择固件库所使用的外设，在新建程序和进行功能变更之前应当首先修改对应的配置。使用相应外设时应调用相关头文件，不使用则注释。<br />在Projects\STM3210E_EVAL\Templates\Inc下。 |
| stm32f1xx_it.h<br />stm32f1xx_it.c           | 外设中断函数文件   | 用户可以加入自己的中断程序代码。一般不用改动。<br />在Projects\STM3210E_EVAL\Templates\Inc下 |
| stm32f1xx_hal_ppp.h<br />stm32f1xx_hal_ppp.c | 外设驱动文件       | 包括相关外设初始化配置和部分功能应用函数。<br />在Drivers\STM32F1xx_HAL_Driver\Inc下 |
| Application.c                                | 用户程序文件       | 常用main.c。                                                 |



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

### 3.0 HAL库MDK工程

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

### 3.2 GPIO

通用输入输出端口。

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

输入浮空、输入上拉、输入下拉、模拟功能

开漏输出、推挽输出、开漏复用输出、推挽复用输出

**寄存器**

见参考手册 8.2

F1:

GPIOx_CRL、GPIOx_CRH、GPIOx_IDR、GPIOx_ODR、GPIOx_BSRR、GPIOx_BRR、GPIOx_LCKR

**通用外设驱动模式：**

1. 初始化。时钟设置、参数设置、IO设置、中断设置。
2. 读函数（可选）。
3. 写函数（可选）。
4. 中断服务函数（可选）。

**GPIO配置步骤：**

1. 使能时钟   __HAL_RCC_GPIOx_CLK_ENABLE()

2. 设置工作模式   HAL_GPIO_Init()
3. 设置输出状态（可选）HAL_GPIO_WritePin()、HAL_GPIO_TogglePin()
4. 读取输入状态（可选）HAL_GPIO_ReadPin()

tips：

1. F1和F4/F7/H7 内部上/下拉放的位置不同。
2. 施密特触发器，把非标准方波整形成方波
3. F1在输出模式，禁止使用内部上下拉
4. ODR和BSRR。建议使用BSRR控制输出

### 3.3 中断

打断CPU执行正常的程序，转而执行处理紧急程序，然后返回原暂停的程序继续运行。

**作用和意义**

实时控制、故障处理、数据传输

高效处理紧急程序，不会一直占用CPU资源。

流程：GPIO->AFIO(F1)/SYSCFG(F4)->EXTI->NVIC->CPU

#### **3.3.1 NVIC**

嵌套向量中断控制器，属于内核。

支持256个中断（16内核+240外部），支持：256个优先级，允许裁剪。ST只使用16个中断优先级

**中断向量表**

定义一块固定的内存，4字节对齐，存放各个中断服务函数程序的首地址。

中断向量表定义在启动文件。**见参考手册9**

**寄存器**

中断使能寄存器ISER、中断失能寄存器ICER、应用程序中断及复位控制寄存器AIRCR、中断优先级寄存器IPR（STM32只使用高4位）

ISER/ICER（中断使能/失能）->IPR（优先级）->AIRCR（控制分组）

**中断优先级**

1. 抢占优先级，高抢占优先级可以打断低抢占优先级
2. 响应优先级，抢占优先级相同时，响应优先级高的先执行，但是不能互相打断
3. 自然优先级，抢占和响应优先级都相同的情况下，自然优先级高的先执行
4. 数值越小，表示优先级越高

**STM32优先级分组**

![image-20230628230040382](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306282300778.png)

一个工程中一般只设置一次中断优先级分组

**见编程手册4.4.5**，自然优先级**见参考手册9.1.2**

**NVIC的使用**

1. 设置中断分组。AIRCR[10:8],HAL_NVIC_SetPriorityGrouping
2. 设置中断优先级。IPRx,bit[7:4],HAL_NVIC_SetPriority
3. 使能中断。ISER,HAL_NVIC_EnableIRQ

#### **3.3.2 EXTI**

外部（扩展）中断事件控制器。**见参考参考手册9.2**

包含20个产生事件/中断请求的边沿检测器，F1总共20条EXTI线。

**中断和事件**

中断：要进入NVIC，有相应的中断服务函数，需要CPU处理

事件：不进入NVIC，仅用于内部硬件自动控制，如：TIM、DMA、ADC

**特性**

F1/F4/F7：

每条EXTI都可以单独配置：选择类型（中断或者事件）、触发方式（上升下降沿）、支持软件触发、开启/屏蔽、有挂起状态

H7：

由其它外设对EXTI产生的事件分为可配置事件和直接事件。**见参考手册20.1**

**AFIO简介F1**

复用功能IO，用于重映射和外部中断映射配置。**见参考手册8.4.3**

1. 调试IO配置 AFIO_MAPR[26:24]
2. 重映射配置 AFIO_MAPR
3. 外部中断配置 AFIO_EXTICR1\~4，配置EXTI中断线0\~15对应到哪个具体IO口

- 配置AFIO寄存器之前要使能AFIO时钟 `__HAL_RCC_AFIO_CLK_ENABLE()`

**SYSCFG（F4/F7/H7）**

系统配置控制器，用于外部中断映射配置等

外部中断配置 SYSCFG_EXTICR1\~4，配置EXTI中断线0\~15对应到哪个具体IO口。

- 配置SYSCFG寄存器之前要使能SYSCFG时钟 `__HAL_RCC_SYSCFG_CLK_ENABLE()`

**如何使用中断**

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306291748580.png" alt="image-20230629174828416" style="zoom:33%;" />

**EXTI配置步骤（GPIO外部中断）**

1. 使能GPIO时钟。
2. 设置GPIO输入模式。上/下拉/浮空输入。
3. 使能AFIO/SYSCFG时钟。设置AFIO/SYSCFG时钟开启寄存器。
4. 设置EXTI和IO对应关系。AFIO_EXTICR/SYSCFG_EXTICR
5. 设置EXTI屏蔽，触发方式。IMR、RTSR/FTSR
6. 设置NVIC。设置优先级分组、设置优先级、使能中断。
7. 设置中断服务函数。编写函数，清除中断标志。

步骤2-5使用HAL_GPIO_Init一步到位

**EXTI的HAL库配置步骤（GPIO外部中断）**

1. 使能GPIO时钟。`__HAL_RCC_GPIOx_CLK_ENABLE`
2. GPIO/AFIO(SYSCFG)/EXTI。`HAL_GPIO_Init`
3. 设置中断分组。`HAL_NVIC_SetPriorityGrouping`
4. 设置中断优先级。`HAL_NVIC_SetPriority`
5. 使能中断。`HAL_NVIC_EnableIRQ`
6. 设计中断服务函数。`EXTIx_IRQHandler`

STM32仅有：EXTI0\~4（5个）、EXTI9\~5（1个）、EXTI15\~10（1个）。7个外部中断服务函数。

**HAL库中断回调处理机制**

中断服务函数->HAL库中断处理公用函数->HAL库数据处理回调函数

### 3.4 串口

#### 3.4.1 通信

单工通信：数据只沿一个方向传输。

半双工通信：数据可以沿两个方向传输，但需要分时进行。

全双工通信：数据可以同时进行双向传输。



同步通信：共用同一时钟信号。

异步通信：没有时钟信号，通过在数据信号中加入起始位和停止位等同步信号。



比特率：每秒传送的比特数。bit/s

波特率：每秒传送的码元数。Baud

比特率 = 波特率 * log2 M,M表示每个码元承载的信息量。

二进制系统中，波特率数值上等于比特率。

![image-20230629212617098](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306292126368.png)

#### 3.4.2 串口

- 串口通信接口：指按位发送和接收的接口。RS-232/485。

- RS-232接口（DB9）数据：

  ​	TXD:串口数据输出   RXD:串口数据输入   GND

**电平对比：**

RS-232电平：逻辑1：-15V ~ -3V   逻辑0：+3V ~ +15V

CMOS电平：逻辑1：3.3V   逻辑0：0V     STM32

TTL电平：逻辑1：5V   逻辑0：0V

RS-232电平不能与CMOS/TTL电平直接交换信息。

电平转换芯片MAX3232/SP3232(3.3V)。MAX232(5V)

**STM32串口与电脑USB通信**

USB/串口转换电路（CH340C）

**RS-232异步通信**

1. 启动位。占1个位长，保持逻辑0电平。
2. 有效数据位。可选5、6、7、8、9个位长，LSB在前，MSB在后。
3. 奇偶校验位。可选1个位长，可以没有。
4. 停止位。必须有，可选占0.5、1、1.5、2个位长，保持逻辑1电平。

![image-20230629214426176](https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202306292144264.png)

#### 3.4.3 STM32的USART

USART通用同步异步收发器。UART通用异步收发器。

USART/UART都可以全双工异步通信。

快速查看STM32外设的数量。查看ST MCU选型手册

**设置USART波特率（F1）**

波特率计算公式：$baud = \frac{f_{ck}}{16 * USARTDIV}$

$USARTDIV = DIV\_Mantissa+(DIV\_Fraction / 16)$

波特比例寄存器BRR

例：波特率115200

$115200 = \frac{72000000}{16 * USARTDIV}$，则USARTDIV算得$39.0625$

```c
uint16_t mantissa;
uint16_t fraction;
mantissa=39;
fraction = 0.0625 * 16 + 0.5;//得到0X01。加0.5四舍五入
USART1->BRR = (mantissa<<4)+fraction
//USART1->BRR = USARTDIV * 16 + 0.5;
//USART1->BRR = (f_ck + baud/2)/baud;
```

**USART寄存器**

控制寄存器1（CR1） 数据寄存器DR  状态寄存器SR

#### 3.4.4 HAL库外设初始化MSP回调机制

`HAL_PPP_Init()`调用MSP回调函数`HAL_PPP_MSPInit()`

`HAL_PPP_MSPInit()`是空函数，__weak修饰，允许用户重新定义。

**HAL库中断回调机制**

用户在中断服务函数`PPP_IRQHandler()`里调用HAL库共用中断函数`HAL_PPP_IRQHandler()`

再调用一系列中断回调函数`HAL_PPP_xxxCallback()`

`HAL_PPP_xxxCallback()`是空函数，__weak修饰，允许用户重新定义。

#### 3.4.5 USART/UART异步通信配置步骤

1. 配置串口工作参数。`HAL_UART_Init()`
2. 串口底层初始化。`HAL_UART_MspInit()`
3. 开启串口异步接收中断。`HAL_UART_Receive_IT()`
4. 设置优先级，使能中断。`HAL_NVIC_SetPriority()`、`HAL_NVIC_EnableIRQ()`
5. 编写中断服务函数。`USARTx_IRQHandler()`、`UARTx_IRQHandler()`
6. 串口数据发送。`USART_DR()`、`HAL_UART_Transmit()`

### 3.5 IWDG

独立看门狗，能产生系统复位信号的计数器。递减的计数器，时钟由独立的RC振荡器提供，计数到0时产生复位。

喂狗。在计数器计数到0之前，重装载计数器的值，防止复位。

**作用：**

用于检测外部干扰或软硬件异常造成程序跑飞的问题。用于高稳定性且对时间精度要求较低的场景。IWDG是异常处理的最后手段，不可依赖。

**寄存器：**

键寄存器IWDG_KR。喂狗、解除PR与RLR寄存器写访问保护、启动看门狗工作。

预分频器寄存器IWDG_PR。

重装载寄存器IWDG_RLR。

状态寄存器IWDG_SR。

**寄存器配置操作步骤**

1. IWDG_KR写入0xCCCC使能IWDG
2. IWDG_KR写入0x5555使能寄存器访问
3. 预分频器IWDG_PR配置预分频器
4. 重载寄存器IWDG_RLR进行写操作
5. 等待寄存器IWDG_SR更新(IWDG_SR = 0x0000 0000)
6. 刷新计数器值为IWDG_RLR的值(IWDG_KR = 0xAAAA)

**IWDG溢出时间计算**

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202307041137550.png" alt="image-20230704113730440" style="zoom:33%;" />

**IWDG配置步骤**

1. HAL_IWDG_Init()。取消PR/RLR寄存器写保护，设置IWDG预分频系数和重装载值，启动IWDG。
2. HAL_IWDG_Refresh()。喂狗，即写入0xAAAA到IWDG_KR

### 3.6 WWDG

窗口看门狗。能产生系统复位信号和提前唤醒中断的计数器。

- 递减计数器，从0x40减到0x3F时复位（T6位跳变到0）。
- 计数器的值大于W[6:0]值时喂狗会复位。
- 提前唤醒中断（EWI）：当递减计数器等于0x40时可产生。

喂狗：在窗口期内装载计数器的值，防止复位。

作用：用于监测单片机程序运行时效是否精准，主要检测软件异常。

**寄存器**

控制寄存器WWDF_CR、配置寄存器WWDG_CFG、状态寄存器WWDG_SR

**WWDG超时时间计算**

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202307042253592.png" alt="image-20230704225332329" style="zoom:33%;" />

WWDG配置步骤

1. WWDG工作参数初始化。`HAL_WWDG_Init()`
2. WWDG Msp初始化。`HAL_WWDG_MspInit()`
3. 设置优先级，使能中断。`HAL_NVIC_SetPriority()`、`HAL_NVIC_Enable()`
4. 编写中断服务函数。`WWDG_IRQHandler()`->`HAL_WWDG_IRQHandler()`
5. 重定义提前唤醒回调函数。`HAL_WWDG_EarlyWakeupCallback()`
6. 在窗口期内喂狗。`HAL_WWDG_Refresh()`

`HAL_WWDG_Init()`。WWDG_CR/WWDG_CFR。使能WWDG，设置预分频系数和窗口值。

`HAL_WWDG_Refresh()`。WWDG_CR重装载计数器（喂狗）

**IWDG和WWDG的主要区别**

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202307042305202.png" alt="image-20230704230550947" style="zoom:50%;" />

### 3.7 定时器

**软件定时**

使用纯软件（CPU死等）的方式实现（延时）功能。

```c
void delay_us(uint32_t us)
{
	us*=72;
	while(us--);
}
```

缺点：1.延时不精准。2.CPU死等。

**定时器定时**

使用精准的时基，通过硬件的方式，实现定时功能。定时器核心就是计数器。

定时器分类：

1. 常规定时器。基本定时器、通用定时器、高级定时器
2. 专用定时器。独立看门狗、窗口看门狗、实时时钟、低功耗定时器
3. 内核定时器。SysTick定时器

#### 3.5.1 基本定时器

TIM6/7。16位递增计数器。16位预分频器。可用于触发DAC。在更新时间（计数器溢出）时，可产生中断DMA请求。

计数模式：

1. 递增。CNT=ARR
2. 递减。CNT=0
3. 中心对齐模式。CNT=ARR-1、CNT=1

**寄存器**

1. 控制寄存器1。TIMx_CR1。 设置ARR寄存器是否具有缓冲、使能计数器。

2. DMA/中断使能寄存器TIMx_DIER。使能更新中断。

3. 状态寄存器TIMx_SR。判断是否发生了更新中断。硬件置1，软件清0。
4. 计数器TIMx_CNT。计数器实时数值，可设置计数器初值，可读可写。
5. 预分频器TIMx_PSC。设置预分频系数，实际预分频系数等于PSC+1。
6. 自动重装载寄存器TIMx_ARR。设置自动重装载值。

**定时器溢出时间计算方法**

<img src="https://raw.githubusercontent.com/pikapika-zrf/BlogImg/master/img/202307051206116.png" alt="image-20230705120629249" style="zoom:33%;" />

##### **基本定时器配置步骤：**

1. 配置定时器基础工作参数。`HAL_TIM_Base_Init()`
2. 定时器基础MSP初始化。`HAL_TIM_Base_MspInit()`
3. 使能更新中断并启动计数器。`HAL_TIM_Base_Start_IT()`
4. 设置优先级，使能中断。`HAL_NVIC_SetPriority()`、`HAL_NVIC_Enable()`
5. 编写中断服务函数。`TIMx_IRQHandler()`->`HAL_TIM_IRQHandler()`
6. 编写定时器更新中断回调函数。`HAL_TIM_PeriodElapsedCallback()`

相关库函数介绍：

1. `HAL_TIM_Base_Init()`。主要寄存器CR1、ARR、PSC。初始化定时器基础参数。
2. `HAL_TIM_Base_MspInit()`。存放NVIC、CLOCK、GPIO初始化代码。
3. `HAL_TIM_Base_Start_IT()`。DIER、CR1。使能更新中断并启动计数器。
4. `HAL_TIM_IRQHandler()`。SR。定时器中断处理公用函数，处理各种中断。
5. `HAL_TIM_PeriodElapsedCallback()`。定时器更新中断回调函数，由用户重定义。

#### 3.5.2 通用定时器

TIM2\3\4\5。16位递增、递减、中心对齐计数器。16位预分频器。可用于触发DAC、ADC。

在更新时间、触发时间、输入捕获、输出比较时会产生中断/DMA请求。

4个独立通道，可用于：输入捕获、输出比较、输出PWM、单脉冲模式。

使用外部信号控制定时器且可实现多个定时器互连的同步电路。

支持编码器和霍尔传感器电路。

**时钟源**

内部时钟、外部时钟模式1、外部时钟模式2、内部触发输入

##### **通用定时器PWM输出配置步骤：**

1. 配置定时器基础工作参数。`HAL_TIM_PWM_Init()`
2. 定时器 PWM输出MSP初始化。`HAL_TIM_PWM_MspInit()`
3. 配置PWM模式/比较值等。`HAL_TIM_PWM_ConfigChannel()`
4. 使能输出并启动计数器。`HAL_TIM_PWM_Start()`
5. 修改比较值控制占空比（可选）。`__HAL_TIM_SET_COMPARE()`

6. 使能通道预装载（可选）。`__HAL_TIM_ENABLE_OCxPRELOAD()`

相关库函数介绍：

1. `HAL_TIM_PWM_Init()`。主要寄存器CR1、ARR、PSC。初始化定时器基础参数。
2. `HAL_TIM_PWM_MspInit()`。存放NVIC、CLOCK、GPIO初始化代码。
3. `HAL_TIM_PWM_ConfigChannel()`。CCMRx、CCRx、CCER。配置PWM模式、比较值、输出极性等。
4. `HAL_TIM_PWM_Start()`。CCER、CR1。使能输出比较并启动计数器。
5. `__HAL_TIM_SET_COMPARE()`。CCRx。修改比较值
6. `__HAL_TIM_ENABLE_OCxPRELOAD()`。CCER。使能通道预装载。

##### **通用定时器输入捕获配置步骤：**

1. 配置定时器基础工作参数。`HAL_TIM_IC_Init()`
2. 定时器输入捕获MSP初始化。`HAL_TIM_IC_MspInit()`
3. 配置输入通道映射、捕获边沿。`HAL_TIM_IC_ConfigChannel()`
4. 设置优先级，使能中断。`HAL_NVIC_SetPriority()`、`HAL_NVIC_Enable()`
5. 使能定时器更新中断。`__HAL_TIM_ENABLE_IT()`
6. 使能捕获、捕获中断及计数器。`HAL_TIM_IC_Start_IT()`

6. 编写中断服务函数。`TIMx_IRQHandler()`->`HAL_TIM_IRQHandler()`
7. 编写更新中断和捕获回调函数。`HAL_TIM_PeriodElapsedCallback()`,`HAL_TIM_IC_CaptureCallback()`

相关库函数介绍：

1. `HAL_TIM_IC_Init()`。主要寄存器CR1、ARR、PSC。初始化定时器基础参数。
2. `HAL_TIM_IC_MspInit()`。存放NVIC、CLOCK、GPIO初始化代码。
3. `HAL_TIM_IC_ConfigChannel()`。CCMRx、CCER。配置通道映射、捕获边沿、分频、滤波等。
4. `__HAL_TIM_ENABLE_IT()`。DIER。使能更新中断。
5. `HAL_TIM_IC_Start_IT()`。CCER、DIER、CR1。使能输入捕获、捕获中断并启动计数器。
6. `HAL_TIM_IRQHandler()`。SR。定时器中断处理公用函数，处理各种中断。
7. `HAL_TIM_PeriodElapsedCallback()`。定时器更新中断回调函数，由用户重定义。
8. `HAL_TIM_IC_CaptureCallback()`。定时器输入捕获回调函数，由用户重定义。

##### **通用定时器脉冲计数配置步骤：**

1. 配置定时器基础工作参数。`HAL_TIM_IC_Init()`
2. 定时器输入捕获MSP初始化。`HAL_TIM_IC_MspInit()`
3. 配置定时器从模式。`HAL_TIM_SlaveConfigSynchro()`
4. 使能输入捕获并启动计数器。`HAL_TIM_IC_Start()`
5. 获取计数器的值。`__HAL_TIM_GET_COUNTER()`
6. 设置计数器的值。`__HAL_TIM_SET_COUNTER()`

相关库函数介绍：

1. `HAL_TIM_IC_Init()`。主要寄存器CR1、ARR、PSC。初始化定时器基础参数。
2. `HAL_TIM_IC_MspInit()`。存放NVIC、CLOCK、GPIO初始化代码。
3. `HAL_TIM_SlaveConfigSynchro()`。SMCR、CCMRx、CCER。配置定时器从模式、触发选择、分频、滤波等。
4. `HAL_TIM_IC_Start()`。CCER、CR1。使能输入捕获并启动计数器。

#### 3.5.3 高级定时器

TIM1/8。通用定时器+重复计数器、死区时间带编程的互补输出、断路输入。

 **重复计数器**

计数器每次溢出使重复计数器减1，再发生一次溢出就发生更新事件。即设置RCR为N，更新事件将在N+1次溢出时发生。

**输出指定个数PWM实验**

配置边沿对齐模式输出PWM。指定输出N个PWM。把N-1写入RCR。在更新中断内，关闭计数器。

高级定时器通道输出必须把MOE位置1

##### **输出指定个数PWM配置步骤：**

1. 配置定时器基础工作参数。`HAL_TIM_PWM_Init()`
2. 定时器 PWM输出MSP初始化。`HAL_TIM_PWM_MspInit()`
3. 配置PWM模式/比较值等。`HAL_TIM_PWM_ConfigChannel()`
4. 设置优先级，使能中断。`HAL_NVIC_SetPriority()`、`HAL_NVIC_Enable()`
5. 使能定时器更新中断。`__HAL_TIM_ENABLE_IT()`
6. 使能输出、主输出并启动计数器。`HAL_TIM_PWM_Start()`
7. 编写中断服务函数。`TIMx_IRQHandler()`->`HAL_TIM_IRQHandler()`
8. 编写更新中断回调函数。`HAL_TIM_PeriodElapsedCallback()`

相关HAL库函数介绍：

`HAL_TIM_GenerateEvent()`。主要寄存器EGR。通过软件产生事件。

**输出比较模式**

CNT=CCRx时，OCRxREF电平翻转。

周期由ARR决定，占空比固定50%，相位由CCRx决定。

##### **输出比较配置步骤：**

1. 配置定时器基础工作参数。`HAL_TIM_OC_Init()`。CR1、ARR、PSC。
2. 定时器输出捕获MSP初始化。`HAL_TIM_OC_MspInit()`。
3. 配置输出比较模式。`HAL_TIM_OC_ConfigChannel()`。CCMRx、CCRx、CCER。
4. 使能通道预装载。`__HAL_TIM_ENABLE_OCxPRELOAD()`。CCMRx。
5. 使能输出、主输出及计数器。`HAL_TIM_OC_Start()`。CR1、CCER、BDTR。
6. 修改捕获/比较寄存器的值。`__HAL_TIM_SET_COMPARE()`。CCRx。

**互补输出带死区**

死区时间计算：

1. 确定$t_{DTS}$的值$f_{DTS} = \frac{F_t}{2^{CKD[1:0]}}$
2. 判断DTG[7:5]，选择计算公式。

使能刹车功能：将TIM_BDTR的BKE位置一，刹车输入信号极性由BKP位位置。

使能刹车功能后，由TIMx_BDTR的MOE、OSSI、OSSR位，TIMx_CR2的OISx、OISxN位，TIMx_CCER的CCxE、CCxNE位控制OCx和OCxN输出状态。**见参考手册13.4.9表75**

发生刹车后：

1. MOE位被清零，OCx和OCxN为无效、空闲或复位状态（OSSI位选择）
2. OCx和OCxN的状态由相关控制位状态决定。
3. BIF位置1，会产生刹车中断。使能TDE位，会产生DMA请求。
4. 如果AOE位置1，在下一个更新事件UEV时，MOE位被自动置1。

**互补输出带死区控制配置步骤：**

1. 配置定时器基础工作参数。`HAL_TIM_PWM_Init()`。CR1、ARR、PSC。
2. 定时器 PWM输出MSP初始化。`HAL_TIM_PWM_MspInit()`。
3. 配置PWM模式/比较值等。`HAL_TIM_PWM_ConfigChannel()`。CCMRx、CCRx、CCER。
4. 设置刹车功能、死区时间。`HAL_TIMEx_ConfigBreakDeadTime()`。BDTR。
5. 使能输出、主输出并启动计数器。`HAL_TIM_PWM_Start()`。CCER、CR1。
6. 使能互补输出、主输出并启动计数器。`HAL_TIMEx_PWMN_Start()`。CCER、CR1。

**PWM输入模式**

1. 配置定时器基础工作参数。`HAL_TIM_IC_Init()`
2. 定时器输入捕获MSP初始化。`HAL_TIM_IC_MspInit()`
3. 配置IC1/2映射、捕获边沿等。`HAL_TIM_IC_ConfigChannel()`
4. 配置从模式，触发源。`HAL_TIM_SlaveConfigSynchro()`
5. 设置优先级，使能中断。`HAL_NVIC_SetPriority()`、`HAL_NVIC_Enable()`
6. 使能输入捕获、捕获中断并启动计数器。`HAL_TIM_IC_Start_IT()`、`HAL_TIM_IC_Start()`
7. 编写中断服务函数。`TIMx_IRQHandler()`->`HAL_TIM_IRQHandler()`
8. 编写输入捕获回调函数。`HAL_TIM_IC_CaptureCallback()`

### 3.8 USMART

正点原子串口调试组件，提高代码调试效率。USMART可以直接通过串口调用用户编写的函数，修改参数。

通过对比用户输入字符串和本地函数名，用函数指针实现调用不同的函数。

### 3.9 RTC实验

实时时钟，计数器，计数频率常为秒。有后备电源$V_{BAT}$。

普通定时器无法掉电运行。RTC能提供秒钟数、能在MCU掉电后运行、低功耗。

RTC和后备寄存器不会被系统或电源复位源复位。`RTC_RPL`、`RTC_ALR`、`RTC_CNT`、`RTC_DIV`寄存器不会被系统复位。

**基本配置：**

1. 使能对RTC的访问。使能PWR&BKP时钟，使能对后备寄存器和RTC的访问权限。`RCC_APB1ENB`（位27，位28），`PWR_CR`（位8）
2. 设置RTC时钟源。激活LSE，设置RTC的技术时钟源为LSE。`RCC_BDCR`（位15，位9:8，位0）
3. 进入配置模式。等待RTOFF位为1，设置CNF位为1。`RTC_CRL`（位5，位4），`RTC_CRH`
4. 设置RTC寄存器。设置分频器、计数值等，一般先只设置分频值，CNT的设置独立。`RTC_PRL`，`RTC_CNT`
5. 退出配置模式。清除CNF位，等待RTOFF为1即配置完成。`RTC_CRL`

相关HAL库驱动介绍

1. `HAL_RTC_Init()`。初始化RTC。
2. `HAL_RTC_MspInit()`。使能RTC时钟。
3. `HAL_RCC_OscConfig()`。开启LSE时钟源。
4. `HAL_RCCEx_PeriphCLKConfig()`。设置RTC时钟源为LSE。
5. `HAL_PWR_EnableBkUpAccess()`。使能备份域的访问权限。
6. `HAL_RTCEx_BKUPWrite/Read()`。读/写备份域数据寄存器。

**RTC基本驱动步骤（F1）：**

1. 使能电源时钟并使能后备域访问。使能电源时钟`__HAL_RCC_PWR_CLK_ENABLE()`，使能备份时钟`__HAL_RCC_BKP_CLK_ENABLE()`，使能备份访问。
2. 开启LSE/选择RTC时钟源/使能RTC时钟。开启LSE时钟源`HAL_RCC_OscConfig()`，选择RTC时钟源`HAL_RCCEx_PeriphCLKConfig()`，使能RTC时钟`__HAL_RCC_RTC_ENABLE()`。
3. 初始化RTC，设置分频置以及工作参数。初始化RTC`HAL_RTC_Init()`，完成RTC底层初始化`HAL_RTC_MspInit()`
4. 设置RTC的日期和时间。操作寄存器方式实现`rtc_set_time`。
5. 获取RTC当前日期和时间。定义rtc_get_time函数。

### 3.10 RNG

随机数发生器，用于生成随机数的程序或硬件。STM32 RNG处理器是以连续模拟噪声为基础的随机数发生器，在主机读数时提供一个32位的随机数。

真随机数TRNG，参考选型手册。F1只有伪随机数。

相关HAL库驱动介绍

1. `HAL_RNG_Init()`。初始化RNG。`RNG_CR`
2. `HAL_RNG_MspInit()`。使能时钟、外设、选择时钟源。
3. `HAL_RCCEx_PeriphCLKConfig()`。设置RNG时钟源为PLL。`RCC_BDCR`
4. `HAL_RNG_GenerateRandomNumber()`。判断DRDY位并读取随机数。`RNG_DR`
5. `__HAL_RCC_RNG_CLK_ENABLE()`。使能RNG时钟。`AHB2ENR`
6. `__HAL_RNG_GET_FLAG()`。获取RNG相关标记。`RNG_SR`

**RNG基本驱动步骤：**

1. 使能RNG时钟。`__HAL_RCC_RNG_CLK_ENABLE()`
2. 初始化随机数发生器。`HAL_RNG_Init()`，`HAL_RNG_MspInit()`，`HAL_RCCEx_PeriphCLKConfig()`
3. 判断DRDY位，读取随机数值。`HAL_RNG_GenerateRandomNumber()`

### 3.11 低功耗

STM32具有运行、睡眠、停止和待机四种工作模式。

低功耗模式：睡眠、停止、待机

相关HAL库驱动介绍

1. `HAL_PWR_EnterSLEEPMode()`。进入睡眠模式。`SCB_SCR`
2. `HAL_PWR_EnterSTOPMode()`。进入停止模式。`PWR_CR`、`SCB_SCR`
3. `HAL_PWR_EnterSTANDBYMode()`。进入待机模式。`PWR_CR`、`SCB_SCR`
4. `HAL_PWR_EnterWakeMode()`。使能WKUP管脚唤醒功能。`PWR_CR`。
5. `__HAL_PWR_CAEAR_FLAG()`。清除PWR的相关标记。`PWR_CR`
6. `__HAL_RCC_PWR_CLK_ENABLE()`。使能电源时钟。`PWR_CR`

**睡眠模式配置步骤**

1. 初始化WKUP为中断触发源。（外设低功耗处理）
2. 进入睡眠模式。`HAL_PWR_EnterSLEEPMode()`
3. 等待WKUP外部中断唤醒。

**停止模式配置步骤**

1. 初始化WKUP为中断触发源。（外设低功耗处理）
2. 进入停止模式。`HAL_PWR_EnterSTOPMode()`
3. 等待WKUP外部中断唤醒。重新设置时钟、重新选择滴答时钟源、使能systick中断。

**待机模式配置步骤**

1. 初始化WKUP为中断触发源。（外设低功耗处理）
2. 使能电源时钟。`__HAL_RCC_PWR_CLK_Enable()`
3. 使能WKUP的唤醒功能。`HAL_PWR_EnableWakeUpPin()`
4. 清除唤醒标记WUF。`__HAL_PWR_CLEAR_FLAG`。
5. 进入待机模式。`HAL_PWR_EnterSTANDBYMode`

### 3.12 DMA

直接存储器访问。无需CPU直接控制传输，通过硬件为RAM和IO设备开辟一条直接传输数据的通道，提高CPU的效率。

相关HAL库驱动介绍

1. `__HAL_RCC_DMAx_CLK_ENABLE()`。使能DMAx时钟。`RCC_AHBENR`。
2. `HAL_DMA_Init()`。初始化DMA。`DMA_CCR`。
3. `HAL_DMA_Start_IT()`。开始DMA传输。`DMA_CCR/CPAR/CMAR/CNDTR`。
4. `__HAL_LINKDMA()`。用来连接DMA和外设句柄。
5. `HAL_UART_Transmit_DMA()`。使能DMA发送，启动传输。`DMA_CCR/CPAR/CMAR/CNDTR/USART_CR3`。
6. `__HAL_DMA_GET_FLAG()`。查询DMA传输通道的状态。`DMA_ISR`。
7. `__HAL_DMA_ENABLE()`。使能DMA外设。`DMA_CCR`。
8. `__HAL_DMA_DISABLE()`。使能DMA外设。`DMA_CCR`。

**DMA方式传输串口数据配置步骤**

1. 使能DMA时钟。`__HAL_RCC_DMA1_CLK_ENABLE`
2. 初始化DMA。`HAL_DMA_Init`函数初始化DMA相关参数，`__HAL_LINKDMA`函数连接DMA和外设
3. 使能串口的DMA发送，启动传输。`HAL_UART_Transmit_DMA`

查询DMA传输状态。`__HAL_DMA_GET_FLAG`查询通道传输状态，`__HAL_DMA_GET_COUNTER`获取当前传输剩余数据量

DMA中断使用。`HAL_NVIC_EnableIRQ
`HAL_NVIc SetPriority`，编写中断服务函数xxx_IRQHandler

### 3.13 ADC

模拟/数字转换器。分为并联比较型（速度快）和逐次逼近型（功耗低）。

- 分辨率。ADC能辨别的最小模拟量，用二进制位数表示。
- 转换时间。完成一次AD转换所需要的时间。转换时间越短，采样率越高。
- 精度。最小精度基础上叠加各种误差参数。精度受ADC性能、温度和气压等影响。
- 量化误差。用数字量近似表示模拟量，采用四舍五入原则产生的误差。

ADC供电电源。$V_{SSA}$接0V、$V_{DDA}$接3.3V通过低通滤波器。

ADC输入电压范围$V_{REF-}<=V_{IN}<=V_{REF+}$($0V<=V_{IN}<=3.3V$)

#### **1.触发源（F1）**

1. ADON位触发转换（F1）

当ADC_CR2寄存器的ADON位为1时，再单独给ADON位写1，只能启动规则组转换。

2. 外部事件触发转换

分为：规则组外部触发和注入组外部触发。

**规则组外部触发方法：**

`EXTTRIG位置1`（使能外部事件触发）的情况下：

​		`EXTSEL[2:0]=111`。选择软件启动    》  `SWSTART位置1`。启动规则通道转换。

​		`EXTSEL[2:0]=其它`。选择硬件启动   》  `等待相关事件发生`。启动规则通道转换。

**注入组外部触发方法：**

`JEXTTRIG位置1`（使能外部事件触发）的情况下：

​		`JEXTSEL[2:0]=111`。选择软件启动    》  `JSWSTART位置1`。启动注入通道转换。

​		`JEXTSEL[2:0]=其它`。选择硬件启动   》  `等待相关事件发生`。启动注入通道转换。

#### **2.转换时间**

PCLK2。APB总线上的时钟-》

ADCPRE[1:0]。RCC_CFGR寄存器-》

ADCCLK。ADC最大时钟频率为14MHz。

ADC转换时间：$T_{CONV}$=采样时间+12.5个周期

采样时间通过SMPx[2:0]设置。

#### **3.数据寄存器**

规则组。转换结果按顺序输出。规则数据寄存器（32位）ADCx_DR

注入组。对应四个寄存器。注入数据寄存器（16位）ADCx_JDRy

分辨率是8/10/12位。由ADCx_CR2寄存器的ALIGN位设置数据对齐方式，可选择：右对齐或者左对齐。

#### 4.中断

规则通道转换结束、 注入通道转换结束、设置了模拟看门狗状态位、
溢出(F1没有)

**DMA请求(只适用于规则组)**
规则组每个通道转换结束后，除了可以产生中断外，还可以产生DMA请求,我们利用DMA及时把转换好的数据传输到指定的内存里，防止数据被覆盖。

#### **5.单次转换模式和连续转换模式**

CONT位选择转换模式。

单次转换模式(只触发一次转换)

- 规则组。转换结果被储存在ADC_DR。EOC(转换结束)标志位被置1。如果设置了EOCIE位，则产生中断。然后ADC停止。
- 注入组。转换结果被储存在ADC_DRJx。JEOC(转换结束)标志位被置1。如果设置了JEOCIE位，则产生中断。然后ADC停止

连续转换模式(自动触发下一次转换）

- 规则组。转换结果被储存在ADC_DR。EOC(转换结束)标志位被置1。如果设置了EOCIE位，则产生中断。
- 注入组。转换结果被储存在ADC_DRJx。JEOC(转换结束)标志位被置1。如果设置了JEOCIE位，则产生中断。自动注入:将JAUTO位置1

**扫描模式**

SCAN位。

- 关闭扫描模式。ADC只转换ADC_SQRx或ADC_JSQR选中的第一个通道进行转换。
- 使用扫描模式。ADC会扫描所有被ADC_SQRx或ADC_JSQR选中的所有通道

**不同模式组合**

- 单次转换模式(不扫描)。只转换一个通道,而且是一次，需等待下一次触发
- 单次转换模式(扫描)。ADC_SQRx和ADC_JSQR选中的所有通道都转换一次
- 连续转换模式(不扫描)。只会转换一个通道,转换完后会自动执行下一次转换
- 连续转换模式(扫描)。ADC SORx和ADC_JSQR选中的所有通道都转换一次，并自动进入下—轮转换

不常用的模式：间断模式。

**单通道ADC实验**

寄存器ADC_CR1（位8、位19:16）、ADC_CR2（位22、位20、位19:17、11、8、3、2、1、0）、ADC_SMPR1/2、ADC_SQR1

**单通道ADC采集实验配置步骤**

1. 配置ADC工作参数、ADC校准。`HAL_ADC_Init()`、`HAL_ADCEx_Calibration_Start()`
2. ADC MSP初始化。`HAL_ADC_MspInit()` 配置NVIC、CLOCK、GPIO等
3. 配置ADC相应通道相关参数。`HAL_ADC_ConfigChannel()`
4. 启动A/D转换。`HAL_ADC_Start()`
5. 等待规则通道转换完成。`HAL_ADC_PollForConversion()`
6. 获取规则通道A/D转换结果。`HAL_ADC_GetValue()`

相关HAL库函数介绍：

1. `HAL_ADC_Init()`。CR1、CR2。配置ADC工作参数
2. `HAL_ADCEx_Calibration_Start()`。CR2。ADC校准
3. `HAL_ADC_MspInit()`。无。存放NVIC、CLOCK、GPIO初始化代码
4. `HAL_RCCEx_PeriphCLKConfig()`。RCC_CFGR。设置扩展外设时钟，如:ADC、RTC等
5. `HAL_ADC_ConfigChannel()`。SQRx、SMPRx。配置ADC相应通道的相关参数
6. `HAL_ADC_Start()`。CR2。启动A/D转换
7. `HAL_ADC_PollForConversion()`。SR。等待规则通道转换完成
8. `HAL_ADC_GetValue()`。DR。获取规则通道A/D转换结果

**单通道ADC采集（DMA处理）实验配置步骤**

1. 初始化DMA。`HAL_DMA_Init()`
2. 将DMA和ADC句柄联系起来。`__HAL_LINKDMA()`
3. 配置ADC工作参数、ADC校准。`HAL_ADC_Init()`、`HAL_ADCEx_Calibration_Start()`
4. ADC MSP初始化。`HAL_ADC_MspInit()` 配置NVIC、CLOCK、GPIO等
5. 配置ADC相应通道相关参数。`HAL_ADC_ConfigChannel()`
6. 使能DMA数据流传输完成中断。`HAL_NVIC_SetPriority()`、`HAL_NVIC_EnableIRQ()`
7. 编写DMA数据流中断服务函数。`DMAx_Channely_lRQHandler()`
8. 启动DMA，开启传输完成中断。`HAL_DMA_Start_IT()`
9. 触发ADC转换，DMA传输数据。`HAL_ADC_Start_DMA()`
10. 等待规则通道转换完成。`HAL_ADC_PollForConversion()`
11. 获取规则通道A/D转换结果。`HAL_ADC_GetValue()`

相关HAL库函数介绍：

1. `HAL_DMA_Start_IT()`。CCRx。启动DMA、开启传输完成中断。
2. `HAL_ADC_Start_DMA()`。CR2。触发ADC转换、使用DMA传输数据。

实验：多通道ADC采集（DMA）、单通道ADC过采样、内部温度传感器、光敏传感器、

### 3.14 DAC

数字/模拟转换器。

- 分辨率表示模拟电压的最小增量，常用二进制位数表示，比如:8、12位等

- 建立时间。表示将一个数字量转换为稳定模拟信号所需的时间

- 精度。转换器实际特性曲线与理想特性曲线之间的最大偏差

  误差源:比例系统误差、失调误差、非线性误差

  原因:元件参数误差、基准电压不稳定、运算放大器零漂等

$V_{SSA}$∶0V，$V_{DDA}$∶2.4V~3.6V

$V_{REF\_}$: 0V，$V_{REF+}$—般为3.3V

DAC输出电压范围：$V_{REF-}\leq V_{OUT}\leq V_{REF+}$

**DAC数据模式**

支持8/12位模式。

8位模式。只能右对齐。DHR8Rx、DHR8RD（双DAC通道转换用）

12位寄存器。右对齐。DHR12Rx、DHR12RD（双DAC通道转换用）。左对齐。DHR12Lx、DHR12LD（双DAC通道转换用）

**触发源**

自动触发、软件触发、外部时间触发

TENx位置0，禁止触发，即自动。经过1个APB1时间周期，DHRx->DORx

 TENx位置1，使能触发。$TSELx[2:0] \neq 111$，外部事件触发。经过3个APB1时钟周期，DHRx->DORx

​	$TSELx[2:0] = 111$，软件触发。经过1个APB1时钟周期，DHRx->DORx

DHRx数据加载到DORx后，模拟输出电压将经过时间$V_{SETTING}$后可用。

**DMA请求**

DAMENx位置1，通过外部事件触发产生DMA请求，DHRx->DORx。

**DAC输出电压**

12位模式下，$DAC输出电压 = (\frac {DORx} {4096}) * V_{REF+}$

8位模式下，$DAC输出电压 = (\frac {DORx} {256}) * V_{REF+}$

#### DAC输出实验

关闭通道1触发（即自动），TEN1位置0。

关闭输出缓冲，BOFF1位置1。

使用12位右对齐模式，将数字量写入DAC_DHR12R1寄存器。

**DAC输出实验配置步骤**

1. 初始化DAC。`HAL_DAC_Init()`、
2. DAC MSP初始化。`HAL_DAC_MspInit()` 配置NVIC、CLOCK、GPIO等配置
3. 配置DAC相应通道相关参数。`HAL_DAC_ConfigChannel()`
4. 启动DA转换。`HAL_DAC_Start()`
5. 设置输出数字量。`HAL_DAC_SetValue()`
6. 读取通道输出数字量（可选）。`HAL_DAC_GetValue()`

相关HAL库函数介绍：

1. `HAL_DAC_Init()`。无。配置DAC工作状态（HAL库内部使用）
2. `HAL_DAC_MspInit()`。无。存放NVIC、CLOCK、GPIO初始化代码
3. `HAL_DAC_ConfigChannel()`。CR。配置DAC相应通道的相关参数
4. `HAL_DAC_Start()`。CR、SWTRIGR。启动D/A转换
5. `HAL_DAC_SetValue()`。DHR12Rx。设置输出数字量
6. `HAL_DAC_GetValue()`。DORx。读取通道输出数字量

 DAC输出三角波实验、 DAC输出正弦波实验、PWM DAC实验

### 3.15 IIC

IIC总线协议，集成电路总线，同步串行半双工通信总线。

- 两根双向信号线。时钟线SCL和数据线SDA。都接上拉电阻，总线空闲时为高电平.
- 总线上可以挂多个主机，多个从机，每个设备都有一个唯一的地址。

起始信号(S)：当SCL为高电平时，SDA从高电平变为低电平

停止信号(P)：当SCL为高电平时，SDA从低电平变为高电平
应答信号：上拉电阻影响下SDA默认为高，从而都拉低SDA就是确认收到数据即

数据先发送高位，以字节传输。

**在数据的传输过程中，SCLK为高电平时，外设模块开始采集SDA数据线上的数据，此时要求SDA数据线上的电平状态必须稳定，当SCLK为低电平时才允许SDA线上的数据跳变成另外一种状态。**

**协议规定，主机每传完一个字节的数据即外设每收到一个字节的数据，外设就要在第9个时钟脉冲到来的时候，将SDA数据线拉低进行应答（ACK）,且必须是稳定的低电平，表示已经收到了一个字节的数据，拉高表示不进行应答（NACK）**；注意这里是外设将SDA数据线拉低，不是主机了

**IIC配置步骤**

1. 使能SCL和SDA对应时钟。`__HAL_RCC_GPIOB_CLK_ENABLE()`
2. 设置GPIO工作模式。SDA开漏/SCL推挽输出模式，使用`HAL_GPIO_Init`初始化。
3. 编写基本信号。起始信号、停止信号、应答信号。
4. 编写读和写函数

### 3.16 SPI

串行外设设备接口。高速、全双工、同步的通信总线。

主设备控制从设备。通过片选控制多个从设备。时钟由主设备提供给从设备。

时钟极性和时钟相位控制两个SPI设备数据采用和交换。

每次SPI是主从设备在交换数据。发一个数据必然会收到一个数据；要收一个数据必须也要先发一个数据。

### 3.17 CAN

控制器局域网。有低速CAN和高速CAN。

CAN也使用差分信号传输数据。CAN总线使用CAN_H和CAN_L的电位差来表示数据电平。电位差分为显性电平和隐性电平，分别表示逻辑0和1。

CAN是一种基于消息广播模式的串行通信总线，即在同一时刻网络上所有节点监测到的数据是一致的，各节点根据报文ID来甄别是否是发给自己的报文。

CAN总线以“帧”（Frame）的形式进行通信。

为保证通信稳定，CAN采用“位同步”机制，实现对电平的正确采样。

筛选器实现选择性的获取报文，降低系统负担。



1. Tips：

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

