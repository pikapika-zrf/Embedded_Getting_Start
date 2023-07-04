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

## 3.5 定时器



































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

