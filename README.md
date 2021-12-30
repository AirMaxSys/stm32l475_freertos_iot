# STM32L475_FREERTOS_IOT

## Development environment

Keil

- Toolchain: MDK-ARM
- Toolchain version: 5.29.0.0
- Compiler: ArmClang
- Compiler version: 6.13.1

GNU

- Toolchain: GCC
- Build system: CMake Makefile

## RTOS

FreeRTOS synopsis

- Version: [FreeRTOS v10.4.4](https://github.com/FreeRTOS/FreeRTOS-Kernel/releases/tag/V10.4.4)

- Manual: [FreeRTOS reference Manual](Docs/RTOS/FreeRTOS_Reference_Manual_V10.0.0.pdf)

- Web: [FreeRTOS website](https://www.freertos.org/RTOS.html)

FreeRTOS port

- Get FreeRTOS kernel source code.

- In ```FreeRTOS-kernel/portable``` directory, finding appropriate port.c and portmacro.h files. Firstly, enter the next level directory according to the toolchain you use. In my case, we use ArmClang compiler, goto ```ARMClang``` subdirectory find ```Use-the-GCC-ports.txt``` file
that means we should use GCC portable files. So enter the ```GCC``` subdurectory. Secondly, enter the next subdirectory according to the MCU/MPU architecture, in my case, use ARM cortex-M4 MCU so entern the ```ARM_CM4F``` and get port.c and portmacro.h files.

- Choosen one memory management algorithm. In the ```FreeRTOS-kernel/portable/MemMang``` directory has 5 kinds of algorithms. In most of case heap_4.c is a good choice.

- Get FreeRTOSConfig.h file. In FreeRTOS github repositories can find many [usage demo](https://github.com/FreeRTOS/FreeRTOS/tree/main/FreeRTOS/Demo), according to your toolchain and MCU/MPU architecture entern one appropriate demo and get the FreeRTOSConfig.h file.

- Configure RTOS. Make sure the following macros are defined in FreeRTOSConfig.h file. Other customisation configuration options can refer to FreeRTOS configuration [website](https://www.freertos.org/a00110.html).

``` cpp
#define xPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler
#define xPortSysTickHandler SysTick_Handler
```
