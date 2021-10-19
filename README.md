# STM32L475VET Pandora Board

## RTOS

### RTX brief

[CMSIS_5](https://github.com/ARM-software/CMSIS_5/releases/tag/5.7.0) RTOS_v2 RTX_5.5.2

[CMSOS-RTOS2 documentation page](https://arm-software.github.io/CMSIS_5/RTOS2/html/index.html)

### RTX port to stm32L475VET

- Make sure stm32cubeMX dones't generete three NVIC handler which are SVC PendSV and SysTick. If you want to HAL_Delay or LL_mDelay function, you can select one TIM as no RTOS timebase of MCU.

- Add SVC PendSV and SysTick NVIC handler assambly file form RTOS/RTX/Source/ARM/irq_cm4f.s to MDK-ARM project. If you want to use GCC or IAR tool set, then select the appropriate file.

- Add RTX source file to your project which from directory RTOS/RTX/Source.

- Add config file from RTOS/RTX/RTX_Config.c (not necessary).

- Add RTX os tick functions implement which from directory Drivers/CMSIS/RTOS2/Source/os_systick.c

- Add header files path to your project.
