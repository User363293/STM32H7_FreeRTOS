# FreeRTOS Port for STM32H743

## Features
- Pure FreeRTOS implementation (no CMSIS wrappers)
- LED blink example
- CubeIDE project

## Requirements
- STM32CubeIDE 1.11.0+
- STM32H7 HAL 1.11.0

## Quick Start
1. Clone this repo
2. Open project in CubeIDE
3. Build and flash

🔧 Project Setup After Code Generation
After generating code through STM32CubeMX you must:

Open Core/Src/stm32h7xx_it.c

Comment out the following interrupt handlers:

c
Copy
/*
void SVC_Handler(void) {
    // Must be handled by FreeRTOS
}

void PendSV_Handler(void) {
    // Must be handled by FreeRTOS
}

void SysTick_Handler(void) {
    // Must be handled by FreeRTOS
}
*/
