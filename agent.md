# Agent Instructions

## Project scope

- Work in `baseboard_test`; the N32G032 project under `docs` is reference-only.
- Target MCU: STM32G474RCTX (STM32G4 HAL/CMSIS, FreeRTOS, STM32CubeIDE-generated project).
- Do not compile or run generated-binary verification during code-only changes.
- Preserve CubeMX-generated sections and use `USER CODE` regions for application logic where practical.

## USART2 / DYP-A22 constraints

- USART2 uses PA2/PA3, 115200 8-N-1, DMA1 channel 1 (TX) and channel 2 (RX), and IDLE-line frame detection.
- The external RS485 transceiver handles direction; firmware must not add DE/RE GPIO switching.
- DYP-A22 traffic is Modbus RTU. CRC16 is transmitted low byte first.
- RX DMA is restarted after each IDLE frame. ISR code only captures a frame; parsing and sensor state updates belong in `Core/Src/DYP/DYPA22.c`.
- Do not change `baseboard_test.ioc` or request a CubeMX regeneration unless a required pin, DMA, interrupt, or peripheral setting is missing. Report such a requirement before dependent code changes.

## Change discipline

- Keep DYP protocol helpers in `Core/Inc/DYP/DYPA22.h` and `Core/Src/DYP/DYPA22.c`.
- Keep interrupt handlers short and avoid blocking calls from ISR context.
- Document functional changes in `README.md`.
