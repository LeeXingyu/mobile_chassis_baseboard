# baseboard_test

## Current target

The project targets the STM32G474RCTX and uses STM32 HAL with FreeRTOS. USART2 (PA2/PA3, 115200 8-N-1) is connected to the DYP-A22 bus through an RS485 transceiver. The transceiver direction is handled externally, so firmware does not control a DE/RE pin.

## USART2/DYP implementation history

- Added DMA RX plus USART2 IDLE-frame handoff from the interrupt handler to the DYP module.
- Added Modbus RTU CRC16, DMA transmission, distance-read and register-write helpers in `Core/Src/DYP/DYPA22.c`.
- Added on-demand DYP configuration helpers for algorithm enable, angle/range levels, and slave ID changes.
- Added RX frame validation and sensor status storage for the two configured sensor IDs.
- Added an independent USB CDC result module; the FreeRTOS task sends one result line after each complete sensor polling round (`DYP,ID1=...mm,ID2=...mm`).
- Added periodic DYP service invocation from the default FreeRTOS task.

## CubeMX status

No CubeMX configuration change is required for this code-only update. The existing USART2, DMA1 channel 1/2, PA2/PA3, and USART2 interrupt settings are used as generated.

Compilation and hardware verification were intentionally not run, per the change-stage constraint.
