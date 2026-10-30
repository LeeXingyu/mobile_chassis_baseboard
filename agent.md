# Agent Instructions

## Project scope

- Work in `baseboard_test`; the N32G032 project under `docs` is reference-only.
- Target MCU: STM32G474RCTX (STM32G4 HAL/CMSIS, FreeRTOS, STM32CubeIDE-generated project).
- Do not compile or run generated-binary verification during code-only changes.
- Treat CubeMX-generated regions as overwriteable. Put private/application code only in
  the matching `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` sections of
  generated files, and use dedicated user-owned source/header files for code that
  does not belong in those sections. Do not place custom code between generated
  sections: rerunning CubeMX can delete it.
- Before changing CubeMX settings or regenerating, identify every touched generated
  file and confirm custom code is in preserved user sections; afterward review the
  diff to ensure the custom code remains. If a needed insertion point is missing,
  stop and report it before proceeding with dependent edits.

## USART2 / DYP-A22 constraints

- USART2 uses PA2/PA3, 115200 8-N-1, DMA1 channel 1 (TX) and channel 2 (RX), and IDLE-line frame detection.
- The external RS485 transceiver handles direction; firmware must not add DE/RE GPIO switching.
- DYP-A22 traffic is Modbus RTU. CRC16 is transmitted low byte first.
- RX DMA is restarted after each IDLE frame. ISR code only captures a frame; parsing and sensor state updates belong in `Core/Src/DYP/DYPA22.c`.
- Do not change `baseboard_test.ioc` or request a CubeMX regeneration unless a required pin, DMA, interrupt, or peripheral setting is missing. Report such a requirement before dependent code changes.

## Change discipline

- ZLAC8015D uses FDCAN2 (PB5 RX/PB6 TX), classic CAN, 500 kbit/s:
  PCLK1 170 MHz / (17 * (1 + 15 + 4)). External transceiver needs no GPIO control.
- Preserve the user's existing FDCAN timing/.ioc changes. No CubeMX change is
  needed for the CANopen implementation: a runtime global filter accepts standard
  data frames despite StdFiltersNbr=0; reject extended IDs and remote frames.
- Keep CANopen in `Core/Src/zlac8015d.c` / `Core/Inc/zlac8015d.h`.
  ISR only queues RX. One statically allocated motor task owns all protocol/TX
  APIs; only the snapshot getter may be called from the CDC task. Do not add a
  second FDCAN RX callback or concurrent caller without redesigning ownership.
- Motor task uses above-normal priority and bounded work every 5 ms, so DYP
  busy waits cannot starve it. Static stack/TCB avoid the 3072-byte RTOS heap.
- Boot sends no CAN commands. CDC provides explicit node/index/subindex commands
  through `cdc_can_command.c/h`; there is no competing periodic SDO probe.
  Never automatically start NMT, enable a wheel or write speed.
  Standard CiA402 helpers remain opt-in, single logical-axis objects only.
- The supplied Version 1.02 hardware manual confirms CiA301/CiA402 and default
  500 kbit/s but lacks dual-axis object mappings and velocity scaling. Obtain
  vendor CANopen protocol/EDS before adding left/right motion control.
- Motor diagnostics are appended by the existing CDC sender in the same USB
  transfer as DYP; never transmit USB from the motor task or CAN ISR.
- CDC RX callback only assembles bounded ASCII lines into a four-command queue.
  Motor task parses and sends; command replies use a persistent single mailbox.
  Both CDC TX helpers run only in the existing default task, respect TxState,
  and consume a command reply only after USB accepts its copied TX buffer.
  Do not add a second USB sender or retry CAN writes due to USB busy.
- CDC numeric arguments are hexadecimal. Reject overflow, invalid lengths and
  extra tokens; never execute truncated lines. Require one command/result at a
  time in normal host operation. SDO completion is distinct from drive-state
  confirmation; NMT has no SDO acknowledgement.

- Keep DYP protocol helpers in `Core/Inc/DYP/DYPA22.h` and `Core/Src/DYP/DYPA22.c`.
- Keep interrupt handlers short and avoid blocking calls from ISR context.
- Keep USB CDC formatting and sending in the CDC module, never in the DYP driver
  or UART interrupt/error callbacks. The sender is owned by the single polling task.
- Preserve per-sensor distance-read diagnostics (return code, rejection reason,
  HAL UART error bits, RX length/raw bytes). UART error callbacks only record
  errors; diagnostics must not silently change bus timing or recovery behavior.
- Document functional changes in `README.md`.
