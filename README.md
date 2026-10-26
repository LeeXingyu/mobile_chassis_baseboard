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

## USART2 read diagnostics

Each polling round now sends the original result line followed by one diagnostic
line per sensor, in the same USB CDC transfer. DYP only records structured data;
`Core/Src/CDC/usb_cdc_result.c` owns formatting and USB transmission. No new task
or timer is created, and the existing polling delay is preserved.

Example of a receive timeout:

```text
DYP,ID1=INVALID,ID2=INVALID
DYP_DIAG,ID=1,SEQ=1,ERR=3,DETAIL=TIMEOUT,TX_HAL=0,UART=0x00000000,MS=800,RX_LEN=0,PARTIAL=1,EX=0x00,HEX=NONE
```

- `SEQ`: global read transaction sequence, cached separately for each sensor.
- `ERR`: unchanged read return code: 0 success, 1 TX/parameter failure,
  2 rejected response, 3 response timeout.
- `DETAIL`: OK, PARAM, TX_FAIL, TIMEOUT, LENGTH, ADDRESS, FUNCTION, BYTE_COUNT,
  CRC, or EXCEPTION. It reports the first failed check, not all possible defects.
- `TX_HAL`: return status of the send helper: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT.
  The existing helper reports ERROR when the UART is not ready. HAL_OK only means
  DMA transmission was accepted, not that the sensor received the request.
- `UART`: HAL error bitmask recorded by the USART2 error callback and sampled at
  read completion: 0x01 parity, 0x02 noise, 0x04 framing, 0x08 overrun,
  0x10 DMA, 0x20 receiver timeout. Multiple bits can be set. This is not a Modbus
  error code; zero does not prove that wiring or the sensor is correct.
- `MS`: elapsed read time in milliseconds.
- `RX_LEN` / `HEX`: number of captured bytes and their raw hexadecimal values;
  NONE means no captured bytes. At most 128 bytes are retained.
- `PARTIAL=1`: timeout snapshot of the DMA buffer, not an IDLE-completed frame;
  it can be empty. A timeout can therefore have nonzero RX_LEN.
- `EX`: Modbus exception byte, populated only for a CRC-valid 0x83 response.

Typical checks: TX_FAIL means inspect UART/DMA state; TIMEOUT with no bytes means
inspect the bus, device address and baud rate; LENGTH with an 8-byte request in
HEX suggests local echo; CRC suggests corrupted or incorrectly framed data.
The first received frame is still used, so this diagnostic change does not fix
echoes, stale frames, UART recovery, or sensor special-distance handling.

Only distance reads are diagnosed here, not optional register writes. Records
are latest-per-sensor snapshots, not a historical queue. If USB is offline or
busy, that round is skipped without blocking or overwriting an in-flight TX
buffer; the next round reports its own results. Call the CDC sender from only
the existing polling task.

## CubeMX status

No CubeMX configuration change is required for this code-only update. The existing USART2, DMA1 channel 1/2, PA2/PA3, and USART2 interrupt settings are used as generated.

Compilation and hardware verification were intentionally not run, per the change-stage constraint.
