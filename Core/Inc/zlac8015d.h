#ifndef ZLAC8015D_H
#define ZLAC8015D_H

#include "stm32g4xx_hal.h"

#define ZLAC_SDO_TIMEOUT_MS 500U

typedef enum {
    ZLAC_IDLE = 0, ZLAC_WAITING, ZLAC_OK, ZLAC_TX_ERROR,
    ZLAC_TIMEOUT, ZLAC_SDO_ABORT, ZLAC_BAD_RESPONSE, ZLAC_BUS_OFF
} ZlacResult;

typedef struct {
    uint32_t sequence, value, abort_code, elapsed_ms;
    uint32_t hal_error, bus_off, last_error, tx_errors, rx_errors;
    uint32_t rx_frames, rx_dropped, heartbeat_ms;
    uint16_t index, emcy_code, last_rx_id;
    uint8_t node, subindex, size, result, tx_hal;
    uint8_t heartbeat_node, heartbeat_state, emcy_node, emcy_register;
    uint8_t last_rx_length, last_rx[8];
} ZlacDiagnostic;

/* All functions except GetDiagnostic belong to ONE motor task, never to ISR.
 * SDOs use the standard default COB-IDs and expedited (1..4 byte) transfers.
 * HAL_OK means queued, NOT acknowledged by the drive; inspect result later. */
HAL_StatusTypeDef Zlac_Init(void);
HAL_StatusTypeDef Zlac_ReadObject(uint8_t node, uint16_t index, uint8_t subindex);
HAL_StatusTypeDef Zlac_WriteObject(uint8_t node, uint16_t index, uint8_t subindex,
                                 uint32_t value, uint8_t size);
/* Explicit unicast NMT only: start=1, stop=2, pre-operational=128.
 * No broadcast/reset command and no automatic motor enable at boot. */
HAL_StatusTypeDef Zlac_SendNmt(uint8_t node, uint8_t command);
void Zlac_Service(void);
/* Coherent latest snapshot, callable by the independent CDC/reporting task. */
void Zlac_GetDiagnostic(ZlacDiagnostic *diagnostic);

/* Standard CiA402 logical-axis objects, subindex 0. These are opt-in APIs;
 * confirm the drive's axis mapping before calling. Each call starts ONE SDO;
 * wait for ZLAC_OK before the next. No left/right mapping is assumed here. */
HAL_StatusTypeDef Zlac402_ReadStatusword(uint8_t node);
HAL_StatusTypeDef Zlac402_WriteControlword(uint8_t node, uint16_t controlword);
HAL_StatusTypeDef Zlac402_SetMode(uint8_t node, int8_t mode);
/* Raw signed 0x60FF value, NOT RPM or m/s until vendor scaling is confirmed. */
HAL_StatusTypeDef Zlac402_SetTargetVelocityRaw(uint8_t node, int32_t velocity);
HAL_StatusTypeDef Zlac402_ReadActualVelocity(uint8_t node);

#endif
