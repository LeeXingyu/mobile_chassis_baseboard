#ifndef CDC_CAN_COMMAND_H
#define CDC_CAN_COMMAND_H

#include "zlac8015d.h"

typedef enum {
    CDC_CAN_SDO_DONE = 0, CDC_CAN_SYNTAX, CDC_CAN_SEND_FAILED, CDC_CAN_NMT_QUEUED
} CdcCanReplyStatus;

typedef struct {
    uint32_t number, rx_dropped;
    ZlacDiagnostic sdo;
    uint8_t status, tx_hal;
} CdcCanReply;

/* USB receive callback: copy bytes only. Lines end with CR, LF or CRLF.
 * Complete lines are queued; overlong/invalid/full-queue lines are discarded. */
void CdcCan_ReceiveBytes(const uint8_t *data, uint32_t length);
/* Called only by the motor task, after Zlac_Service(). No USB TX here. */
void CdcCan_Service(void);
/* Single CDC sender peeks a persistent reply, and consumes it ONLY after USB
 * accepts the copied TX buffer. Backpressure prevents replies being overwritten. */
uint8_t CdcCan_PeekReply(CdcCanReply *reply);
void CdcCan_ReplySubmitted(void);

#endif
