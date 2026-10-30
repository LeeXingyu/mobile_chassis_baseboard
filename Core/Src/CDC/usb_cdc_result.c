#include "usb_cdc_result.h"
#include "usbd_cdc_if.h"
#include "zlac8015d.h"
#include "cdc_can_command.h"
#include <stdio.h>
#include <stdarg.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Both senders run in the same task and respect the endpoint's TX ownership. */
static uint8_t usb_tx_ready(void)
{
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
        (hUsbDeviceFS.pClassData == NULL)) {
        return 0U;
    }
    return (((USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData)->TxState == 0U) ? 1U : 0U;
}

/* Append bounded text; snprintf returns the required size even after truncation. */
static uint8_t append_text(uint8_t *buffer, uint16_t capacity, uint16_t *offset,
                           const char *format, ...)
{
    va_list args;
    int written;
    if (*offset >= capacity) {
        return 0U;
    }
    va_start(args, format);
    written = vsnprintf((char *)&buffer[*offset], capacity - *offset, format, args);
    va_end(args);
    if ((written < 0) || ((unsigned int)written >= (unsigned int)(capacity - *offset))) {
        return 0U;
    }
    *offset = (uint16_t)(*offset + written);
    return 1U;
}

/* Keep human-readable reason names in the reporting module, not the DYP driver. */
static const char *diagnostic_name(uint8_t detail)
{
    switch (detail) {
    case ULTRASONIC_DIAG_OK: return "OK";
    case ULTRASONIC_DIAG_PARAM: return "PARAM";
    case ULTRASONIC_DIAG_TX: return "TX_FAIL";
    case ULTRASONIC_DIAG_TIMEOUT: return "TIMEOUT";
    case ULTRASONIC_DIAG_LENGTH: return "LENGTH";
    case ULTRASONIC_DIAG_ADDRESS: return "ADDRESS";
    case ULTRASONIC_DIAG_FUNCTION: return "FUNCTION";
    case ULTRASONIC_DIAG_BYTE_COUNT: return "BYTE_COUNT";
    case ULTRASONIC_DIAG_CRC: return "CRC";
    case ULTRASONIC_DIAG_EXCEPTION: return "EXCEPTION";
    default: return "UNKNOWN";
    }
}

/* Motor task only records structured diagnostics; CDC remains owned by the
 * existing reporting task. Append to the same transfer to avoid USB TX races. */
static uint8_t append_motor_diagnostic(uint8_t *buffer, uint16_t capacity,
                                       uint16_t *offset)
{
    static ZlacDiagnostic motor;
    Zlac_GetDiagnostic(&motor);
    /* Small groups limit variadic argument stack use in the 512-byte sender task. */
    if (append_text(buffer, capacity, offset,
        "ZLAC,ID=%u,SEQ=%lu,RESULT=%u,OBJ=%04X:%02X,VALUE=0x%08lX,",
        (unsigned int)motor.node, (unsigned long)motor.sequence,
        (unsigned int)motor.result, (unsigned int)motor.index,
        (unsigned int)motor.subindex, (unsigned long)motor.value) == 0U) {
        return 0U;
    }
    if (append_text(buffer, capacity, offset,
        "ABORT=0x%08lX,MS=%lu,TX_HAL=%u,HAL=0x%08lX,BO=%lu,LEC=%lu,",
        (unsigned long)motor.abort_code, (unsigned long)motor.elapsed_ms,
        (unsigned int)motor.tx_hal, (unsigned long)motor.hal_error,
        (unsigned long)motor.bus_off, (unsigned long)motor.last_error) == 0U) {
        return 0U;
    }
    if (append_text(buffer, capacity, offset,
        "TEC=%lu,REC=%lu,RX=%lu,DROP=%lu,HB=%u:%u,",
        (unsigned long)motor.tx_errors, (unsigned long)motor.rx_errors,
        (unsigned long)motor.rx_frames, (unsigned long)motor.rx_dropped,
        (unsigned int)motor.heartbeat_node, (unsigned int)motor.heartbeat_state) == 0U) {
        return 0U;
    }
    if (append_text(buffer, capacity, offset,
        "EMCY=%u:%04X:%02X,LAST=%03X,LEN=%u,HEX=",
        (unsigned int)motor.emcy_node, (unsigned int)motor.emcy_code,
        (unsigned int)motor.emcy_register, (unsigned int)motor.last_rx_id,
        (unsigned int)motor.last_rx_length) == 0U) {
        return 0U;
    }
    for (uint8_t i = 0U; i < motor.last_rx_length; ++i) {
        if (append_text(buffer, capacity, offset, "%02X",
                        (unsigned int)motor.last_rx[i]) == 0U) {
            return 0U;
        }
    }
    return append_text(buffer, capacity, offset, "\r\n");
}

/* CDC transmission is asynchronous, so this buffer must remain valid while USB is busy. */
uint8_t UsbCdc_SendUltrasonicResults(const Ultrasonic_Sensor_t *sensors, uint8_t count)
{
    static uint8_t buffer[APP_TX_DATA_SIZE];
    uint16_t offset = 0U;
    USBD_CDC_HandleTypeDef *hcdc;

    if ((sensors == NULL) || (count == 0U)) {
        return 1U;
    }
    /* Never overwrite the static buffer while the USB endpoint still owns it. */
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
        (hUsbDeviceFS.pClassData == NULL)) {
        return 2U;
    }
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0U) {
        return 2U;
    }
    if (append_text(buffer, sizeof(buffer), &offset, "DYP,") == 0U) {
        return 1U;
    }
    for (uint8_t index = 0U; index < count && offset < sizeof(buffer); ++index) {
        uint8_t appended;
        if (sensors[index].is_valid != 0U) {
            appended = append_text(buffer, sizeof(buffer), &offset,
                               "ID%u=%umm%s", (unsigned int)sensors[index].id,
                               (unsigned int)sensors[index].distance,
                               (index + 1U < count) ? "," : "\r\n");
        } else {
            appended = append_text(buffer, sizeof(buffer), &offset,
                               "ID%u=INVALID%s", (unsigned int)sensors[index].id,
                               (index + 1U < count) ? "," : "\r\n");
        }
        if (appended == 0U) {
            return 1U;
        }
    }
    /* Submit all lines in one transfer so diagnostics do not compete with results. */
    for (uint8_t index = 0U; index < count; ++index) {
        const Ultrasonic_Diagnostic_t *diag = &sensors[index].diagnostic;
        if ((diag->rx_length > RS485_BUFF_SIZE) ||
            (append_text(buffer, sizeof(buffer), &offset,
                "DYP_DIAG,ID=%u,SEQ=%lu,ERR=%u,DETAIL=%s,TX_HAL=%u,UART=0x%08lX,"
                "MS=%lu,RX_LEN=%u,PARTIAL=%u,EX=0x%02X,HEX=",
                (unsigned int)sensors[index].id, (unsigned long)diag->sequence,
                (unsigned int)diag->result, diagnostic_name(diag->detail),
                (unsigned int)diag->tx_status, (unsigned long)diag->uart_error,
                (unsigned long)diag->elapsed_ms, (unsigned int)diag->rx_length,
                (unsigned int)diag->rx_partial, (unsigned int)diag->exception_code) == 0U)) {
            return 1U;
        }
        if ((diag->rx_length == 0U) &&
            (append_text(buffer, sizeof(buffer), &offset, "NONE") == 0U)) {
            return 1U;
        }
        for (uint16_t byte = 0U; byte < diag->rx_length; ++byte) {
            if (append_text(buffer, sizeof(buffer), &offset, "%s%02X",
                            (byte != 0U) ? " " : "", (unsigned int)diag->rx[byte]) == 0U) {
                return 1U;
            }
        }
        if (append_text(buffer, sizeof(buffer), &offset, "\r\n") == 0U) {
            return 1U;
        }
    }
    if (append_motor_diagnostic(buffer, sizeof(buffer), &offset) == 0U) {
        return 1U;
    }
    return (CDC_Transmit_FS(buffer, offset) == USBD_OK) ? 0U : 2U;
}

/* Human-readable completion results stay in the CDC module, not CAN ISR. */
static const char *can_reply_name(const CdcCanReply *reply)
{
    switch (reply->status) {
    case CDC_CAN_SYNTAX: return "SYNTAX";
    case CDC_CAN_SEND_FAILED: return "SEND_FAILED";
    case CDC_CAN_NMT_QUEUED: return "NMT_QUEUED";
    default:
        switch (reply->sdo.result) {
        case ZLAC_OK: return "OK";
        case ZLAC_TIMEOUT: return "TIMEOUT";
        case ZLAC_SDO_ABORT: return "SDO_ABORT";
        case ZLAC_BAD_RESPONSE: return "BAD_RESPONSE";
        case ZLAC_BUS_OFF: return "BUS_OFF";
        default: return "TX_ERROR";
        }
    }
}

uint8_t UsbCdc_SendCanReply(void)
{
    static uint8_t buffer[384];
    static CdcCanReply reply;
    uint16_t offset = 0U;
    if (usb_tx_ready() == 0U) {
        return 2U;
    }
    if (CdcCan_PeekReply(&reply) == 0U) {
        return 1U;
    }
    if ((append_text(buffer, sizeof(buffer), &offset,
            "CAN_REPLY,CMD=%lu,RESULT=%s,ID=%u,OBJ=%04X:%02X,",
            (unsigned long)reply.number, can_reply_name(&reply),
            (unsigned int)reply.sdo.node, (unsigned int)reply.sdo.index,
            (unsigned int)reply.sdo.subindex) == 0U) ||
        (append_text(buffer, sizeof(buffer), &offset,
            "SIZE=%u,VALUE=0x%08lX,ABORT=0x%08lX,",
            (unsigned int)reply.sdo.size, (unsigned long)reply.sdo.value,
            (unsigned long)reply.sdo.abort_code) == 0U) ||
        (append_text(buffer, sizeof(buffer), &offset,
            "TX_HAL=%u,MS=%lu,RX_DROP=%lu\r\n",
            (unsigned int)reply.tx_hal, (unsigned long)reply.sdo.elapsed_ms,
            (unsigned long)reply.rx_dropped) == 0U)) {
        return 2U;
    }
    if (CDC_Transmit_FS(buffer, offset) != USBD_OK) {
        return 2U;
    }
    /* Retire only after submission, never resend the CAN write on USB busy. */
    CdcCan_ReplySubmitted();
    return 0U;
}
