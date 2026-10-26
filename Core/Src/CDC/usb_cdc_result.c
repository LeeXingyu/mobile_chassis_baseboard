#include "usb_cdc_result.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

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
    return (CDC_Transmit_FS(buffer, offset) == USBD_OK) ? 0U : 2U;
}
