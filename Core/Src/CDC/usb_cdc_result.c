#include "usb_cdc_result.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

/* CDC transmission is asynchronous, so this buffer must remain valid while USB is busy. */
uint8_t UsbCdc_SendUltrasonicResults(const Ultrasonic_Sensor_t *sensors, uint8_t count)
{
    static uint8_t buffer[APP_TX_DATA_SIZE];
    uint16_t offset;

    if ((sensors == NULL) || (count == 0U)) {
        return 1U;
    }
    offset = (uint16_t)snprintf((char *)buffer, sizeof(buffer), "DYP,");
    if (offset >= sizeof(buffer)) {
        return 1U;
    }
    for (uint8_t index = 0U; index < count && offset < sizeof(buffer); ++index) {
        int written;
        if (sensors[index].is_valid != 0U) {
            written = snprintf((char *)&buffer[offset], sizeof(buffer) - offset,
                               "ID%u=%umm%s", sensors[index].id, sensors[index].distance,
                               (index + 1U < count) ? "," : "\r\n");
        } else {
            written = snprintf((char *)&buffer[offset], sizeof(buffer) - offset,
                               "ID%u=INVALID%s", sensors[index].id,
                               (index + 1U < count) ? "," : "\r\n");
        }
        if ((written < 0) || ((uint16_t)written >= (sizeof(buffer) - offset))) {
            /* Do not pass a truncated line or a length beyond the static buffer. */
            return 1U;
        }
        offset = (uint16_t)(offset + (uint16_t)written);
    }
    return (CDC_Transmit_FS(buffer, offset) == USBD_OK) ? 0U : 2U;
}
