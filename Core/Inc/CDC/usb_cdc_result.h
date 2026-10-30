#ifndef INC_CDC_USB_CDC_RESULT_H_
#define INC_CDC_USB_CDC_RESULT_H_

#include "DYP/DYPA22.h"

/* Send DYP results/diagnostics plus motor CANopen snapshot;
 * 0 sent, 1 invalid/overflow, 2 USB busy/offline.
 * Call only from the single polling task; the asynchronous TX buffer is shared. */
uint8_t UsbCdc_SendUltrasonicResults(const Ultrasonic_Sensor_t *sensors, uint8_t count);

/* Send a queued CAN command reply independently of DYP result availability.
 * Same single CDC TX task only. Returns 0 submitted, 1 no reply, 2 busy/offline. */
uint8_t UsbCdc_SendCanReply(void);

#endif /* INC_CDC_USB_CDC_RESULT_H_ */
