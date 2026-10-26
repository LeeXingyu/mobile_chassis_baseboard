#ifndef INC_CDC_USB_CDC_RESULT_H_
#define INC_CDC_USB_CDC_RESULT_H_

#include "DYP/DYPA22.h"

/* Send results plus per-sensor diagnostics; 0 sent, 1 invalid/overflow, 2 USB busy/offline.
 * Call only from the single polling task; the asynchronous TX buffer is shared. */
uint8_t UsbCdc_SendUltrasonicResults(const Ultrasonic_Sensor_t *sensors, uint8_t count);

#endif /* INC_CDC_USB_CDC_RESULT_H_ */
