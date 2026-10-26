#ifndef INC_CDC_USB_CDC_RESULT_H_
#define INC_CDC_USB_CDC_RESULT_H_

#include "DYP/DYPA22.h"

/* Send one completed ultrasonic polling result through the USB CDC port. */
uint8_t UsbCdc_SendUltrasonicResults(const Ultrasonic_Sensor_t *sensors, uint8_t count);

#endif /* INC_CDC_USB_CDC_RESULT_H_ */
