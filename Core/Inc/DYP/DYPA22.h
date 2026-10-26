#ifndef INC_DYP_DYPA22_H_
#define INC_DYP_DYPA22_H_

#include "main.h"

#define RS485_BUFF_SIZE (128U)
#define SENSOR_COUNT (2U)
#define ULTRASONIC_REG_ANGLE_LEVEL (0x0208U)
#define ULTRASONIC_REG_RANGE_LEVEL (0x021FU)
#define ULTRASONIC_REG_SLAVE_ID (0x0200U)
#define ULTRASONIC_ANGLE_LEVEL_4 (0x0004U)
#define ULTRASONIC_RANGE_LEVEL_4 (0x0004U)
#define ULTRASONIC_RESP_TIMEOUT_MS (800U)
#define ULTRASONIC_WRITE_TIMEOUT_MS (80U)

extern uint8_t ULTRASONIC_rx_buf[RS485_BUFF_SIZE];
extern uint8_t ULTRASONIC_rx_process_buf[RS485_BUFF_SIZE];
extern uint8_t ULTRASONIC_tx_buf[RS485_BUFF_SIZE];
extern uint8_t ULTRASONIC_tx_process_buf[RS485_BUFF_SIZE];

typedef enum {
    ULTRASONIC_DIAG_OK = 0,
    ULTRASONIC_DIAG_PARAM,
    ULTRASONIC_DIAG_TX,
    ULTRASONIC_DIAG_TIMEOUT,
    ULTRASONIC_DIAG_LENGTH,
    ULTRASONIC_DIAG_ADDRESS,
    ULTRASONIC_DIAG_FUNCTION,
    ULTRASONIC_DIAG_BYTE_COUNT,
    ULTRASONIC_DIAG_CRC,
    ULTRASONIC_DIAG_EXCEPTION
} Ultrasonic_DiagCode_t;

typedef struct {
    uint32_t sequence;
    uint32_t uart_error;
    uint32_t elapsed_ms;
    uint16_t rx_length;
    uint8_t result; /* Existing read return code: 0 OK, 1 TX/parameter, 2 frame, 3 timeout. */
    uint8_t detail;
    uint8_t tx_status; /* HAL_OK / HAL_ERROR / HAL_BUSY / HAL_TIMEOUT. */
    uint8_t exception_code;
    uint8_t rx_partial; /* 1: timeout snapshot from DMA, not an IDLE-completed frame. */
    uint8_t rx[RS485_BUFF_SIZE];
} Ultrasonic_Diagnostic_t;

typedef struct {
    uint8_t id;
    uint16_t distance;
    uint8_t is_valid;
    uint8_t error_cnt;
    Ultrasonic_Diagnostic_t diagnostic;
} Ultrasonic_Sensor_t;

extern Ultrasonic_Sensor_t g_sensors[SENSOR_COUNT];

/* Capture one USART2 DMA frame after the IDLE interrupt. Call from ISR only. */
void Ultrasonic_UartIdleHandler(void);
/* Calculate the Modbus RTU CRC16 value (wire order is low byte first). */
uint16_t Ultrasonic_ModbusCrc16(const uint8_t *data, uint16_t length);
/* Append CRC and start a non-blocking USART2 DMA transmission. */
HAL_StatusTypeDef Ultrasonic_ModbusSend(const uint8_t *payload, uint16_t length);
/* Read the distance register of one DYP-A22 sensor. Returns zero on success. */
uint8_t Ultrasonic_ModbusReadDistance(uint8_t slave_id, uint16_t *distance);
/* Write one DYP-A22 holding register and validate its echo response. */
uint8_t Ultrasonic_ModbusWriteRegister(uint8_t slave_id, uint16_t reg_addr, uint16_t value);
/* Enable or disable the sensor internal measurement algorithm. */
uint8_t Ultrasonic_SetAlgorithm(uint8_t slave_id, uint8_t enable);
/* Configure one sensor's angle and range levels. */
uint8_t Ultrasonic_ConfigSensorLevels(uint8_t slave_id, uint16_t angle_level,
                                      uint16_t range_level);
/* Change a sensor address; address zero is rejected. */
uint8_t Ultrasonic_SetSlaveId(uint8_t old_id, uint8_t new_id);
/* Return the latest cached result for a configured sensor index. */
const Ultrasonic_Sensor_t *Ultrasonic_GetSensor(uint8_t index);
/* Poll sensors periodically; returns 1 after a complete sensor round. */
uint8_t Ultrasonic_ServiceTick(void);

#endif /* INC_DYP_DYPA22_H_ */
