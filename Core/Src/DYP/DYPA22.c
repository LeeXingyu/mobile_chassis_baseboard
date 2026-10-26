#include "DYPA22.h"
#include "usart.h"
#include <string.h>

uint8_t ULTRASONIC_rx_buf[RS485_BUFF_SIZE];
uint8_t ULTRASONIC_rx_process_buf[RS485_BUFF_SIZE];
uint8_t ULTRASONIC_tx_buf[RS485_BUFF_SIZE];
uint8_t ULTRASONIC_tx_process_buf[RS485_BUFF_SIZE];

static volatile uint16_t s_rx_frame_length;
static volatile uint8_t s_rx_frame_ready;
static volatile uint32_t s_uart_error;
static Ultrasonic_Diagnostic_t s_read_diagnostic;

/* Preserve transient HAL errors before a subsequent RX restart clears ErrorCode. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        s_uart_error |= huart->ErrorCode;
    }
}

Ultrasonic_Sensor_t g_sensors[SENSOR_COUNT] = {
    {.id = 1U}, {.id = 2U}
};

uint16_t Ultrasonic_ModbusCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    while (length-- != 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 1U) != 0U) ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                                     : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

/* Stop the current DMA reception, publish the frame, and immediately re-arm RX. */
void Ultrasonic_UartIdleHandler(void)
{
    uint16_t length = (uint16_t)(RS485_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
    (void)HAL_UART_DMAStop(&huart2);
    if ((length != 0U) && (length <= RS485_BUFF_SIZE) && (s_rx_frame_ready == 0U)) {
        memcpy(ULTRASONIC_rx_process_buf, ULTRASONIC_rx_buf, length);
        s_rx_frame_length = length;
        s_rx_frame_ready = 1U;
    }
    memset(ULTRASONIC_rx_buf, 0, sizeof(ULTRASONIC_rx_buf));
    (void)HAL_UART_Receive_DMA(&huart2, ULTRASONIC_rx_buf, RS485_BUFF_SIZE);
}

/* Build a complete Modbus RTU frame and hand it to the USART2 TX DMA channel. */
HAL_StatusTypeDef Ultrasonic_ModbusSend(const uint8_t *payload, uint16_t length)
{
    uint16_t crc;
    if ((payload == NULL) || (length == 0U) || (length > (RS485_BUFF_SIZE - 2U)) ||
        (huart2.gState != HAL_UART_STATE_READY)) {
        return HAL_ERROR;
    }
    memcpy(ULTRASONIC_tx_process_buf, payload, length);
    crc = Ultrasonic_ModbusCrc16(payload, length);
    ULTRASONIC_tx_process_buf[length] = (uint8_t)(crc & 0xFFU);
    ULTRASONIC_tx_process_buf[length + 1U] = (uint8_t)(crc >> 8U);
    memcpy(ULTRASONIC_tx_buf, ULTRASONIC_tx_process_buf, length + 2U);
    return HAL_UART_Transmit_DMA(&huart2, ULTRASONIC_tx_buf, (uint16_t)(length + 2U));
}

/* Move the most recent frame out of the ISR-owned buffer atomically. */
static uint8_t take_rx_frame(uint8_t *frame, uint16_t capacity, uint16_t *length)
{
    uint8_t ready;
    __disable_irq();
    ready = s_rx_frame_ready;
    if (ready != 0U) {
        *length = s_rx_frame_length;
        if (*length > capacity) {
            *length = capacity;
        }
        memcpy(frame, ULTRASONIC_rx_process_buf, *length);
        s_rx_frame_ready = 0U;
    }
    __enable_irq();
    return ready;
}

/* Check address, function code, and the trailing Modbus CRC. */
static uint8_t validate_response(const uint8_t *frame, uint16_t length,
                                 uint8_t slave_id, uint8_t function)
{
    uint16_t received_crc;
    if ((length < 4U) || (frame[0] != slave_id) || (frame[1] != function)) {
        return 0U;
    }
    received_crc = (uint16_t)frame[length - 2U] |
                   (uint16_t)((uint16_t)frame[length - 1U] << 8U);
    return (Ultrasonic_ModbusCrc16(frame, (uint16_t)(length - 2U)) == received_crc) ? 1U : 0U;
}

/* Finalize task-owned diagnostics without coupling the protocol layer to USB. */
static uint8_t finish_read(uint8_t result, Ultrasonic_DiagCode_t detail, uint32_t start)
{
    s_read_diagnostic.result = result;
    s_read_diagnostic.detail = (uint8_t)detail;
    s_read_diagnostic.uart_error = s_uart_error | huart2.ErrorCode;
    s_read_diagnostic.elapsed_ms = HAL_GetTick() - start;
    return result;
}

/* Record the first observed frame without filtering echoes or changing read behavior. */
uint8_t Ultrasonic_ModbusReadDistance(uint8_t slave_id, uint16_t *distance)
{
    const uint8_t request[] = {slave_id, 0x03U, 0x01U, 0x01U, 0x00U, 0x01U};
    uint8_t frame[RS485_BUFF_SIZE];
    uint16_t length = 0U;
    uint32_t start = HAL_GetTick();
    uint32_t sequence = s_read_diagnostic.sequence + 1U;
    memset(&s_read_diagnostic, 0, sizeof(s_read_diagnostic));
    s_read_diagnostic.sequence = sequence;
    s_uart_error = 0U;
    if (distance == NULL) {
        return finish_read(1U, ULTRASONIC_DIAG_PARAM, start);
    }
    s_read_diagnostic.tx_status = (uint8_t)Ultrasonic_ModbusSend(request, sizeof(request));
    if (s_read_diagnostic.tx_status != HAL_OK) {
        return finish_read(1U, ULTRASONIC_DIAG_TX, start);
    }
    while ((HAL_GetTick() - start) < ULTRASONIC_RESP_TIMEOUT_MS) {
        if (take_rx_frame(frame, sizeof(frame), &length) != 0U) {
            s_read_diagnostic.rx_length = length;
            memcpy(s_read_diagnostic.rx, frame, length);
            if ((length == 5U) && validate_response(frame, length, slave_id, 0x83U)) {
                s_read_diagnostic.exception_code = frame[2];
                return finish_read(2U, ULTRASONIC_DIAG_EXCEPTION, start);
            }
            if (length != 7U) {
                return finish_read(2U, ULTRASONIC_DIAG_LENGTH, start);
            }
            if (frame[0] != slave_id) {
                return finish_read(2U, ULTRASONIC_DIAG_ADDRESS, start);
            }
            if (frame[1] != 0x03U) {
                return finish_read(2U, ULTRASONIC_DIAG_FUNCTION, start);
            }
            if (frame[2] != 0x02U) {
                return finish_read(2U, ULTRASONIC_DIAG_BYTE_COUNT, start);
            }
            if (validate_response(frame, length, slave_id, 0x03U) == 0U) {
                return finish_read(2U, ULTRASONIC_DIAG_CRC, start);
            }
            *distance = (uint16_t)(((uint16_t)frame[3] << 8U) | frame[4]);
            return finish_read(0U, ULTRASONIC_DIAG_OK, start);
        }
    }
    /* Snapshot partial bytes on timeout without stopping DMA or changing RX ownership. */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (s_rx_frame_ready != 0U) {
            s_read_diagnostic.rx_length = s_rx_frame_length;
            memcpy(s_read_diagnostic.rx, ULTRASONIC_rx_process_buf, s_rx_frame_length);
        } else if (huart2.RxState == HAL_UART_STATE_BUSY_RX) {
            uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);
            if (remaining <= RS485_BUFF_SIZE) {
                s_read_diagnostic.rx_length = (uint16_t)(RS485_BUFF_SIZE - remaining);
                s_read_diagnostic.rx_partial = 1U;
                memcpy(s_read_diagnostic.rx, ULTRASONIC_rx_buf, s_read_diagnostic.rx_length);
            }
        }
        __set_PRIMASK(primask);
    }
    return finish_read(3U, ULTRASONIC_DIAG_TIMEOUT, start);
}

/* Send function 0x06 and require the sensor to echo address and value. */
uint8_t Ultrasonic_ModbusWriteRegister(uint8_t slave_id, uint16_t reg_addr, uint16_t value)
{
    const uint8_t request[] = {slave_id, 0x06U, (uint8_t)(reg_addr >> 8U), (uint8_t)reg_addr,
                                (uint8_t)(value >> 8U), (uint8_t)value};
    uint8_t frame[RS485_BUFF_SIZE];
    uint16_t length = 0U;
    uint32_t start = HAL_GetTick();
    if (Ultrasonic_ModbusSend(request, sizeof(request)) != HAL_OK) {
        return 1U;
    }
    while ((HAL_GetTick() - start) < ULTRASONIC_WRITE_TIMEOUT_MS) {
        if (take_rx_frame(frame, sizeof(frame), &length) != 0U) {
            if ((length == 8U) && validate_response(frame, length, slave_id, 0x06U) &&
                (memcmp(&frame[2], &request[2], 4U) == 0)) {
                return 0U;
            }
            return 2U;
        }
    }
    return 3U;
}

/* Register 0x0227 controls the DYP-A22 internal algorithm switch. */
uint8_t Ultrasonic_SetAlgorithm(uint8_t slave_id, uint8_t enable)
{
    return Ultrasonic_ModbusWriteRegister(slave_id, 0x0227U, (enable != 0U) ? 1U : 0U);
}

/* Apply the angle and range levels as two validated Modbus write transactions. */
uint8_t Ultrasonic_ConfigSensorLevels(uint8_t slave_id, uint16_t angle_level,
                                      uint16_t range_level)
{
    uint8_t result = Ultrasonic_ModbusWriteRegister(slave_id,
                                                    ULTRASONIC_REG_ANGLE_LEVEL,
                                                    angle_level);
    if (result != 0U) {
        return result;
    }
    return Ultrasonic_ModbusWriteRegister(slave_id, ULTRASONIC_REG_RANGE_LEVEL, range_level);
}

/* Change the address only when both old and new addresses are valid Modbus IDs. */
uint8_t Ultrasonic_SetSlaveId(uint8_t old_id, uint8_t new_id)
{
    if ((old_id == 0U) || (new_id == 0U)) {
        return 1U;
    }
    return Ultrasonic_ModbusWriteRegister(old_id, ULTRASONIC_REG_SLAVE_ID, new_id);
}

const Ultrasonic_Sensor_t *Ultrasonic_GetSensor(uint8_t index)
{
    return (index < SENSOR_COUNT) ? &g_sensors[index] : NULL;
}

/* Round-robin polling service for the configured sensor IDs. */
uint8_t Ultrasonic_ServiceTick(void)
{
    static uint32_t next_poll;
    static uint8_t sensor_index;
    uint16_t distance;
    if ((int32_t)(HAL_GetTick() - next_poll) < 0) {
        return 0U;
    }
    next_poll = HAL_GetTick() + 20U;
    if (Ultrasonic_ModbusReadDistance(g_sensors[sensor_index].id, &distance) == 0U) {
        g_sensors[sensor_index].distance = distance;
        g_sensors[sensor_index].is_valid = 1U;
        g_sensors[sensor_index].error_cnt = 0U;
    } else {
        g_sensors[sensor_index].is_valid = 0U;
        if (g_sensors[sensor_index].error_cnt != 0xFFU) {
            ++g_sensors[sensor_index].error_cnt;
        }
    }
    g_sensors[sensor_index].diagnostic = s_read_diagnostic;
    sensor_index = (uint8_t)((sensor_index + 1U) % SENSOR_COUNT);
    if (sensor_index == 0U) {
        return 1U;
    }
    return 0U;
}
