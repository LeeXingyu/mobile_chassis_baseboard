#include "zlac8015d.h"
#include "fdcan.h"
#include <string.h>

#define RX_QUEUE_SIZE 16U
typedef struct {
    uint16_t id;
    uint8_t length, data[8];
} CanFrame;

static CanFrame rx_queue[RX_QUEUE_SIZE];
static volatile uint32_t rx_head, rx_tail, rx_dropped;
static ZlacDiagnostic diag;
static uint32_t started_ms, tx_buffer;
static uint8_t initialized, write_pending;

/* CANopen multi-byte fields are little endian, independent of host casts. */
static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

/* Standard classic CAN only; this HAL version takes an unshifted DLC value. */
static HAL_StatusTypeDef send_frame(uint16_t id, const uint8_t *data, uint8_t size)
{
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = size;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if (initialized == 0U) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, data);
}

HAL_StatusTypeDef Zlac_Init(void)
{
    HAL_StatusTypeDef status;
    if (initialized != 0U) {
        return HAL_BUSY;
    }
    memset(&diag, 0, sizeof(diag));
    rx_head = rx_tail = rx_dropped = 0U;
    /* Zero filter elements in CubeMX are sufficient: global acceptance routes
     * standard data frames to FIFO0. Extended IDs and remote frames are rejected. */
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0,
                                         FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE);
    if (status == HAL_OK) {
        status = HAL_FDCAN_Start(&hfdcan2);
    }
    if (status == HAL_OK) {
        status = HAL_FDCAN_ActivateNotification(&hfdcan2,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0U);
        if (status != HAL_OK) {
            (void)HAL_FDCAN_Stop(&hfdcan2);
        }
    }
    initialized = (status == HAL_OK) ? 1U : 0U;
    diag.tx_hal = (uint8_t)status;
    diag.hal_error = HAL_FDCAN_GetError(&hfdcan2);
    if (status != HAL_OK) {
        diag.result = ZLAC_TX_ERROR;
    }
    return status;
}

/* IRQ only drains hardware into a bounded single-producer/single-consumer queue.
 * No protocol processing, USB, delays or RTOS allocation in interrupt context. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    if (handle != &hfdcan2) {
        return;
    }
    if ((interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
        ++rx_dropped;
    }
    /* G4 FIFO0 contains three elements; bound ISR time even under bus traffic. */
    for (uint32_t count = 0U; count < 3U; ++count) {
        uint32_t next;
        if (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) == 0U) {
            break;
        }
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            ++rx_dropped;
            break;
        }
        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN) || (header.DataLength > 8U)) {
            continue;
        }
        next = (rx_head + 1U) % RX_QUEUE_SIZE;
        if (next == rx_tail) {
            ++rx_dropped;
            continue;
        }
        rx_queue[rx_head].id = (uint16_t)header.Identifier;
        rx_queue[rx_head].length = (uint8_t)header.DataLength;
        memset(rx_queue[rx_head].data, 0, 8U);
        memcpy(rx_queue[rx_head].data, data, header.DataLength);
        __DMB();
        rx_head = next;
    }
}

/* One outstanding SDO: matching node/index/subindex prevents unrelated frames
 * from completing a request. The application must not call this from two tasks. */
static HAL_StatusTypeDef request(uint8_t node, uint16_t index, uint8_t subindex,
                                uint32_t value, uint8_t size, uint8_t write)
{
    uint8_t data[8] = {0};
    HAL_StatusTypeDef status;
    if ((node == 0U) || (node > 127U) ||
        ((write != 0U) && ((size == 0U) || (size > 4U)))) {
        return HAL_ERROR;
    }
    if (diag.result == ZLAC_WAITING) {
        return HAL_BUSY;
    }
    /* Process stale buffered frames before marking a new transaction active. */
    Zlac_Service();
    ++diag.sequence;
    diag.node = node;
    diag.index = index;
    diag.subindex = subindex;
    diag.value = diag.abort_code = diag.elapsed_ms = 0U;
    diag.size = 0U;
    write_pending = write;
    data[0] = (write != 0U) ? (uint8_t)(0x23U | ((4U - size) << 2U)) : 0x40U;
    data[1] = (uint8_t)index;
    data[2] = (uint8_t)(index >> 8U);
    data[3] = subindex;
    for (uint8_t i = 0U; i < size; ++i) {
        data[4U + i] = (uint8_t)(value >> (8U * i));
    }
    started_ms = HAL_GetTick();
    status = send_frame((uint16_t)(0x600U + node), data, 8U);
    diag.tx_hal = (uint8_t)status;
    diag.result = (status == HAL_OK) ? ZLAC_WAITING : ZLAC_TX_ERROR;
    if (status == HAL_OK) {
        tx_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan2);
    }
    return status;
}

HAL_StatusTypeDef Zlac_ReadObject(uint8_t node, uint16_t index, uint8_t subindex)
{
    return request(node, index, subindex, 0U, 0U, 0U);
}

HAL_StatusTypeDef Zlac_WriteObject(uint8_t node, uint16_t index, uint8_t subindex,
                                 uint32_t value, uint8_t size)
{
    return request(node, index, subindex, value, size, 1U);
}

/* CiA402 helpers intentionally do not chain enable steps or infer dual-axis
 * offsets. The caller must inspect statusword and SDO result between commands. */
HAL_StatusTypeDef Zlac402_ReadStatusword(uint8_t node)
{
    return Zlac_ReadObject(node, 0x6041U, 0U);
}

HAL_StatusTypeDef Zlac402_WriteControlword(uint8_t node, uint16_t controlword)
{
    return Zlac_WriteObject(node, 0x6040U, 0U, controlword, 2U);
}

HAL_StatusTypeDef Zlac402_SetMode(uint8_t node, int8_t mode)
{
    return Zlac_WriteObject(node, 0x6060U, 0U, (uint8_t)mode, 1U);
}

HAL_StatusTypeDef Zlac402_SetTargetVelocityRaw(uint8_t node, int32_t velocity)
{
    return Zlac_WriteObject(node, 0x60FFU, 0U, (uint32_t)velocity, 4U);
}

HAL_StatusTypeDef Zlac402_ReadActualVelocity(uint8_t node)
{
    return Zlac_ReadObject(node, 0x606CU, 0U);
}

HAL_StatusTypeDef Zlac_SendNmt(uint8_t node, uint8_t command)
{
    /* Use a full backing word because HAL copies payload in 32-bit units. */
    uint8_t data[8] = {command, node, 0U};
    if ((node == 0U) || (node > 127U) ||
        ((command != 1U) && (command != 2U) && (command != 128U))) {
        return HAL_ERROR;
    }
    if (diag.result == ZLAC_WAITING) {
        return HAL_BUSY;
    }
    return send_frame(0U, data, 2U);
}

/* Parse only standard CANopen services whose layouts are defined by CiA301.
 * Manufacturer-specific dual-axis objects/scales require the vendor EDS. */
static void receive_frame(const CanFrame *frame)
{
    const uint8_t *data = frame->data;
    ++diag.rx_frames;
    diag.last_rx_id = frame->id;
    diag.last_rx_length = frame->length;
    memcpy(diag.last_rx, data, 8U);
    if ((frame->id >= 0x701U) && (frame->id <= 0x77FU) && (frame->length == 1U)) {
        diag.heartbeat_node = (uint8_t)(frame->id - 0x700U);
        diag.heartbeat_state = data[0];
        diag.heartbeat_ms = HAL_GetTick();
    }
    if ((frame->id >= 0x81U) && (frame->id <= 0xFFU) && (frame->length == 8U)) {
        diag.emcy_node = (uint8_t)(frame->id - 0x80U);
        diag.emcy_code = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
        diag.emcy_register = data[2];
    }
    if ((diag.result != ZLAC_WAITING) || (frame->id != (0x580U + diag.node))) {
        return;
    }
    if (frame->length < 4U) {
        return; /* Cannot correlate a truncated response with this SDO. */
    }
    if ((data[1] != (uint8_t)diag.index) ||
        (data[2] != (uint8_t)(diag.index >> 8U)) || (data[3] != diag.subindex)) {
        return;
    }
    diag.elapsed_ms = HAL_GetTick() - started_ms;
    if (frame->length != 8U) {
        diag.result = ZLAC_BAD_RESPONSE;
    } else if (data[0] == 0x80U) {
        diag.abort_code = read_le32(&data[4]);
        diag.result = ZLAC_SDO_ABORT;
    } else if ((write_pending != 0U) && (data[0] == 0x60U)) {
        diag.result = ZLAC_OK;
    } else if ((write_pending == 0U) &&
               ((data[0] == 0x43U) || (data[0] == 0x47U) ||
                (data[0] == 0x4BU) || (data[0] == 0x4FU))) {
        diag.size = (uint8_t)(4U - ((data[0] >> 2U) & 3U));
        diag.value = 0U;
        for (uint8_t i = 0U; i < diag.size; ++i) {
            diag.value |= (uint32_t)data[4U + i] << (8U * i);
        }
        diag.result = ZLAC_OK;
    } else {
        diag.result = ZLAC_BAD_RESPONSE; /* Segmented/unsized SDO not supported. */
    }
}

void Zlac_Service(void)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef counters = {0};
    if (initialized == 0U) {
        return;
    }
    for (uint32_t count = 0U; count < RX_QUEUE_SIZE; ++count) {
        CanFrame frame;
        if (rx_tail == rx_head) {
            break;
        }
        __DMB();
        frame = rx_queue[rx_tail];
        __DMB();
        rx_tail = (rx_tail + 1U) % RX_QUEUE_SIZE;
        receive_frame(&frame);
    }
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol);
    (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &counters);
    diag.bus_off = protocol.BusOff;
    diag.last_error = protocol.LastErrorCode;
    diag.tx_errors = counters.TxErrorCnt;
    diag.rx_errors = counters.RxErrorCnt;
    diag.hal_error = HAL_FDCAN_GetError(&hfdcan2);
    if (diag.result == ZLAC_WAITING) {
        diag.elapsed_ms = HAL_GetTick() - started_ms;
        if ((protocol.BusOff != 0U) || (diag.elapsed_ms >= ZLAC_SDO_TIMEOUT_MS)) {
            /* Do not leave an old queued request transmitting after timeout. */
            (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, tx_buffer);
            diag.result = (protocol.BusOff != 0U) ? ZLAC_BUS_OFF : ZLAC_TIMEOUT;
        }
    }
}

void Zlac_GetDiagnostic(ZlacDiagnostic *diagnostic)
{
    uint32_t primask;
    if (diagnostic == NULL) {
        return;
    }
    /* Short snapshot prevents a task switch or RX interrupt during the copy. */
    primask = __get_PRIMASK();
    __disable_irq();
    *diagnostic = diag;
    diagnostic->rx_dropped = rx_dropped;
    __set_PRIMASK(primask);
}
