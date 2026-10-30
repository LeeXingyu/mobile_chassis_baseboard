#include "cdc_can_command.h"
#include <string.h>

#define COMMAND_LENGTH 80U
#define COMMAND_SLOTS 5U /* Ring reserves one slot: four complete queued commands. */
static char rx_line[COMMAND_LENGTH];
static char lines[COMMAND_SLOTS][COMMAND_LENGTH];
static uint32_t rx_length;
static uint8_t discard_line;
static volatile uint32_t head, tail, dropped;
static CdcCanReply reply;
static volatile uint8_t reply_ready;
static uint8_t sdo_pending;
static uint32_t command_number;

/* USB packets are a byte stream: assemble split lines and handle several lines
 * per packet. Never execute a partial/truncated command, especially a write. */
void CdcCan_ReceiveBytes(const uint8_t *data, uint32_t length)
{
    if (data == NULL) {
        return;
    }
    for (uint32_t i = 0U; i < length; ++i) {
        uint8_t ch = data[i];
        if ((ch == '\r') || (ch == '\n')) {
            if (discard_line != 0U) {
                ++dropped;
            } else if (rx_length != 0U) {
                uint32_t next = (head + 1U) % COMMAND_SLOTS;
                if (next == tail) {
                    ++dropped;
                } else {
                    rx_line[rx_length] = '\0';
                    memcpy(lines[head], rx_line, rx_length + 1U);
                    __DMB();
                    head = next;
                }
            }
            rx_length = 0U;
            discard_line = 0U;
        } else if (discard_line == 0U) {
            if (((ch < 0x20U) && (ch != '\t')) || (ch > 0x7EU) ||
                (rx_length >= COMMAND_LENGTH - 1U)) {
                discard_line = 1U;
            } else {
                rx_line[rx_length++] = (char)ch;
            }
        }
    }
}

/* Split in place without strtok/sscanf or dynamic allocation. */
static uint8_t split_tokens(char *line, char **tokens, uint8_t capacity)
{
    uint8_t count = 0U;
    while (*line != '\0') {
        while ((*line == ' ') || (*line == '\t')) {
            ++line;
        }
        if (*line == '\0') {
            break;
        }
        if (count == capacity) {
            return 0U;
        }
        tokens[count++] = line;
        while ((*line != '\0') && (*line != ' ') && (*line != '\t')) {
            ++line;
        }
        if (*line != '\0') {
            *line++ = '\0';
        }
    }
    return count;
}

/* Every numeric token is hexadecimal, optionally prefixed with 0x. Reject
 * overflow rather than silently truncating a node, object or speed value. */
static uint8_t parse_hex(const char *text, uint32_t maximum, uint32_t *value)
{
    uint32_t number = 0U;
    if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X'))) {
        text += 2;
    }
    if (*text == '\0') {
        return 0U;
    }
    while (*text != '\0') {
        uint32_t digit;
        char ch = *text++;
        if ((ch >= '0') && (ch <= '9')) {
            digit = (uint32_t)(ch - '0');
        } else if ((ch >= 'a') && (ch <= 'f')) {
            digit = (uint32_t)(ch - 'a') + 10U;
        } else if ((ch >= 'A') && (ch <= 'F')) {
            digit = (uint32_t)(ch - 'A') + 10U;
        } else {
            return 0U;
        }
        if ((digit > maximum) || (number > (maximum - digit) / 16U)) {
            return 0U;
        }
        number = number * 16U + digit;
    }
    *value = number;
    return 1U;
}

/* Publish the fully populated mailbox only after the response is complete. */
static void publish_reply(uint8_t status, HAL_StatusTypeDef tx_status)
{
    reply.status = status;
    reply.tx_hal = (uint8_t)tx_status;
    __DMB();
    reply_ready = 1U;
}

/* One command/SDO at a time. Reply backpressure means an offline/busy USB host
 * cannot cause queued writes to execute without preserving their results. */
void CdcCan_Service(void)
{
    static char line[COMMAND_LENGTH];
    char *tokens[7];
    uint32_t node, index, subindex, size, value;
    uint8_t count;
    HAL_StatusTypeDef status;
    if (reply_ready != 0U) {
        return;
    }
    if (sdo_pending != 0U) {
        Zlac_GetDiagnostic(&reply.sdo);
        if (reply.sdo.result != ZLAC_WAITING) {
            sdo_pending = 0U;
            publish_reply(CDC_CAN_SDO_DONE, (HAL_StatusTypeDef)reply.sdo.tx_hal);
        }
        return;
    }
    if (tail == head) {
        return;
    }
    __DMB();
    memcpy(line, lines[tail], sizeof(line));
    __DMB();
    tail = (tail + 1U) % COMMAND_SLOTS;
    memset(&reply, 0, sizeof(reply));
    reply.number = ++command_number;
    count = split_tokens(line, tokens, 7U);
    if ((count == 3U) && (strcmp(tokens[0], "NMT") == 0) &&
        (parse_hex(tokens[1], 127U, &node) != 0U) && (node != 0U) &&
        (parse_hex(tokens[2], 255U, &value) != 0U) &&
        ((value == 1U) || (value == 2U) || (value == 128U))) {
        reply.sdo.node = (uint8_t)node;
        reply.sdo.value = value;
        status = Zlac_SendNmt((uint8_t)node, (uint8_t)value);
        /* NMT has no SDO confirmation: QUEUED never means drive state changed. */
        publish_reply((status == HAL_OK) ? CDC_CAN_NMT_QUEUED : CDC_CAN_SEND_FAILED,
                      status);
        return;
    }
    if (((count == 5U) || (count == 7U)) &&
        (strcmp(tokens[0], "SDO") == 0) &&
        (parse_hex(tokens[2], 127U, &node) != 0U) && (node != 0U) &&
        (parse_hex(tokens[3], 65535U, &index) != 0U) &&
        (parse_hex(tokens[4], 255U, &subindex) != 0U)) {
        if ((count == 5U) && (strcmp(tokens[1], "R") == 0)) {
            status = Zlac_ReadObject((uint8_t)node, (uint16_t)index, (uint8_t)subindex);
        } else if ((count == 7U) && (strcmp(tokens[1], "W") == 0) &&
                   (parse_hex(tokens[5], 4U, &size) != 0U) && (size != 0U) &&
                   (parse_hex(tokens[6], UINT32_MAX, &value) != 0U) &&
                   ((size == 4U) || ((value >> (size * 8U)) == 0U))) {
            status = Zlac_WriteObject((uint8_t)node, (uint16_t)index,
                                      (uint8_t)subindex, value, (uint8_t)size);
        } else {
            publish_reply(CDC_CAN_SYNTAX, HAL_ERROR);
            return;
        }
        Zlac_GetDiagnostic(&reply.sdo);
        if (status == HAL_OK) {
            sdo_pending = 1U;
        } else {
            publish_reply(CDC_CAN_SEND_FAILED, status);
        }
        return;
    }
    publish_reply(CDC_CAN_SYNTAX, HAL_ERROR);
}

uint8_t CdcCan_PeekReply(CdcCanReply *destination)
{
    uint32_t primask;
    uint8_t available;
    if (destination == NULL) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    available = reply_ready;
    if (available != 0U) {
        *destination = reply;
        destination->rx_dropped = dropped;
    }
    __set_PRIMASK(primask);
    return available;
}

void CdcCan_ReplySubmitted(void)
{
    /* Only the single CDC TX task may consume this mailbox. */
    __DMB();
    reply_ready = 0U;
}
