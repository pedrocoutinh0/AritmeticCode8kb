#include "uart_protocol.h"

void uart_protocol_init(uart_protocol_t *protocol) {
    if (protocol == 0) {
        return;
    }

    protocol->state = UART_PARSE_WAIT_SOF;
    protocol->checksum = 0u;
    protocol->frame.cmd = 0u;
    protocol->frame.len = 0u;
    protocol->payload_index = 0u;
    protocol->expected_checksum = 0u;
    protocol->error_code = 0u;
}

static void parser_reset(uart_protocol_t *protocol) {
    protocol->state = UART_PARSE_WAIT_SOF;
    protocol->checksum = 0u;
    protocol->frame.cmd = 0u;
    protocol->frame.len = 0u;
    protocol->payload_index = 0u;
    protocol->expected_checksum = 0u;
}

bool uart_protocol_process_byte(uart_protocol_t *protocol, uint8_t byte, uart_frame_t *out_frame) {
    if ((protocol == 0) || (out_frame == 0)) {
        return false;
    }

    switch (protocol->state) {
    case UART_PARSE_WAIT_SOF:
        if (byte == APP_UART_SOF) {
            protocol->state = UART_PARSE_WAIT_CMD;
            protocol->checksum = 0u;
            protocol->payload_index = 0u;
            protocol->error_code = 0u;
        }
        break;

    case UART_PARSE_WAIT_CMD:
        protocol->frame.cmd = byte;
        protocol->checksum ^= byte;
        protocol->state = UART_PARSE_WAIT_LEN_L;
        break;

    case UART_PARSE_WAIT_LEN_L:
        protocol->frame.len = byte;
        protocol->checksum ^= byte;
        protocol->state = UART_PARSE_WAIT_LEN_H;
        break;

    case UART_PARSE_WAIT_LEN_H:
        protocol->frame.len |= ((uint16_t)byte << 8u);
        protocol->checksum ^= byte;
        if (protocol->frame.len > APP_UART_RX_PAYLOAD_MAX) {
            protocol->error_code = ERR_OVERSIZE;
            parser_reset(protocol);
        } else if (protocol->frame.len == 0u) {
            protocol->state = UART_PARSE_WAIT_CHECKSUM;
        } else {
            protocol->state = UART_PARSE_WAIT_PAYLOAD;
        }
        break;

    case UART_PARSE_WAIT_PAYLOAD:
        protocol->frame.payload[protocol->payload_index] = byte;
        protocol->checksum ^= byte;
        protocol->payload_index++;
        if (protocol->payload_index >= protocol->frame.len) {
            protocol->state = UART_PARSE_WAIT_CHECKSUM;
        }
        break;

    case UART_PARSE_WAIT_CHECKSUM:
        protocol->expected_checksum = byte;
        if (protocol->checksum == protocol->expected_checksum) {
            *out_frame = protocol->frame;
            parser_reset(protocol);
            return true;
        }
        protocol->error_code = ERR_BAD_CHECKSUM;
        parser_reset(protocol);
        break;

    default:
        parser_reset(protocol);
        break;
    }

    return false;
}

uint16_t uart_protocol_build_frame(uint8_t cmd,
                                   const uint8_t *payload,
                                   uint16_t len,
                                   uint8_t *out_buffer,
                                   uint16_t out_capacity) {
    uint16_t i;
    uint16_t total_len;
    uint8_t checksum;

    if (out_buffer == 0) {
        return 0u;
    }

    if ((len > 0u) && (payload == 0)) {
        return 0u;
    }

    total_len = (uint16_t)(1u + 1u + 2u + len + 1u);
    if (len > APP_UART_TX_PAYLOAD_MAX || total_len > out_capacity) {
        return 0u;
    }

    checksum = 0u;
    out_buffer[0] = APP_UART_SOF;
    out_buffer[1] = cmd;
    checksum ^= cmd;

    out_buffer[2] = (uint8_t)(len & 0xFFu);
    checksum ^= out_buffer[2];
    out_buffer[3] = (uint8_t)((len >> 8u) & 0xFFu);
    checksum ^= out_buffer[3];

    for (i = 0u; i < len; i++) {
        out_buffer[4u + i] = payload[i];
        checksum ^= payload[i];
    }

    out_buffer[4u + len] = checksum;
    return total_len;
}
