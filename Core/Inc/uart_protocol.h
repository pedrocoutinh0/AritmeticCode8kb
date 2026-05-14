#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

typedef struct {
    uint8_t cmd;
    uint16_t len;
    uint8_t payload[APP_UART_RX_PAYLOAD_MAX];
} uart_frame_t;

typedef enum {
    UART_PARSE_WAIT_SOF = 0,
    UART_PARSE_WAIT_CMD,
    UART_PARSE_WAIT_LEN_L,
    UART_PARSE_WAIT_LEN_H,
    UART_PARSE_WAIT_PAYLOAD,
    UART_PARSE_WAIT_CHECKSUM
} uart_parse_state_t;

typedef struct {
    uart_parse_state_t state;
    uint8_t checksum;
    uart_frame_t frame;
    uint16_t payload_index;
    uint8_t expected_checksum;
    uint8_t error_code;
} uart_protocol_t;

void uart_protocol_init(uart_protocol_t *protocol);
bool uart_protocol_process_byte(uart_protocol_t *protocol, uint8_t byte, uart_frame_t *out_frame);
uint16_t uart_protocol_build_frame(uint8_t cmd,
                                   const uint8_t *payload,
                                   uint16_t len,
                                   uint8_t *out_buffer,
                                   uint16_t out_capacity);

#endif
