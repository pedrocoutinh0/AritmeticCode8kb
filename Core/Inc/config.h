#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/*
 * Configuracao principal para a versao de 4 KB RAM.
 * Para a placa do professor, troque APP_MAX_INPUT_BYTES para 8192.
 */
#define APP_MAX_INPUT_BYTES         8192u
#define APP_UART_RX_PAYLOAD_MAX     32u
#define APP_UART_TX_PAYLOAD_MAX     32u
#define APP_ARITH_BLOCK_BYTES       128u

/* Frame: SOF + CMD + LEN(2) + PAYLOAD + CHK */
#define APP_UART_SOF                0xA5u
#define APP_UART_FRAME_OVERHEAD     5u
#define APP_UART_FRAME_MAX_BYTES    (APP_UART_FRAME_OVERHEAD + APP_UART_RX_PAYLOAD_MAX)

/* Timeouts de recepcao/transmissao em milissegundos */
#define APP_UART_RX_TIMEOUT_MS      20u
#define APP_UART_TX_TIMEOUT_MS      200u

/* Codificador aritmetico WNC em 16 bits */
#define ARITH_SYMBOL_COUNT          256u
#define ARITH_WNC_TOTAL_MAX         ((1u << 14u) - 1u)
#define ARITH_HEADER_BYTES          (4u + (ARITH_SYMBOL_COUNT * 2u))

/* Comandos do protocolo */
#define CMD_START                   0x01u
#define CMD_DATA                    0x02u
#define CMD_END_INPUT               0x03u
#define CMD_RUN                     0x04u

#define CMD_ACK                     0x79u
#define CMD_NACK                    0x1Fu
#define CMD_COMPRESSED_DATA         0x81u
#define CMD_END_OUTPUT              0x82u
#define CMD_ERROR                   0xE0u

/* Codigos de erro */
#define ERR_BAD_CHECKSUM            0x01u
#define ERR_BAD_STATE               0x02u
#define ERR_OVERSIZE                0x03u
#define ERR_TIMEOUT                 0x04u
#define ERR_ENCODER                 0x05u

#endif
