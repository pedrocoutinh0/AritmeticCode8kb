/**
 * @file      uart_protocol.c
 * @brief     FSM e Parser do Protocolo Camada de Enlace (UART).
 * @details   Apresentação Geral: Reconstrução assíncrona de frames de comunicação e validação lógica.
 * Permissões de Uso: Acadêmico.  Livre para reutilização em projetos de sistemas embarcados.
 * Como usar: Executar `uart_protocol_process_byte` iterativamente passando dados seriais crus lidos da placa.
 * Entrada: stream de bytes nativos e variáveis.
 * Saída: Estrutura isolada uart_frame_t empacotada livre de corrupção.
 * Contexto: Trabalho de disciplina de Sistemas Embarcados (SEMB).
 * Plataforma Alvo: STM32.
 *
 * @author    Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo
 * @date      Maio de 2026
 */

#include "uart_protocol.h"

/**
 * @brief  Inicializa ou reseta duramente as variáveis e estado principal do parser UART.
 * @param  protocol Ponteiro para estrutura uart_protocol_t alocada pela aplicação.
 * @return void
 */
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

/**
 * @brief  Reinicia a máquina de estado suavemente após concluir um pacote ou identificar erro.
 * @param  protocol Instância do protocolo.
 * @return void
 */
static void parser_reset(uart_protocol_t *protocol) {
    protocol->state = UART_PARSE_WAIT_SOF;
    protocol->checksum = 0u;
    protocol->frame.cmd = 0u;
    protocol->frame.len = 0u;
    protocol->payload_index = 0u;
    protocol->expected_checksum = 0u;
}

/**
 * @brief  Consome iterativamente a serial para estruturar os blocos do protocolo usando FSM.
 * @param  protocol Instância do estado atual.
 * @param  byte O byte que acaba de emergir na HAL_UART_Receive.
 * @param  out_frame Onde o bloco final estruturado será retornado.
 * @return bool True se o byte finalizou um frame com integridade validada; False se a FSM segue pendente.
 */
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

/**
 * @brief  Inversores da recepção: Encapsula dados puros no formato frame estruturado (SOF + HEADER + PAYLOAD + CHECKSUM).
 * @param  cmd Comando associado à ação desejada (ex: CMD_ACK).
 * @param  payload Ponteiro de arranjo contendo os dados a serem transmitidos.
 * @param  len Tamanho em bytes da carga util.
 * @param  out_buffer Arranjo pré-alocado pela aplicação onde o fluxo linear de bytes será montado.
 * @param  out_capacity Capacidade física máxima de armazenamento do out_buffer (evita estouros).
 * @return uint16_t Total de bytes preenchidos no buffer final para envio físico.
 */
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
