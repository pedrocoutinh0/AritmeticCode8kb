/**
 * @file      uart_protocol.h
 * @brief     Declarações de Tipos e Protótipos para o Parser da Camada de Enlace (UART).
 * @details   Apresentação Geral: Define a estrutura do frame binário de comunicação e a FSM de recepção serial.
 * Permissões de Uso: Acadêmico. Livre para reutilização em projetos de sistemas embarcados.
 * Como usar: Instanciar a estrutura `uart_protocol_t` no `main.c` e invocar `uart_protocol_init` antes do loop.
 * Entrada: Bytes individuais da USART através da função `uart_protocol_process_byte`.
 * Saída: Quadros lógicos completos (`uart_frame_t`) contendo comandos decodificados e validados via Checksum.
 * Contexto: Trabalho da disciplina de Sistemas Embarcados (SEMB).
 * Plataforma Alvo: STM32F030R8T6 / STM32F030F4P6.
 *
 * @author    Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo
 * @date      Maio de 2026
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/**
 * @brief Estrutura que representa um pacote de dados lógico decodificado.
 */
typedef struct {
    uint8_t cmd;                              /**< Identificador da instrução ou evento (START, DATA, ACK, etc) */
    uint16_t len;                             /**< Comprimento real do payload útil contido no frame */
    uint8_t payload[APP_UART_RX_PAYLOAD_MAX]; /**< Buffer estático transiente que armazena a carga de dados */
} uart_frame_t;

/**
 * @brief Estados internos da Máquina de Estados Finita (FSM) do Parser Serial.
 */
typedef enum {
    UART_PARSE_WAIT_SOF = 0,  /**< Estado inicial: aguarda o byte delimitador de início (0xA5) */
    UART_PARSE_WAIT_CMD,      /**< Aguarda o byte identificador do comando */
    UART_PARSE_WAIT_LEN_L,    /**< Aguarda o byte menos significativo do tamanho (LSB) */
    UART_PARSE_WAIT_LEN_H,    /**< Aguarda o byte mais significativo do tamanho (MSB) */
    UART_PARSE_WAIT_PAYLOAD,  /**< Consome bytes seriais preenchendo o buffer até atingir LEN */
    UART_PARSE_WAIT_CHECKSUM  /**< Intercepta o byte de validação e executa o batimento por XOR */
} uart_parse_state_t;

/**
 * @brief Estrutura operacional que retém o estado em tempo real da máquina do parser.
 */
typedef struct {
    uart_parse_state_t state;   /**< Estado atual em que a FSM de enlace se encontra */
    uint8_t checksum;           /**< Acumulador dinâmico do cálculo XOR do pacote em recepção */
    uart_frame_t frame;         /**< Instância do frame transiente que está sendo progressivamente montado */
    uint16_t payload_index;     /**< Cursor/índice de escrita para preenchimento ordenado do payload */
    uint8_t expected_checksum;  /**< Byte de checksum extraído do final do pacote físico recebido */
    uint8_t error_code;         /**< Armazena códigos de erro se houver falha de validação (ex: ERR_BAD_CHECKSUM) */
} uart_protocol_t;

/* --- Funções de Interface Pública --- */

/**
 * @brief  Inicializa ou reseta duramente as variáveis e estado principal do parser UART.
 * @param  protocol Ponteiro para a estrutura de controle do protocolo.
 * @return void
 */
void uart_protocol_init(uart_protocol_t *protocol);

/**
 * @brief  Consome iterativamente a serial para estruturar os blocos do protocolo usando FSM.
 * @param  protocol Ponteiro para o contexto do estado do protocolo.
 * @param  byte O byte bruto capturado diretamente do registrador de recepção UART.
 * @param  out_frame Ponteiro de saída onde o pacote será depositado caso sua integridade seja confirmada.
 * @return bool Retorna true se um frame completo e livre de erros foi fechado; false caso contrário.
 */
bool uart_protocol_process_byte(uart_protocol_t *protocol, uint8_t byte, uart_frame_t *out_frame);

/**
 * @brief  Inversores da recepção: Encapsula dados puros no formato frame estruturado (SOF + HEADER + PAYLOAD + CHECKSUM).
 * @param  cmd Comando associado à ação desejada (ex: CMD_ACK).
 * @param  payload Ponteiro de arranjo contendo os dados a serem transmitidos.
 * @param  len Tamanho em bytes da carga util.
 * @param  out_buffer Arranjo pré-alocado pela aplicação onde o fluxo linear de bytes será montado.
 * @param  out_capacity Capacidade física máxima de armazenamento do out_buffer (evita estouros).
 * @return uint16_t Quantidade total de bytes efetivamente preenchidos no buffer final para envio físico.
 */
uint16_t uart_protocol_build_frame(uint8_t cmd,
                                   const uint8_t *payload,
                                   uint16_t len,
                                   uint8_t *out_buffer,
                                   uint16_t out_capacity);

#endif /* UART_PROTOCOL_H */
