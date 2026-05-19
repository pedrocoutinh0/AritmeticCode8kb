/**
 * @file      config.h
 * @brief     Configurações Globais e Macros do Sistema.
 * @details   Apresentação Geral: Centraliza os parâmetros de dimensionamento de memória, limites da UART, comandos do protocolo e códigos de erro.
 * Permissões de Uso: Acadêmico (SEMB).
 * Como usar: Incluir este cabeçalho em qualquer módulo que dependa de limites estruturais ou mapeamento do protocolo serial.
 * Contexto: Trabalho da disciplina de Sistemas Embarcados (SEMB).
 * Plataforma Alvo: STM32F030R8T6.
 *
 * @author    Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo
 * @date      Maio de 2026
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* --- Parâmetros de Dimensionamento de Memória --- */
/** @brief Memória máxima alocada para o arquivo de entrada (Para a placa do professor, 8192 bytes) */
#define APP_MAX_INPUT_BYTES         8192u
/** @brief Tamanho máximo do payload de recepção em um único frame UART */
#define APP_UART_RX_PAYLOAD_MAX     32u
/** @brief Tamanho máximo do payload de transmissão em um único frame UART */
#define APP_UART_TX_PAYLOAD_MAX     32u
/** @brief Tamanho do bloco isolado para processamento aritmético */
#define APP_ARITH_BLOCK_BYTES       128u

/* --- Estrutura do Frame UART --- */
/** @brief Identificador de Start of Frame (SOF) da camada de enlace */
#define APP_UART_SOF                0xA5u
/** @brief Sobrecarga fixa do pacote lógico: SOF(1) + CMD(1) + LEN(2) + CHK(1) */
#define APP_UART_FRAME_OVERHEAD     5u
/** @brief Tamanho absoluto máximo do buffer em RAM para segurar um frame físico */
#define APP_UART_FRAME_MAX_BYTES    (APP_UART_FRAME_OVERHEAD + APP_UART_RX_PAYLOAD_MAX)

/* --- Timeouts --- */
/** @brief Timeout da interrupção para leitura serial (ms) */
#define APP_UART_RX_TIMEOUT_MS      20u
/** @brief Timeout para escrita serial bloqueante (ms) */
#define APP_UART_TX_TIMEOUT_MS      200u

/* --- Constantes do Codificador Aritmético WNC (16 bits) --- */
/** @brief Quantidade de símbolos distintos no modelo de probabilidade (0 a 255) */
#define ARITH_SYMBOL_COUNT          256u
/** @brief Limite de saturação do acumulador de frequência (evita underflow no range de precisão) */
#define ARITH_WNC_TOTAL_MAX         ((1u << 14u) - 1u)
/** @brief Sobrecarga extra do cabeçalho em caso de encapsulamento estrutural do algoritmo */
#define ARITH_HEADER_BYTES          (4u + (ARITH_SYMBOL_COUNT * 2u))

/* --- Comandos do Protocolo (Camada de Aplicação) --- */
#define CMD_START                   0x01u   /**< Início de sessão (traz o tamanho total do arquivo) */
#define CMD_DATA                    0x02u   /**< Envio sequencial de bloco (chunk) do arquivo original */
#define CMD_END_INPUT               0x03u   /**< Aviso de conclusão do tráfego do arquivo original */
#define CMD_RUN                     0x04u   /**< Comando de gatilho suplementar para finalização */

#define CMD_ACK                     0x79u   /**< Confirmação positiva (Acknowledge) do controle Stop-and-Wait */
#define CMD_NACK                    0x1Fu   /**< Rejeição de frame por quebra de fluxo ou inconsistência */
#define CMD_COMPRESSED_DATA         0x81u   /**< Identificador do bitstream de saída (early emit ou final) */
#define CMD_END_OUTPUT              0x82u   /**< Flag de encerramento, contendo tamanho comprimido final */
#define CMD_ERROR                   0xE0u   /**< Falha catastrófica da FSM ou motor algébrico do STM32 */

/* --- Códigos de Erro de Exceção --- */
#define ERR_BAD_CHECKSUM            0x01u   /**< Corrupção de barramento físico (XOR falhou) */
#define ERR_BAD_STATE               0x02u   /**< Ordem de comando inválida na FSM (ex: CMD_DATA em IDLE) */
#define ERR_OVERSIZE                0x03u   /**< Arquivo declarado excede a reserva de RAM definida em APP_MAX_INPUT_BYTES */
#define ERR_TIMEOUT                 0x04u   /**< Atraso crítico de I/O na interface */
#define ERR_ENCODER                 0x05u   /**< Erro da operação bitwise de compressão da placa */

#endif /* APP_CONFIG_H */