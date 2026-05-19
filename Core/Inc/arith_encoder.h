/**
 * @file      arith_encoder.h
 * @brief     Protótipos e Definições de Tipos do Motor de Codificação Aritmética.
 * @details   Apresentação Geral: Declara o contexto matemático e as operações de subdivisão de sub-intervalos inteiros.
 * Permissões de Uso: Código de uso acadêmico (SEMB). Livre para modificação e reuso com fins educacionais.
 * Como usar: Incluir este arquivo nos módulos que exigem compressão. Chamar `arith_start` antes de processar os dados.
 * Entrada: Símbolos unitários de 8 bits (uint8_t).
 * Saída: Status operacionais (`arith_status_t`) e acionamento de callback de emissão de bytes comprimidos.
 * Contexto: Trabalho da disciplina de Sistemas Embarcados (SEMB).
 * Plataforma Alvo: STM32F030R8T6.
 *
 * @author    Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo
 * @date      Maio de 2026
 */

#ifndef ARITH_ENCODER_H
#define ARITH_ENCODER_H

#include <stdint.h>
#include "config.h"

/**
 * @brief Códigos de status de retorno das funções do codificador aritmético.
 */
typedef enum {
    ARITH_STATUS_OK = 0,             /**< Operação concluída com sucesso */
    ARITH_STATUS_INVALID_ARG = 1,    /**< Argumento nulo ou inválido passado à função */
    ARITH_STATUS_MODEL_ERROR = 2,    /**< Inconsistência interna no modelo estatístico */
    ARITH_STATUS_OUTPUT_ERROR = 3    /**< Falha de hardware ou estouro ao ejetar os bits de saída */
} arith_status_t;

/**
 * @brief  Definição de tipo para o ponteiro de função (*callback*) de emissão.
 * @param  byte O byte bruto comprimido gerado.
 * @param  user Ponteiro de contexto opcional fornecido pelo usuário.
 * @return arith_status_t Status de sucesso ou falha no tratamento do byte.
 */
typedef arith_status_t (*arith_emit_fn)(uint8_t byte, void *user);

/**
 * @brief Estrutura de controle que encapsula o estado interno do codificador.
 */
typedef struct {
    uint16_t freq[ARITH_SYMBOL_COUNT]; /**< Tabela dinâmica de frequências acumuladas (Histograma) */
    uint16_t total;                    /**< Somatório global das frequências da tabela atual */
    uint32_t low;                      /**< Limite inferior do intervalo matemático atual (Precisão 16-bits) */
    uint32_t high;                     /**< Limite superior do intervalo matemático atual (Precisão 16-bits) */
    uint16_t follow_bits;              /**< Contador de bits pendentes para prevenção de sobrefluxo (*underflow*) */
    uint8_t current_byte;              /**< Acumulador de bits para formação do próximo byte de saída */
    uint8_t bit_count;                 /**< Quantidade atual de bits válidos guardados em current_byte */
    arith_emit_fn emit;                /**< Ponteiro para a função de callback de transmissão de dados */
    void *emit_user;                   /**< Contexto opcional do usuário repassado ao callback */
} arith_ctx_t;

/* --- Funções de Interface Pública --- */

/**
 * @brief  Inicializa contadores estatísticos e range ao estado original de entropia plena.
 * @param  ctx Ponteiro para a memória que abrigará o contexto.
 * @return void
 */
void arith_init(arith_ctx_t *ctx);

/**
 * @brief  Inicia formalmente uma sessão do encoder atribuindo funções de callback.
 * @param  ctx Ponteiro para a estrutura de contexto do algoritmo.
 * @param  emit Função disparada sempre que um byte comprimido é fechado.
 * @param  user Ponteiro genérico para passagem de contexto de aplicação.
 * @return arith_status_t Código de status operacional (ARITH_STATUS_OK em caso de sucesso).
 */
arith_status_t arith_start(arith_ctx_t *ctx, arith_emit_fn emit, void *user);

/**
 * @brief  Rotina CORE: Modela as partições estritas de alta e baixa probabilidade com renormalização iterativa.
 * @param  ctx Ponteiro para a estrutura de contexto do algoritmo.
 * @param  symbol Byte cru recebido que deve ser processado e comprimido.
 * @return arith_status_t Código de status operacional.
 */
arith_status_t arith_process_byte(arith_ctx_t *ctx, uint8_t symbol);

/**
 * @brief  Finaliza a fração, emite os bits remanescentes para definir inequivocamente o espaço sub-probabilístico e aplica flux.
 * @param  ctx Ponteiro para a estrutura de contexto do algoritmo.
 * @return arith_status_t Código de status operacional.
 */
arith_status_t arith_finish(arith_ctx_t *ctx);

#endif /* ARITH_ENCODER_H */
