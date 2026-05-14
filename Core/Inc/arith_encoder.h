#ifndef ARITH_ENCODER_H
#define ARITH_ENCODER_H

#include <stdint.h>

#include "config.h"

typedef enum {
    ARITH_STATUS_OK = 0,
    ARITH_STATUS_INVALID_ARG = 1,
    ARITH_STATUS_MODEL_ERROR = 2,
    ARITH_STATUS_OUTPUT_ERROR = 3
} arith_status_t;

typedef arith_status_t (*arith_emit_fn)(uint8_t byte, void *user);

typedef struct {
    uint16_t freq[ARITH_SYMBOL_COUNT];
    uint16_t total;
    uint32_t low;
    uint32_t high;
    uint16_t follow_bits;
    uint8_t current_byte;
    uint8_t bit_count;
    arith_emit_fn emit;
    void *emit_user;
} arith_ctx_t;

void arith_init(arith_ctx_t *ctx);
arith_status_t arith_start(arith_ctx_t *ctx, arith_emit_fn emit, void *user);
arith_status_t arith_process_byte(arith_ctx_t *ctx, uint8_t symbol);
arith_status_t arith_finish(arith_ctx_t *ctx);

#endif
