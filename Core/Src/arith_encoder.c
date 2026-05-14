#include "arith_encoder.h"

#include <string.h>

static arith_status_t bit_writer_put_bit(arith_ctx_t *ctx, uint8_t bit) {
    arith_status_t status;
    if ((ctx == NULL) || (ctx->emit == NULL)) {
        return ARITH_STATUS_INVALID_ARG;
    }
    ctx->current_byte = (uint8_t)((ctx->current_byte << 1u) | (bit & 1u));
    ctx->bit_count++;
    if (ctx->bit_count == 8u) {
        status = ctx->emit(ctx->current_byte, ctx->emit_user);
        if (status != ARITH_STATUS_OK) {
            return status;
        }
        ctx->current_byte = 0u;
        ctx->bit_count = 0u;
    }
    return ARITH_STATUS_OK;
}

static arith_status_t bit_writer_bit_plus_follow(arith_ctx_t *ctx, uint8_t bit) {
    arith_status_t status = bit_writer_put_bit(ctx, bit);
    if (status != ARITH_STATUS_OK) {
        return status;
    }

    while (ctx->follow_bits > 0u) {
        status = bit_writer_put_bit(ctx, (uint8_t)(bit ^ 1u));
        if (status != ARITH_STATUS_OK) {
            return status;
        }
        ctx->follow_bits--;
    }

    return ARITH_STATUS_OK;
}

static arith_status_t bit_writer_flush(arith_ctx_t *ctx) {
    while (ctx->bit_count != 0u) {
        arith_status_t status = bit_writer_put_bit(ctx, 0u);
        if (status != ARITH_STATUS_OK) {
            return status;
        }
    }
    return ARITH_STATUS_OK;
}

void arith_init(arith_ctx_t *ctx) {
    uint16_t i;
    if (ctx == NULL) {
        return;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    for (i = 0u; i < ARITH_SYMBOL_COUNT; i++) {
        ctx->freq[i] = 1u;
    }
    ctx->total = ARITH_SYMBOL_COUNT;
    ctx->low = 0u;
    ctx->high = ((1u << 16u) - 1u);
}

static void arith_rescale_model(arith_ctx_t *ctx) {
    uint16_t i;
    uint32_t sum = 0u;
    for (i = 0u; i < ARITH_SYMBOL_COUNT; i++) {
        uint16_t v = (uint16_t)((ctx->freq[i] + 1u) >> 1u);
        if (v == 0u) {
            v = 1u;
        }
        ctx->freq[i] = v;
        sum += v;
    }
    ctx->total = (uint16_t)sum;
}

arith_status_t arith_start(arith_ctx_t *ctx, arith_emit_fn emit, void *user) {
    if ((ctx == NULL) || (emit == NULL)) {
        return ARITH_STATUS_INVALID_ARG;
    }
    arith_init(ctx);
    ctx->emit = emit;
    ctx->emit_user = user;
    return ARITH_STATUS_OK;
}

arith_status_t arith_process_byte(arith_ctx_t *ctx, uint8_t symbol) {
    uint16_t i;
    uint32_t cum_low = 0u;
    uint32_t cum_high;
    uint64_t range;
    arith_status_t status;
    const uint32_t max_value = ((1u << 16u) - 1u);
    const uint32_t half = (1u << 15u);
    const uint32_t first_qtr = (1u << 14u);
    const uint32_t third_qtr = first_qtr * 3u;

    if ((ctx == NULL) || (ctx->emit == NULL)) {
        return ARITH_STATUS_INVALID_ARG;
    }

    for (i = 0u; i < symbol; i++) {
        cum_low += ctx->freq[i];
    }
    cum_high = cum_low + ctx->freq[symbol];

    range = (uint64_t)((ctx->high - ctx->low) + 1u);
    ctx->high = (uint32_t)(ctx->low + (uint32_t)((range * cum_high) / ctx->total) - 1u);
    ctx->low = (uint32_t)(ctx->low + (uint32_t)((range * cum_low) / ctx->total));

    for (;;) {
        if (ctx->high < half) {
            status = bit_writer_bit_plus_follow(ctx, 0u);
            if (status != ARITH_STATUS_OK) {
                return status;
            }
            ctx->low = (ctx->low << 1u) & max_value;
            ctx->high = ((ctx->high << 1u) | 1u) & max_value;
        } else if (ctx->low >= half) {
            status = bit_writer_bit_plus_follow(ctx, 1u);
            if (status != ARITH_STATUS_OK) {
                return status;
            }
            ctx->low = ((ctx->low - half) << 1u) & max_value;
            ctx->high = (((ctx->high - half) << 1u) | 1u) & max_value;
        } else if ((ctx->low >= first_qtr) && (ctx->high < third_qtr)) {
            ctx->follow_bits++;
            ctx->low = ((ctx->low - first_qtr) << 1u) & max_value;
            ctx->high = (((ctx->high - first_qtr) << 1u) | 1u) & max_value;
        } else {
            break;
        }
    }

    if (ctx->freq[symbol] < 0xFFFFu) {
        ctx->freq[symbol]++;
        ctx->total++;
    }
    if (ctx->total >= ARITH_WNC_TOTAL_MAX) {
        arith_rescale_model(ctx);
    }

    return ARITH_STATUS_OK;
}

arith_status_t arith_finish(arith_ctx_t *ctx) {
    arith_status_t status;
    if ((ctx == NULL) || (ctx->emit == NULL)) {
        return ARITH_STATUS_INVALID_ARG;
    }

    ctx->follow_bits++;
    if (ctx->low < (1u << 14u)) {
        status = bit_writer_bit_plus_follow(ctx, 0u);
    } else {
        status = bit_writer_bit_plus_follow(ctx, 1u);
    }
    if (status != ARITH_STATUS_OK) {
        return status;
    }

    status = bit_writer_flush(ctx);
    if (status != ARITH_STATUS_OK) {
        return status;
    }
    return ARITH_STATUS_OK;
}
