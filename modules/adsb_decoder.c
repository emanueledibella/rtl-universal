#include "header/adsb_decoder.h"

#include <stdio.h>
#include <string.h>

#define adsb_PREAMBLE_SAMPLES 16u
#define adsb_SAMPLES_PER_BIT 2u
#define adsb_MIN_DETECT_SAMPLES 256u
#define adsb_SHORT_FRAME_BITS 56u
#define adsb_LONG_FRAME_BITS 112u

static float adsb_hist_get(const adsb_ctx_t *ctx, uint64_t sample_index) {
    return ctx->mag_history[sample_index % adsb_MAG_HISTORY_LEN];
}

static void adsb_hist_put(adsb_ctx_t *ctx, uint64_t sample_index, float sample) {
    ctx->mag_history[sample_index % adsb_MAG_HISTORY_LEN] = sample;
}

static float adsb_max2(float a, float b) {
    return (a > b) ? a : b;
}

static float adsb_min2(float a, float b) {
    return (a < b) ? a : b;
}

static size_t adsb_mode_s_frame_bits_from_df(uint8_t df) {
    return (df >= 16u) ? adsb_LONG_FRAME_BITS : adsb_SHORT_FRAME_BITS;
}

static void pack_bits_to_bytes_msb(const uint8_t *bits, size_t nbits, uint8_t *out);

static int adsb_is_preamble(const adsb_ctx_t *ctx, uint64_t start_sample) {
    static const unsigned int hi_pos[4] = { 0u, 2u, 7u, 9u };
    static const unsigned int lo_pos[12] = { 1u, 3u, 4u, 5u, 6u, 8u, 10u, 11u, 12u, 13u, 14u, 15u };
    float p0 = adsb_hist_get(ctx, start_sample + hi_pos[0]);
    float p1 = adsb_hist_get(ctx, start_sample + hi_pos[1]);
    float p2 = adsb_hist_get(ctx, start_sample + hi_pos[2]);
    float p3 = adsb_hist_get(ctx, start_sample + hi_pos[3]);
    float high_avg = (p0 + p1 + p2 + p3) * 0.25f;
    float low_avg = 0.0f;
    float low_max = 0.0f;
    float peak_min = adsb_min2(adsb_min2(p0, p1), adsb_min2(p2, p3));

    for (size_t i = 0; i < sizeof(lo_pos) / sizeof(lo_pos[0]); i++) {
        float v = adsb_hist_get(ctx, start_sample + lo_pos[i]);
        low_avg += v;
        low_max = adsb_max2(low_max, v);
    }
    low_avg /= (float)(sizeof(lo_pos) / sizeof(lo_pos[0]));
    if (low_avg < 1e-3f) low_avg = 1e-3f;

    if (peak_min < ctx->threshold) return 0;
    if (high_avg < 2.0f * low_avg) return 0;
    if (low_max > 0.85f * peak_min) return 0;

    return 1;
}

static void adsb_deliver_frame(adsb_ctx_t *ctx, const uint8_t *bits, size_t frame_bits) {
    uint8_t packed[14];
    size_t frame_bytes;

    if (!ctx || !bits || !ctx->on_frame_cb) return;
    if (frame_bits == 0u || frame_bits > sizeof(ctx->frame_bits_buf)) return;

    frame_bytes = (frame_bits + 7u) / 8u;
    if (frame_bytes > sizeof(packed)) return;

    pack_bits_to_bytes_msb(bits, frame_bits, packed);
    ctx->on_frame_cb(ctx->on_frame_user, packed, frame_bits);
    ctx->clean_frame_count++;
}

static void adsb_emit_frame_bits(adsb_ctx_t *ctx, const uint8_t *bits, size_t frame_bits) {
    if (!ctx || !bits) return;
    if (frame_bits == 0u || frame_bits > sizeof(ctx->frame_bits_buf)) return;

    if (ctx->on_bit_cb) {
        for (size_t i = 0; i < frame_bits; i++) {
            ctx->on_bit_cb(ctx->on_bit_user, bits[i], ctx->clean_bit_count + i);
        }
    }

    ctx->clean_bit_count += frame_bits;
    adsb_deliver_frame(ctx, bits, frame_bits);
}

static int adsb_try_emit_captured_frame(adsb_ctx_t *ctx) {
    uint8_t bits[adsb_LONG_FRAME_BITS];
    uint8_t weak[adsb_LONG_FRAME_BITS];
    size_t capture_bits;
    size_t frame_bits;
    uint64_t data_start;
    uint8_t df;
    int weak_bits = 0;

    if (!ctx) return 0;

    capture_bits = adsb_LONG_FRAME_BITS;
    if (capture_bits > sizeof(bits)) return 0;

    data_start = ctx->capture_start_sample + adsb_PREAMBLE_SAMPLES;
    for (size_t i = 0; i < capture_bits; i++) {
        float early = adsb_hist_get(ctx, data_start + (uint64_t)(i * adsb_SAMPLES_PER_BIT));
        float late = adsb_hist_get(ctx, data_start + (uint64_t)(i * adsb_SAMPLES_PER_BIT + 1u));
        float hi = adsb_max2(early, late);
        float lo = adsb_min2(early, late);

        weak[i] = (uint8_t)((hi < 0.75f * ctx->threshold || hi < 1.10f * lo) ? 1u : 0u);
        bits[i] = (uint8_t)((early >= late) ? 1u : 0u);
    }

    df = 0u;
    for (size_t i = 0; i < 5u; i++) {
        df = (uint8_t)((df << 1u) | bits[i]);
    }

    frame_bits = adsb_mode_s_frame_bits_from_df(df);
    if (frame_bits == 0u || frame_bits > capture_bits) {
        ctx->rejected_frame_count++;
        return 0;
    }

    for (size_t i = 0; i < frame_bits; i++) {
        weak_bits += weak[i] ? 1 : 0;
    }

    if (weak_bits > (int)(frame_bits / 4u)) {
        ctx->rejected_frame_count++;
        return 0;
    }

    adsb_emit_frame_bits(ctx, bits, frame_bits);
    return 1;
}

static void pack_bits_to_bytes_msb(const uint8_t *bits, size_t nbits, uint8_t *out) {
    size_t nbytes = (nbits + 7u) / 8u;
    memset(out, 0, nbytes);
    for (size_t i = 0; i < nbits; i++) {
        if (bits[i] & 1u) {
            size_t by = i / 8u;
            int bi = (int)(7u - (i % 8u));
            out[by] |= (uint8_t)(1u << bi);
        }
    }
}

static uint32_t bits_get_u32(const uint8_t *bytes, int start_bit, int bit_len) {
    uint32_t v = 0;
    for (int i = 0; i < bit_len; i++) {
        int bit_index = start_bit + i;
        int byte_i = bit_index / 8;
        int bit_i = 7 - (bit_index % 8);
        int bit = (bytes[byte_i] >> bit_i) & 1;
        v = (v << 1) | (uint32_t)bit;
    }
    return v;
}

static uint64_t bits_get_u64(const uint8_t *bytes, int start_bit, int bit_len) {
    uint64_t v = 0;
    for (int i = 0; i < bit_len; i++) {
        int bit_index = start_bit + i;
        int byte_i = bit_index / 8;
        int bit_i = 7 - (bit_index % 8);
        int bit = (bytes[byte_i] >> bit_i) & 1;
        v = (v << 1) | (uint64_t)bit;
    }
    return v;
}

void adsb_on_clean_frame(const uint8_t *frame, size_t frame_bits) {
    size_t frame_bytes = frame_bits / 8u;
    uint32_t df;
    uint32_t ca;
    uint32_t icao;
    uint32_t me;
    uint32_t pi;

    printf("[adsb] frame bits=%zu bytes=%zu hex=", frame_bits, frame_bytes);
    for (size_t i = 0; i < frame_bytes; i++) {
        printf("%02X", frame[i]);
    }
    printf("\n");

    if (frame_bits < adsb_SHORT_FRAME_BITS) {
        printf("[adsb] short/incomplete frame\n");
        return;
    }

    df = bits_get_u32(frame, 0, 5);
    ca = bits_get_u32(frame, 5, 3);

    if (frame_bits == adsb_LONG_FRAME_BITS) {
        uint32_t aa = bits_get_u32(frame, 8, 24);
        uint64_t me = bits_get_u64(frame, 32, 56);
        uint32_t parity = bits_get_u32(frame, 88, 24);
        printf("[adsb] parsed df=%u ca=%u aa=%06X me=%014llX parity=%06X\n",
               df, ca, aa, (unsigned long long)me, parity);
        return;
    }

    {
        uint32_t payload = bits_get_u32(frame, 8, 24);
        uint32_t parity = bits_get_u32(frame, 32, 24);
        printf("[adsb] parsed df=%u ca=%u payload=%06X parity=%06X\n",
               df, ca, payload, parity);
    }
}

static void adsb_on_frame_default(void *user, const uint8_t *frame, size_t frame_bits) {
    (void)user;
    adsb_on_clean_frame(frame, frame_bits);
}

static void adsb_emit_frame_from_clean_bits(adsb_ctx_t *ctx) {
    size_t frame_bits = (size_t)ctx->frame_bits_len;
    adsb_deliver_frame(ctx, ctx->frame_bits_buf, frame_bits);
}

static void adsb_on_demod_sample_cb(void *user, float sample) {
    if (!user) return;
    adsb_process_am_sample((adsb_ctx_t *)user, sample);
}

void adsb_init(adsb_ctx_t *ctx, int fs_demod) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->fs_demod = fs_demod;
    ctx->threshold = 1.6f;      // normalized pulse height over local average
    ctx->frame_bits_target = (int)adsb_LONG_FRAME_BITS;
    ctx->on_frame_cb = adsb_on_frame_default;

    printf("[adsb] init fs_demod=%d frame_bits_target=%d\n", fs_demod, ctx->frame_bits_target);
}

void adsb_get_demod_config(demod_config_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->kind = DEMOD_KIND_AM;
    cfg->input_fs = 2000000;
    cfg->output_fs = 2000000;
    cfg->u.am.dc_alpha = 0.0005f;
}

demod_output_t adsb_get_demod_output(adsb_ctx_t *ctx) {
    demod_output_t out;
    memset(&out, 0, sizeof(out));
    out.on_float = adsb_on_demod_sample_cb;
    out.user = ctx;
    return out;
}

void adsb_set_callbacks(adsb_ctx_t *ctx,
                        adsb_on_bit_cb_t on_bit_cb, void *on_bit_user,
                        adsb_on_frame_cb_t on_frame_cb, void *on_frame_user) {
    if (!ctx) return;
    ctx->on_bit_cb = on_bit_cb;
    ctx->on_bit_user = on_bit_user;
    ctx->on_frame_cb = on_frame_cb ? on_frame_cb : adsb_on_frame_default;
    ctx->on_frame_user = on_frame_user;
}

void adsb_set_frame_bits_target(adsb_ctx_t *ctx, int frame_bits_target) {
    if (!ctx) return;
    if (frame_bits_target < 1 || frame_bits_target > (int)sizeof(ctx->frame_bits_buf)) return;
    ctx->frame_bits_target = frame_bits_target;
    ctx->frame_bits_len = 0;
}

void adsb_feed_clean_bit(adsb_ctx_t *ctx, uint8_t bit) {
    if (!ctx) return;

    bit = (uint8_t)(bit ? 1u : 0u);

    if (ctx->on_bit_cb) {
        ctx->on_bit_cb(ctx->on_bit_user, bit, ctx->clean_bit_count);
    }

    ctx->clean_bit_count++;

    if (ctx->frame_bits_len < (int)sizeof(ctx->frame_bits_buf)) {
        ctx->frame_bits_buf[ctx->frame_bits_len++] = bit;
    } else {
        // overflow safety
        ctx->frame_bits_len = 0;
        return;
    }

    if (ctx->frame_bits_len == ctx->frame_bits_target) {
        adsb_emit_frame_from_clean_bits(ctx);
        ctx->frame_bits_len = 0;
    }
}

void adsb_feed_clean_bits(adsb_ctx_t *ctx, const uint8_t *bits, size_t nbits) {
    if (!ctx || !bits) return;
    for (size_t i = 0; i < nbits; i++) {
        adsb_feed_clean_bit(ctx, bits[i]);
    }
}

void adsb_test_emit_example(adsb_ctx_t *ctx) {
    // Known ADS-B long frame (112 bits) often used in examples.
    // Hex: 8D40621D58C382D690C8AC2863A7
    static const uint8_t bits[112] = {
        1,0,0,0,1,1,0,1, 0,1,0,0,0,0,0,0,
        0,1,1,0,0,0,1,0, 0,0,0,1,1,1,0,1,
        0,1,0,1,1,0,0,0, 1,1,0,0,0,0,1,1,
        1,0,0,0,0,0,1,0, 1,1,0,1,0,1,1,0,
        1,0,0,1,0,0,0,0, 1,1,0,0,1,0,0,0,
        1,0,1,0,1,1,0,0, 0,0,1,0,1,0,0,0,
        0,1,1,0,0,0,1,1, 1,0,1,0,0,1,1,1
    };

    if (!ctx) return;
    printf("[adsb] emitting synthetic clean-bit frame\n");
    adsb_feed_clean_bits(ctx, bits, sizeof(bits) / sizeof(bits[0]));
}

void adsb_process_am_sample(adsb_ctx_t *ctx, float sample) {
    uint64_t sample_index;
    size_t needed_samples;
    float a;
    float baseline;
    float norm;

    if (!ctx) return;
    if (ctx->frame_bits_target < 1 || ctx->frame_bits_target > (int)sizeof(ctx->frame_bits_buf)) return;

    a = sample;
    if (a < 0.0f) a = -a;

    if (ctx->ema_abs <= 0.0f) {
        ctx->ema_abs = a;
    } else {
        ctx->ema_abs = 0.9995f * ctx->ema_abs + 0.0005f * a;
    }

    baseline = (ctx->ema_abs > 1e-3f) ? ctx->ema_abs : 1e-3f;
    norm = a / baseline;

    sample_index = ctx->demod_sample_count;
    adsb_hist_put(ctx, sample_index, norm);

    if (norm > ctx->threshold) {
        ctx->pulse_count++;
    }

    ctx->demod_sample_count++;

    needed_samples = (size_t)adsb_PREAMBLE_SAMPLES + (size_t)ctx->frame_bits_target * (size_t)adsb_SAMPLES_PER_BIT;
    if (needed_samples >= adsb_MAG_HISTORY_LEN) return;

    if (!ctx->capture_active
        && ctx->demod_sample_count >= adsb_MIN_DETECT_SAMPLES
        && sample_index + 1u >= adsb_PREAMBLE_SAMPLES) {
        uint64_t start_sample = sample_index + 1u - adsb_PREAMBLE_SAMPLES;
        if (start_sample >= ctx->next_search_sample && adsb_is_preamble(ctx, start_sample)) {
            ctx->capture_active = 1;
            ctx->capture_start_sample = start_sample;
            ctx->next_search_sample = start_sample + (uint64_t)needed_samples;
            ctx->detected_preamble_count++;
        }
    }

    if (ctx->capture_active) {
        uint64_t capture_end = ctx->capture_start_sample + (uint64_t)needed_samples;
        if (ctx->demod_sample_count >= capture_end) {
            (void)adsb_try_emit_captured_frame(ctx);
            ctx->capture_active = 0;
        }
    }

    if (ctx->fs_demod > 0 && (ctx->demod_sample_count % (uint64_t)ctx->fs_demod) == 0) {
        // printf("[adsb] demod samples=%llu pulses=%u preambles=%llu clean_frames=%llu rejected=%llu avg_abs=%.3f\n",
        //        (unsigned long long)ctx->demod_sample_count,
        //        ctx->pulse_count,
        //        (unsigned long long)ctx->detected_preamble_count,
        //        (unsigned long long)ctx->clean_frame_count,
        //        (unsigned long long)ctx->rejected_frame_count,
        //        ctx->ema_abs);
        ctx->pulse_count = 0;
    }
}

void adsb_flush(adsb_ctx_t *ctx) {
    if (!ctx) return;
    printf("[adsb] flush demod_samples=%llu clean_bits=%llu clean_frames=%llu preambles=%llu rejected=%llu\n",
           (unsigned long long)ctx->demod_sample_count,
           (unsigned long long)ctx->clean_bit_count,
           (unsigned long long)ctx->clean_frame_count,
           (unsigned long long)ctx->detected_preamble_count,
           (unsigned long long)ctx->rejected_frame_count);
}
