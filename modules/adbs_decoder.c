#include "header/adbs_decoder.h"

#include <stdio.h>
#include <string.h>

#define ADBS_PREAMBLE_SAMPLES 16u
#define ADBS_SAMPLES_PER_BIT 2u
#define ADBS_MIN_DETECT_SAMPLES 256u

static float adbs_hist_get(const adbs_ctx_t *ctx, uint64_t sample_index) {
    return ctx->mag_history[sample_index % ADBS_MAG_HISTORY_LEN];
}

static void adbs_hist_put(adbs_ctx_t *ctx, uint64_t sample_index, float sample) {
    ctx->mag_history[sample_index % ADBS_MAG_HISTORY_LEN] = sample;
}

static float adbs_max2(float a, float b) {
    return (a > b) ? a : b;
}

static float adbs_min2(float a, float b) {
    return (a < b) ? a : b;
}

static int adbs_is_preamble(const adbs_ctx_t *ctx, uint64_t start_sample) {
    static const unsigned int hi_pos[4] = { 0u, 2u, 7u, 9u };
    static const unsigned int lo_pos[12] = { 1u, 3u, 4u, 5u, 6u, 8u, 10u, 11u, 12u, 13u, 14u, 15u };
    float p0 = adbs_hist_get(ctx, start_sample + hi_pos[0]);
    float p1 = adbs_hist_get(ctx, start_sample + hi_pos[1]);
    float p2 = adbs_hist_get(ctx, start_sample + hi_pos[2]);
    float p3 = adbs_hist_get(ctx, start_sample + hi_pos[3]);
    float high_avg = (p0 + p1 + p2 + p3) * 0.25f;
    float low_avg = 0.0f;
    float low_max = 0.0f;
    float peak_min = adbs_min2(adbs_min2(p0, p1), adbs_min2(p2, p3));

    for (size_t i = 0; i < sizeof(lo_pos) / sizeof(lo_pos[0]); i++) {
        float v = adbs_hist_get(ctx, start_sample + lo_pos[i]);
        low_avg += v;
        low_max = adbs_max2(low_max, v);
    }
    low_avg /= (float)(sizeof(lo_pos) / sizeof(lo_pos[0]));
    if (low_avg < 1e-3f) low_avg = 1e-3f;

    if (peak_min < ctx->threshold) return 0;
    if (high_avg < 3.0f * low_avg) return 0;
    if (low_max > 0.60f * peak_min) return 0;

    return 1;
}

static int adbs_try_emit_captured_frame(adbs_ctx_t *ctx) {
    uint8_t bits[112];
    size_t frame_bits;
    uint64_t data_start;
    int weak_bits = 0;

    if (!ctx) return 0;

    frame_bits = (size_t)ctx->frame_bits_target;
    if (frame_bits == 0u || frame_bits > sizeof(bits)) return 0;

    data_start = ctx->capture_start_sample + ADBS_PREAMBLE_SAMPLES;
    for (size_t i = 0; i < frame_bits; i++) {
        float early = adbs_hist_get(ctx, data_start + (uint64_t)(i * ADBS_SAMPLES_PER_BIT));
        float late = adbs_hist_get(ctx, data_start + (uint64_t)(i * ADBS_SAMPLES_PER_BIT + 1u));
        float hi = adbs_max2(early, late);
        float lo = adbs_min2(early, late);

        if (hi < 0.85f * ctx->threshold || hi < 1.20f * lo) {
            weak_bits++;
        }

        bits[i] = (uint8_t)((early >= late) ? 1u : 0u);
    }

    if (weak_bits > (int)(frame_bits / 8u)) {
        ctx->rejected_frame_count++;
        return 0;
    }

    adbs_feed_clean_bits(ctx, bits, frame_bits);
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

void adbs_on_clean_frame(const uint8_t *frame, size_t frame_bits) {
    size_t frame_bytes = frame_bits / 8u;

    printf("[ADBS] frame bits=%zu bytes=%zu hex=", frame_bits, frame_bytes);
    for (size_t i = 0; i < frame_bytes; i++) {
        printf("%02X", frame[i]);
    }
    printf("\n");

    // Basic fields ready for protocol conversion work.
    uint32_t df = bits_get_u32(frame, 0, 5);
    uint32_t ca = bits_get_u32(frame, 5, 3);
    uint32_t icao = bits_get_u32(frame, 8, 24);
    uint32_t me = bits_get_u32(frame, 32, 56);
    uint32_t pi = bits_get_u32(frame, 88, 24);
    printf("[ADBS] parsed df=%u ca=%u icao=%06X me=%014lX pi=%06X\n", df, ca, icao, me, pi);
}

static void adbs_on_frame_default(void *user, const uint8_t *frame, size_t frame_bits) {
    (void)user;
    adbs_on_clean_frame(frame, frame_bits);
}

static void adbs_emit_frame_from_clean_bits(adbs_ctx_t *ctx) {
    if (!ctx || !ctx->on_frame_cb) return;

    size_t frame_bits = (size_t)ctx->frame_bits_len;
    size_t frame_bytes = (frame_bits + 7u) / 8u;
    uint8_t packed[14];
    if (frame_bytes > sizeof(packed)) return;

    pack_bits_to_bytes_msb(ctx->frame_bits_buf, frame_bits, packed);
    ctx->on_frame_cb(ctx->on_frame_user, packed, frame_bits);
    ctx->clean_frame_count++;
}

static void adbs_on_demod_sample_cb(void *user, float sample) {
    if (!user) return;
    adbs_process_am_sample((adbs_ctx_t *)user, sample);
}

void adbs_init(adbs_ctx_t *ctx, int fs_demod) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->fs_demod = fs_demod;
    ctx->threshold = 2.0f;      // normalized pulse height over local average
    ctx->frame_bits_target = 112;
    ctx->on_frame_cb = adbs_on_frame_default;

    printf("[ADBS] init fs_demod=%d frame_bits_target=%d\n", fs_demod, ctx->frame_bits_target);
}

void adbs_get_demod_config(demod_config_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->kind = DEMOD_KIND_AM;
    cfg->input_fs = 2000000;
    cfg->output_fs = 2000000;
    cfg->u.am.dc_alpha = 0.0005f;
}

demod_output_t adbs_get_demod_output(adbs_ctx_t *ctx) {
    demod_output_t out;
    memset(&out, 0, sizeof(out));
    out.on_float = adbs_on_demod_sample_cb;
    out.user = ctx;
    return out;
}

void adbs_set_callbacks(adbs_ctx_t *ctx,
                        adbs_on_bit_cb_t on_bit_cb, void *on_bit_user,
                        adbs_on_frame_cb_t on_frame_cb, void *on_frame_user) {
    if (!ctx) return;
    ctx->on_bit_cb = on_bit_cb;
    ctx->on_bit_user = on_bit_user;
    ctx->on_frame_cb = on_frame_cb ? on_frame_cb : adbs_on_frame_default;
    ctx->on_frame_user = on_frame_user;
}

void adbs_set_frame_bits_target(adbs_ctx_t *ctx, int frame_bits_target) {
    if (!ctx) return;
    if (frame_bits_target < 1 || frame_bits_target > (int)sizeof(ctx->frame_bits_buf)) return;
    ctx->frame_bits_target = frame_bits_target;
    ctx->frame_bits_len = 0;
}

void adbs_feed_clean_bit(adbs_ctx_t *ctx, uint8_t bit) {
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
        adbs_emit_frame_from_clean_bits(ctx);
        ctx->frame_bits_len = 0;
    }
}

void adbs_feed_clean_bits(adbs_ctx_t *ctx, const uint8_t *bits, size_t nbits) {
    if (!ctx || !bits) return;
    for (size_t i = 0; i < nbits; i++) {
        adbs_feed_clean_bit(ctx, bits[i]);
    }
}

void adbs_test_emit_example(adbs_ctx_t *ctx) {
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
    printf("[ADBS] emitting synthetic clean-bit frame\n");
    adbs_feed_clean_bits(ctx, bits, sizeof(bits) / sizeof(bits[0]));
}

void adbs_process_am_sample(adbs_ctx_t *ctx, float sample) {
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
    adbs_hist_put(ctx, sample_index, norm);

    if (norm > ctx->threshold) {
        ctx->pulse_count++;
    }

    ctx->demod_sample_count++;

    needed_samples = (size_t)ADBS_PREAMBLE_SAMPLES + (size_t)ctx->frame_bits_target * (size_t)ADBS_SAMPLES_PER_BIT;
    if (needed_samples >= ADBS_MAG_HISTORY_LEN) return;

    if (!ctx->capture_active
        && ctx->demod_sample_count >= ADBS_MIN_DETECT_SAMPLES
        && sample_index + 1u >= ADBS_PREAMBLE_SAMPLES) {
        uint64_t start_sample = sample_index + 1u - ADBS_PREAMBLE_SAMPLES;
        if (start_sample >= ctx->next_search_sample && adbs_is_preamble(ctx, start_sample)) {
            ctx->capture_active = 1;
            ctx->capture_start_sample = start_sample;
            ctx->next_search_sample = start_sample + (uint64_t)needed_samples;
            ctx->detected_preamble_count++;
        }
    }

    if (ctx->capture_active) {
        uint64_t capture_end = ctx->capture_start_sample + (uint64_t)needed_samples;
        if (ctx->demod_sample_count >= capture_end) {
            (void)adbs_try_emit_captured_frame(ctx);
            ctx->capture_active = 0;
        }
    }

    if (ctx->fs_demod > 0 && (ctx->demod_sample_count % (uint64_t)ctx->fs_demod) == 0) {
        // printf("[ADBS] demod samples=%llu pulses=%u preambles=%llu clean_frames=%llu rejected=%llu avg_abs=%.3f\n",
        //        (unsigned long long)ctx->demod_sample_count,
        //        ctx->pulse_count,
        //        (unsigned long long)ctx->detected_preamble_count,
        //        (unsigned long long)ctx->clean_frame_count,
        //        (unsigned long long)ctx->rejected_frame_count,
        //        ctx->ema_abs);
        ctx->pulse_count = 0;
    }
}

void adbs_flush(adbs_ctx_t *ctx) {
    if (!ctx) return;
    printf("[ADBS] flush demod_samples=%llu clean_bits=%llu clean_frames=%llu preambles=%llu rejected=%llu\n",
           (unsigned long long)ctx->demod_sample_count,
           (unsigned long long)ctx->clean_bit_count,
           (unsigned long long)ctx->clean_frame_count,
           (unsigned long long)ctx->detected_preamble_count,
           (unsigned long long)ctx->rejected_frame_count);
}
