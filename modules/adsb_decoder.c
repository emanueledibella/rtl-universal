#include "header/adsb_decoder.h"
#include "header/adsb_protocol.h"
#include "output.h"

#include <stdio.h>
#include <string.h>

#define PREAMBLE_SAMPLES 16u
#define SAMPLES_PER_BIT 2u
#define MIN_DETECT_SAMPLES 256u
#define SHORT_FRAME_BITS 56u
#define LONG_FRAME_BITS 112u
#define MIN_PREAMBLE_PEAK_TO_NOISE 2.1f
#define MAX_PREAMBLE_LOW_TO_PEAK 0.82f
#define MIN_BIT_HIGH_TO_LOW 1.20f
#define MAX_WEAK_BIT_DIVISOR 8u

static float hist_get(const adsb_ctx_t *ctx, uint64_t sample_index) {
    return ctx->mag_history[sample_index % MAG_HISTORY_LEN];
}

static void hist_put(adsb_ctx_t *ctx, uint64_t sample_index, float sample) {
    ctx->mag_history[sample_index % MAG_HISTORY_LEN] = sample;
}

static float max2(float a, float b) {
    return (a > b) ? a : b;
}

static float min2(float a, float b) {
    return (a < b) ? a : b;
}

static size_t mode_s_frame_bits_from_df(uint8_t df) {
    return (df >= 16u) ? LONG_FRAME_BITS : SHORT_FRAME_BITS;
}

static void pack_bits_to_bytes_msb(const uint8_t *bits, size_t nbits, uint8_t *out);

static int is_preamble(const adsb_ctx_t *ctx, uint64_t start_sample) {
    static const unsigned int hi_pos[4] = { 0u, 2u, 7u, 9u };
    static const unsigned int lo_pos[12] = { 1u, 3u, 4u, 5u, 6u, 8u, 10u, 11u, 12u, 13u, 14u, 15u };
    float p0 = hist_get(ctx, start_sample + hi_pos[0]);
    float p1 = hist_get(ctx, start_sample + hi_pos[1]);
    float p2 = hist_get(ctx, start_sample + hi_pos[2]);
    float p3 = hist_get(ctx, start_sample + hi_pos[3]);
    float high_avg = (p0 + p1 + p2 + p3) * 0.25f;
    float low_avg = 0.0f;
    float low_max = 0.0f;
    float peak_min = min2(min2(p0, p1), min2(p2, p3));

    for (size_t i = 0; i < sizeof(lo_pos) / sizeof(lo_pos[0]); i++) {
        float v = hist_get(ctx, start_sample + lo_pos[i]);
        low_avg += v;
        low_max = max2(low_max, v);
    }
    low_avg /= (float)(sizeof(lo_pos) / sizeof(lo_pos[0]));
    if (low_avg < 1e-3f) low_avg = 1e-3f;

    if (peak_min < ctx->threshold) return 0;
    if (high_avg < MIN_PREAMBLE_PEAK_TO_NOISE * low_avg) return 0;
    if (low_max > MAX_PREAMBLE_LOW_TO_PEAK * peak_min) return 0;

    return 1;
}

static void deliver_frame(adsb_ctx_t *ctx, const uint8_t *bits, size_t frame_bits) {
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

static void emit_frame_bits(adsb_ctx_t *ctx, const uint8_t *bits, size_t frame_bits) {
    if (!ctx || !bits) return;
    if (frame_bits == 0u || frame_bits > sizeof(ctx->frame_bits_buf)) return;

    if (ctx->on_bit_cb) {
        for (size_t i = 0; i < frame_bits; i++) {
            ctx->on_bit_cb(ctx->on_bit_user, bits[i], ctx->clean_bit_count + i);
        }
    }

    ctx->clean_bit_count += frame_bits;
    deliver_frame(ctx, bits, frame_bits);
}

static int try_emit_captured_frame(adsb_ctx_t *ctx) {
    uint8_t bits[LONG_FRAME_BITS];
    uint8_t weak[LONG_FRAME_BITS];
    size_t capture_bits;
    size_t frame_bits;
    uint64_t data_start;
    uint8_t df;
    int weak_bits = 0;

    if (!ctx) return 0;

    capture_bits = LONG_FRAME_BITS;
    if (capture_bits > sizeof(bits)) return 0;

    data_start = ctx->capture_start_sample + PREAMBLE_SAMPLES;
    for (size_t i = 0; i < capture_bits; i++) {
        float early = hist_get(ctx, data_start + (uint64_t)(i * SAMPLES_PER_BIT));
        float late = hist_get(ctx, data_start + (uint64_t)(i * SAMPLES_PER_BIT + 1u));
        float hi = max2(early, late);
        float lo = min2(early, late);

        weak[i] = (uint8_t)((hi < 0.75f * ctx->threshold
                             || hi < MIN_BIT_HIGH_TO_LOW * lo) ? 1u : 0u);
        bits[i] = (uint8_t)((early >= late) ? 1u : 0u);
    }

    df = 0u;
    for (size_t i = 0; i < 5u; i++) {
        df = (uint8_t)((df << 1u) | bits[i]);
    }

    frame_bits = mode_s_frame_bits_from_df(df);
    if (frame_bits == 0u || frame_bits > capture_bits) {
        ctx->rejected_frame_count++;
        ctx->quality_rejected_count++;
        return 0;
    }

    for (size_t i = 0; i < frame_bits; i++) {
        weak_bits += weak[i] ? 1 : 0;
    }

    if (weak_bits > (int)(frame_bits / MAX_WEAK_BIT_DIVISOR)) {
        ctx->rejected_frame_count++;
        ctx->quality_rejected_count++;
        return 0;
    }

    emit_frame_bits(ctx, bits, frame_bits);
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

void on_clean_frame(const uint8_t *frame, size_t frame_bits) {
    (void)adsb_protocol_handle_frame(frame, frame_bits);
}

static void on_frame_default(void *user, const uint8_t *frame, size_t frame_bits) {
    adsb_ctx_t *ctx = (adsb_ctx_t *)user;
    if (!ctx || !frame || (frame_bits != SHORT_FRAME_BITS && frame_bits != LONG_FRAME_BITS)) {
        return;
    }
    if (adsb_protocol_handle_frame(frame, frame_bits)) {
        ctx->valid_frame_count++;
    } else {
        ctx->rejected_frame_count++;
        ctx->crc_error_count++;
    }
}

static void emit_frame_from_clean_bits(adsb_ctx_t *ctx) {
    size_t frame_bits = (size_t)ctx->frame_bits_len;
    deliver_frame(ctx, ctx->frame_bits_buf, frame_bits);
}

static void on_demod_sample_cb(void *user, float sample) {
    if (!user) return;
    process_am_sample((adsb_ctx_t *)user, sample);
}

void init(adsb_ctx_t *ctx, int fs_demod) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->fs_demod = fs_demod;
    ctx->threshold = 1.6f;      // normalized pulse height over local average
    ctx->frame_bits_target = (int)LONG_FRAME_BITS;
    ctx->on_frame_cb = on_frame_default;
    ctx->on_frame_user = ctx;

    fprintf(output_diagnostics(), "[adsb] init fs_demod=%d frame_bits_target=%d\n",
            fs_demod, ctx->frame_bits_target);
}

void get_demod_config(demod_config_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->kind = DEMOD_KIND_AM;
    cfg->input_fs = 2000000;
    cfg->output_fs = 2000000;
    /* Preserve the pulse envelope; process_am_sample maintains its own baseline. */
    cfg->u.am.dc_alpha = 0.0f;
}

demod_output_t get_demod_output(adsb_ctx_t *ctx) {
    demod_output_t out;
    memset(&out, 0, sizeof(out));
    out.on_float = on_demod_sample_cb;
    out.user = ctx;
    return out;
}

void set_callbacks(adsb_ctx_t *ctx,
                   adsb_on_bit_cb_t on_bit_cb, void *on_bit_user,
                   adsb_on_frame_cb_t on_frame_cb, void *on_frame_user) {
    if (!ctx) return;
    ctx->on_bit_cb = on_bit_cb;
    ctx->on_bit_user = on_bit_user;
    ctx->on_frame_cb = on_frame_cb ? on_frame_cb : on_frame_default;
    ctx->on_frame_user = on_frame_user;
}

void set_frame_bits_target(adsb_ctx_t *ctx, int frame_bits_target) {
    if (!ctx) return;
    if (frame_bits_target < 1 || frame_bits_target > (int)sizeof(ctx->frame_bits_buf)) return;
    ctx->frame_bits_target = frame_bits_target;
    ctx->frame_bits_len = 0;
}

void feed_clean_bit(adsb_ctx_t *ctx, uint8_t bit) {
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
        emit_frame_from_clean_bits(ctx);
        ctx->frame_bits_len = 0;
    }
}

void feed_clean_bits(adsb_ctx_t *ctx, const uint8_t *bits, size_t nbits) {
    if (!ctx || !bits) return;
    for (size_t i = 0; i < nbits; i++) {
        feed_clean_bit(ctx, bits[i]);
    }
}

void process_am_sample(adsb_ctx_t *ctx, float sample) {
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
    hist_put(ctx, sample_index, norm);

    if (norm > ctx->threshold) {
        ctx->pulse_count++;
    }

    ctx->demod_sample_count++;

    needed_samples = (size_t)PREAMBLE_SAMPLES + (size_t)ctx->frame_bits_target * (size_t)SAMPLES_PER_BIT;
    if (needed_samples >= MAG_HISTORY_LEN) return;

    if (!ctx->capture_active
        && ctx->demod_sample_count >= MIN_DETECT_SAMPLES
        && sample_index + 1u >= PREAMBLE_SAMPLES) {
        uint64_t start_sample = sample_index + 1u - PREAMBLE_SAMPLES;
        if (start_sample >= ctx->next_search_sample && is_preamble(ctx, start_sample)) {
            ctx->capture_active = 1;
            ctx->capture_start_sample = start_sample;
            ctx->next_search_sample = start_sample + (uint64_t)needed_samples;
            ctx->detected_preamble_count++;
        }
    }

    if (ctx->capture_active) {
        uint64_t capture_end = ctx->capture_start_sample + (uint64_t)needed_samples;
        if (ctx->demod_sample_count >= capture_end) {
            (void)try_emit_captured_frame(ctx);
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

void flush(adsb_ctx_t *ctx) {
    if (!ctx) return;
    fprintf(output_diagnostics(),
            "[adsb] flush demod_samples=%llu clean_bits=%llu candidates=%llu valid=%llu preambles=%llu crc_errors=%llu non_adsb=%llu quality_rejected=%llu rejected=%llu\n",
            (unsigned long long)ctx->demod_sample_count,
            (unsigned long long)ctx->clean_bit_count,
            (unsigned long long)ctx->clean_frame_count,
            (unsigned long long)ctx->valid_frame_count,
            (unsigned long long)ctx->detected_preamble_count,
            (unsigned long long)ctx->crc_error_count,
            (unsigned long long)ctx->non_adsb_candidate_count,
            (unsigned long long)ctx->quality_rejected_count,
            (unsigned long long)ctx->rejected_frame_count);
}

int adsb_test_emit_examples(adsb_ctx_t *ctx) {
    static const uint8_t identification[14] = {
        0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3,
        0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98
    };
    static const unsigned int preamble_high[4] = { 0u, 2u, 7u, 9u };
    uint64_t frames_before;
    uint64_t preambles_before;
    uint64_t quality_before;
    uint64_t valid_before;
    int protocol_ok;
    int quality_gate_ok;

    if (!ctx) return 0;
    protocol_ok = adsb_protocol_emit_test_examples();
    frames_before = ctx->clean_frame_count;

    /* Establish a stable noise baseline before the synthetic burst. */
    for (size_t i = 0; i < MIN_DETECT_SAMPLES; i++) process_am_sample(ctx, 10.0f);
    for (size_t i = 0; i < PREAMBLE_SAMPLES; i++) {
        float amplitude = 10.0f;
        for (size_t p = 0; p < sizeof(preamble_high) / sizeof(preamble_high[0]); p++) {
            if (i == preamble_high[p]) amplitude = 50.0f;
        }
        process_am_sample(ctx, amplitude);
    }
    for (size_t i = 0; i < LONG_FRAME_BITS; i++) {
        uint8_t bit = (uint8_t)((identification[i / 8u] >> (7u - (i % 8u))) & 1u);
        process_am_sample(ctx, bit ? 50.0f : 10.0f);
        process_am_sample(ctx, bit ? 10.0f : 50.0f);
    }

    /* A preamble followed by low-contrast data must not reach the decoder. */
    for (size_t i = 0; i < 64u; i++) process_am_sample(ctx, 10.0f);
    preambles_before = ctx->detected_preamble_count;
    quality_before = ctx->quality_rejected_count;
    valid_before = ctx->valid_frame_count;
    for (size_t i = 0; i < PREAMBLE_SAMPLES; i++) {
        float amplitude = 10.0f;
        for (size_t p = 0; p < sizeof(preamble_high) / sizeof(preamble_high[0]); p++) {
            if (i == preamble_high[p]) amplitude = 50.0f;
        }
        process_am_sample(ctx, amplitude);
    }
    for (size_t i = 0; i < LONG_FRAME_BITS; i++) {
        uint8_t bit = (uint8_t)((identification[i / 8u] >> (7u - (i % 8u))) & 1u);
        process_am_sample(ctx, bit ? 12.0f : 10.0f);
        process_am_sample(ctx, bit ? 10.0f : 12.0f);
    }
    quality_gate_ok = ctx->detected_preamble_count == preambles_before + 1u
                      && ctx->quality_rejected_count == quality_before + 1u
                      && ctx->valid_frame_count == valid_before;

    fprintf(output_diagnostics(), "[ADSB] ppm_test_result=%s decoded=%llu expected=1\n",
            ctx->clean_frame_count - frames_before == 1u ? "PASS" : "FAIL",
            (unsigned long long)(ctx->clean_frame_count - frames_before));
    fprintf(output_diagnostics(), "[ADSB] quality_gate_test=%s\n",
            quality_gate_ok ? "PASS" : "FAIL");
    return protocol_ok && ctx->clean_frame_count - frames_before == 1u
           && quality_gate_ok;
}
