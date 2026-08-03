#include "header/gmsk_demod.h"

#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TWO_PI_F 6.28318530717958647692f

static void destroy_channel(gmsk_demod_channel_t *channel) {
    if (!channel) return;
    if (channel->mixer) nco_crcf_destroy(channel->mixer);
    if (channel->decimator) firdecim_crcf_destroy(channel->decimator);
    if (channel->timing) symsync_crcf_destroy(channel->timing);
    if (channel->demod) gmskdem_destroy(channel->demod);
    free(channel->decim_buf);
    memset(channel, 0, sizeof(*channel));
}

int gmsk_demod_init(gmsk_demod_ctx_t *ctx, const gmsk_demod_config_t *cfg,
                    gmsk_demod_output_cb_t out_cb,
                    gmsk_demod_channel_output_cb_t out_channel_cb,
                    void *out_user) {
    float *coefficients = NULL;
    unsigned int filter_len;
    float cutoff;

    if (!ctx || !cfg || (!out_cb && !out_channel_cb)) return 0;
    if (cfg->input_fs <= 0 || cfg->output_fs <= 0 || cfg->symbol_rate == 0u) return 0;
    if (cfg->input_fs % cfg->output_fs != 0) return 0;
    if (cfg->output_fs % (int)cfg->symbol_rate != 0) return 0;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    if (ctx->cfg.channel_count == 0u) ctx->cfg.channel_count = 1u;
    if (ctx->cfg.channel_count > GMSK_DEMOD_MAX_CHANNELS) return 0;
    ctx->out_cb = out_cb;
    ctx->out_channel_cb = out_channel_cb;
    ctx->out_user = out_user;
    ctx->decim = (unsigned int)(cfg->input_fs / cfg->output_fs);
    if (ctx->decim < 2u || cfg->output_fs / (int)cfg->symbol_rate < 2) return 0;

    /* 18 kHz low-pass isolates either 25 kHz AIS channel before decimation. */
    filter_len = 8u * ctx->decim + 1u;
    cutoff = 18000.0f / (float)cfg->input_fs;
    coefficients = (float *)calloc(filter_len, sizeof(*coefficients));
    if (!coefficients) return 0;
    if (liquid_firdes_kaiser(filter_len, cutoff, 70.0f, 0.0f, coefficients) != LIQUID_OK) {
        free(coefficients);
        return 0;
    }

    for (unsigned int i = 0; i < ctx->cfg.channel_count; i++) {
        gmsk_demod_channel_t *channel = &ctx->channel[i];
        unsigned int input_k = (unsigned int)(cfg->output_fs / (int)cfg->symbol_rate);
        channel->mixer = nco_crcf_create(LIQUID_NCO);
        channel->decimator = firdecim_crcf_create(ctx->decim, coefficients, filter_len);
        channel->timing = symsync_crcf_create_kaiser(input_k, cfg->m, cfg->bt, 32u);
        channel->demod = gmskdem_create(2u, cfg->m, cfg->bt);
        channel->decim_buf = (liquid_float_complex *)calloc(
            ctx->decim, sizeof(*channel->decim_buf));
        if (!channel->mixer || !channel->decimator || !channel->timing
            || !channel->demod || !channel->decim_buf) {
            free(coefficients);
            gmsk_demod_flush(ctx);
            return 0;
        }
        (void)nco_crcf_set_frequency(
            channel->mixer,
            TWO_PI_F * cfg->channel_offset_hz[i] / (float)cfg->input_fs);
        (void)symsync_crcf_set_output_rate(channel->timing, 2u);
        (void)symsync_crcf_set_lf_bw(channel->timing, 0.01f);
    }
    free(coefficients);
    return 1;
}

static void emit_timed_sample(gmsk_demod_ctx_t *ctx, unsigned int index,
                              liquid_float_complex sample) {
    gmsk_demod_channel_t *channel = &ctx->channel[index];
    unsigned int symbol = 0u;
    channel->sym_buf[channel->sym_idx++] = sample;
    if (channel->sym_idx < 2u) return;
    (void)gmskdem_demodulate(channel->demod, channel->sym_buf, &symbol);
    if (ctx->out_channel_cb) {
        ctx->out_channel_cb(ctx->out_user, index, (uint8_t)(symbol & 1u));
    } else if (ctx->out_cb) {
        ctx->out_cb(ctx->out_user, (uint8_t)(symbol & 1u));
    }
    channel->sym_idx = 0u;
}

void gmsk_demod_process_raw_iq_u8(gmsk_demod_ctx_t *ctx,
                                   const unsigned char *buf, uint32_t len) {
    if (!ctx || !buf) return;
    for (uint32_t p = 0; p + 1u < len; p += 2u) {
        liquid_float_complex input = ((float)buf[p] - 127.5f) / 127.5f
                                     + _Complex_I
                                           * (((float)buf[p + 1u] - 127.5f) / 127.5f);
        for (unsigned int i = 0; i < ctx->cfg.channel_count; i++) {
            gmsk_demod_channel_t *channel = &ctx->channel[i];
            liquid_float_complex mixed;
            liquid_float_complex decimated;
            liquid_float_complex synchronized[8];
            unsigned int synchronized_count = 0u;

            (void)nco_crcf_mix_down(channel->mixer, input, &mixed);
            (void)nco_crcf_step(channel->mixer);
            channel->decim_buf[channel->decim_count++] = mixed;
            if (channel->decim_count < ctx->decim) continue;
            channel->decim_count = 0u;
            (void)firdecim_crcf_execute(channel->decimator,
                                        channel->decim_buf, &decimated);
            (void)symsync_crcf_execute(channel->timing, &decimated, 1u,
                                       synchronized, &synchronized_count);
            for (unsigned int j = 0; j < synchronized_count; j++) {
                emit_timed_sample(ctx, i, synchronized[j]);
            }
        }
    }
}

void gmsk_demod_flush(gmsk_demod_ctx_t *ctx) {
    if (!ctx) return;
    for (unsigned int i = 0; i < GMSK_DEMOD_MAX_CHANNELS; i++) {
        destroy_channel(&ctx->channel[i]);
    }
    memset(ctx, 0, sizeof(*ctx));
}

typedef struct {
    uint8_t bits[GMSK_DEMOD_MAX_CHANNELS][1024];
    size_t count[GMSK_DEMOD_MAX_CHANNELS];
} gmsk_test_capture_t;

static void gmsk_test_capture(void *user, unsigned int channel, uint8_t bit) {
    gmsk_test_capture_t *capture = (gmsk_test_capture_t *)user;
    if (!capture || channel >= GMSK_DEMOD_MAX_CHANNELS) return;
    if (capture->count[channel] < sizeof(capture->bits[channel])) {
        capture->bits[channel][capture->count[channel]++] = bit & 1u;
    }
}

static int best_sequence_match(const uint8_t *captured, size_t captured_len,
                               const uint8_t *expected, size_t expected_start,
                               size_t compare_len) {
    int best = 0;
    if (!captured || !expected || captured_len < compare_len) return 0;
    for (size_t offset = 0; offset + compare_len <= captured_len; offset++) {
        int same = 0;
        int inverted = 0;
        for (size_t i = 0; i < compare_len; i++) {
            if (captured[offset + i] == expected[expected_start + i]) same++;
            else inverted++;
        }
        if (same > best) best = same;
        if (inverted > best) best = inverted;
    }
    return best;
}

int gmsk_demod_self_test(void) {
    enum { TEST_SYMBOLS = 500, TEST_K = 250, COMPARE_SYMBOLS = 160 };
    gmsk_demod_config_t cfg;
    gmsk_demod_ctx_t demod;
    gmsk_test_capture_t capture;
    gmskmod mod_a = NULL;
    gmskmod mod_b = NULL;
    liquid_float_complex samples_a[TEST_K];
    liquid_float_complex samples_b[TEST_K];
    unsigned char raw[TEST_K * 2];
    uint8_t expected[2][TEST_SYMBOLS];
    uint16_t lfsr_a = 0x1D3u;
    uint16_t lfsr_b = 0x16Fu;
    float phase_a = 0.0f;
    float phase_b = 0.0f;
    const float step_a = TWO_PI_F * -25000.0f / 2400000.0f;
    const float step_b = TWO_PI_F * 25000.0f / 2400000.0f;
    int result = 0;

    memset(&cfg, 0, sizeof(cfg));
    memset(&demod, 0, sizeof(demod));
    memset(&capture, 0, sizeof(capture));
    cfg.input_fs = 2400000;
    cfg.output_fs = 96000;
    cfg.symbol_rate = 9600u;
    cfg.m = 3u;
    cfg.bt = 0.4f;
    cfg.channel_count = 2u;
    cfg.channel_offset_hz[0] = -25000.0f;
    cfg.channel_offset_hz[1] = 25000.0f;
    mod_a = gmskmod_create(TEST_K, cfg.m, cfg.bt);
    mod_b = gmskmod_create(TEST_K, cfg.m, cfg.bt);
    if (!mod_a || !mod_b
        || !gmsk_demod_init(&demod, &cfg, NULL, gmsk_test_capture, &capture)) {
        goto cleanup;
    }
    for (size_t symbol = 0; symbol < TEST_SYMBOLS; symbol++) {
        lfsr_a = (uint16_t)((lfsr_a >> 1u)
                            ^ (uint16_t)(-(int)(lfsr_a & 1u) & 0x120u));
        lfsr_b = (uint16_t)((lfsr_b >> 1u)
                            ^ (uint16_t)(-(int)(lfsr_b & 1u) & 0x17Cu));
        expected[0][symbol] = (uint8_t)(lfsr_a & 1u);
        expected[1][symbol] = (uint8_t)(lfsr_b & 1u);
        (void)gmskmod_modulate(mod_a, expected[0][symbol], samples_a);
        (void)gmskmod_modulate(mod_b, expected[1][symbol], samples_b);
        for (size_t i = 0; i < TEST_K; i++) {
            liquid_float_complex carrier_a = cosf(phase_a) + _Complex_I * sinf(phase_a);
            liquid_float_complex carrier_b = cosf(phase_b) + _Complex_I * sinf(phase_b);
            liquid_float_complex sample = 0.38f * samples_a[i] * carrier_a
                                          + 0.38f * samples_b[i] * carrier_b;
            float scaled_i = 127.5f + 115.0f * crealf(sample);
            float scaled_q = 127.5f + 115.0f * cimagf(sample);
            if (scaled_i < 0.0f) scaled_i = 0.0f;
            if (scaled_i > 255.0f) scaled_i = 255.0f;
            if (scaled_q < 0.0f) scaled_q = 0.0f;
            if (scaled_q > 255.0f) scaled_q = 255.0f;
            raw[i * 2u] = (unsigned char)lrintf(scaled_i);
            raw[i * 2u + 1u] = (unsigned char)lrintf(scaled_q);
            phase_a += step_a;
            phase_b += step_b;
            if (phase_a < -TWO_PI_F) phase_a += TWO_PI_F;
            if (phase_b > TWO_PI_F) phase_b -= TWO_PI_F;
        }
        gmsk_demod_process_raw_iq_u8(&demod, raw, sizeof(raw));
    }
    result = best_sequence_match(capture.bits[0], capture.count[0], expected[0],
                                 80u, COMPARE_SYMBOLS) >= 150
             && best_sequence_match(capture.bits[1], capture.count[1], expected[1],
                                    80u, COMPARE_SYMBOLS) >= 150;

cleanup:
    if (mod_a) gmskmod_destroy(mod_a);
    if (mod_b) gmskmod_destroy(mod_b);
    gmsk_demod_flush(&demod);
    return result;
}
