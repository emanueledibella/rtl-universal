#include "header/sstv_module.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "output.h"

#define SSTV_INPUT_FS 240000
#define SSTV_AUDIO_FS 48000
#define SSTV_VIS_BLOCK_MS 10.0

enum {
    VIS_WAIT_LEADER = 0,
    VIS_WAIT_BREAK,
    VIS_WAIT_SECOND_LEADER,
    VIS_WAIT_START,
    VIS_WAIT_DATA,
    VIS_READ_DATA,
    VIS_WAIT_STOP
};

static int equals_icase(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static const char *mode_name(sstv_mode_t mode) {
    switch (mode) {
    case SSTV_MODE_PD120:
        return "pd120";
    case SSTV_MODE_MARTIN_M1:
        return "martin-m1";
    case SSTV_MODE_AUTO:
    default:
        return "auto";
    }
}

static void emit_event(const char *event, sstv_mode_t mode, const char *path,
                       unsigned int line, unsigned int total) {
    output_format_t format = output_get_format();
    if (format == OUTPUT_FORMAT_QUIET) return;
    if (format == OUTPUT_FORMAT_JSON) {
        printf("{\"protocol\":\"sstv\",\"event\":\"%s\",\"mode\":\"%s\"",
               event, mode_name(mode));
        if (path) printf(",\"path\":\"%s\"", path);
        if (total > 0u) printf(",\"line\":%u,\"total_lines\":%u", line, total);
        printf("}\n");
    } else if (format == OUTPUT_FORMAT_CSV) {
        printf("sstv,%s,%s,%s,%u,%u\n", event, mode_name(mode), path ? path : "",
               line, total);
    } else if (strcmp(event, "progress") != 0 || line == 1u || line == total
               || line % 16u == 0u) {
        printf("[SSTV] event=%s mode=%s", event, mode_name(mode));
        if (path) printf(" file=%s", path);
        if (total > 0u) printf(" line=%u/%u", line, total);
        printf("\n");
    }
}

static unsigned int mode_width(sstv_mode_t mode) {
    return mode == SSTV_MODE_PD120 ? 640u : 320u;
}

static unsigned int mode_height(sstv_mode_t mode) {
    return mode == SSTV_MODE_PD120 ? 496u : 256u;
}

static unsigned int mode_scan_lines(sstv_mode_t mode) {
    return mode == SSTV_MODE_PD120 ? 248u : 256u;
}

static double mode_sync_ms(sstv_mode_t mode) {
    return mode == SSTV_MODE_PD120 ? 20.0 : 4.862;
}

static uint8_t clamp_byte(double value) {
    if (value <= 0.0) return 0u;
    if (value >= 255.0) return 255u;
    return (uint8_t)lrint(value);
}

static int ensure_directory(const char *path) {
    char partial[512];
    size_t length;
    if (!path || path[0] == '\0') return 0;
    length = strlen(path);
    if (length >= sizeof(partial)) return 0;
    memcpy(partial, path, length + 1u);
    for (size_t i = 1u; i <= length; i++) {
        if (partial[i] != '/' && partial[i] != '\0') continue;
        {
            char saved = partial[i];
            struct stat info;
            partial[i] = '\0';
            if (partial[0] != '\0' && stat(partial, &info) != 0) {
                if (mkdir(partial, 0755) != 0 && errno != EEXIST) return 0;
            } else if (partial[0] != '\0' && !S_ISDIR(info.st_mode)) {
                return 0;
            }
            partial[i] = saved;
        }
    }
    return 1;
}

static double tone_to_level(double frequency) {
    return fmin(255.0, fmax(0.0, (frequency - 1500.0) * (255.0 / 800.0)));
}

static void clear_line_channels(sstv_module_t *ctx) {
    memset(ctx->channel_sum, 0, sizeof(ctx->channel_sum));
    memset(ctx->channel_count, 0, sizeof(ctx->channel_count));
}

static void set_pixel_rgb(sstv_module_t *ctx, unsigned int x, unsigned int y,
                          double red, double green, double blue) {
    size_t position;
    if (x >= SSTV_MAX_WIDTH || y >= SSTV_MAX_HEIGHT) return;
    position = ((size_t)y * SSTV_MAX_WIDTH + x) * 3u;
    ctx->image[position] = clamp_byte(red);
    ctx->image[position + 1u] = clamp_byte(green);
    ctx->image[position + 2u] = clamp_byte(blue);
}

static double channel_level(const sstv_module_t *ctx, unsigned int channel,
                            unsigned int x) {
    if (ctx->channel_count[channel][x] == 0u) return 0.0;
    return ctx->channel_sum[channel][x] / ctx->channel_count[channel][x];
}

static int write_ppm(sstv_module_t *ctx) {
    char path[1024];
    char stamp[64];
    time_t now = time(NULL);
    struct tm local;
    FILE *file;
    unsigned int width = mode_width(ctx->active_mode);
    unsigned int height = mode_height(ctx->active_mode);

    if (!ensure_directory(ctx->save_dir)) {
        fprintf(stderr, "[SSTV] cannot create output directory %s\n", ctx->save_dir);
        return 0;
    }
    if (localtime_r(&now, &local) == NULL
        || strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local) == 0u) {
        strcpy(stamp, "unknown_time");
    }
    if (snprintf(path, sizeof(path), "%s/sstv_%s_%s.ppm", ctx->save_dir,
                 mode_name(ctx->active_mode), stamp) >= (int)sizeof(path)) {
        fprintf(stderr, "[SSTV] output path is too long\n");
        return 0;
    }
    file = fopen(path, "wb");
    if (!file) {
        fprintf(stderr, "[SSTV] cannot write %s: %s\n", path, strerror(errno));
        return 0;
    }
    fprintf(file, "P6\n%u %u\n255\n", width, height);
    for (unsigned int y = 0u; y < height; y++) {
        const uint8_t *row = ctx->image + (size_t)y * SSTV_MAX_WIDTH * 3u;
        if (fwrite(row, 3u, width, file) != width) {
            fclose(file);
            fprintf(stderr, "[SSTV] write error for %s\n", path);
            return 0;
        }
    }
    if (fclose(file) != 0) return 0;
    ctx->images_saved++;
    emit_event("image-saved", ctx->active_mode, path, 0u, 0u);
    return 1;
}

static void finish_scan_line(sstv_module_t *ctx) {
    unsigned int width = mode_width(ctx->active_mode);
    if (ctx->active_mode == SSTV_MODE_PD120) {
        unsigned int y0 = ctx->scan_line * 2u;
        unsigned int y1 = y0 + 1u;
        for (unsigned int x = 0u; x < width; x++) {
            double luma0 = channel_level(ctx, 0u, x);
            double v = channel_level(ctx, 1u, x) - 128.0;
            double u = channel_level(ctx, 2u, x) - 128.0;
            double luma1 = channel_level(ctx, 3u, x);
            set_pixel_rgb(ctx, x, y0, luma0 + 1.402 * v,
                          luma0 - 0.344136 * u - 0.714136 * v,
                          luma0 + 1.772 * u);
            set_pixel_rgb(ctx, x, y1, luma1 + 1.402 * v,
                          luma1 - 0.344136 * u - 0.714136 * v,
                          luma1 + 1.772 * u);
        }
    } else {
        for (unsigned int x = 0u; x < width; x++) {
            double green = channel_level(ctx, 0u, x);
            double blue = channel_level(ctx, 1u, x);
            double red = channel_level(ctx, 2u, x);
            set_pixel_rgb(ctx, x, ctx->scan_line, red, green, blue);
        }
    }
    ctx->scan_line++;
    ctx->lines_decoded++;
    emit_event("progress", ctx->active_mode, NULL, ctx->scan_line,
               mode_scan_lines(ctx->active_mode));
    if (ctx->scan_line >= mode_scan_lines(ctx->active_mode)) {
        (void)write_ppm(ctx);
        ctx->scan_line = 0u;
        memset(ctx->image, 0, sizeof(ctx->image));
        if (ctx->requested_mode == SSTV_MODE_AUTO) ctx->active_mode = SSTV_MODE_AUTO;
    }
}

static void accumulate_channel_sample(sstv_module_t *ctx, unsigned int channel,
                                      double field_start_ms, double field_ms,
                                      double line_ms) {
    double relative_ms = line_ms - field_start_ms;
    unsigned int width = mode_width(ctx->active_mode);
    unsigned int x;
    if (relative_ms < 0.0 || relative_ms >= field_ms || ctx->tone_hz < 1300.0
        || ctx->tone_hz > 2500.0) return;
    x = (unsigned int)(relative_ms * width / field_ms);
    if (x >= width) x = width - 1u;
    ctx->channel_sum[channel][x] += tone_to_level(ctx->tone_hz);
    ctx->channel_count[channel][x]++;
}

static void process_active_line(sstv_module_t *ctx) {
    double line_ms = 1000.0 * (double)ctx->line_sample / ctx->sample_rate;
    double end_ms;
    if (ctx->active_mode == SSTV_MODE_PD120) {
        const double start = 22.08;
        const double field = 121.6;
        for (unsigned int channel = 0u; channel < 4u; channel++) {
            accumulate_channel_sample(ctx, channel, start + field * channel,
                                      field, line_ms);
        }
        end_ms = start + field * 4.0;
    } else {
        const double sync = 4.862;
        const double porch = 0.572;
        const double field = 146.432;
        double start = sync + porch;
        accumulate_channel_sample(ctx, 0u, start, field, line_ms);
        start += field + porch;
        accumulate_channel_sample(ctx, 1u, start, field, line_ms);
        start += field + porch;
        accumulate_channel_sample(ctx, 2u, start, field, line_ms);
        end_ms = start + field;
    }
    ctx->line_sample++;
    if (line_ms >= end_ms) {
        finish_scan_line(ctx);
        ctx->line_active = 0;
        ctx->sync_run_samples = 0u;
    }
}

static int classify_vis_tone(double frequency) {
    static const int tones[] = { 1100, 1200, 1300, 1900 };
    int best = 0;
    double error = 1e9;
    if (!isfinite(frequency)) return 0;
    for (size_t i = 0u; i < sizeof(tones) / sizeof(tones[0]); i++) {
        double current = fabs(frequency - tones[i]);
        if (current < error) {
            error = current;
            best = tones[i];
        }
    }
    return error <= 90.0 ? best : 0;
}

static void activate_vis_mode(sstv_module_t *ctx, uint8_t vis) {
    sstv_mode_t detected = SSTV_MODE_AUTO;
    if (vis == 95u) detected = SSTV_MODE_PD120;
    else if (vis == 44u) detected = SSTV_MODE_MARTIN_M1;
    if (detected == SSTV_MODE_AUTO) {
        ctx->rejected_headers++;
        emit_event("unsupported-vis", SSTV_MODE_AUTO, NULL, vis, 0u);
    } else {
        ctx->active_mode = detected;
        ctx->vis_headers++;
        ctx->scan_line = 0u;
        ctx->sync_run_samples = 0u;
        ctx->line_active = 0;
        memset(ctx->image, 0, sizeof(ctx->image));
        emit_event("vis-detected", detected, NULL, 0u, 0u);
    }
    ctx->vis_state = VIS_WAIT_LEADER;
    ctx->vis_tone_blocks = 0u;
}

static void process_vis_block(sstv_module_t *ctx, int tone) {
    switch (ctx->vis_state) {
    case VIS_WAIT_LEADER:
        ctx->vis_tone_blocks = tone == 1900 ? ctx->vis_tone_blocks + 1u : 0u;
        if (ctx->vis_tone_blocks >= 25u) {
            ctx->vis_state = VIS_WAIT_BREAK;
            ctx->vis_tone_blocks = 0u;
        }
        break;
    case VIS_WAIT_BREAK:
        if (tone == 1200) {
            ctx->vis_state = VIS_WAIT_SECOND_LEADER;
            ctx->vis_tone_blocks = 0u;
        } else if (tone != 1900) {
            ctx->vis_state = VIS_WAIT_LEADER;
        }
        break;
    case VIS_WAIT_SECOND_LEADER:
        ctx->vis_tone_blocks = tone == 1900 ? ctx->vis_tone_blocks + 1u : 0u;
        if (ctx->vis_tone_blocks >= 25u) {
            ctx->vis_state = VIS_WAIT_START;
            ctx->vis_tone_blocks = 0u;
        }
        break;
    case VIS_WAIT_START:
        if (tone == 1200) {
            ctx->vis_tone_blocks++;
            if (ctx->vis_tone_blocks >= 2u) ctx->vis_state = VIS_WAIT_DATA;
        } else if (tone != 1900) {
            ctx->vis_state = VIS_WAIT_LEADER;
            ctx->vis_tone_blocks = 0u;
        }
        break;
    case VIS_WAIT_DATA:
        if (tone == 1100 || tone == 1300) {
            ctx->vis_state = VIS_READ_DATA;
            ctx->vis_bit_blocks = 0u;
            ctx->vis_ones = 0u;
            ctx->vis_zeros = 0u;
            ctx->vis_bit_index = 0u;
            ctx->vis_value = 0u;
            ctx->vis_parity = 0u;
        } else {
            break;
        }
        /* The current block is the first data-bit block. */
        /* fall through */
    case VIS_READ_DATA:
        if (tone != 1100 && tone != 1300) {
            ctx->rejected_headers++;
            ctx->vis_state = VIS_WAIT_LEADER;
            break;
        }
        if (tone == 1100) ctx->vis_ones++; else ctx->vis_zeros++;
        ctx->vis_bit_blocks++;
        if (ctx->vis_bit_blocks >= 3u) {
            uint8_t bit = ctx->vis_ones > ctx->vis_zeros ? 1u : 0u;
            if (ctx->vis_bit_index < 7u) {
                ctx->vis_value |= (uint8_t)(bit << ctx->vis_bit_index);
                ctx->vis_parity ^= bit;
            } else if (bit != ctx->vis_parity) {
                ctx->rejected_headers++;
                ctx->vis_state = VIS_WAIT_LEADER;
                break;
            }
            ctx->vis_bit_index++;
            ctx->vis_bit_blocks = 0u;
            ctx->vis_ones = 0u;
            ctx->vis_zeros = 0u;
            if (ctx->vis_bit_index == 8u) ctx->vis_state = VIS_WAIT_STOP;
        }
        break;
    case VIS_WAIT_STOP:
        if (tone == 1200) activate_vis_mode(ctx, ctx->vis_value);
        else if (tone != 1100 && tone != 1300) {
            ctx->rejected_headers++;
            ctx->vis_state = VIS_WAIT_LEADER;
        }
        break;
    default:
        ctx->vis_state = VIS_WAIT_LEADER;
        break;
    }
}

static void sstv_on_sample(void *user, float sample) {
    sstv_module_t *ctx = (sstv_module_t *)user;
    int rising;
    if (!ctx) return;
    ctx->audio_samples++;
    ctx->samples_since_crossing++;
    rising = ctx->previous_sample <= 0.0f && sample > 0.0f;
    if (rising && ctx->samples_since_crossing >= 10u
        && ctx->samples_since_crossing <= 96u) {
        double measured = (double)ctx->sample_rate / ctx->samples_since_crossing;
        ctx->tone_hz = measured;
        ctx->samples_since_crossing = 0u;
    } else if (rising) {
        ctx->samples_since_crossing = 0u;
    }
    ctx->previous_sample = sample;

    if (ctx->active_mode == SSTV_MODE_AUTO) {
        if (ctx->tone_hz > 0.0) {
            ctx->tone_sum += ctx->tone_hz;
            ctx->tone_samples++;
        }
        ctx->tone_block_samples++;
        if (ctx->tone_block_samples >= (unsigned int)(ctx->sample_rate * SSTV_VIS_BLOCK_MS / 1000.0)) {
            /* Use the most recent full-cycle estimate. Averaging a whole block
             * smears the exact 30 ms VIS transitions into adjacent bits. */
            process_vis_block(ctx, classify_vis_tone(ctx->tone_hz));
            ctx->tone_sum = 0.0;
            ctx->tone_samples = 0u;
            ctx->tone_block_samples = 0u;
        }
        return;
    }

    if (ctx->line_active) {
        process_active_line(ctx);
        return;
    }
    if (fabs(ctx->tone_hz - 1200.0) <= 90.0) ctx->sync_run_samples++;
    else ctx->sync_run_samples = 0u;
    if (ctx->sync_run_samples
        >= (uint64_t)llround(ctx->sample_rate * mode_sync_ms(ctx->active_mode) / 1000.0 * 0.65)) {
        clear_line_channels(ctx);
        ctx->line_active = 1;
        ctx->line_sample = ctx->sync_run_samples;
    }
}

void sstv_module_reset(sstv_module_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->requested_mode = SSTV_MODE_AUTO;
    ctx->active_mode = SSTV_MODE_AUTO;
    ctx->sample_rate = SSTV_AUDIO_FS;
    strcpy(ctx->save_dir, ".");
}

int sstv_module_set_mode(sstv_module_t *ctx, const char *name) {
    sstv_mode_t mode;
    if (!ctx) return 0;
    if (!name || equals_icase(name, "auto")) mode = SSTV_MODE_AUTO;
    else if (equals_icase(name, "pd120") || equals_icase(name, "pd-120")) {
        mode = SSTV_MODE_PD120;
    } else if (equals_icase(name, "martin-m1") || equals_icase(name, "martin1")
               || equals_icase(name, "m1")) {
        mode = SSTV_MODE_MARTIN_M1;
    } else {
        return 0;
    }
    ctx->requested_mode = mode;
    ctx->active_mode = mode;
    return 1;
}

int sstv_module_set_save_dir(sstv_module_t *ctx, const char *path) {
    size_t len;
    if (!ctx || !path || path[0] == '\0') return 0;
    len = strlen(path);
    if (len >= sizeof(ctx->save_dir)) return 0;
    memcpy(ctx->save_dir, path, len + 1u);
    return 1;
}

int sstv_module_init(sstv_module_t *ctx, const demod_config_t *cfg) {
    sstv_mode_t requested;
    char save_dir[sizeof(ctx->save_dir)];
    if (!ctx || !cfg) return 0;
    requested = ctx->requested_mode;
    memcpy(save_dir, ctx->save_dir, sizeof(save_dir));
    sstv_module_reset(ctx);
    ctx->requested_mode = requested;
    ctx->active_mode = requested;
    memcpy(ctx->save_dir, save_dir, sizeof(ctx->save_dir));
    ctx->sample_rate = cfg->output_fs;
    return 1;
}

void sstv_module_get_demod_config(sstv_module_t *ctx, demod_config_t *cfg) {
    (void)ctx;
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->kind = DEMOD_KIND_FM;
    cfg->input_fs = SSTV_INPUT_FS;
    cfg->output_fs = SSTV_AUDIO_FS;
    cfg->u.fm.dc_alpha = 0.0005f;
}

demod_output_t sstv_module_get_demod_output(sstv_module_t *ctx) {
    demod_output_t output;
    memset(&output, 0, sizeof(output));
    output.on_float = sstv_on_sample;
    output.user = ctx;
    return output;
}

void sstv_module_flush(sstv_module_t *ctx) {
    if (!ctx) return;
    fprintf(output_diagnostics(),
            "[SSTV] samples=%llu vis=%llu lines=%llu images=%llu rejected_headers=%llu\n",
            (unsigned long long)ctx->audio_samples,
            (unsigned long long)ctx->vis_headers,
            (unsigned long long)ctx->lines_decoded,
            (unsigned long long)ctx->images_saved,
            (unsigned long long)ctx->rejected_headers);
}

static void feed_tone(sstv_module_t *ctx, double frequency, double milliseconds,
                      double *phase) {
    uint64_t samples = (uint64_t)llround(ctx->sample_rate * milliseconds / 1000.0);
    double step = 2.0 * M_PI * frequency / ctx->sample_rate;
    for (uint64_t i = 0u; i < samples; i++) {
        sstv_on_sample(ctx, (float)(0.8 * sin(*phase)));
        *phase += step;
        if (*phase >= 2.0 * M_PI) *phase -= 2.0 * M_PI;
    }
}

int sstv_module_run_test(sstv_module_t *ctx) {
    double phase = 0.0;
    uint8_t vis = 95u;
    uint8_t parity = 0u;
    uint64_t before;
    uint64_t pd_lines;
    uint64_t martin_before;
    uint64_t martin_lines;
    if (!ctx) return 0;
    ctx->requested_mode = SSTV_MODE_AUTO;
    ctx->active_mode = SSTV_MODE_AUTO;
    feed_tone(ctx, 1900.0, 320.0, &phase);
    feed_tone(ctx, 1200.0, 20.0, &phase);
    feed_tone(ctx, 1900.0, 320.0, &phase);
    feed_tone(ctx, 1200.0, 30.0, &phase);
    for (unsigned int bit = 0u; bit < 7u; bit++) {
        uint8_t value = (uint8_t)((vis >> bit) & 1u);
        parity ^= value;
        feed_tone(ctx, value ? 1100.0 : 1300.0, 30.0, &phase);
    }
    feed_tone(ctx, parity ? 1100.0 : 1300.0, 30.0, &phase);
    feed_tone(ctx, 1200.0, 30.0, &phase);
    before = ctx->lines_decoded;
    feed_tone(ctx, 1200.0, 20.0, &phase);
    feed_tone(ctx, 1500.0, 2.08, &phase);
    feed_tone(ctx, 1800.0, 121.6, &phase);
    feed_tone(ctx, 1900.0, 121.6, &phase);
    feed_tone(ctx, 1700.0, 121.6, &phase);
    feed_tone(ctx, 2100.0, 121.6, &phase);
    feed_tone(ctx, 1900.0, 5.0, &phase);
    pd_lines = ctx->lines_decoded - before;

    ctx->requested_mode = SSTV_MODE_MARTIN_M1;
    ctx->active_mode = SSTV_MODE_MARTIN_M1;
    ctx->scan_line = 0u;
    ctx->sync_run_samples = 0u;
    ctx->line_active = 0;
    martin_before = ctx->lines_decoded;
    feed_tone(ctx, 1200.0, 4.862, &phase);
    feed_tone(ctx, 1500.0, 0.572, &phase);
    feed_tone(ctx, 1800.0, 146.432, &phase);
    feed_tone(ctx, 1500.0, 0.572, &phase);
    feed_tone(ctx, 1700.0, 146.432, &phase);
    feed_tone(ctx, 1500.0, 0.572, &phase);
    feed_tone(ctx, 2100.0, 146.432, &phase);
    feed_tone(ctx, 1900.0, 5.0, &phase);
    martin_lines = ctx->lines_decoded - martin_before;
    fprintf(output_diagnostics(),
            "[SSTV] test_result=%s pd120_lines=%llu martin_m1_lines=%llu expected=1+1\n",
            pd_lines == 1u && martin_lines == 1u ? "PASS" : "FAIL",
            (unsigned long long)pd_lines, (unsigned long long)martin_lines);
    return pd_lines == 1u && martin_lines == 1u;
}
