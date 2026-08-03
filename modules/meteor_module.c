#include "header/meteor_module.h"

#include <errno.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "output.h"

extern char **environ;

static volatile sig_atomic_t meteor_child_pid = -1;

static int copy_option(char *destination, size_t size, const char *source) {
    size_t len;
    if (!destination || size == 0u || !source || source[0] == '\0') return 0;
    len = strlen(source);
    if (len >= size) return 0;
    memcpy(destination, source, len + 1u);
    return 1;
}

void meteor_module_reset(meteor_module_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    strcpy(ctx->executable, "satdump");
    strcpy(ctx->pipeline, "meteor_m2-x_lrpt");
    strcpy(ctx->output_dir, "meteor-output");
    strcpy(ctx->satellite, "Auto");
    ctx->sample_rate = 1000000u;
}

int meteor_module_set_executable(meteor_module_t *ctx, const char *path) {
    return ctx && copy_option(ctx->executable, sizeof(ctx->executable), path);
}

int meteor_module_set_pipeline(meteor_module_t *ctx, const char *name) {
    const char *pipeline = NULL;
    if (!ctx || !name) return 0;
    if (strcmp(name, "m2") == 0 || strcmp(name, "72k-qpsk") == 0
        || strcmp(name, "meteor_m2_lrpt") == 0) {
        pipeline = "meteor_m2_lrpt";
    } else if (strcmp(name, "m2-x") == 0 || strcmp(name, "72k") == 0
               || strcmp(name, "72k-oqpsk") == 0
               || strcmp(name, "meteor_m2-x_lrpt") == 0) {
        pipeline = "meteor_m2-x_lrpt";
    } else if (strcmp(name, "m2-x-80k") == 0 || strcmp(name, "80k") == 0
               || strcmp(name, "meteor_m2-x_lrpt_80k") == 0) {
        pipeline = "meteor_m2-x_lrpt_80k";
    }
    return pipeline && copy_option(ctx->pipeline, sizeof(ctx->pipeline), pipeline);
}

int meteor_module_set_output_dir(meteor_module_t *ctx, const char *path) {
    return ctx && copy_option(ctx->output_dir, sizeof(ctx->output_dir), path);
}

int meteor_module_set_satellite(meteor_module_t *ctx, const char *name) {
    if (!ctx || !name) return 0;
    if (strcmp(name, "Auto") != 0 && strcmp(name, "auto") != 0
        && strcmp(name, "M2") != 0 && strcmp(name, "M2-2") != 0
        && strcmp(name, "M2-3") != 0 && strcmp(name, "M2-4") != 0) {
        return 0;
    }
    return copy_option(ctx->satellite, sizeof(ctx->satellite),
                       strcmp(name, "auto") == 0 ? "Auto" : name);
}

void meteor_module_set_timeout(meteor_module_t *ctx, unsigned int seconds) {
    if (ctx) ctx->timeout_seconds = seconds;
}

static void log_command(char *const argv[]) {
    FILE *diagnostics = output_diagnostics();
    fputs("[METEOR] backend-command:", diagnostics);
    for (size_t i = 0u; argv[i]; i++) fprintf(diagnostics, " %s", argv[i]);
    fputc('\n', diagnostics);
}

static int spawn_and_wait(meteor_module_t *ctx, char *const argv[]) {
    pid_t pid;
    int status;
    int result;
    if (!ctx || !argv || !argv[0]) return 0;
    log_command(argv);
    result = posix_spawnp(&pid, ctx->executable, NULL, NULL, argv, environ);
    if (result != 0) {
        if (result == ENOENT) {
            fprintf(stderr,
                    "[METEOR] SatDump non trovato. Installa SatDump oppure usa --satdump /percorso/satdump.\n");
        } else {
            fprintf(stderr, "[METEOR] cannot start %s: %s\n", ctx->executable,
                    strerror(result));
        }
        return 0;
    }
    meteor_child_pid = (sig_atomic_t)pid;
    do {
        result = waitpid(pid, &status, 0) < 0 ? errno : 0;
    } while (result == EINTR);
    meteor_child_pid = -1;
    if (result != 0) {
        fprintf(stderr, "[METEOR] waitpid failed: %s\n", strerror(result));
        return 0;
    }
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) return 1;
    if (WIFEXITED(status)) {
        fprintf(stderr, "[METEOR] SatDump exited with status %d\n", WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
        fprintf(stderr, "[METEOR] SatDump stopped by signal %d\n", WTERMSIG(status));
    }
    return 0;
}

int meteor_module_run_live(meteor_module_t *ctx, double frequency_mhz,
                           const char *device_id, int manual_gain, int gain_db,
                           int have_ppm, int ppm) {
    char sample_rate[32];
    char frequency[32];
    char gain[32];
    char correction[32];
    char timeout[32];
    char *argv[32];
    size_t argc = 0u;
    if (!ctx || frequency_mhz <= 0.0) return 0;
    snprintf(sample_rate, sizeof(sample_rate), "%u", ctx->sample_rate);
    snprintf(frequency, sizeof(frequency), "%.0f", frequency_mhz * 1e6);
    snprintf(gain, sizeof(gain), "%d", gain_db);
    snprintf(correction, sizeof(correction), "%d", ppm);
    snprintf(timeout, sizeof(timeout), "%u", ctx->timeout_seconds);

    argv[argc++] = ctx->executable;
    argv[argc++] = "legacy";
    argv[argc++] = "live";
    argv[argc++] = ctx->pipeline;
    argv[argc++] = ctx->output_dir;
    argv[argc++] = "--source";
    argv[argc++] = "rtlsdr";
    if (device_id && device_id[0]) {
        argv[argc++] = "--source_id";
        argv[argc++] = (char *)device_id;
    }
    argv[argc++] = "--samplerate";
    argv[argc++] = sample_rate;
    argv[argc++] = "--frequency";
    argv[argc++] = frequency;
    if (manual_gain) {
        argv[argc++] = "--gain";
        argv[argc++] = gain;
    } else {
        argv[argc++] = "--agc";
    }
    if (have_ppm) {
        argv[argc++] = "--ppm_correction";
        argv[argc++] = correction;
    }
    if (ctx->timeout_seconds > 0u) {
        argv[argc++] = "--timeout";
        argv[argc++] = timeout;
    }
    argv[argc++] = "--satellite_number";
    argv[argc++] = ctx->satellite;
    argv[argc++] = "--fill_missing";
    argv[argc] = NULL;
    return spawn_and_wait(ctx, argv);
}

int meteor_module_run_offline(meteor_module_t *ctx, const char *input_path) {
    char sample_rate[32];
    char *argv[24];
    size_t argc = 0u;
    if (!ctx || !input_path || input_path[0] == '\0') return 0;
    snprintf(sample_rate, sizeof(sample_rate), "%u", ctx->sample_rate);
    argv[argc++] = ctx->executable;
    argv[argc++] = "pipeline";
    argv[argc++] = ctx->pipeline;
    argv[argc++] = "baseband";
    argv[argc++] = (char *)input_path;
    argv[argc++] = ctx->output_dir;
    argv[argc++] = "--samplerate";
    argv[argc++] = sample_rate;
    argv[argc++] = "--baseband_format";
    argv[argc++] = "cu8";
    argv[argc++] = "--satellite_number";
    argv[argc++] = ctx->satellite;
    argv[argc++] = "--fill_missing";
    argv[argc] = NULL;
    return spawn_and_wait(ctx, argv);
}

int meteor_module_run_test(meteor_module_t *ctx) {
    int ok = ctx && ctx->sample_rate == 1000000u
             && (strcmp(ctx->pipeline, "meteor_m2_lrpt") == 0
                 || strcmp(ctx->pipeline, "meteor_m2-x_lrpt") == 0
                 || strcmp(ctx->pipeline, "meteor_m2-x_lrpt_80k") == 0)
             && ctx->output_dir[0] != '\0';
    fprintf(output_diagnostics(),
            "[METEOR] test_result=%s backend=satdump pipeline=%s samplerate=%u output=%s\n",
            ok ? "PASS" : "FAIL", ctx ? ctx->pipeline : "-",
            ctx ? ctx->sample_rate : 0u, ctx ? ctx->output_dir : "-");
    return ok;
}

void meteor_module_stop(void) {
    pid_t pid = (pid_t)meteor_child_pid;
    if (pid > 0) (void)kill(pid, SIGINT);
}
