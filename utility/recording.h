#pragma once

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RECORDING_PATH_MAX 4096

typedef enum {
    RECORDING_FORMAT_NONE = 0,
    RECORDING_AUDIO_WAV_S16,
    RECORDING_AUDIO_WAV_F32,
    RECORDING_AUDIO_S16LE,
    RECORDING_AUDIO_F32LE,
    RECORDING_IQ_CU8,
    RECORDING_IQ_CS16LE,
    RECORDING_IQ_CF32LE,
    RECORDING_IQ_WAV_S16,
    RECORDING_IQ_SIGMF_CU8,
} recording_format_t;

typedef struct {
    uint64_t sample_start;
    uint32_t frequency_hz;
} recording_capture_t;

typedef struct {
    pthread_mutex_t lock;
    int lock_ready;
    FILE *file;
    recording_format_t format;
    int sample_rate;
    uint32_t center_frequency_hz;
    uint64_t data_bytes;
    uint64_t sample_frames;
    int write_failed;
    char data_path[RECORDING_PATH_MAX];
    char metadata_path[RECORDING_PATH_MAX];
    recording_capture_t *captures;
    size_t capture_count;
    size_t capture_capacity;
} recording_writer_t;

void recording_writer_init(recording_writer_t *writer);
void recording_writer_destroy(recording_writer_t *writer);

int recording_parse_audio_format(const char *name, recording_format_t *format);
int recording_parse_iq_format(const char *name, recording_format_t *format);
const char *recording_format_name(recording_format_t format);

int recording_writer_start_audio(recording_writer_t *writer,
                                 const char *path,
                                 recording_format_t format,
                                 int sample_rate);
int recording_writer_start_iq(recording_writer_t *writer,
                              const char *path,
                              recording_format_t format,
                              int sample_rate,
                              uint32_t center_frequency_hz);
uint64_t recording_writer_stop(recording_writer_t *writer,
                               int *write_failed);
int recording_writer_is_active(recording_writer_t *writer);
int recording_writer_get_path(recording_writer_t *writer,
                              char *path,
                              size_t path_size);

void recording_writer_write_audio_f32(recording_writer_t *writer,
                                      const float *samples,
                                      uint32_t count);
void recording_writer_write_iq_u8(recording_writer_t *writer,
                                  const unsigned char *samples,
                                  uint32_t byte_count);
void recording_writer_note_frequency(recording_writer_t *writer,
                                     uint32_t frequency_hz);

#ifdef __cplusplus
}
#endif
