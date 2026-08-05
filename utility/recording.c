#include "recording.h"

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RECORDING_CONVERSION_CHUNK 1024u
#define WAV_MAX_DATA_BYTES ((uint64_t)UINT32_MAX - 36u)

static int equals_icase(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static int is_audio_format(recording_format_t format) {
    return format >= RECORDING_AUDIO_WAV_S16
           && format <= RECORDING_AUDIO_F32LE;
}

static int is_iq_format(recording_format_t format) {
    return format >= RECORDING_IQ_CU8
           && format <= RECORDING_IQ_SIGMF_CU8;
}

static int is_wav_format(recording_format_t format) {
    return format == RECORDING_AUDIO_WAV_S16
           || format == RECORDING_AUDIO_WAV_F32
           || format == RECORDING_IQ_WAV_S16;
}

static int copy_path(char *destination, size_t size, const char *source) {
    size_t length;
    if (!destination || size == 0u || !source || source[0] == '\0') return 0;
    length = strlen(source);
    if (length >= size) return 0;
    memcpy(destination, source, length + 1u);
    return 1;
}

static int ends_with(const char *value, const char *suffix) {
    size_t value_length;
    size_t suffix_length;
    if (!value || !suffix) return 0;
    value_length = strlen(value);
    suffix_length = strlen(suffix);
    return value_length >= suffix_length
           && strcmp(value + value_length - suffix_length, suffix) == 0;
}

static int prepare_sigmf_paths(recording_writer_t *writer, const char *path) {
    size_t base_length;
    char base[RECORDING_PATH_MAX];

    if (!writer || !path) return 0;
    if (!copy_path(base, sizeof(base), path)) return 0;
    if (ends_with(base, ".sigmf-data")) {
        base[strlen(base) - strlen(".sigmf-data")] = '\0';
    } else if (ends_with(base, ".sigmf-meta")) {
        base[strlen(base) - strlen(".sigmf-meta")] = '\0';
    }
    base_length = strlen(base);
    if (base_length + strlen(".sigmf-data") >= sizeof(writer->data_path)
        || base_length + strlen(".sigmf-meta") >= sizeof(writer->metadata_path)) {
        return 0;
    }
    (void)snprintf(writer->data_path, sizeof(writer->data_path),
                   "%s.sigmf-data", base);
    (void)snprintf(writer->metadata_path, sizeof(writer->metadata_path),
                   "%s.sigmf-meta", base);
    return 1;
}

static int write_bytes(FILE *file, const void *data, size_t size) {
    return file && data && fwrite(data, 1u, size, file) == size;
}

static int write_u16_le(FILE *file, uint16_t value) {
    unsigned char bytes[2];
    bytes[0] = (unsigned char)(value & 0xffu);
    bytes[1] = (unsigned char)((value >> 8u) & 0xffu);
    return write_bytes(file, bytes, sizeof(bytes));
}

static int write_u32_le(FILE *file, uint32_t value) {
    unsigned char bytes[4];
    bytes[0] = (unsigned char)(value & 0xffu);
    bytes[1] = (unsigned char)((value >> 8u) & 0xffu);
    bytes[2] = (unsigned char)((value >> 16u) & 0xffu);
    bytes[3] = (unsigned char)((value >> 24u) & 0xffu);
    return write_bytes(file, bytes, sizeof(bytes));
}

static int write_wav_header(recording_writer_t *writer) {
    uint16_t audio_format;
    uint16_t channels;
    uint16_t bits_per_sample;
    uint16_t block_align;
    uint32_t byte_rate;

    if (!writer || !writer->file || writer->sample_rate <= 0) return 0;
    audio_format = writer->format == RECORDING_AUDIO_WAV_F32 ? 3u : 1u;
    channels = writer->format == RECORDING_IQ_WAV_S16 ? 2u : 1u;
    bits_per_sample = writer->format == RECORDING_AUDIO_WAV_F32 ? 32u : 16u;
    block_align = (uint16_t)(channels * (bits_per_sample / 8u));
    byte_rate = (uint32_t)writer->sample_rate * (uint32_t)block_align;

    return write_bytes(writer->file, "RIFF", 4u)
           && write_u32_le(writer->file, 36u)
           && write_bytes(writer->file, "WAVEfmt ", 8u)
           && write_u32_le(writer->file, 16u)
           && write_u16_le(writer->file, audio_format)
           && write_u16_le(writer->file, channels)
           && write_u32_le(writer->file, (uint32_t)writer->sample_rate)
           && write_u32_le(writer->file, byte_rate)
           && write_u16_le(writer->file, block_align)
           && write_u16_le(writer->file, bits_per_sample)
           && write_bytes(writer->file, "data", 4u)
           && write_u32_le(writer->file, 0u);
}

static int add_capture_locked(recording_writer_t *writer,
                              uint32_t frequency_hz) {
    recording_capture_t *replacement;
    size_t new_capacity;
    if (!writer || writer->format != RECORDING_IQ_SIGMF_CU8) return 1;
    if (writer->capture_count > 0u
        && writer->captures[writer->capture_count - 1u].frequency_hz
           == frequency_hz) {
        return 1;
    }
    if (writer->capture_count == writer->capture_capacity) {
        new_capacity = writer->capture_capacity == 0u
                           ? 8u : writer->capture_capacity * 2u;
        replacement = (recording_capture_t *)realloc(
            writer->captures, new_capacity * sizeof(*replacement));
        if (!replacement) return 0;
        writer->captures = replacement;
        writer->capture_capacity = new_capacity;
    }
    writer->captures[writer->capture_count].sample_start = writer->sample_frames;
    writer->captures[writer->capture_count].frequency_hz = frequency_hz;
    writer->capture_count++;
    return 1;
}

static int write_sigmf_metadata_locked(recording_writer_t *writer) {
    FILE *metadata;
    if (!writer || writer->format != RECORDING_IQ_SIGMF_CU8
        || writer->metadata_path[0] == '\0') {
        return 1;
    }
    metadata = fopen(writer->metadata_path, "wb");
    if (!metadata) return 0;
    (void)fprintf(metadata,
                  "{\n  \"global\": {\n"
                  "    \"core:datatype\": \"cu8\",\n"
                  "    \"core:sample_rate\": %d,\n"
                  "    \"core:version\": \"1.2.0\",\n"
                  "    \"core:recorder\": \"rtl-universal\",\n"
                  "    \"core:description\": \"RTL-SDR IQ recording\"\n"
                  "  },\n  \"captures\": [",
                  writer->sample_rate);
    for (size_t i = 0u; i < writer->capture_count; i++) {
        (void)fprintf(metadata,
                      "%s\n    { \"core:sample_start\": %llu, "
                      "\"core:frequency\": %u }",
                      i == 0u ? "" : ",",
                      (unsigned long long)writer->captures[i].sample_start,
                      writer->captures[i].frequency_hz);
    }
    (void)fputs("\n  ],\n  \"annotations\": []\n}\n", metadata);
    if (fclose(metadata) != 0) return 0;
    return 1;
}

static void finalize_wav_locked(recording_writer_t *writer) {
    long end_position;
    uint32_t data_size;
    if (!writer || !writer->file || !is_wav_format(writer->format)) return;
    data_size = (uint32_t)writer->data_bytes;
    end_position = ftell(writer->file);
    if (fseek(writer->file, 4L, SEEK_SET) != 0
        || !write_u32_le(writer->file, 36u + data_size)
        || fseek(writer->file, 40L, SEEK_SET) != 0
        || !write_u32_le(writer->file, data_size)) {
        writer->write_failed = 1;
    }
    if (end_position >= 0L && fseek(writer->file, end_position, SEEK_SET) != 0) {
        writer->write_failed = 1;
    }
}

static uint64_t stop_locked(recording_writer_t *writer, int *write_failed) {
    uint64_t bytes;
    if (!writer) return 0u;
    bytes = writer->data_bytes;
    if (writer->file) {
        int close_result;
        finalize_wav_locked(writer);
        if (fflush(writer->file) != 0) writer->write_failed = 1;
        close_result = fclose(writer->file);
        if (close_result != 0) writer->write_failed = 1;
        writer->file = NULL;
        if (!write_sigmf_metadata_locked(writer)) writer->write_failed = 1;
    }
    if (write_failed) *write_failed = writer->write_failed;
    writer->format = RECORDING_FORMAT_NONE;
    return bytes;
}

static int start_locked(recording_writer_t *writer,
                        const char *path,
                        recording_format_t format,
                        int sample_rate,
                        uint32_t center_frequency_hz) {
    if (!writer || writer->file || sample_rate <= 0 || !path || !*path) return 0;
    writer->data_bytes = 0u;
    writer->sample_frames = 0u;
    writer->write_failed = 0;
    writer->capture_count = 0u;
    writer->metadata_path[0] = '\0';
    writer->format = format;
    writer->sample_rate = sample_rate;
    writer->center_frequency_hz = center_frequency_hz;
    if (format == RECORDING_IQ_SIGMF_CU8) {
        if (!prepare_sigmf_paths(writer, path)) goto fail;
    } else if (!copy_path(writer->data_path, sizeof(writer->data_path), path)) {
        goto fail;
    }
    writer->file = fopen(writer->data_path, "wb");
    if (!writer->file) goto fail;
    if (is_wav_format(format) && !write_wav_header(writer)) goto fail;
    if (format == RECORDING_IQ_SIGMF_CU8
        && !add_capture_locked(writer, center_frequency_hz)) {
        goto fail;
    }
    return 1;

fail:
    if (writer->file) {
        (void)fclose(writer->file);
        writer->file = NULL;
    }
    writer->format = RECORDING_FORMAT_NONE;
    return 0;
}

void recording_writer_init(recording_writer_t *writer) {
    if (!writer) return;
    memset(writer, 0, sizeof(*writer));
    if (pthread_mutex_init(&writer->lock, NULL) == 0) writer->lock_ready = 1;
}

void recording_writer_destroy(recording_writer_t *writer) {
    if (!writer || !writer->lock_ready) return;
    (void)recording_writer_stop(writer, NULL);
    (void)pthread_mutex_destroy(&writer->lock);
    writer->lock_ready = 0;
    free(writer->captures);
    writer->captures = NULL;
    writer->capture_capacity = 0u;
    writer->capture_count = 0u;
}

int recording_parse_audio_format(const char *name, recording_format_t *format) {
    if (!name || !format) return 0;
    if (equals_icase(name, "wav") || equals_icase(name, "wav-s16")) {
        *format = RECORDING_AUDIO_WAV_S16;
    } else if (equals_icase(name, "wav-f32")) {
        *format = RECORDING_AUDIO_WAV_F32;
    } else if (equals_icase(name, "s16") || equals_icase(name, "s16le")) {
        *format = RECORDING_AUDIO_S16LE;
    } else if (equals_icase(name, "f32") || equals_icase(name, "f32le")) {
        *format = RECORDING_AUDIO_F32LE;
    } else {
        return 0;
    }
    return 1;
}

int recording_parse_iq_format(const char *name, recording_format_t *format) {
    if (!name || !format) return 0;
    if (equals_icase(name, "cu8") || equals_icase(name, "u8")) {
        *format = RECORDING_IQ_CU8;
    } else if (equals_icase(name, "cs16") || equals_icase(name, "cs16le")) {
        *format = RECORDING_IQ_CS16LE;
    } else if (equals_icase(name, "cf32") || equals_icase(name, "cf32le")) {
        *format = RECORDING_IQ_CF32LE;
    } else if (equals_icase(name, "wav-iq")
               || equals_icase(name, "wav-iq-s16")) {
        *format = RECORDING_IQ_WAV_S16;
    } else if (equals_icase(name, "sigmf")
               || equals_icase(name, "sigmf-cu8")) {
        *format = RECORDING_IQ_SIGMF_CU8;
    } else {
        return 0;
    }
    return 1;
}

const char *recording_format_name(recording_format_t format) {
    switch (format) {
    case RECORDING_AUDIO_WAV_S16: return "wav-s16";
    case RECORDING_AUDIO_WAV_F32: return "wav-f32";
    case RECORDING_AUDIO_S16LE: return "s16le";
    case RECORDING_AUDIO_F32LE: return "f32le";
    case RECORDING_IQ_CU8: return "cu8";
    case RECORDING_IQ_CS16LE: return "cs16le";
    case RECORDING_IQ_CF32LE: return "cf32le";
    case RECORDING_IQ_WAV_S16: return "wav-iq-s16";
    case RECORDING_IQ_SIGMF_CU8: return "sigmf-cu8";
    case RECORDING_FORMAT_NONE:
    default: return "none";
    }
}

int recording_writer_start_audio(recording_writer_t *writer,
                                 const char *path,
                                 recording_format_t format,
                                 int sample_rate) {
    int result;
    if (!writer || !writer->lock_ready || !is_audio_format(format)) return 0;
    (void)pthread_mutex_lock(&writer->lock);
    result = start_locked(writer, path, format, sample_rate, 0u);
    (void)pthread_mutex_unlock(&writer->lock);
    return result;
}

int recording_writer_start_iq(recording_writer_t *writer,
                              const char *path,
                              recording_format_t format,
                              int sample_rate,
                              uint32_t center_frequency_hz) {
    int result;
    if (!writer || !writer->lock_ready || !is_iq_format(format)) return 0;
    (void)pthread_mutex_lock(&writer->lock);
    result = start_locked(writer, path, format, sample_rate,
                          center_frequency_hz);
    (void)pthread_mutex_unlock(&writer->lock);
    return result;
}

uint64_t recording_writer_stop(recording_writer_t *writer,
                               int *write_failed) {
    uint64_t bytes;
    if (!writer || !writer->lock_ready) return 0u;
    (void)pthread_mutex_lock(&writer->lock);
    bytes = stop_locked(writer, write_failed);
    (void)pthread_mutex_unlock(&writer->lock);
    return bytes;
}

int recording_writer_is_active(recording_writer_t *writer) {
    int active;
    if (!writer || !writer->lock_ready) return 0;
    (void)pthread_mutex_lock(&writer->lock);
    active = writer->file != NULL;
    (void)pthread_mutex_unlock(&writer->lock);
    return active;
}

int recording_writer_get_path(recording_writer_t *writer,
                              char *path,
                              size_t path_size) {
    int result;
    if (!writer || !writer->lock_ready || !path || path_size == 0u) return 0;
    (void)pthread_mutex_lock(&writer->lock);
    result = copy_path(path, path_size, writer->data_path);
    (void)pthread_mutex_unlock(&writer->lock);
    return result;
}

static int16_t normalized_to_s16(float sample) {
    float clipped = fmaxf(-1.0f, fminf(1.0f, sample));
    return (int16_t)lrintf(clipped * 32767.0f);
}

static int write_s16_chunk(FILE *file, const int16_t *values, size_t count) {
    unsigned char bytes[RECORDING_CONVERSION_CHUNK * 2u];
    if (!file || !values || count > RECORDING_CONVERSION_CHUNK) return 0;
    for (size_t i = 0u; i < count; i++) {
        uint16_t value = (uint16_t)values[i];
        bytes[i * 2u] = (unsigned char)(value & 0xffu);
        bytes[i * 2u + 1u] = (unsigned char)((value >> 8u) & 0xffu);
    }
    return write_bytes(file, bytes, count * 2u);
}

static int write_f32_chunk(FILE *file, const float *values, size_t count) {
    unsigned char bytes[RECORDING_CONVERSION_CHUNK * 4u];
    if (!file || !values || count > RECORDING_CONVERSION_CHUNK) return 0;
    for (size_t i = 0u; i < count; i++) {
        uint32_t value;
        memcpy(&value, &values[i], sizeof(value));
        bytes[i * 4u] = (unsigned char)(value & 0xffu);
        bytes[i * 4u + 1u] = (unsigned char)((value >> 8u) & 0xffu);
        bytes[i * 4u + 2u] = (unsigned char)((value >> 16u) & 0xffu);
        bytes[i * 4u + 3u] = (unsigned char)((value >> 24u) & 0xffu);
    }
    return write_bytes(file, bytes, count * 4u);
}

void recording_writer_write_audio_f32(recording_writer_t *writer,
                                      const float *samples,
                                      uint32_t count) {
    uint32_t offset = 0u;
    if (!writer || !writer->lock_ready || !samples || count == 0u) return;
    (void)pthread_mutex_lock(&writer->lock);
    if (!writer->file || !is_audio_format(writer->format)
        || writer->write_failed) {
        (void)pthread_mutex_unlock(&writer->lock);
        return;
    }
    if (is_wav_format(writer->format)) {
        uint64_t bytes_per_frame = writer->format == RECORDING_AUDIO_WAV_F32
                                       ? 4u : 2u;
        uint64_t available = WAV_MAX_DATA_BYTES - writer->data_bytes;
        if ((uint64_t)count > available / bytes_per_frame) {
            count = (uint32_t)(available / bytes_per_frame);
            writer->write_failed = 1;
        }
    }
    while (offset < count) {
        uint32_t chunk = count - offset;
        if (chunk > RECORDING_CONVERSION_CHUNK) chunk = RECORDING_CONVERSION_CHUNK;
        if (writer->format == RECORDING_AUDIO_WAV_F32
            || writer->format == RECORDING_AUDIO_F32LE) {
            if (!write_f32_chunk(writer->file, samples + offset, chunk)) {
                writer->write_failed = 1;
                break;
            }
            writer->data_bytes += (uint64_t)chunk * 4u;
        } else {
            int16_t converted[RECORDING_CONVERSION_CHUNK];
            for (uint32_t i = 0u; i < chunk; i++) {
                converted[i] = normalized_to_s16(samples[offset + i]);
            }
            if (!write_s16_chunk(writer->file, converted, chunk)) {
                writer->write_failed = 1;
                break;
            }
            writer->data_bytes += (uint64_t)chunk * 2u;
        }
        writer->sample_frames += chunk;
        offset += chunk;
    }
    (void)pthread_mutex_unlock(&writer->lock);
}

void recording_writer_write_iq_u8(recording_writer_t *writer,
                                  const unsigned char *samples,
                                  uint32_t byte_count) {
    uint32_t offset = 0u;
    if (!writer || !writer->lock_ready || !samples || byte_count < 2u) return;
    byte_count &= ~1u;
    (void)pthread_mutex_lock(&writer->lock);
    if (!writer->file || !is_iq_format(writer->format)
        || writer->write_failed) {
        (void)pthread_mutex_unlock(&writer->lock);
        return;
    }
    if (writer->format == RECORDING_IQ_CU8
        || writer->format == RECORDING_IQ_SIGMF_CU8) {
        if (!write_bytes(writer->file, samples, byte_count)) {
            writer->write_failed = 1;
        } else {
            writer->data_bytes += byte_count;
            writer->sample_frames += byte_count / 2u;
        }
        (void)pthread_mutex_unlock(&writer->lock);
        return;
    }
    if (writer->format == RECORDING_IQ_WAV_S16) {
        uint64_t available = WAV_MAX_DATA_BYTES - writer->data_bytes;
        uint64_t requested_frames = byte_count / 2u;
        if (requested_frames > available / 4u) {
            byte_count = (uint32_t)((available / 4u) * 2u);
            writer->write_failed = 1;
        }
    }
    while (offset < byte_count) {
        uint32_t component_count = byte_count - offset;
        if (component_count > RECORDING_CONVERSION_CHUNK) {
            component_count = RECORDING_CONVERSION_CHUNK;
        }
        if (writer->format == RECORDING_IQ_CF32LE) {
            float converted[RECORDING_CONVERSION_CHUNK];
            for (uint32_t i = 0u; i < component_count; i++) {
                converted[i] = ((float)samples[offset + i] - 127.5f) / 127.5f;
            }
            if (!write_f32_chunk(writer->file, converted, component_count)) {
                writer->write_failed = 1;
                break;
            }
            writer->data_bytes += (uint64_t)component_count * 4u;
        } else {
            int16_t converted[RECORDING_CONVERSION_CHUNK];
            for (uint32_t i = 0u; i < component_count; i++) {
                converted[i] = (int16_t)(((int)samples[offset + i] - 128) << 8);
            }
            if (!write_s16_chunk(writer->file, converted, component_count)) {
                writer->write_failed = 1;
                break;
            }
            writer->data_bytes += (uint64_t)component_count * 2u;
        }
        writer->sample_frames += component_count / 2u;
        offset += component_count;
    }
    (void)pthread_mutex_unlock(&writer->lock);
}

void recording_writer_note_frequency(recording_writer_t *writer,
                                     uint32_t frequency_hz) {
    if (!writer || !writer->lock_ready || frequency_hz == 0u) return;
    (void)pthread_mutex_lock(&writer->lock);
    if (writer->file && writer->format == RECORDING_IQ_SIGMF_CU8
        && !add_capture_locked(writer, frequency_hz)) {
        writer->write_failed = 1;
    }
    writer->center_frequency_hz = frequency_hz;
    (void)pthread_mutex_unlock(&writer->lock);
}
