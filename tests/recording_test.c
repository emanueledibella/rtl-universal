#include "recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static uint32_t read_u32_le(const unsigned char *bytes) {
    return (uint32_t)bytes[0]
           | ((uint32_t)bytes[1] << 8u)
           | ((uint32_t)bytes[2] << 16u)
           | ((uint32_t)bytes[3] << 24u);
}

static long file_size(const char *path) {
    FILE *file = fopen(path, "rb");
    long size;
    if (!file) return -1L;
    if (fseek(file, 0L, SEEK_END) != 0) {
        (void)fclose(file);
        return -1L;
    }
    size = ftell(file);
    (void)fclose(file);
    return size;
}

static int test_audio_wav(const char *path) {
    recording_writer_t writer;
    recording_format_t format;
    float samples[] = { -1.0f, -0.25f, 0.25f, 1.0f };
    unsigned char header[44];
    FILE *file;
    int failed = 0;

    recording_writer_init(&writer);
    if (!recording_parse_audio_format("wav-s16", &format)
        || !recording_writer_start_audio(&writer, path, format, 48000)) {
        recording_writer_destroy(&writer);
        return 0;
    }
    recording_writer_write_audio_f32(&writer, samples, 4u);
    if (recording_writer_stop(&writer, &failed) != 8u || failed) {
        recording_writer_destroy(&writer);
        return 0;
    }
    recording_writer_destroy(&writer);
    file = fopen(path, "rb");
    if (!file) return 0;
    if (fread(header, 1u, sizeof(header), file) != sizeof(header)) {
        (void)fclose(file);
        return 0;
    }
    (void)fclose(file);
    return memcmp(header, "RIFF", 4u) == 0
           && memcmp(header + 8u, "WAVE", 4u) == 0
           && read_u32_le(header + 24u) == 48000u
           && read_u32_le(header + 40u) == 8u
           && file_size(path) == 52L;
}

static int test_iq_format(const char *path,
                          const char *name,
                          long expected_size) {
    recording_writer_t writer;
    recording_format_t format;
    unsigned char iq[] = { 0u, 255u, 127u, 128u };
    int failed = 0;
    recording_writer_init(&writer);
    if (!recording_parse_iq_format(name, &format)
        || !recording_writer_start_iq(&writer, path, format,
                                      2400000, 145500000u)) {
        recording_writer_destroy(&writer);
        return 0;
    }
    recording_writer_write_iq_u8(&writer, iq, sizeof(iq));
    (void)recording_writer_stop(&writer, &failed);
    recording_writer_destroy(&writer);
    return !failed && file_size(path) == expected_size;
}

static int test_sigmf(const char *base) {
    recording_writer_t writer;
    recording_format_t format;
    unsigned char iq[] = { 0u, 255u, 127u, 128u };
    char data_path[512];
    char metadata_path[512];
    char metadata[2048];
    FILE *file;
    size_t count;
    int failed = 0;

    (void)snprintf(data_path, sizeof(data_path), "%s.sigmf-data", base);
    (void)snprintf(metadata_path, sizeof(metadata_path), "%s.sigmf-meta", base);
    recording_writer_init(&writer);
    if (!recording_parse_iq_format("sigmf-cu8", &format)
        || !recording_writer_start_iq(&writer, base, format,
                                      2400000, 145500000u)) {
        recording_writer_destroy(&writer);
        return 0;
    }
    recording_writer_write_iq_u8(&writer, iq, sizeof(iq));
    recording_writer_note_frequency(&writer, 146000000u);
    if (recording_writer_stop(&writer, &failed) != sizeof(iq) || failed) {
        recording_writer_destroy(&writer);
        return 0;
    }
    recording_writer_destroy(&writer);
    file = fopen(metadata_path, "rb");
    if (!file) return 0;
    count = fread(metadata, 1u, sizeof(metadata) - 1u, file);
    metadata[count] = '\0';
    (void)fclose(file);
    return file_size(data_path) == (long)sizeof(iq)
           && strstr(metadata, "\"core:datatype\": \"cu8\"")
           && strstr(metadata, "\"core:sample_rate\": 2400000")
           && strstr(metadata, "\"core:frequency\": 145500000")
           && strstr(metadata, "\"core:frequency\": 146000000")
           && strstr(metadata, "\"core:sample_start\": 2");
}

int main(void) {
    char prefix[384];
    char path[512];
    int ok = 1;
    (void)snprintf(prefix, sizeof(prefix),
                   "/tmp/rtl-universal-recording-test-%ld", (long)getpid());

    (void)snprintf(path, sizeof(path), "%s-audio.wav", prefix);
    ok = test_audio_wav(path) && ok;
    (void)remove(path);

    (void)snprintf(path, sizeof(path), "%s-iq.cu8", prefix);
    ok = test_iq_format(path, "cu8", 4L) && ok;
    (void)remove(path);
    (void)snprintf(path, sizeof(path), "%s-iq.cs16", prefix);
    ok = test_iq_format(path, "cs16le", 8L) && ok;
    (void)remove(path);
    (void)snprintf(path, sizeof(path), "%s-iq.cf32", prefix);
    ok = test_iq_format(path, "cf32le", 16L) && ok;
    (void)remove(path);
    (void)snprintf(path, sizeof(path), "%s-iq.wav", prefix);
    ok = test_iq_format(path, "wav-iq-s16", 52L) && ok;
    (void)remove(path);

    ok = test_sigmf(prefix) && ok;
    (void)snprintf(path, sizeof(path), "%s.sigmf-data", prefix);
    (void)remove(path);
    (void)snprintf(path, sizeof(path), "%s.sigmf-meta", prefix);
    (void)remove(path);

    if (!ok) {
        fprintf(stderr, "recording test FAILED\n");
        return 1;
    }
    printf("recording test PASS audio=WAV IQ=CU8/CS16/CF32/WAV/SigMF\n");
    return 0;
}
