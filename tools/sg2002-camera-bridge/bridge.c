#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

typedef struct {
  uint8_t *data[8];
  int data_size[8];
  int count;
} mmf_stream_t;

typedef enum {
  MMF_VENC_RCMODE_CBR = 0,
  MMF_VENC_RCMODE_VBR,
  MMF_VENC_RCMODE_FIXQP,
} mmf_venc_rc_mode_e;

typedef struct {
  uint8_t type;
  int w;
  int h;
  int fmt;
  uint8_t jpg_quality;
  int gop;
  int intput_fps;
  int output_fps;
  int bitrate;
  mmf_venc_rc_mode_e rc_mode;
} mmf_venc_cfg_t;

extern int mmf_init(void);
extern int mmf_deinit(void);
extern int mmf_vi_init(void);
extern int mmf_vi_deinit(void);
extern int mmf_get_vi_unused_channel(void);
extern int mmf_add_vi_channel(int ch, int width, int height, int format);
extern int mmf_del_vi_channel(int ch);
extern int mmf_vi_frame_pop(int ch, void **data, int *len, int *width,
                            int *height, int *format);
extern void mmf_vi_frame_free(int ch);
extern int mmf_venc_unused_channel(void);
extern int mmf_add_venc_channel(int ch, mmf_venc_cfg_t *cfg);
extern int mmf_del_venc_channel(int ch);
extern int mmf_venc_push(int ch, uint8_t *data, int width, int height,
                         int format);
extern int mmf_venc_pop(int ch, mmf_stream_t *stream);
extern int mmf_venc_free(int ch);
extern int mmf_invert_format_to_mmf(int maix_format);
extern int mmf_vb_config_of_vi(uint32_t size, uint32_t count);

enum {
  kMailboxMagic = 0x4d435a52u, /* "RZCM" in little-endian memory. */
  kMailboxVersion = 1u,
  kMailboxEmpty = 0u,
  kMailboxReady = 1u,
  kMailboxPhysicalAddress = 0x8ffde000u,
  kMailboxMapSize = 0x21000u,
  kMailboxHeaderSize = 64u,
  kDefaultWidth = 640,
  kDefaultHeight = 480,
  kDefaultFps = 5,
  kDefaultBitrateKbps = 48,
  kDefaultGop = 15,
  kMaixNv21Format = 8,
  // The active SC035HGS profile produces 640x480 NV21. The vendor VI-only
  // program opens this VPSS output once, closes it, then opens it again before
  // frames become available to user space.
  kCaptureWidth = 640,
  kCaptureHeight = 480,
  // The vendor VI/VPSS path sizes its backing buffer from the sensor maximum
  // (2560x1440 NV21), even when the exposed channel is downscaled to 640x480.
  // Without this pool, base_get_chn_buffer() waits forever and no frame can be
  // popped from the channel.
  kViPoolSize = 2560 * 1440 * 3 / 2,
  kViPoolCount = 3,
};

typedef struct {
  uint32_t magic;
  uint32_t version;
  volatile uint32_t state;
  uint32_t sequence;
  uint32_t length;
  uint32_t timestamp_ms;
  uint16_t width;
  uint16_t height;
  uint32_t crc32;
  uint32_t producer_drops;
  uint32_t reserved[7];
  uint8_t data[];
} CameraMailbox;

_Static_assert(sizeof(CameraMailbox) == kMailboxHeaderSize,
               "camera mailbox header must remain wire-compatible");

static volatile sig_atomic_t stop_requested;

/* The CVI userspace DSOs were built with a GCC 10 runtime that exported the
 * byte-sized legacy __sync helpers. Modern RISC-V libgcc hides those symbols.
 * Implement them with a naturally aligned 32-bit CAS, which is supported by
 * every SG2002 toolchain and preserves atomicity with adjacent byte fields. */
static uint8_t AtomicUpdateByte(volatile uint8_t *pointer, uint8_t value,
                                bool add) {
  const uintptr_t address = (uintptr_t)pointer;
  volatile uint32_t *const word = (volatile uint32_t *)(address & ~(uintptr_t)3u);
  const unsigned shift = (unsigned)(address & 3u) * 8u;
  const uint32_t mask = 0xffu << shift;
  uint32_t observed = __atomic_load_n(word, __ATOMIC_SEQ_CST);
  for (;;) {
    const uint8_t previous = (uint8_t)(observed >> shift);
    const uint8_t updated = add ? (uint8_t)(previous + value)
                                : (uint8_t)(previous & value);
    const uint32_t desired = (observed & ~mask) | ((uint32_t)updated << shift);
    if (__atomic_compare_exchange_n(word, &observed, desired, false,
                                    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
      return previous;
    }
  }
}

uint8_t CompatSyncFetchAndAdd1(volatile void *pointer, uint8_t value)
    __asm__("__sync_fetch_and_add_1");
uint8_t CompatSyncFetchAndAdd1(volatile void *pointer, uint8_t value) {
  return AtomicUpdateByte((volatile uint8_t *)pointer, value, true);
}

uint8_t CompatSyncFetchAndAnd1(volatile void *pointer, uint8_t value)
    __asm__("__sync_fetch_and_and_1");
uint8_t CompatSyncFetchAndAnd1(volatile void *pointer, uint8_t value) {
  return AtomicUpdateByte((volatile uint8_t *)pointer, value, false);
}

bool CompatAtomicCompareExchange1(volatile void *pointer, void *expected,
                                  uint8_t desired, bool weak,
                                  int success_order, int failure_order)
    __asm__("__atomic_compare_exchange_1");
bool CompatAtomicCompareExchange1(volatile void *pointer, void *expected,
                                  uint8_t desired, bool weak,
                                  int success_order, int failure_order) {
  (void)weak;
  (void)success_order;
  (void)failure_order;
  const uintptr_t address = (uintptr_t)pointer;
  volatile uint32_t *const word = (volatile uint32_t *)(address & ~(uintptr_t)3u);
  const unsigned shift = (unsigned)(address & 3u) * 8u;
  const uint32_t mask = 0xffu << shift;
  uint8_t *const expected_byte = (uint8_t *)expected;
  uint32_t observed = __atomic_load_n(word, __ATOMIC_SEQ_CST);
  for (;;) {
    const uint8_t current = (uint8_t)(observed >> shift);
    if (current != *expected_byte) {
      *expected_byte = current;
      return false;
    }
    const uint32_t updated =
        (observed & ~mask) | ((uint32_t)desired << shift);
    if (__atomic_compare_exchange_n(word, &observed, updated, false,
                                    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
      return true;
    }
  }
}

static void HandleSignal(int signal_number) {
  (void)signal_number;
  stop_requested = 1;
}

static uint32_t Crc32(const uint8_t *bytes, size_t length) {
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1u) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
  }
  return ~crc;
}

static uint32_t MonotonicMilliseconds(void) {
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    return 0;
  }
  const uint64_t milliseconds =
      (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
  return (uint32_t)milliseconds;
}

static int WaitForEmpty(volatile CameraMailbox *mailbox) {
  while (!stop_requested) {
    __sync_synchronize();
    if (mailbox->state == kMailboxEmpty) {
      return 0;
    }
    usleep(1000);
  }
  return -1;
}

static int PublishAccessUnit(volatile CameraMailbox *mailbox,
                             const mmf_stream_t *stream, uint32_t sequence,
                             int width, int height) {
  if (stream->count <= 0 || stream->count > 8) {
    fprintf(stderr, "invalid VENC pack count: %d\n", stream->count);
    return -1;
  }

  size_t length = 0;
  for (int i = 0; i < stream->count; ++i) {
    if (stream->data[i] == NULL || stream->data_size[i] <= 0 ||
        (size_t)stream->data_size[i] > kMailboxMapSize - kMailboxHeaderSize - length) {
      fprintf(stderr, "invalid or oversized VENC access unit\n");
      return -1;
    }
    length += (size_t)stream->data_size[i];
  }

  if (WaitForEmpty(mailbox) != 0) {
    return -1;
  }

  size_t offset = 0;
  for (int i = 0; i < stream->count; ++i) {
    memcpy((void *)(mailbox->data + offset), stream->data[i],
           (size_t)stream->data_size[i]);
    offset += (size_t)stream->data_size[i];
  }

  mailbox->magic = kMailboxMagic;
  mailbox->version = kMailboxVersion;
  mailbox->sequence = sequence;
  mailbox->length = (uint32_t)length;
  mailbox->timestamp_ms = MonotonicMilliseconds();
  mailbox->width = (uint16_t)width;
  mailbox->height = (uint16_t)height;
  mailbox->crc32 = Crc32((const uint8_t *)mailbox->data, length);
  __sync_synchronize();
  mailbox->state = kMailboxReady;
  __sync_synchronize();
  return 0;
}

static void Usage(const char *program) {
  fprintf(stderr,
          "Usage: %s [--output file.h264] [--frames count] "
          "[--width px] [--height px] [--fps n] [--bitrate kbps]\n",
          program);
}

static int ParsePositive(const char *text, const char *name) {
  char *end = NULL;
  errno = 0;
  const long value = strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || value <= 0 || value > 65535) {
    fprintf(stderr, "invalid %s: %s\n", name, text);
    return -1;
  }
  return (int)value;
}

int main(int argc, char **argv) {
  const char *output_path = NULL;
  int frame_limit = 0;
  int width = kDefaultWidth;
  int height = kDefaultHeight;
  int fps = kDefaultFps;
  int bitrate = kDefaultBitrateKbps;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
      output_path = argv[++i];
    } else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
      frame_limit = ParsePositive(argv[++i], "frame count");
    } else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
      width = ParsePositive(argv[++i], "width");
    } else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
      height = ParsePositive(argv[++i], "height");
    } else if (strcmp(argv[i], "--fps") == 0 && i + 1 < argc) {
      fps = ParsePositive(argv[++i], "fps");
    } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
      bitrate = ParsePositive(argv[++i], "bitrate");
    } else {
      Usage(argv[0]);
      return 2;
    }
  }
  if (frame_limit < 0 || width < 0 || height < 0 || fps < 0 || bitrate < 0 ||
      (width & 1) != 0 || (height & 1) != 0) {
    Usage(argv[0]);
    return 2;
  }
  if (width != kCaptureWidth || height != kCaptureHeight) {
    fprintf(stderr, "SC035HGS bridge requires %dx%d encoding\n", kCaptureWidth,
            kCaptureHeight);
    return 2;
  }

  signal(SIGINT, HandleSignal);
  signal(SIGTERM, HandleSignal);

  FILE *output = NULL;
  int memory_fd = -1;
  volatile CameraMailbox *mailbox = MAP_FAILED;
  if (output_path != NULL) {
    output = fopen(output_path, "wb");
    if (output == NULL) {
      perror("fopen output");
      return 1;
    }
  } else {
    memory_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memory_fd < 0) {
      perror("open /dev/mem");
      return 1;
    }
    mailbox = mmap(NULL, kMailboxMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
                   memory_fd, kMailboxPhysicalAddress);
    if (mailbox == MAP_FAILED) {
      perror("mmap camera mailbox");
      close(memory_fd);
      return 1;
    }
    mailbox->magic = kMailboxMagic;
    mailbox->version = kMailboxVersion;
    mailbox->state = kMailboxEmpty;
    __sync_synchronize();
  }

  int result = 1;
  int vi_channel = -1;
  int venc_channel = -1;
  bool mmf_ready = false;
  bool vi_ready = false;
  bool vi_channel_ready = false;
  bool venc_ready = false;
  const int format = mmf_invert_format_to_mmf(kMaixNv21Format);

  if (format < 0 || mmf_vb_config_of_vi(kViPoolSize, kViPoolCount) != 0 ||
      mmf_init() != 0) {
    fprintf(stderr, "MMF system initialization failed\n");
    goto cleanup;
  }
  mmf_ready = true;

  if (mmf_vi_init() != 0) {
    fprintf(stderr, "VI initialization failed\n");
    goto cleanup;
  }
  vi_ready = true;
  vi_channel = mmf_get_vi_unused_channel();
  if (vi_channel < 0 ||
      mmf_add_vi_channel(vi_channel, kCaptureWidth, kCaptureHeight, format) != 0) {
    fprintf(stderr, "VI channel initialization failed\n");
    goto cleanup;
  }
  if (mmf_del_vi_channel(vi_channel) != 0 ||
      mmf_add_vi_channel(vi_channel, kCaptureWidth, kCaptureHeight, format) != 0) {
    fprintf(stderr, "VI channel restart failed\n");
    goto cleanup;
  }
  vi_channel_ready = true;

  venc_channel = mmf_venc_unused_channel();
  mmf_venc_cfg_t venc = {
      .type = 2,
      .w = width,
      .h = height,
      .fmt = format,
      .jpg_quality = 0,
      .gop = kDefaultGop,
      .intput_fps = fps,
      .output_fps = fps,
      .bitrate = bitrate,
      .rc_mode = MMF_VENC_RCMODE_CBR,
  };
  if (venc_channel < 0 || mmf_add_venc_channel(venc_channel, &venc) != 0) {
    fprintf(stderr, "H.264 VENC channel initialization failed\n");
    goto cleanup;
  }
  venc_ready = true;

  fprintf(stderr,
          "camera bridge ready: capture %dx%d, encode %dx%d %d fps %d kbit/s, VI=%d VENC=%d\n",
          kCaptureWidth, kCaptureHeight, width, height, fps, bitrate, vi_channel,
          venc_channel);

  uint32_t sequence = 0;
  while (!stop_requested && (frame_limit == 0 || sequence < (uint32_t)frame_limit)) {
    void *frame_data = NULL;
    int frame_length = 0;
    int frame_width = 0;
    int frame_height = 0;
    int frame_format = 0;
    if (mmf_vi_frame_pop(vi_channel, &frame_data, &frame_length, &frame_width,
                         &frame_height, &frame_format) != 0) {
      fprintf(stderr, "VI frame timeout\n");
      continue;
    }

    const int push_result =
        mmf_venc_push(venc_channel, frame_data, frame_width, frame_height, frame_format);
    mmf_vi_frame_free(vi_channel);
    if (push_result != 0) {
      fprintf(stderr, "VENC push failed\n");
      break;
    }

    mmf_stream_t stream = {0};
    if (mmf_venc_pop(venc_channel, &stream) != 0) {
      fprintf(stderr, "VENC pop failed\n");
      break;
    }

    bool frame_ok = true;
    if (output != NULL) {
      for (int i = 0; i < stream.count; ++i) {
        if (stream.data[i] == NULL || stream.data_size[i] <= 0 ||
            fwrite(stream.data[i], 1, (size_t)stream.data_size[i], output) !=
                (size_t)stream.data_size[i]) {
          frame_ok = false;
          break;
        }
      }
    } else if (PublishAccessUnit(mailbox, &stream, sequence, width, height) != 0) {
      frame_ok = false;
    }

    if (mmf_venc_free(venc_channel) != 0) {
      fprintf(stderr, "VENC release failed\n");
      frame_ok = false;
    }
    if (!frame_ok) {
      break;
    }
    ++sequence;
    if ((sequence % (uint32_t)fps) == 0) {
      fprintf(stderr, "published %u access units\n", sequence);
    }
  }
  result = 0;

cleanup:
  if (venc_ready) {
    mmf_del_venc_channel(venc_channel);
  }
  if (vi_channel_ready) {
    mmf_del_vi_channel(vi_channel);
  }
  if (vi_ready) {
    mmf_vi_deinit();
  }
  if (mmf_ready) {
    mmf_deinit();
  }
  if (output != NULL) {
    fclose(output);
  }
  if (mailbox != MAP_FAILED) {
    munmap((void *)mailbox, kMailboxMapSize);
  }
  if (memory_fd >= 0) {
    close(memory_fd);
  }
  return result;
}
