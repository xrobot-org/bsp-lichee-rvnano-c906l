#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

enum {
  kDefaultAddress = 0x30,
};

static void Usage(const char *program) {
  fprintf(stderr,
          "Usage: %s [--bus /dev/i2c-2] [--address 0x30] read [reg ...]\n"
          "       %s [--bus /dev/i2c-2] [--address 0x30] write reg value\n",
          program, program);
}

static int ParseNumber(const char *text, unsigned maximum, unsigned *value) {
  char *end = NULL;
  errno = 0;
  const unsigned long parsed = strtoul(text, &end, 0);
  if (errno != 0 || end == text || *end != '\0' || parsed > maximum) {
    return -1;
  }
  *value = (unsigned)parsed;
  return 0;
}

static int ReadRegister(int fd, uint16_t reg, uint8_t *value) {
  uint8_t register_bytes[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
  return write(fd, register_bytes, sizeof(register_bytes)) ==
                 (ssize_t)sizeof(register_bytes) &&
             read(fd, value, 1) == 1
         ? 0
         : -1;
}

static int WriteRegister(int fd, uint16_t reg, uint8_t value) {
  uint8_t bytes[3] = {(uint8_t)(reg >> 8), (uint8_t)reg, value};
  return write(fd, bytes, sizeof(bytes)) == (ssize_t)sizeof(bytes) ? 0 : -1;
}

int main(int argc, char **argv) {
  const char *bus = "/dev/i2c-2";
  unsigned address = kDefaultAddress;
  int argument = 1;
  while (argument < argc && strncmp(argv[argument], "--", 2) == 0) {
    if (strcmp(argv[argument], "--bus") == 0 && argument + 1 < argc) {
      bus = argv[argument + 1];
    } else if (strcmp(argv[argument], "--address") == 0 &&
               argument + 1 < argc &&
               ParseNumber(argv[argument + 1], 0x7f, &address) == 0) {
    } else {
      Usage(argv[0]);
      return 2;
    }
    argument += 2;
  }

  if (argument >= argc) {
    Usage(argv[0]);
    return 2;
  }

  const int fd = open(bus, O_RDWR);
  if (fd < 0) {
    perror(bus);
    return 1;
  }
  if (ioctl(fd, I2C_SLAVE_FORCE, address) != 0) {
    fprintf(stderr, "select I2C address 0x%02x failed: %s\n", address,
            strerror(errno));
    close(fd);
    return 1;
  }

  int result = 0;
  if (strcmp(argv[argument], "read") == 0) {
    static const uint16_t default_registers[] = {
        0x0100, 0x3107, 0x3108, 0x3e01, 0x3e02, 0x3e06,
        0x3e07, 0x3e08, 0x3e09, 0x4500, 0x4501, 0x5011,
        0x503d, 0x0601,
    };
    const int first_register = argument + 1;
    const size_t register_count = first_register < argc
                                      ? (size_t)(argc - first_register)
                                      : sizeof(default_registers) /
                                            sizeof(default_registers[0]);
    for (size_t i = 0; i < register_count; ++i) {
      unsigned parsed_register = 0;
      const uint16_t reg = first_register < argc
                               ? (ParseNumber(argv[first_register + (int)i], 0xffff,
                                              &parsed_register) == 0
                                      ? (uint16_t)parsed_register
                                      : UINT16_MAX)
                               : default_registers[i];
      uint8_t value = 0;
      if (reg == UINT16_MAX || ReadRegister(fd, reg, &value) != 0) {
        fprintf(stderr, "read 0x%04x failed: %s\n", reg, strerror(errno));
        result = 1;
        continue;
      }
      printf("0x%04x = 0x%02x\n", reg, value);
    }
  } else if (strcmp(argv[argument], "write") == 0 && argument + 2 == argc - 1) {
    unsigned reg = 0;
    unsigned value = 0;
    if (ParseNumber(argv[argument + 1], 0xffff, &reg) != 0 ||
        ParseNumber(argv[argument + 2], 0xff, &value) != 0) {
      Usage(argv[0]);
      result = 2;
    } else if (WriteRegister(fd, (uint16_t)reg, (uint8_t)value) != 0) {
      fprintf(stderr, "write 0x%04x failed: %s\n", reg, strerror(errno));
      result = 1;
    } else {
      uint8_t readback = 0;
      if (ReadRegister(fd, (uint16_t)reg, &readback) != 0) {
        fprintf(stderr, "write succeeded, readback failed: %s\n", strerror(errno));
        result = 1;
      } else {
        printf("0x%04x: wrote 0x%02x, read back 0x%02x\n", reg, value, readback);
        result = readback == (uint8_t)value ? 0 : 1;
      }
    }
  } else {
    Usage(argv[0]);
    result = 2;
  }

  close(fd);
  return result;
}
