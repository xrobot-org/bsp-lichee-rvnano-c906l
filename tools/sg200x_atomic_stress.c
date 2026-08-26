// SPDX-License-Identifier: MIT
// Run this on SG2002 Linux: gcc -O2 -std=c11 -pthread sg200x_atomic_stress.c -o atomic-stress
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

struct stress_state {
  _Atomic uint32_t counter;
  _Atomic uint32_t ready;
  _Atomic int start;
  uint32_t iterations;
};

static void *increment(void *argument)
{
  struct stress_state *state = argument;

  atomic_fetch_add_explicit(&state->ready, 1u, memory_order_release);
  while (!atomic_load_explicit(&state->start, memory_order_acquire))
    ;
  for (uint32_t index = 0; index < state->iterations; ++index)
    atomic_fetch_add_explicit(&state->counter, 1u, memory_order_seq_cst);
  return NULL;
}

static int parse_positive(const char *text, uint32_t *value)
{
  char *end = NULL;
  unsigned long parsed = strtoul(text, &end, 0);
  if (*text == '\0' || *end != '\0' || parsed == 0 || parsed > UINT32_MAX)
    return -1;
  *value = (uint32_t)parsed;
  return 0;
}

int main(int argc, char **argv)
{
  uint32_t threads = 4u;
  uint32_t iterations = 2000000u;
  struct stress_state state = {0};
  pthread_t *workers;
  uint64_t expected;

  if (argc > 1 && parse_positive(argv[1], &threads) != 0) {
    fprintf(stderr, "usage: %s [threads [iterations]]\n", argv[0]);
    return 2;
  }
  if (argc > 2 && parse_positive(argv[2], &iterations) != 0) {
    fprintf(stderr, "usage: %s [threads [iterations]]\n", argv[0]);
    return 2;
  }
  if (argc > 3 || threads > 256u) {
    fprintf(stderr, "threads must be in [1, 256]\n");
    return 2;
  }
  expected = (uint64_t)threads * iterations;
  if (expected > UINT32_MAX) {
    fprintf(stderr, "threads * iterations must fit in uint32_t\n");
    return 2;
  }
  if (!atomic_is_lock_free(&state.counter)) {
    fputs("FAIL: uint32_t atomic is not lock-free\n", stderr);
    return 1;
  }

  workers = calloc(threads, sizeof(*workers));
  if (workers == NULL) {
    perror("calloc");
    return 1;
  }
  state.iterations = iterations;
  for (uint32_t index = 0; index < threads; ++index) {
    int error = pthread_create(&workers[index], NULL, increment, &state);
    if (error != 0) {
      errno = error;
      perror("pthread_create");
      return 1;
    }
  }
  while (atomic_load_explicit(&state.ready, memory_order_acquire) != threads)
    ;
  atomic_store_explicit(&state.start, 1, memory_order_release);
  for (uint32_t index = 0; index < threads; ++index) {
    int error = pthread_join(workers[index], NULL);
    if (error != 0) {
      errno = error;
      perror("pthread_join");
      return 1;
    }
  }
  free(workers);

  const uint32_t observed = atomic_load_explicit(&state.counter, memory_order_acquire);
  printf("threads=%" PRIu32 " iterations=%" PRIu32 " expected=%" PRIu64
         " observed=%" PRIu32 " lock_free=yes\n",
         threads, iterations, expected, observed);
  return observed == expected ? 0 : 1;
}
