#include <stdio.h>
#include <stdint.h>

uint8_t __sync_fetch_and_add_1(volatile void *pointer, uint8_t value) {
  volatile uint8_t *byte = (volatile uint8_t *)pointer;
  uint8_t old = *byte;
  *byte = (uint8_t)(old + value);
  return old;
}

uint8_t __sync_fetch_and_and_1(volatile void *pointer, uint8_t value) {
  volatile uint8_t *byte = (volatile uint8_t *)pointer;
  uint8_t old = *byte;
  *byte = (uint8_t)(old & value);
  return old;
}

extern int mmf_init(void);
extern int mmf_deinit(void);
extern int mmf_vi_init(void);
extern int mmf_vi_deinit(void);
extern int mmf_get_vi_unused_channel(void);
extern int mmf_add_vi_channel(int, int, int, int);
extern int mmf_del_vi_channel(int);
extern int mmf_vi_frame_pop(int, void **, int *, int *, int *, int *);
extern void mmf_vi_frame_free(int);
extern int mmf_invert_format_to_mmf(int);

int main(void) {
  printf("invert 0=%d 1=%d 2=%d 8=%d 9=%d 10=%d\n",
         mmf_invert_format_to_mmf(0), mmf_invert_format_to_mmf(1),
         mmf_invert_format_to_mmf(2), mmf_invert_format_to_mmf(8),
         mmf_invert_format_to_mmf(9), mmf_invert_format_to_mmf(10));
  printf("init=%d vi=%d\n", mmf_init(), mmf_vi_init());
  const int channel = mmf_get_vi_unused_channel();
  const int format = mmf_invert_format_to_mmf(8);
  /* Mirror the vendor VI-only test. The first open closes immediately; its
   * driver uses that transaction to settle the sensor/VPSS route. */
  const int first_add = mmf_add_vi_channel(channel, 2560, 1440, format);
  printf("channel=%d first_add=%d format=%d\n", channel, first_add, format);
  if (first_add == 0) {
    printf("first_del=%d\n", mmf_del_vi_channel(channel));
  }
  printf("second_add=%d\n", mmf_add_vi_channel(channel, 2560, 1440, format));
  for (int i = 0; i < 10; ++i) {
    void *data = NULL;
    int length = 0;
    int width = 0;
    int height = 0;
    int frame_format = 0;
    const int result = mmf_vi_frame_pop(channel, &data, &length, &width,
                                        &height, &frame_format);
    printf("pop %d: result=%d data=%p length=%d size=%dx%d format=%d\n",
           i, result, data, length, width, height, frame_format);
    if (result == 0) mmf_vi_frame_free(channel);
  }
  mmf_del_vi_channel(channel);
  mmf_vi_deinit();
  mmf_deinit();
  return 0;
}
