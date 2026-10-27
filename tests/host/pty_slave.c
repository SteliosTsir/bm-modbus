/*
 * Host build of the real slave engine on a pseudo-terminal, with the wall
 * clock. Prints the pty path on stdout, then serves until SIGTERM. Used by
 * pty_master.py (an independent master written from the spec) for an end-to-end
 * test. usage: pty_slave <slave_id> <baud> [rs232|rs485]
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <modbus/modbus_rtu_slave.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static int fd;
static uint16_t hr[16];
static uint8_t coil[16];
static unsigned de_toggles;

static modbus_exception_t rh(uint16_t a, uint16_t n, uint16_t *o) {
  if (a + n > 16)
    return MODBUS_EX_ILLEGAL_DATA_ADDR;
  for (int i = 0; i < n; i++)
    o[i] = hr[a + i];
  return 0;
}
static modbus_exception_t wh(uint16_t a, uint16_t v) {
  if (a >= 16)
    return MODBUS_EX_ILLEGAL_DATA_ADDR;
  hr[a] = v;
  return 0;
}
static modbus_exception_t whs(uint16_t a, uint16_t n, const uint16_t *s) {
  if (a + n > 16)
    return MODBUS_EX_ILLEGAL_DATA_ADDR;
  for (int i = 0; i < n; i++)
    hr[a + i] = s[i];
  return 0;
}
static modbus_exception_t rc(uint16_t a, uint16_t n, uint8_t *d) {
  if (a + n > 16)
    return MODBUS_EX_ILLEGAL_DATA_ADDR;
  memset(d, 0, (n + 7) / 8);
  for (int i = 0; i < n; i++)
    if (coil[a + i])
      d[i / 8] |= 1 << (i % 8);
  return 0;
}
static modbus_exception_t wc(uint16_t a, bool s) {
  if (a >= 16)
    return MODBUS_EX_ILLEGAL_DATA_ADDR;
  coil[a] = s;
  return 0;
}
static modbus_exception_t wcs(uint16_t a, uint16_t n, const uint8_t *s) {
  if (a + n > 16)
    return MODBUS_EX_ILLEGAL_DATA_ADDR;
  for (int i = 0; i < n; i++)
    coil[a + i] = (s[i / 8] >> (i % 8)) & 1;
  return 0;
}
static const modbus_backend_t BE = {.read_coils = rc,
                                    .read_multi_holding_registers = rh,
                                    .read_input_registers = rh,
                                    .write_coil = wc,
                                    .write_single_holding_register = wh,
                                    .write_multi_coils = wcs,
                                    .write_multi_holding_registers = whs};

static int s_getc(void *c) {
  (void)c;
  uint8_t b;
  ssize_t n = read(fd, &b, 1);
  return n == 1 ? b : -EAGAIN;
}
static void s_putc(void *c, uint8_t b) {
  (void)c;
  if (write(fd, &b, 1) != 1)
    perror("write");
}
static uint64_t s_now(void *c) {
  (void)c;
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1000000ull + t.tv_nsec / 1000;
}
static void s_flush(void *c) {
  (void)c;
  tcdrain(fd);
}
static void s_set_tx(void *c, bool en) {
  (void)c;
  de_toggles++;
  (void)en;
}
static volatile sig_atomic_t stop;
static void on_term(int s) {
  (void)s;
  stop = 1;
}

int main(int argc, char **argv) {
  if (argc < 3)
    return 2;
  bool rs485 = argc > 3 && !strcmp(argv[3], "rs485");
  fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  grantpt(fd);
  unlockpt(fd);
  struct termios t;
  tcgetattr(fd, &t);
  cfmakeraw(&t);
  tcsetattr(fd, TCSANOW, &t);
  printf("%s\n", ptsname(fd));
  fflush(stdout);
  signal(SIGTERM, on_term);
  modbus_serial_ops_t ops = {
      s_getc, s_putc, s_now, s_flush, rs485 ? s_set_tx : NULL, NULL, NULL};
  modbus_slave_cfg_t cfg = {.baud = (uint32_t)atoi(argv[2]),
                            .parity = MODBUS_PARITY_EVEN,
                            .stop_bits = 1,
                            .link = rs485 ? MODBUS_LINK_RS485_2W
                                          : MODBUS_LINK_RS232,
                            .de_lead_us = 20,
                            .de_trail_us = 20};
  modbus_slave_t s;
  if (modbus_slave_init(&s, (uint8_t)atoi(argv[1]), &BE, &ops, &cfg))
    return 3;
  while (!stop) {
    modbus_slave_poll(&s);
    usleep(100);
  }
  fprintf(stderr, "answered=%u ignored=%u dropped=%u de_toggles=%u\n",
          s.stats.frames_answered, s.stats.frames_ignored,
          s.stats.frames_dropped, de_toggles);
  return 0;
}
