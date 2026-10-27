/*
 * Modbus RTU master demo: master() polls a temperature/humidity sensor every few seconds and
 * prints the readings on the SDK console. Selected by main.c (build with ROLE=master).
 *
 * The console UART and the Modbus UART must be different, otherwise the readings would be
 * printed onto the bus. Give the Modbus UART with -DMODBUS_UART_BASE=0x... (see uart_port.h).
 * Under QEMU there is only one UART. With -DSENSOR_SHARED_UART the Modbus traffic and the log lines
 * share it: every log line is sent as 0xFF "LOG:" text '\n'. 0xFF is a reserved address, so it can
 * never start a Modbus frame, and tools/qemu_serial_bridge.py (which sits between QEMU and the real
 * USB-RS485 adapter) takes those lines out and prints them on your terminal instead of putting
 * them on the bus. Do not use SENSOR_SHARED_UART on a real bus without that bridge.
 *
 * Defaults match the common "XY-MD02" style RS-485 sensor (9600 8N1, address 1, input registers
 * 1 = temperature, 2 = humidity). Override with -D on DEMO_FLAGS:
 *   -DSENSOR_ADDR=n  -DSENSOR_FC=3|4  -DSENSOR_FIRST_REG=n  -DSENSOR_TEMP_INDEX=n  -DSENSOR_HUM_INDEX=n
 *   (registers FIRST_REG and FIRST_REG+1 are read; the INDEX values say which of the two is which)
 *   -DSENSOR_BAUD=n  -DSENSOR_PARITY_EVEN  -DSENSOR_STOP_BITS=2  -DSENSOR_RS232
 *   -DSENSOR_PERIOD_MS=n  -DSENSOR_TIMEOUT_MS=n   (the sensor tested here needs ~0.6-1 s to start its reply)
 *   -DSENSOR_REPLY_GAP_MS=n   tolerate pauses of up to n ms inside the sensor's reply (default 200, 0 = strict
 *                             standard framing); this sensor sends its reply with pauses and prints text
 *   -DSENSOR_TEMP_DIV=n  -DSENSOR_HUM_DIV=n   register value / DIV = reading (10, 100, ...; HUM_DIV=0 prints the raw value)
 *   -DMODBUS_DEMO_DE_LEAD_US=n -DMODBUS_DEMO_DE_TRAIL_US=n  (RS-485 transceiver switching times)
 */
#include <modbus/modbus_master.h>
#include "uart_port.h"
#include <target_config.h>
#include <utils/utils.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>

#ifndef SENSOR_ADDR
#define SENSOR_ADDR		1
#endif
#ifndef SENSOR_FC
#define SENSOR_FC		4
#endif
#ifndef SENSOR_FIRST_REG
#define SENSOR_FIRST_REG	1
#endif
#ifndef SENSOR_TEMP_INDEX
#define SENSOR_TEMP_INDEX	0
#endif
#ifndef SENSOR_HUM_INDEX
#define SENSOR_HUM_INDEX	1
#endif
#ifndef SENSOR_BAUD
#define SENSOR_BAUD		9600
#endif
#ifndef SENSOR_STOP_BITS
#define SENSOR_STOP_BITS	1
#endif
#ifndef SENSOR_REPLY_GAP_MS
#define SENSOR_REPLY_GAP_MS	200
#endif
#ifndef SENSOR_TEMP_DIV
#define SENSOR_TEMP_DIV		100	/* this sensor reports hundredths of a degree (the datasheet says tenths) */
#endif
#ifndef SENSOR_HUM_DIV
#define SENSOR_HUM_DIV		0	/* unknown for this sensor: 0 = print the raw register value */
#endif
#ifndef SENSOR_PERIOD_MS
#define SENSOR_PERIOD_MS	5000
#endif
#ifndef SENSOR_TIMEOUT_MS
#define SENSOR_TIMEOUT_MS	2000
#endif
#ifndef MODBUS_DEMO_DE_LEAD_US
#define MODBUS_DEMO_DE_LEAD_US	0
#endif
#ifndef MODBUS_DEMO_DE_TRAIL_US
#define MODBUS_DEMO_DE_TRAIL_US	0
#endif

#if defined(MODBUS_ROLE_MASTER) && !defined(MODBUS_UART_BASE) && !defined(SENSOR_SHARED_UART)
#error "The sensor demo needs its own UART: add -DMODBUS_UART_BASE=0x... to DEMO_FLAGS (or -DSENSOR_SHARED_UART for a QEMU test only)"
#endif

static uint64_t port_now_us(void *ctx)
{
	(void)ctx;
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000u;
}
static int  port_getc(void *ctx) { (void)ctx; return uart_port_getc(); }
static void port_putc(void *ctx, uint8_t b) { (void)ctx; uart_port_putc_raw(b); }
static void port_flush(void *ctx) { (void)ctx; uart_port_tx_flush(); }
#ifndef SENSOR_RS232
static void port_set_tx(void *ctx, bool en) { (void)ctx; uart_port_set_rts(en); }
#endif

static const modbus_serial_ops_t port_ops = {
	.getc = port_getc, .putc = port_putc, .now_us = port_now_us, .tx_flush = port_flush,
#ifndef SENSOR_RS232
	.set_tx = port_set_tx,
#endif
};

/* Log lines use the SDK's own INF()/WRN()/ERR() (utils/utils.h), i.e. plain printf to the console.
 * On a separate console UART nothing else is needed. On the shared UART (SENSOR_SHARED_UART) every
 * line is wrapped as 0xFF "LOG:" ... '\n' so that the bridge can tell it apart from Modbus traffic
 * and never forwards it to the sensor. */
static void log_begin(void)
{
#ifdef SENSOR_SHARED_UART
	static const char tag[] = "LOG:";
	uart_port_putc_raw(0xFF);
	for (int i = 0; tag[i]; i++) uart_port_putc_raw((uint8_t)tag[i]);
#endif
}

static void log_end(void)
{
#ifdef SENSOR_SHARED_UART
	uart_port_putc_raw('\n');
#else
	printf("\n");
#endif
}

#define LOGLINE(level, fmt, ...) \
	do { log_begin(); level(fmt, ##__VA_ARGS__); log_end(); } while (0)

/* "-12.34", "45.6" or just "123" depending on div (1, 10, 100, ...) */
static void fmt_scaled(char *out, size_t n, int v, int div)
{
	int digits = 0;
	for (int d = div; d > 1; d /= 10) digits++;
	int a = v < 0 ? -v : v;
	if (div <= 1) snprintf(out, n, "%d", v);
	else snprintf(out, n, "%s%d.%0*d", v < 0 ? "-" : "", a / div, digits, a % div);
}

void master(void)
{
	static modbus_master_t m;
	modbus_slave_cfg_t cfg = {
		.baud = SENSOR_BAUD,
#ifdef SENSOR_PARITY_EVEN
		.parity = MODBUS_PARITY_EVEN,
#else
		.parity = MODBUS_PARITY_NONE,
#endif
		.stop_bits = SENSOR_STOP_BITS,
#ifdef SENSOR_RS232
		.link = MODBUS_LINK_RS232,
#else
		.link = MODBUS_LINK_RS485_2W,
#endif
		.de_lead_us = MODBUS_DEMO_DE_LEAD_US,
		.de_trail_us = MODBUS_DEMO_DE_TRAIL_US,
	};

#ifdef SENSOR_SHARED_UART
	uart_port_set_format(cfg.parity != MODBUS_PARITY_NONE, cfg.stop_bits);
#else
	uart_port_init(cfg.baud, cfg.parity != MODBUS_PARITY_NONE, cfg.stop_bits);
#endif

	if (modbus_master_init(&m, &port_ops, &cfg, SENSOR_TIMEOUT_MS, 2) != 0) {
		LOGLINE(ERR, "sensor demo: bad configuration");
		for (;;) { }
	}
	modbus_master_set_tolerant(&m, SENSOR_REPLY_GAP_MS);
	LOGLINE(INF, "sensor demo: polling slave %d every %d ms", SENSOR_ADDR, SENSOR_PERIOD_MS);

	/*
	 * After every poll one machine-readable line "DATA {json}" is sent through the same log channel. The
	 * bridge (tools/qemu_serial_bridge.py --dashboard PORT) turns those lines into a live web page.
	 * Values are raw register values; "td"/"hd" are the divisors to turn them into degrees / percent.
	 * "up" is the time since boot in ms, "rtt" the duration of the Modbus transaction, "jit" how late
	 * this poll started compared with its schedule (the loop below is paced by the timer, not by sleeping),
	 * "first" the uptime at the first good reading (boot to first value).
	 */
	uint64_t next = port_now_us(NULL);
	unsigned seq = 0, first_ms = 0;
	for (;;) {
		uint16_t v[2] = { 0, 0 };
		uint64_t t0 = port_now_us(NULL);
		int jit_us = (int)((int64_t)(t0 - next));
		int rc = modbus_master_read_regs(&m, SENSOR_ADDR, SENSOR_FC, SENSOR_FIRST_REG, 2, v);
		unsigned rtt_ms = (unsigned)((port_now_us(NULL) - t0) / 1000u);
		if (rc == 0) {
			int t = (int16_t)v[SENSOR_TEMP_INDEX], h = (int16_t)v[SENSOR_HUM_INDEX];
			char ts[16], hs[16];
			fmt_scaled(ts, sizeof ts, t, SENSOR_TEMP_DIV);
			fmt_scaled(hs, sizeof hs, h, SENSOR_HUM_DIV);
			LOGLINE(INF, "T=%s C  H=%s%s", ts, hs, SENSOR_HUM_DIV ? " %" : " (raw)");
			if (!first_ms) first_ms = (unsigned)(port_now_us(NULL) / 1000u);
		} else if (rc == -EPROTO) {
			LOGLINE(WRN, "sensor answered with exception %d", modbus_master_exception(&m));
		} else if (rc == -ETIME) {
			LOGLINE(WRN, "no answer from the sensor (check wiring, address, baud rate, parity)");
		} else {
			LOGLINE(WRN, "damaged answer from the sensor (error %d)", rc);
		}

		log_begin();
		printf("DATA {\"seq\":%u,\"up\":%u,\"rc\":%d,\"t\":%d,\"h\":%d,\"td\":%d,\"hd\":%d,\"rtt\":%u,\"jit\":%d,"
		       "\"ok\":%u,\"retry\":%u,\"tmo\":%u,\"bad\":%u,\"exc\":%u,\"first\":%u}",
		       ++seq, (unsigned)(port_now_us(NULL) / 1000u), rc, (int16_t)v[SENSOR_TEMP_INDEX], (int16_t)v[SENSOR_HUM_INDEX],
		       SENSOR_TEMP_DIV, SENSOR_HUM_DIV ? SENSOR_HUM_DIV : 1, rtt_ms, jit_us,
		       m.stats.ok, m.stats.retries, m.stats.timeouts, m.stats.bad_replies, m.stats.exceptions, first_ms);
		log_end();

		next += (uint64_t)SENSOR_PERIOD_MS * 1000u;
		uint64_t now = port_now_us(NULL);
		if (next < now) next = now;		/* a poll took longer than the period: do not try to catch up */
		while (port_now_us(NULL) < next) { }
	}
}