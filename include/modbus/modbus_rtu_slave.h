/*
 * Modbus RTU slave over a serial line (RS-232, RS-485 2-wire or 4-wire).
 *
 * This is the part that turns "bytes arrive, bytes can be sent" into the
 * state machine of "Modbus over Serial Line V1.02" section 2.5.1:
 *
 *   INIT --t3.5 silence--> IDLE --byte--> RECEPTION --t3.5 silence--> process
 *                           ^                                          |
 *                           +------------- reply (EMISSION) <----------+
 *
 * It is poll based and does not use the SDK directly: everything hardware
 * related comes in through modbus_serial_ops_t, so the same code runs on the
 * board (see demo/modbus_slave.c) and on a host against a simulated line
 * (tests/host/).
 *
 * Call modbus_slave_poll() often. Byte times are taken when a byte is READ,
 * not when it arrived, so the loop must come back at least every ~t3.5
 * (1.75 ms above 19200 baud); otherwise one frame can be seen as two and
 * both halves are dropped (CRC), which looks like a lost request to the master.
 */
#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H

#include <modbus/modbus_rtu.h>

typedef struct {
	/* Required. Returns a byte 0..255, -EAGAIN when none is waiting, or -EIO
	 * for a framing/parity/overrun/break error (like uart_getc()). */
	int  (*getc)(void *ctx);
	/* Required. Sends one byte as is (no '\n' translation). */
	void (*putc)(void *ctx, uint8_t byte);
	/* Required: monotonic microseconds. */
	uint64_t (*now_us)(void *ctx);
	/* Wait until the last stop bit has left the wire. Required for RS-485,
	 * strongly advised otherwise. */
	void (*tx_flush)(void *ctx);
	/* Drive the RS-485 driver-enable (DE/RE) line. Required for RS-485. */
	void (*set_tx)(void *ctx, bool enable);
	/* Optional sleep; if NULL the engine spins on now_us(). */
	void (*delay_us)(void *ctx, uint32_t us);
	void *ctx;
} modbus_serial_ops_t;

typedef enum { MODBUS_PARITY_NONE, MODBUS_PARITY_ODD, MODBUS_PARITY_EVEN } modbus_parity_t;

typedef enum {
	MODBUS_LINK_RS232,	/* point to point, no driver control */
	MODBUS_LINK_RS485_2W,	/* half duplex on one pair: we hear our own transmission */
	MODBUS_LINK_RS485_4W	/* separate pairs, but the slave driver is still shared */
} modbus_link_t;

typedef struct {
	uint32_t baud;
	modbus_parity_t parity;	/* the spec default is EVEN */
	uint8_t stop_bits;	/* 1 or 2 (2 when parity is NONE, per spec) */
	modbus_link_t link;
	uint32_t de_lead_us;	/* DE high -> first start bit (transceiver enable time) */
	uint32_t de_trail_us;	/* last stop bit -> DE low */
	/*
	 * Drop a frame if two of its bytes are more than t1.5 apart. The spec
	 * requires it, but it is only reliable when the loop polls much faster
	 * than t1.5 (750 us above 19200 baud), so it is off by default.
	 */
	bool strict_t15;
} modbus_slave_cfg_t;

typedef struct {
	uint32_t frames_answered;	/* replies sent */
	uint32_t frames_ignored;	/* complete frames that need no reply: bad CRC, other
					 * slave, broadcast, too short */
	uint32_t frames_dropped;	/* frames discarded for t1.5, overflow or line error */
	uint32_t line_errors;		/* -EIO reports */
	uint32_t bytes_discarded;	/* bytes thrown away in INIT / after our own reply */
} modbus_slave_stats_t;

typedef enum { MODBUS_ST_INIT, MODBUS_ST_IDLE, MODBUS_ST_RECEPTION } modbus_slave_state_t;

typedef struct {
	modbus_rtu_t rtu;
	modbus_serial_ops_t ops;
	modbus_slave_cfg_t cfg;
	uint32_t t15_us, t35_us;
	modbus_slave_state_t state;
	uint64_t last_us;	/* time of last byte seen (INIT/RECEPTION) */
	size_t rx_len;
	bool bad, overflow, err_pending;
	uint8_t rx[MODBUS_RTU_MAX_ADU_LEN];
	uint8_t tx[MODBUS_RTU_MAX_ADU_LEN];
	modbus_slave_stats_t stats;
} modbus_slave_t;

/* 0 or -EINVAL (bad id, NULL backend, missing mandatory op, baud 0, RS-485
 * without set_tx/tx_flush, stop_bits not 1 or 2). */
int modbus_slave_init(modbus_slave_t *s, uint8_t slave_id, const modbus_backend_t *backend,
		      const modbus_serial_ops_t *ops, const modbus_slave_cfg_t *cfg);

/* Services the line. Returns 1 if a reply was sent during this call, else 0. */
int modbus_slave_poll(modbus_slave_t *s);

/* Frame gap times used by the engine (character length is never below 11 bits) */
uint32_t modbus_slave_t15_us(uint32_t baud, modbus_parity_t parity, uint8_t stop_bits);
uint32_t modbus_slave_t35_us(uint32_t baud, modbus_parity_t parity, uint8_t stop_bits);

#endif /* MODBUS_RTU_SLAVE_H */
