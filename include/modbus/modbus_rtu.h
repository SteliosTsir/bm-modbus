/*
 * Modbus RTU framing: [addr][PDU...][CRC lo][CRC hi]. Pure data in / data out,
 * no UART and no timing (see modbus_rtu_slave.h for that).
 */
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <modbus/modbus_pdu.h>
#include <errno.h>

#define MODBUS_RTU_MAX_ADU_LEN 256
#define MODBUS_BROADCAST_ID    0x00
#define MODBUS_SLAVE_ID_MIN    1
#define MODBUS_SLAVE_ID_MAX    247	/* 248..255 are reserved */

typedef enum { 
	MODBUS_PARITY_NONE,
	MODBUS_PARITY_ODD,
	MODBUS_PARITY_EVEN
} modbus_parity_t;

typedef enum {
	MODBUS_LINK_RS232,	/* point to point, no driver control */
	MODBUS_LINK_RS485_2W,	/* half duplex on one pair: we hear our own transmission */
	MODBUS_LINK_RS485_4W	/* separate pairs, but the slave driver is still shared */
} modbus_link_t;

typedef struct {
	uint8_t slave_id;
	const modbus_backend_t *backend;
} modbus_rtu_t;

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

/* Returns 0, or -EINVAL for a NULL argument, a NULL backend, or a slave id
 * outside 1..247 (0 is the broadcast address, 248..255 are reserved). */
int modbus_rtu_init(modbus_rtu_t *rtu, uint8_t slave_id, const modbus_backend_t *backend);

/* CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF) */
uint16_t modbus_rtu_crc16(const uint8_t *buf, size_t len);

/*
 * Checks a received frame and builds the response frame.
 * Returns the response length, or 0 when nothing must be sent: bad length,
 * bad CRC, other slave's address, or a broadcast. A broadcast write is
 * executed (silently); a broadcast read is ignored without touching the
 * device. Never writes more than max_tx_len bytes.
 */
size_t modbus_rtu_process_frame(const modbus_rtu_t *rtu, const uint8_t *rx_frame, size_t rx_len,
				uint8_t *tx_frame, size_t max_tx_len);

/* Inter-frame gap t3.5 in us for 11-bit characters; fixed 1750 us above 19200 baud. */
uint32_t modbus_rtu_t35_us(uint32_t baud);

/* Inter-character timeout t1.5 in us; fixed 750 us above 19200 baud. */
uint32_t modbus_rtu_t15_us(uint32_t baud);

#endif /* MODBUS_RTU_H */
