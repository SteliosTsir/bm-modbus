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

typedef struct {
	uint8_t slave_id;
	const modbus_backend_t *backend;
} modbus_rtu_t;

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
