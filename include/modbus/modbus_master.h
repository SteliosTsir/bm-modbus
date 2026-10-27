/*
 * Modbus RTU master (client): sends a request to one slave and waits for the
 * answer. Works on RS-232 and RS-485 through the same modbus_serial_ops_t as
 * the slave engine.
 *
 * Calls block until the transaction is finished (reply, exception, or all
 * retries timed out). Rules from "Modbus over Serial Line V1.02" that it keeps:
 *  - at least t3.5 of silence before every request,
 *  - a reply must come from the slave addressed, with the same function code
 *    (or that code | 0x80 and an exception code),
 *  - a reply with a bad CRC or length is dropped and the request repeated,
 *  - a broadcast (slave 0) is never answered: the master just waits long
 *    enough for the slaves to finish.
 *
 * Return values: 0 on success, otherwise a negative errno:
 *   -EINVAL     bad argument (quantity out of the spec's range, NULL, slave > 247)
 *   -ETIME  no reply after all retries
 *   -EBADMSG    replies arrived but were damaged or did not match the request
 *   -EPROTO  the slave answered with an exception; see modbus_master_exception()
 */
#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <modbus/modbus_rtu_slave.h>	/* modbus_serial_ops_t, modbus_slave_cfg_t */

typedef struct {
	uint32_t requests;	/* transactions started */
	uint32_t ok;		/* finished with a good reply (or sent broadcast) */
	uint32_t retries;	/* repeated requests */
	uint32_t timeouts;	/* attempts with no reply */
	uint32_t bad_replies;	/* attempts with a damaged or unexpected reply */
	uint32_t exceptions;	/* exception replies */
} modbus_master_stats_t;

typedef struct {
	modbus_serial_ops_t ops;
	modbus_slave_cfg_t cfg;
	uint32_t t35_us;
	uint32_t reply_gap_us;		/* 0 = strict (t3.5 ends a reply); else tolerant mode, see modbus_master_set_tolerant() */
	uint32_t timeout_ms;		/* wait for the first byte of the reply */
	uint32_t broadcast_wait_ms;	/* pause after a broadcast */
	uint8_t retries;		/* extra attempts after the first */
	uint8_t last_exception;
	uint64_t last_bus_us;		/* last time the line was active */
	uint8_t tx[MODBUS_RTU_MAX_ADU_LEN];
	uint8_t rx[MODBUS_RTU_MAX_ADU_LEN];
	modbus_master_stats_t stats;
} modbus_master_t;

/* The cfg fields baud/parity/stop_bits/link/de_lead_us/de_trail_us are used.
 * Suggested: timeout_ms 500, retries 2. 0 or -EINVAL like modbus_slave_init. */
int modbus_master_init(modbus_master_t *m, const modbus_serial_ops_t *ops, const modbus_slave_cfg_t *cfg,
		       uint32_t timeout_ms, uint8_t retries);

/*
 * Tolerant reply mode for sloppy slaves. The standard says a pause of t3.5 inside a frame ends it, and
 * that is the default. Some cheap sensors send their reply with longer pauses between bytes (seen: an
 * XY-MD02 variant, with gaps of several ms and a reply that starts ~0.6 s after the request) and may print
 * text lines on the same wire. With gap_ms > 0 the master instead
 *   - ignores bytes before the reply that are not the slave's address,
 *   - ends the reply as soon as the expected number of bytes (or an exception reply) has arrived,
 *   - otherwise ends it after gap_ms of silence instead of t3.5.
 * gap_ms == 0 returns to the strict behaviour. The CRC, address, function code and length checks stay.
 */
void modbus_master_set_tolerant(modbus_master_t *m, uint32_t gap_ms);

/* FC 03 (holding) or 04 (input): reads count (1..125) registers into out[] */
int modbus_master_read_regs(modbus_master_t *m, uint8_t slave, uint8_t fc, uint16_t addr,
			    uint16_t count, uint16_t *out);
/* FC 01 (coils) or 02 (discrete inputs): count (1..2000) bits, packed LSB first, out needs (count+7)/8 bytes */
int modbus_master_read_bits(modbus_master_t *m, uint8_t slave, uint8_t fc, uint16_t addr,
			    uint16_t count, uint8_t *out);
/* FC 06 / FC 05 / FC 10. Slave 0 = broadcast. */
int modbus_master_write_reg(modbus_master_t *m, uint8_t slave, uint16_t addr, uint16_t value);
int modbus_master_write_coil(modbus_master_t *m, uint8_t slave, uint16_t addr, bool on);
int modbus_master_write_regs(modbus_master_t *m, uint8_t slave, uint16_t addr, uint16_t count,
			     const uint16_t *values);	/* count 1..123 */

/* Exception code of the last -EPROTO (1 illegal function, 2 illegal address, 3 illegal value, 4 device failure ...) */
static inline uint8_t modbus_master_exception(const modbus_master_t *m) { return m->last_exception; }

#endif /* MODBUS_MASTER_H */