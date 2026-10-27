#include <modbus/modbus_master.h>

static void wait_us(modbus_master_t *m, uint32_t us)
{
	if (us == 0) return;
	if (m->ops.delay_us) {
		m->ops.delay_us(m->ops.ctx, us);
	} else {
		uint64_t start = m->ops.now_us(m->ops.ctx);
		while (m->ops.now_us(m->ops.ctx) - start < us) { }
	}
}

static void wait_ms(modbus_master_t *m, uint32_t ms)
{
	while (ms--) wait_us(m, 1000);
}

static bool is_rs485(const modbus_slave_cfg_t *cfg)
{
	return cfg->link == MODBUS_LINK_RS485_2W || cfg->link == MODBUS_LINK_RS485_4W;
}

int modbus_master_init(modbus_master_t *m, const modbus_serial_ops_t *ops, const modbus_slave_cfg_t *cfg,
		       uint32_t timeout_ms, uint8_t retries)
{
	if (!m || !ops || !cfg || !ops->getc || !ops->putc || !ops->now_us || timeout_ms == 0)
		return -EINVAL;
	if (cfg->baud == 0 || (cfg->stop_bits != 1 && cfg->stop_bits != 2))
		return -EINVAL;
	if (cfg->link != MODBUS_LINK_RS232 && !is_rs485(cfg))
		return -EINVAL;
	if (is_rs485(cfg) && (!ops->set_tx || !ops->tx_flush))
		return -EINVAL;

	m->ops = *ops;
	m->cfg = *cfg;
	m->t35_us = modbus_slave_t35_us(cfg->baud, cfg->parity, cfg->stop_bits);
	m->reply_gap_us = 0;
	m->timeout_ms = timeout_ms;
	m->broadcast_wait_ms = 100;	/* the spec suggests 100..200 ms turnaround */
	m->retries = retries;
	m->last_exception = 0;
	m->stats.requests = m->stats.ok = m->stats.retries = 0;
	m->stats.timeouts = m->stats.bad_replies = m->stats.exceptions = 0;
	if (m->ops.set_tx) m->ops.set_tx(m->ops.ctx, false);
	m->last_bus_us = m->ops.now_us(m->ops.ctx);
	return 0;
}

void modbus_master_set_tolerant(modbus_master_t *m, uint32_t gap_ms)
{
	if (m) m->reply_gap_us = gap_ms * 1000u;
}

static void send_frame(modbus_master_t *m, size_t n)
{
	if (m->ops.set_tx) {
		m->ops.set_tx(m->ops.ctx, true);
		wait_us(m, m->cfg.de_lead_us);
	}
	for (size_t i = 0; i < n; i++)
		m->ops.putc(m->ops.ctx, m->tx[i]);
	if (m->ops.tx_flush) m->ops.tx_flush(m->ops.ctx);
	if (m->ops.set_tx) {
		wait_us(m, m->cfg.de_trail_us);
		m->ops.set_tx(m->ops.ctx, false);
	}
	/* The receiver may have heard our own frame (2-wire RS-485): drop it. The slave cannot
	 * start answering before t3.5 after our last byte, so nothing real is lost here. */
	if (is_rs485(&m->cfg)) {
		for (int i = 0; i < 2 * MODBUS_RTU_MAX_ADU_LEN; i++)
			if (m->ops.getc(m->ops.ctx) == -EAGAIN) break;
	}
	m->last_bus_us = m->ops.now_us(m->ops.ctx);
}

/* Collects one reply: waits up to timeout for the first byte, then until t3.5 of silence (strict mode),
 * or, in tolerant mode, until the expected length has arrived or reply_gap_us of silence has passed.
 * exp_total is the exact length of a normal reply including address and CRC (0 = unknown).
 * Returns the length (>0), -ETIME, or -EBADMSG (line error or overflow). */
static int receive_frame(modbus_master_t *m, uint8_t slave, uint8_t fc, size_t exp_total)
{
	size_t len = 0;
	bool bad = false;
	uint64_t last = 0;
	uint64_t deadline = m->ops.now_us(m->ops.ctx) + (uint64_t)m->timeout_ms * 1000u;
	uint32_t gap = m->reply_gap_us ? m->reply_gap_us : m->t35_us;

	for (;;) {
		int r = m->ops.getc(m->ops.ctx);
		if (r >= 0) {
			if (m->reply_gap_us && len == 0 && (uint8_t)r != slave)
				continue;	/* text or noise before the reply (tolerant mode) */
			last = m->ops.now_us(m->ops.ctx);
			if (len < sizeof(m->rx)) m->rx[len++] = (uint8_t)r;
			else bad = true;
			if (m->reply_gap_us && !bad && len >= 2) {
				if (m->rx[1] == (uint8_t)(fc | 0x80) && len >= 5) break;	/* exception reply */
				if (exp_total && m->rx[1] == fc && len >= exp_total) break;	/* complete */
			}
		} else if (r == -EIO) {
			bad = true;	/* the damaged byte is still delivered by the next getc() */
		} else {
			uint64_t now = m->ops.now_us(m->ops.ctx);
			if (len > 0 && now - last >= gap) break;
			if (len == 0 && now >= deadline) return -ETIME;
		}
	}
	m->last_bus_us = last;
	return bad ? -EBADMSG : (int)len;
}

/*
 * One transaction. req is the request PDU. On success *resp points at the reply PDU inside
 * m->rx and *resp_len is its length. expect_len is the exact reply PDU length (0 = any).
 */
static int transact(modbus_master_t *m, uint8_t slave, const uint8_t *req, size_t req_len,
		    size_t expect_len, const uint8_t **resp, size_t *resp_len)
{
	if (slave > MODBUS_SLAVE_ID_MAX || req_len < 1 || req_len + 3 > sizeof(m->tx))
		return -EINVAL;

	m->tx[0] = slave;
	for (size_t i = 0; i < req_len; i++) m->tx[1 + i] = req[i];
	uint16_t crc = modbus_rtu_crc16(m->tx, 1 + req_len);
	size_t n = 1 + req_len;
	m->tx[n++] = (uint8_t)(crc & 0xFF);
	m->tx[n++] = (uint8_t)(crc >> 8);

	uint8_t fc = req[0];
	int err = -ETIME;
	m->stats.requests++;

	for (unsigned attempt = 0; attempt <= m->retries; attempt++) {
		if (attempt) m->stats.retries++;

		/* at least t3.5 of silence before a request */
		while (m->ops.now_us(m->ops.ctx) - m->last_bus_us < m->t35_us)
			wait_us(m, 50);
		/* old bytes (late replies, noise) must not be taken for this answer */
		for (int i = 0; i < 2 * MODBUS_RTU_MAX_ADU_LEN; i++)
			if (m->ops.getc(m->ops.ctx) == -EAGAIN) break;

		send_frame(m, n);

		if (slave == MODBUS_BROADCAST_ID) {
			wait_ms(m, m->broadcast_wait_ms);
			m->last_bus_us = m->ops.now_us(m->ops.ctx);
			m->stats.ok++;
			*resp = NULL; *resp_len = 0;
			return 0;
		}

		int len = receive_frame(m, slave, fc, expect_len ? expect_len + 3 : 0);
		if (len == -ETIME) { m->stats.timeouts++; err = -ETIME; continue; }
		if (len < 0) { m->stats.bad_replies++; err = -EBADMSG; continue; }

		const uint8_t *rx = m->rx;
		size_t rl = (size_t)len;
		bool good = rl >= 5 && rx[0] == slave &&
			    modbus_rtu_crc16(rx, rl - 2) == (uint16_t)(rx[rl - 2] | ((uint16_t)rx[rl - 1] << 8));
		if (good && rx[1] == (uint8_t)(fc | 0x80)) {
			if (rl != 5) good = false;
			else {
				m->last_exception = rx[2];
				m->stats.exceptions++;
				m->stats.ok++;
				return -EPROTO;
			}
		}
		if (good && (rx[1] != fc || (expect_len && rl - 3 != expect_len)))
			good = false;
		if (!good) { m->stats.bad_replies++; err = -EBADMSG; continue; }

		*resp = &rx[1];
		*resp_len = rl - 3;
		m->stats.ok++;
		return 0;
	}
	return err;
}

int modbus_master_read_regs(modbus_master_t *m, uint8_t slave, uint8_t fc, uint16_t addr,
			    uint16_t count, uint16_t *out)
{
	if (!m || !out || (fc != 3 && fc != 4) || slave == MODBUS_BROADCAST_ID ||
	    count < 1 || count > MODBUS_MAX_READ_REGS)
		return -EINVAL;
	uint8_t req[5] = { fc, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)(count >> 8), (uint8_t)count };
	const uint8_t *r; size_t rl;
	int rc = transact(m, slave, req, sizeof req, 2 + 2u * count, &r, &rl);
	if (rc) return rc;
	if (r[1] != 2u * count) return -EBADMSG;
	for (uint16_t i = 0; i < count; i++)
		out[i] = (uint16_t)(((uint16_t)r[2 + 2 * i] << 8) | r[3 + 2 * i]);
	return 0;
}

int modbus_master_read_bits(modbus_master_t *m, uint8_t slave, uint8_t fc, uint16_t addr,
			    uint16_t count, uint8_t *out)
{
	if (!m || !out || (fc != 1 && fc != 2) || slave == MODBUS_BROADCAST_ID ||
	    count < 1 || count > MODBUS_MAX_READ_BITS)
		return -EINVAL;
	uint8_t req[5] = { fc, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)(count >> 8), (uint8_t)count };
	size_t nb = ((size_t)count + 7) / 8;
	const uint8_t *r; size_t rl;
	int rc = transact(m, slave, req, sizeof req, 2 + nb, &r, &rl);
	if (rc) return rc;
	if (r[1] != nb) return -EBADMSG;
	for (size_t i = 0; i < nb; i++) out[i] = r[2 + i];
	return 0;
}

static int write_echo(modbus_master_t *m, uint8_t slave, const uint8_t *req, size_t req_len, size_t cmp_len)
{
	const uint8_t *r; size_t rl;
	int rc = transact(m, slave, req, req_len, 5, &r, &rl);
	if (rc) return rc;
	if (slave == MODBUS_BROADCAST_ID) return 0;
	for (size_t i = 0; i < cmp_len; i++)
		if (r[i] != req[i]) return -EBADMSG;
	return 0;
}

int modbus_master_write_reg(modbus_master_t *m, uint8_t slave, uint16_t addr, uint16_t value)
{
	if (!m) return -EINVAL;
	uint8_t req[5] = { 6, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)(value >> 8), (uint8_t)value };
	return write_echo(m, slave, req, sizeof req, 5);
}

int modbus_master_write_coil(modbus_master_t *m, uint8_t slave, uint16_t addr, bool on)
{
	if (!m) return -EINVAL;
	uint8_t req[5] = { 5, (uint8_t)(addr >> 8), (uint8_t)addr, on ? 0xFF : 0x00, 0x00 };
	return write_echo(m, slave, req, sizeof req, 5);
}

int modbus_master_write_regs(modbus_master_t *m, uint8_t slave, uint16_t addr, uint16_t count,
			     const uint16_t *values)
{
	if (!m || !values || count < 1 || count > MODBUS_MAX_WRITE_REGS) return -EINVAL;
	uint8_t req[6 + 2 * MODBUS_MAX_WRITE_REGS];
	req[0] = 0x10;
	req[1] = (uint8_t)(addr >> 8); req[2] = (uint8_t)addr;
	req[3] = (uint8_t)(count >> 8); req[4] = (uint8_t)count;
	req[5] = (uint8_t)(2 * count);
	for (uint16_t i = 0; i < count; i++) {
		req[6 + 2 * i] = (uint8_t)(values[i] >> 8);
		req[7 + 2 * i] = (uint8_t)values[i];
	}
	/* reply echoes address and quantity */
	return write_echo(m, slave, req, 6 + 2u * count, 5);
}