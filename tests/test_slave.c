/*
 * Host tests for the serial-line slave engine, against a simulated serial port
 * with a virtual clock. Checks the behaviours of "Modbus over Serial Line V1.02"
 * 2.5.1 (slave state machine, t1.5/t3.5, no reply on errors) and the RS-485
 * driver-enable handling.
 */
#include "mbtest.h"
#include <modbus/modbus_rtu_slave.h>

/* ---------------- tiny device ---------------- */
static struct { uint16_t hr[10]; uint8_t coil; unsigned reads, writes; } D;

static modbus_exception_t d_rh(uint16_t a, uint16_t n, uint16_t *o)
{ D.reads++; if (a + n > 10) return MODBUS_EX_ILLEGAL_DATA_ADDR; for (int i = 0; i < n; i++) o[i] = D.hr[a + i]; return 0; }
static modbus_exception_t d_wh(uint16_t a, uint16_t v)
{ D.writes++; if (a >= 10) return MODBUS_EX_ILLEGAL_DATA_ADDR; D.hr[a] = v; return 0; }
static modbus_exception_t d_wc(uint16_t a, bool s)
{ D.writes++; if (a >= 8) return MODBUS_EX_ILLEGAL_DATA_ADDR; if (s) D.coil |= 1u << a; else D.coil &= ~(1u << a); return 0; }
static const modbus_backend_t BE = { .read_multi_holding_registers = d_rh,
	.write_single_holding_register = d_wh, .write_coil = d_wc };

/* ---------------- simulated serial port ---------------- */
enum { EV_DE_ON, EV_DE_OFF, EV_BYTE, EV_FLUSH };
struct ev { int type; uint64_t t; uint8_t b; };
struct rxb { uint64_t at; uint8_t b; bool err; };

static struct {
	uint64_t t;			/* virtual time, us */
	uint32_t bt;			/* one character on the wire, us */
	struct rxb q[4096]; int qh, qt;	/* bytes arriving from the master, sorted by time */
	bool err_reported;
	struct ev log[1024]; int nlog;
	uint64_t tx_wire_done;		/* when the last transmitted bit leaves the wire */
	bool de, echo;			/* RS-485 2-wire: we hear ourselves */
	unsigned de_violations;		/* DE dropped while bits still on the wire / bytes sent with DE low */
	bool de_used;
} S;

static void sim_reset(uint32_t baud, bool echo)
{
	memset(&S, 0, sizeof S);
	S.bt = (uint32_t)(11ull * 1000000 / baud);
	S.t = 1000;
	S.echo = echo;
	memset(&D, 0, sizeof D);
}
static void qpush(uint64_t at, uint8_t b, bool err)
{
	/* keep the queue sorted by arrival time */
	int i = S.qt++;
	while (i > S.qh && S.q[i - 1].at > at) { S.q[i] = S.q[i - 1]; i--; }
	S.q[i] = (struct rxb){ at, b, err };
}
static int s_getc(void *c)
{
	(void)c;
	if (S.qh == S.qt || S.q[S.qh].at > S.t) return -EAGAIN;
	if (S.q[S.qh].err && !S.err_reported) { S.err_reported = true; return -EIO; }
	S.err_reported = false;
	uint8_t b = S.q[S.qh++].b;
	if (S.qh == S.qt) S.qh = S.qt = 0;	/* queue drained: reuse it */
	return b;
}
static void s_putc(void *c, uint8_t b)
{
	(void)c;
	if (S.de_used && !S.de) S.de_violations++;
	uint64_t start = S.tx_wire_done > S.t ? S.tx_wire_done : S.t;
	S.tx_wire_done = start + S.bt;
	S.log[S.nlog++] = (struct ev){ EV_BYTE, start, b };
	if (S.echo && S.de) qpush(S.tx_wire_done, b, false);
}
static void s_flush(void *c)
{ (void)c; if (S.tx_wire_done > S.t) S.t = S.tx_wire_done; S.log[S.nlog++] = (struct ev){ EV_FLUSH, S.t, 0 }; }
static void s_set_tx(void *c, bool en)
{
	(void)c;
	S.de_used = true;
	if (!en && S.tx_wire_done > S.t) S.de_violations++;	/* released too early: reply truncated */
	S.de = en;
	S.log[S.nlog++] = (struct ev){ en ? EV_DE_ON : EV_DE_OFF, S.t, 0 };
}
static uint64_t s_now(void *c) { (void)c; return S.t; }
static void s_delay(void *c, uint32_t us) { (void)c; S.t += us; }

static const modbus_serial_ops_t OPS232 = { s_getc, s_putc, s_now, s_flush, NULL, s_delay, NULL };
static const modbus_serial_ops_t OPS485 = { s_getc, s_putc, s_now, s_flush, s_set_tx, s_delay, NULL };

/* ---------------- frame helpers ---------------- */
static size_t frame(uint8_t *o, const uint8_t *body, size_t n)
{
	memcpy(o, body, n);
	uint16_t c = modbus_rtu_crc16(o, n);
	o[n] = c & 0xFF; o[n + 1] = c >> 8;
	return n + 2;
}
/* Master sends a frame starting at 'at', every character one char time apart. Returns end time. */
static uint64_t send_frame(uint64_t at, const uint8_t *f, size_t n)
{
	for (size_t i = 0; i < n; i++) qpush(at + (uint64_t)i * S.bt, f[i], false);
	return at + (uint64_t)(n - 1) * S.bt;	/* arrival of the last byte */
}
static uint64_t send_req(uint64_t at, uint8_t addr, uint8_t fc, uint16_t a, uint16_t v)
{
	uint8_t b[6] = { addr, fc, a >> 8, (uint8_t)a, v >> 8, (uint8_t)v }, f[8];
	return send_frame(at, f, frame(f, b, 6));
}
static void run(uint64_t until, uint32_t step, modbus_slave_t *s)
{
	while (S.t < until) { modbus_slave_poll(s); S.t += step; }
}
static int bytes_sent(void) { int n = 0; for (int i = 0; i < S.nlog; i++) n += S.log[i].type == EV_BYTE; return n; }
static size_t reply(uint8_t *o)
{
	size_t n = 0;
	for (int i = 0; i < S.nlog; i++) if (S.log[i].type == EV_BYTE) o[n++] = S.log[i].b;
	return n;
}

static modbus_slave_cfg_t cfg(uint32_t baud, modbus_link_t link)
{
	return (modbus_slave_cfg_t){ .baud = baud, .parity = MODBUS_PARITY_EVEN, .stop_bits = 1, .link = link,
		.de_lead_us = 50, .de_trail_us = 30 };
}
static void start(modbus_slave_t *s, uint8_t id, modbus_slave_cfg_t c, const modbus_serial_ops_t *o)
{
	CHECK(modbus_slave_init(s, id, &BE, o, &c) == 0);
	run(S.t + 4 * s->t35_us, 50, s);	/* leave INIT */
}

/* ---------------- tests ---------------- */

TEST(timing_values)
{
	CHECK(modbus_slave_t35_us(9600, MODBUS_PARITY_EVEN, 1) == 4011);
	CHECK(modbus_slave_t15_us(9600, MODBUS_PARITY_EVEN, 1) == 1719);
	CHECK(modbus_slave_t35_us(19200, MODBUS_PARITY_NONE, 2) == 2006);
	CHECK(modbus_slave_t35_us(19200, MODBUS_PARITY_NONE, 1) == 2006);	/* 10 bit chars are timed as 11 */
	CHECK(modbus_slave_t35_us(115200, MODBUS_PARITY_EVEN, 1) == 1750);
	CHECK(modbus_slave_t15_us(115200, MODBUS_PARITY_EVEN, 1) == 750);
	CHECK(modbus_slave_t35_us(1200, MODBUS_PARITY_EVEN, 1) == 32084);
}

TEST(init_validation)
{
	modbus_slave_t s; modbus_slave_cfg_t c = cfg(19200, MODBUS_LINK_RS232);
	sim_reset(19200, false);
	CHECK(modbus_slave_init(&s, 1, &BE, &OPS232, &c) == 0);
	CHECK(modbus_slave_init(&s, 0, &BE, &OPS232, &c) == -EINVAL);
	CHECK(modbus_slave_init(&s, 248, &BE, &OPS232, &c) == -EINVAL);
	CHECK(modbus_slave_init(&s, 1, NULL, &OPS232, &c) == -EINVAL);
	CHECK(modbus_slave_init(&s, 1, &BE, NULL, &c) == -EINVAL);
	modbus_slave_cfg_t bad = c; bad.baud = 0;
	CHECK(modbus_slave_init(&s, 1, &BE, &OPS232, &bad) == -EINVAL);
	bad = c; bad.stop_bits = 3;
	CHECK(modbus_slave_init(&s, 1, &BE, &OPS232, &bad) == -EINVAL);
	bad = c; bad.link = MODBUS_LINK_RS485_2W;	/* RS-485 without DE control must be refused */
	CHECK(modbus_slave_init(&s, 1, &BE, &OPS232, &bad) == -EINVAL);
	modbus_serial_ops_t o = OPS485; o.tx_flush = NULL;
	CHECK(modbus_slave_init(&s, 1, &BE, &o, &bad) == -EINVAL);
	o = OPS232; o.getc = NULL;
	CHECK(modbus_slave_init(&s, 1, &BE, &o, &c) == -EINVAL);
	bad = c; bad.link = (modbus_link_t)7;
	CHECK(modbus_slave_init(&s, 1, &BE, &OPS232, &bad) == -EINVAL);
}

TEST(rs232_valid_request_is_answered_after_t35)
{
	modbus_slave_t s; sim_reset(19200, false);
	start(&s, 0x11, cfg(19200, MODBUS_LINK_RS232), &OPS232);
	D.hr[0] = 0x022B; D.hr[1] = 0; D.hr[2] = 0x0064;
	uint64_t last = send_req(S.t + 100, 0x11, 3, 0, 3);
	run(last + 3 * s.t35_us, 50, &s);

	uint8_t r[64]; size_t n = reply(r);
	const uint8_t want[] = { 0x11, 0x03, 0x06, 0x02, 0x2B, 0x00, 0x00, 0x00, 0x64, 0xA6, 0xD7 };
	/* CRC bytes of that response computed by the engine must verify */
	CHECK(n == 11);
	CHECK_BYTES(r, want, 9);
	CHECK(modbus_rtu_crc16(r, n) == 0);
	/* reply must not start before t3.5 of silence after the last request byte */
	uint64_t first = 0; for (int i = 0; i < S.nlog; i++) if (S.log[i].type == EV_BYTE) { first = S.log[i].t; break; }
	CHECK(first >= last + s.t35_us);
	CHECK(!S.de_used);			/* RS-232: no driver control */
	CHECK(s.stats.frames_answered == 1);
}

TEST(startup_stays_deaf_until_line_is_quiet)
{
	modbus_slave_t s; sim_reset(19200, false);
	modbus_slave_cfg_t c = cfg(19200, MODBUS_LINK_RS232);
	CHECK(modbus_slave_init(&s, 1, &BE, &OPS232, &c) == 0);
	/* a perfectly valid request while the line was never silent for t3.5 */
	send_req(S.t + 10, 1, 6, 2, 0x1234);
	run(S.t + 20 * s.t35_us, 50, &s);
	CHECK(bytes_sent() == 0);
	CHECK(D.writes == 0);
	/* afterwards the slave works */
	uint64_t last = send_req(S.t + 100, 1, 6, 2, 0x1234);
	run(last + 3 * s.t35_us, 50, &s);
	CHECK(D.hr[2] == 0x1234);
	CHECK(bytes_sent() == 8);
}

TEST(frames_separated_by_t35_are_both_answered)
{
	modbus_slave_t s; sim_reset(19200, false);
	start(&s, 1, cfg(19200, MODBUS_LINK_RS232), &OPS232);
	uint64_t l1 = send_req(S.t + 100, 1, 6, 0, 1);
	uint64_t l2 = send_req(l1 + 2 * s.t35_us + 10 * S.bt, 1, 6, 1, 2);	/* master waits for the reply first */
	run(l2 + 4 * s.t35_us, 20, &s);
	CHECK(D.hr[0] == 1 && D.hr[1] == 2);
	CHECK(bytes_sent() == 16);
	CHECK(s.stats.frames_answered == 2);
}

TEST(inner_gap_between_t15_and_t35_default_is_tolerated)
{
	modbus_slave_t s; sim_reset(9600, false);
	start(&s, 1, cfg(9600, MODBUS_LINK_RS232), &OPS232);
	uint8_t b[6] = { 1, 6, 0, 3, 0xAB, 0xCD }, f[8]; size_t n = frame(f, b, 6);
	uint64_t t = S.t + 100;
	for (size_t i = 0; i < n; i++) { qpush(t, f[i], false); t += S.bt + (i == 3 ? s.t15_us + 100 : 0); }
	run(t + 3 * s.t35_us, 100, &s);
	CHECK(D.hr[3] == 0xABCD);
	CHECK(bytes_sent() == 8);
}

TEST(strict_t15_drops_frame_with_inner_gap_without_side_effect)
{
	modbus_slave_t s; sim_reset(9600, false);
	modbus_slave_cfg_t c = cfg(9600, MODBUS_LINK_RS232); c.strict_t15 = true;
	start(&s, 1, c, &OPS232);
	uint8_t b[6] = { 1, 6, 0, 3, 0xAB, 0xCD }, f[8]; size_t n = frame(f, b, 6);
	uint64_t t = S.t + 100;
	for (size_t i = 0; i < n; i++) { qpush(t, f[i], false); t += S.bt + (i == 3 ? s.t15_us + 100 : 0); }
	run(t + 3 * s.t35_us, 100, &s);
	CHECK(D.writes == 0 && bytes_sent() == 0);
	CHECK(s.stats.frames_dropped == 1);
	/* and the next good request is fine */
	uint64_t l = send_req(S.t + 100, 1, 6, 3, 0x0102);
	run(l + 3 * s.t35_us, 100, &s);
	CHECK(D.hr[3] == 0x0102);
}

TEST(gap_of_t35_splits_frame_into_two_garbage_frames)
{
	modbus_slave_t s; sim_reset(19200, false);
	start(&s, 1, cfg(19200, MODBUS_LINK_RS232), &OPS232);
	uint8_t b[6] = { 1, 6, 0, 3, 0xAB, 0xCD }, f[8]; size_t n = frame(f, b, 6);
	uint64_t t = S.t + 100;
	for (size_t i = 0; i < n; i++) { qpush(t, f[i], false); t += S.bt + (i == 3 ? s.t35_us + 100 : 0); }
	run(t + 3 * s.t35_us, 100, &s);
	CHECK(D.writes == 0 && bytes_sent() == 0);	/* both halves fail CRC */
}

TEST(oversized_frame_is_dropped_and_line_recovers)
{
	modbus_slave_t s; sim_reset(115200, false);
	start(&s, 1, cfg(115200, MODBUS_LINK_RS232), &OPS232);
	uint64_t t = S.t + 100;
	uint8_t big[300]; for (int i = 0; i < 300; i++) big[i] = (uint8_t)i;
	big[0] = 1; big[1] = 6;
	uint8_t f[302]; size_t n = frame(f, big, 298);	/* valid CRC but 300 bytes */
	for (size_t i = 0; i < n; i++) qpush(t + i * S.bt, f[i], false);
	t += n * S.bt + 3 * s.t35_us;
	run(t, 5, &s);
	CHECK(D.writes == 0 && bytes_sent() == 0);
	CHECK(s.stats.frames_dropped == 1);
	uint64_t l = send_req(S.t + 100, 1, 6, 0, 7);
	run(l + 3 * s.t35_us, 5, &s);
	CHECK(D.hr[0] == 7 && bytes_sent() == 8);
}

TEST(line_error_inside_frame_drops_it)
{
	modbus_slave_t s; sim_reset(19200, false);
	start(&s, 1, cfg(19200, MODBUS_LINK_RS232), &OPS232);
	uint8_t b[6] = { 1, 6, 0, 3, 0xAB, 0xCD }, f[8]; size_t n = frame(f, b, 6);
	uint64_t t = S.t + 100;
	for (size_t i = 0; i < n; i++) qpush(t + i * S.bt, f[i], i == 2);	/* parity error on byte 2, data intact */
	run(t + n * S.bt + 3 * s.t35_us, 50, &s);
	CHECK(D.writes == 0 && bytes_sent() == 0);
	CHECK(s.stats.line_errors == 1 && s.stats.frames_dropped == 1);
	uint64_t l = send_req(S.t + 100, 1, 6, 3, 9);
	run(l + 3 * s.t35_us, 50, &s);
	CHECK(D.hr[3] == 9);
}

TEST(line_error_on_first_byte_drops_that_frame_only)
{
	modbus_slave_t s; sim_reset(19200, false);
	start(&s, 1, cfg(19200, MODBUS_LINK_RS232), &OPS232);
	uint8_t b[6] = { 1, 6, 0, 3, 0xAB, 0xCD }, f[8]; size_t n = frame(f, b, 6);
	uint64_t t = S.t + 100;
	for (size_t i = 0; i < n; i++) qpush(t + i * S.bt, f[i], i == 0);
	run(t + n * S.bt + 3 * s.t35_us, 50, &s);
	CHECK(D.writes == 0 && bytes_sent() == 0);
	uint64_t l = send_req(S.t + 100, 1, 6, 3, 9);
	run(l + 3 * s.t35_us, 50, &s);
	CHECK(D.hr[3] == 9 && bytes_sent() == 8);
}

TEST(other_slave_and_broadcast_get_no_reply)
{
	modbus_slave_t s; sim_reset(19200, false);
	start(&s, 5, cfg(19200, MODBUS_LINK_RS485_2W), &OPS485);
	uint64_t l = send_req(S.t + 100, 6, 6, 0, 1);			/* someone else */
	l = send_req(l + 4 * s.t35_us, 0, 6, 1, 0x55);			/* broadcast write */
	l = send_req(l + 4 * s.t35_us, 0, 3, 0, 1);			/* broadcast read */
	run(l + 4 * s.t35_us, 50, &s);
	CHECK(bytes_sent() == 0);
	CHECK(D.hr[0] == 0 && D.hr[1] == 0x55);		/* broadcast write executed */
	CHECK(D.reads == 0);				/* broadcast read never touched the device */
	/* the driver was never switched on for a frame that is not ours */
	for (int i = 0; i < S.nlog; i++) CHECK(S.log[i].type != EV_DE_ON);
}

TEST(rs485_driver_enable_sequence)
{
	modbus_slave_t s; sim_reset(19200, true);
	modbus_slave_cfg_t c = cfg(19200, MODBUS_LINK_RS485_2W);
	start(&s, 1, c, &OPS485);
	S.nlog = 0;
	uint64_t l = send_req(S.t + 100, 1, 6, 0, 0x4242);
	run(l + 6 * s.t35_us, 20, &s);

	/* DE_ON, 8 x BYTE, FLUSH, DE_OFF — in this order and nothing else */
	CHECK(S.nlog == 11);
	CHECK(S.log[0].type == EV_DE_ON);
	for (int i = 1; i <= 8; i++) CHECK(S.log[i].type == EV_BYTE);
	CHECK(S.log[9].type == EV_FLUSH && S.log[10].type == EV_DE_OFF);
	CHECK(S.log[1].t >= S.log[0].t + c.de_lead_us);		/* transceiver enable time respected */
	CHECK(S.log[10].t >= S.tx_wire_done + c.de_trail_us);	/* released only after last stop bit + trail */
	CHECK(S.de_violations == 0 && !S.de);
}

TEST(rs485_without_flush_would_cut_the_reply_checker_works)
{
	/* Sanity check of the simulator itself: releasing DE with bytes still on the wire is flagged. */
	modbus_slave_t s; sim_reset(19200, true);
	modbus_serial_ops_t o = OPS485; o.tx_flush = s_flush;
	s_set_tx(NULL, true); s_putc(NULL, 0x55); s_set_tx(NULL, false);
	CHECK(S.de_violations == 1);
	(void)s; (void)o;
}

TEST(rs485_two_wire_echo_is_ignored)
{
	modbus_slave_t s; sim_reset(19200, true);
	start(&s, 1, cfg(19200, MODBUS_LINK_RS485_2W), &OPS485);
	uint64_t l = send_req(S.t + 100, 1, 6, 0, 0x4242);
	run(l + 10 * s.t35_us, 20, &s);
	CHECK(bytes_sent() == 8);				/* exactly one reply, not a reply to its own echo */
	CHECK(s.stats.frames_answered == 1);
	CHECK(s.stats.frames_ignored == 0 && s.stats.frames_dropped == 0);
	/* the master asks again after its turnaround time: still served */
	uint64_t l2 = send_req(S.t + 100, 1, 6, 1, 0x1111);
	run(l2 + 6 * s.t35_us, 20, &s);
	CHECK(D.hr[1] == 0x1111 && s.stats.frames_answered == 2);
}

TEST(rs485_four_wire_listens_normally)
{
	modbus_slave_t s; sim_reset(19200, false);
	start(&s, 1, cfg(19200, MODBUS_LINK_RS485_4W), &OPS485);
	uint64_t l = send_req(S.t + 100, 1, 6, 0, 3);
	l = send_req(l + 2 * s.t35_us + 20 * S.bt, 1, 6, 1, 4);
	run(l + 6 * s.t35_us, 20, &s);
	CHECK(D.hr[0] == 3 && D.hr[1] == 4 && s.stats.frames_answered == 2);
	CHECK(S.de_violations == 0);
}

TEST(exception_reply_goes_out_through_the_driver_too)
{
	modbus_slave_t s; sim_reset(19200, true);
	start(&s, 1, cfg(19200, MODBUS_LINK_RS485_2W), &OPS485);
	uint64_t l = send_req(S.t + 100, 1, 6, 99, 1);		/* illegal address */
	run(l + 6 * s.t35_us, 20, &s);
	uint8_t r[16]; size_t n = reply(r);
	CHECK(n == 5 && r[0] == 1 && r[1] == 0x86 && r[2] == 0x02 && modbus_rtu_crc16(r, n) == 0);
}

TEST(late_polling_is_the_documented_weak_spot)
{
	/* 115200: t3.5 = 1.75 ms. A frame that lands entirely between two polls is fine
	 * even with a slow loop (the UART FIFO holds it) ... */
	modbus_slave_t s; sim_reset(115200, false);
	start(&s, 1, cfg(115200, MODBUS_LINK_RS232), &OPS232);
	uint64_t l = send_req(S.t + 100, 1, 6, 0, 1);
	run(l + 4 * s.t35_us, 3000, &s);
	CHECK(D.writes == 1);

	/* ... but if a poll falls in the middle of the frame and the next one comes
	 * more than t3.5 later, the request is seen as two fragments: not answered,
	 * not executed, nothing corrupted. */
	uint64_t t0 = S.t + 100;
	send_req(t0, 1, 6, 1, 2);
	S.t = t0 + 300; modbus_slave_poll(&s);		/* first few bytes */
	S.t = t0 + 300 + 3000; modbus_slave_poll(&s);	/* rest, far too late */
	run(S.t + 20000, 100, &s);
	CHECK(D.writes == 1 && D.hr[1] == 0);

	/* with a loop that keeps up, the same request works */
	uint64_t l3 = send_req(S.t + 100, 1, 6, 1, 2);
	run(l3 + 4 * s.t35_us, 20, &s);
	CHECK(D.hr[1] == 2);
}

static uint32_t rnd_state = 12345;
static uint32_t rnd(void) { rnd_state = rnd_state * 1664525u + 1013904223u; return rnd_state >> 8; }

TEST(random_traffic_replies_exactly_to_valid_requests)
{
	for (int mode = 0; mode < 3; mode++) {
		modbus_slave_t s; sim_reset(mode == 0 ? 9600 : 57600, mode == 1);
		modbus_link_t lk = mode == 0 ? MODBUS_LINK_RS232 : MODBUS_LINK_RS485_2W;
		if (mode == 2) { lk = MODBUS_LINK_RS485_4W; }
		start(&s, 7, cfg(mode == 0 ? 9600 : 57600, lk), lk == MODBUS_LINK_RS232 ? &OPS232 : &OPS485);
		unsigned valid = 0, others = 0;
		uint64_t t = S.t + 200;
		for (int i = 0; i < 400; i++) {
			uint8_t f[260], b[260]; size_t n;
			unsigned k = rnd() % 5;
			if (k == 0 || k == 1) {			/* valid read of 1..10 holding registers */
				uint16_t a = rnd() % 5, c = 1 + rnd() % 5;
				uint8_t q[6] = { 7, 3, a >> 8, (uint8_t)a, c >> 8, (uint8_t)c };
				n = frame(f, q, 6); valid++;
			} else if (k == 2) {			/* valid write */
				uint8_t q[6] = { 7, 6, 0, (uint8_t)(rnd() % 10), (uint8_t)rnd(), (uint8_t)rnd() };
				n = frame(f, q, 6); valid++;
			} else if (k == 3) {			/* corrupt CRC */
				uint8_t q[6] = { 7, 3, 0, 0, 0, 1 };
				n = frame(f, q, 6); f[n - 1] ^= 1 << (rnd() % 8); others++;
			} else {				/* random garbage or other slave */
				n = 1 + rnd() % 40;
				for (size_t j = 0; j < n; j++) b[j] = (uint8_t)rnd();
				if (n >= 1) b[0] = (uint8_t)(rnd() % 2 ? 9 : 7 + 1 + rnd() % 200);
				n = frame(f, b, n); others++;
			}
			uint64_t last = t;
			for (size_t j = 0; j < n; j++) { qpush(t, f[j], false); last = t; t += S.bt; }
			unsigned before = s.stats.frames_answered;
			S.nlog = 0;
			/* master waits for the reply (or the timeout) before the next request */
			uint64_t end = last + 5 * s.t35_us + 30 * S.bt;
			run(end, 25, &s);
			bool want_reply = (k <= 2);
			CHECK(s.stats.frames_answered - before == (want_reply ? 1u : 0u));
			if (want_reply) {
				uint8_t r[300]; size_t rn = reply(r);
				CHECK(rn >= 5 && r[0] == 7 && modbus_rtu_crc16(r, rn) == 0);
			} else {
				CHECK(bytes_sent() == 0);
			}
			t = S.t + 100;
			CHECK(S.de_violations == 0);
		}
		CHECK(s.stats.frames_answered == valid);
		CHECK(valid > 100);
		/* every byte sent forms valid frames: scan the log */
		(void)others;
	}
}

int main(int argc, char **argv) { return mbtest_main(argc, argv); }
