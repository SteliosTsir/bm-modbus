/*
 * Host tests for the Modbus PDU and RTU-frame layers.
 *
 * The expected bytes come from the Modbus Application Protocol spec V1.1b3
 * (function examples), the Modbus over Serial Line spec V1.02 and the widely
 * published Modicon guide example (11 03 00 6B 00 03 76 87).
 * They are checked against the implementation, never derived from it.
 */
#include "mbtest.h"
#include "api.h"

/* ------------------------------------------------------------------ */
/* A small device model behind the backend callbacks                   */
/* ------------------------------------------------------------------ */

#define NC 2048		/* coils and discrete inputs */
#define NR 256		/* registers */

static struct {
	uint8_t  coil[NC], di[NC];
	uint16_t hr[NR], ir[NR];
	unsigned read_calls, write_calls;
	unsigned violations;	/* backend called with arguments the spec forbids */
	int      force_ex;	/* non-zero: every callback returns this */
} M;

static bool bit(const uint8_t *src, unsigned i) { return (src[i / 8] >> (i % 8)) & 1u; }

static modbus_exception_t rd_bits(const uint8_t *s, uint16_t a, uint16_t n, uint8_t *d)
{
	M.read_calls++;
	if (n < 1 || n > 2000) M.violations++;
	if (M.force_ex) return (modbus_exception_t)M.force_ex;
	if ((unsigned)a + n > NC) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	memset(d, 0, (n + 7u) / 8u);
	for (unsigned i = 0; i < n; i++) if (s[a + i]) d[i / 8] |= (uint8_t)(1u << (i % 8));
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_rc(uint16_t a, uint16_t n, uint8_t *d) { return rd_bits(M.coil, a, n, d); }
static modbus_exception_t be_rd(uint16_t a, uint16_t n, uint8_t *d) { return rd_bits(M.di, a, n, d); }
static modbus_exception_t be_rh(uint16_t a, uint16_t n, uint16_t *d)
{
	M.read_calls++;
	if (n < 1 || n > 125) M.violations++;
	if (M.force_ex) return (modbus_exception_t)M.force_ex;
	if ((unsigned)a + n > NR) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (unsigned i = 0; i < n; i++) d[i] = M.hr[a + i];
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_ri(uint16_t a, uint16_t n, uint16_t *d)
{
	M.read_calls++;
	if (n < 1 || n > 125) M.violations++;
	if (M.force_ex) return (modbus_exception_t)M.force_ex;
	if ((unsigned)a + n > NR) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (unsigned i = 0; i < n; i++) d[i] = M.ir[a + i];
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_wc(uint16_t a, bool s)
{
	M.write_calls++;
	if (M.force_ex) return (modbus_exception_t)M.force_ex;
	if (a >= NC) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	M.coil[a] = s;
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_wh(uint16_t a, uint16_t v)
{
	M.write_calls++;
	if (M.force_ex) return (modbus_exception_t)M.force_ex;
	if (a >= NR) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	M.hr[a] = v;
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_wcs(uint16_t a, uint16_t n, const uint8_t *s)
{
	M.write_calls++;
	if (n < 1 || n > 1968) M.violations++;
	if (M.force_ex) return (modbus_exception_t)M.force_ex;
	if ((unsigned)a + n > NC) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (unsigned i = 0; i < n; i++) M.coil[a + i] = bit(s, i);
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_whs(uint16_t a, uint16_t n, const uint16_t *s)
{
	M.write_calls++;
	if (n < 1 || n > 123) M.violations++;
	if (M.force_ex) return (modbus_exception_t)M.force_ex;
	if ((unsigned)a + n > NR) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (unsigned i = 0; i < n; i++) M.hr[a + i] = s[i];
	return MODBUS_EX_NONE;
}

static const modbus_backend_t BE = {
	.read_coils = be_rc, .read_discrete_inputs = be_rd,
	.read_multi_holding_registers = be_rh, .read_input_registers = be_ri,
	.write_coil = be_wc, .write_single_holding_register = be_wh,
	.write_multi_coils = be_wcs, .write_multi_holding_registers = be_whs,
};

static void model_reset(void) { memset(&M, 0, sizeof M); }

/* Runs a request through the PDU layer with exact-size heap buffers, so
 * ASan flags any read or write outside what the caller handed over. */
static size_t pdu(const uint8_t *req, size_t rlen, uint8_t *out, size_t outmax)
{
	uint8_t *rq = malloc(rlen ? rlen : 1), *rs = malloc(outmax ? outmax : 1);
	memcpy(rq, req, rlen);
	size_t n = modbus_process_pdu(&BE, rq, rlen, rs, outmax);
	CHECK(n <= outmax);
	memcpy(out, rs, n);
	free(rq); free(rs);
	return n;
}
#define PDU(req, resp, nresp) do { uint8_t o_[256]; size_t n_ = pdu(req, sizeof(req), o_, 253); \
	CHECK(n_ == sizeof(resp)); CHECK_BYTES(o_, resp, sizeof(resp)); (void)nresp; } while (0)

static void pdu_expect_ex(const uint8_t *req, size_t rlen, uint8_t ex)
{
	uint8_t o[256];
	size_t n = pdu(req, rlen, o, 253);
	if (!(n == 2 && o[0] == (uint8_t)(req[0] | 0x80) && o[1] == ex))
		fprintf(stderr, "CHECK failed %s:%d: FC %02X len %zu expected exception %02X, got %zu bytes: %02X %02X\n",
			__FILE__, __LINE__, req[0], rlen, ex, n, n > 0 ? o[0] : 0, n > 1 ? o[1] : 0);
	CHECK(n == 2 && o[0] == (uint8_t)(req[0] | 0x80) && o[1] == ex);
}

/* ------------------------------------------------------------------ */
/* CRC                                                                 */
/* ------------------------------------------------------------------ */

static uint16_t crc_ref(const uint8_t *b, size_t n)
{
	uint16_t c = 0xFFFF;
	for (size_t i = 0; i < n; i++) {
		c ^= b[i];
		for (int k = 0; k < 8; k++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
	}
	return c;
}

TEST(crc_published_vectors)
{
	const uint8_t a[] = { 0x11, 0x03, 0x00, 0x6B, 0x00, 0x03 };	/* -> 76 87 */
	const uint8_t b[] = { 0x11, 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40 };	/* -> 49 AD */
	const uint8_t c[] = { 0x01, 0x07 };				/* -> 41 E2 */
	CHECK(modbus_rtu_crc16(a, sizeof a) == 0x8776);	/* low byte 76 goes first */
	CHECK(modbus_rtu_crc16(b, sizeof b) == 0xAD49);
	CHECK(modbus_rtu_crc16(c, sizeof c) == 0xE241);
	CHECK(modbus_rtu_crc16(a, 0) == 0xFFFF);
}

TEST(crc_table_matches_bitwise_reference)
{
	srand(1);
	uint8_t buf[300];
	for (int it = 0; it < 20000; it++) {
		size_t n = (size_t)(rand() % 300);
		for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)rand();
		CHECK(modbus_rtu_crc16(buf, n) == crc_ref(buf, n));
	}
}

/* ------------------------------------------------------------------ */
/* Function code examples from the Modbus Application Protocol spec   */
/* ------------------------------------------------------------------ */

TEST(spec_fc01_read_coils)	/* coils 20-38 */
{
	model_reset();
	/* Spec: CD 6B 05 -> LSB-first, coil 20 is bit 0 of the first byte */
	const uint8_t bits[3] = { 0xCD, 0x6B, 0x05 };
	for (int i = 0; i < 19; i++) M.coil[19 + i] = bit(bits, i);
	const uint8_t rq[] = { 0x01, 0x00, 0x13, 0x00, 0x13 }, rs[] = { 0x01, 0x03, 0xCD, 0x6B, 0x05 };
	PDU(rq, rs, 0);
}

TEST(spec_fc02_read_discrete_inputs)	/* inputs 197-218 */
{
	model_reset();
	const uint8_t bits[3] = { 0xAC, 0xDB, 0x35 };
	for (int i = 0; i < 22; i++) M.di[196 + i] = bit(bits, i);
	const uint8_t rq[] = { 0x02, 0x00, 0xC4, 0x00, 0x16 }, rs[] = { 0x02, 0x03, 0xAC, 0xDB, 0x35 };
	PDU(rq, rs, 0);
}

TEST(spec_fc03_read_holding_registers)
{
	model_reset();
	M.hr[0x6B] = 0x022B; M.hr[0x6C] = 0x0000; M.hr[0x6D] = 0x0064;
	const uint8_t rq[] = { 0x03, 0x00, 0x6B, 0x00, 0x03 };
	const uint8_t rs[] = { 0x03, 0x06, 0x02, 0x2B, 0x00, 0x00, 0x00, 0x64 };
	PDU(rq, rs, 0);
}

TEST(spec_fc04_read_input_registers)
{
	model_reset();
	M.ir[8] = 0x000A;
	const uint8_t rq[] = { 0x04, 0x00, 0x08, 0x00, 0x01 }, rs[] = { 0x04, 0x02, 0x00, 0x0A };
	PDU(rq, rs, 0);
}

TEST(spec_fc05_write_single_coil)
{
	model_reset();
	const uint8_t rq[] = { 0x05, 0x00, 0xAC, 0xFF, 0x00 };
	PDU(rq, rq, 0);			/* response echoes the request */
	CHECK(M.coil[0xAC] == 1);
	const uint8_t off[] = { 0x05, 0x00, 0xAC, 0x00, 0x00 };
	PDU(off, off, 0);
	CHECK(M.coil[0xAC] == 0);
}

TEST(spec_fc06_write_single_register)
{
	model_reset();
	const uint8_t rq[] = { 0x06, 0x00, 0x01, 0x00, 0x03 };
	PDU(rq, rq, 0);
	CHECK(M.hr[1] == 3);
}

TEST(spec_fc0f_write_multiple_coils)	/* 10 coils from address 19 */
{
	model_reset();
	const uint8_t rq[] = { 0x0F, 0x00, 0x13, 0x00, 0x0A, 0x02, 0xCD, 0x01 };
	const uint8_t rs[] = { 0x0F, 0x00, 0x13, 0x00, 0x0A };
	PDU(rq, rs, 0);
	const uint8_t want[10] = { 1, 0, 1, 1, 0, 0, 1, 1, 1, 0 };	/* 0xCD LSB first, then 0x01 -> bit 8=1, bit 9=0 */
	CHECK_BYTES(&M.coil[19], want, 10);
	CHECK(M.coil[18] == 0 && M.coil[29] == 0);
}

TEST(spec_fc10_write_multiple_registers)
{
	model_reset();
	const uint8_t rq[] = { 0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02 };
	const uint8_t rs[] = { 0x10, 0x00, 0x01, 0x00, 0x02 };
	PDU(rq, rs, 0);
	CHECK(M.hr[1] == 0x000A && M.hr[2] == 0x0102 && M.hr[0] == 0 && M.hr[3] == 0);
}

/* ------------------------------------------------------------------ */
/* Exceptions                                                          */
/* ------------------------------------------------------------------ */

TEST(exc_unsupported_function_codes)
{
	model_reset();
	for (int fc = 0; fc < 256; fc++) {
		if ((fc >= 1 && fc <= 6) || fc == 0x0F || fc == 0x10) continue;
		uint8_t rq[5] = { (uint8_t)fc, 0, 0, 0, 1 };
		pdu_expect_ex(rq, sizeof rq, 0x01);
	}
	CHECK(M.read_calls == 0 && M.write_calls == 0);
}

TEST(exc_quantity_out_of_range_is_illegal_data_value)
{
	model_reset();
	/* { fc, count } : count 0 and max+1 must give exception 03, backend untouched */
	static const struct { uint8_t fc; uint16_t max; } t[] = {
		{ 0x01, 2000 }, { 0x02, 2000 }, { 0x03, 125 }, { 0x04, 125 } };
	for (unsigned i = 0; i < 4; i++) {
		uint8_t z[5]  = { t[i].fc, 0, 0, 0, 0 };
		uint8_t hi[5] = { t[i].fc, 0, 0, (uint8_t)((t[i].max + 1) >> 8), (uint8_t)(t[i].max + 1) };
		pdu_expect_ex(z, 5, 0x03);
		pdu_expect_ex(hi, 5, 0x03);
		uint8_t ffff[5] = { t[i].fc, 0, 0, 0xFF, 0xFF };
		pdu_expect_ex(ffff, 5, 0x03);
	}
	/* write multiple coils: 1..1968 */
	uint8_t wc0[6] = { 0x0F, 0, 0, 0, 0, 0 };
	pdu_expect_ex(wc0, 6, 0x03);
	uint8_t wc_hi[6 + 247]; memset(wc_hi, 0, sizeof wc_hi);	/* 1969 coils: one past the limit */
	wc_hi[0] = 0x0F; wc_hi[3] = (uint8_t)(1969 >> 8); wc_hi[4] = (uint8_t)(1969 & 0xFF); wc_hi[5] = 247;
	pdu_expect_ex(wc_hi, sizeof wc_hi, 0x03);
	/* write multiple registers: 1..123 */
	uint8_t wr0[6] = { 0x10, 0, 0, 0, 0, 0 };
	pdu_expect_ex(wr0, 6, 0x03);
	uint8_t wr124[6 + 248]; memset(wr124, 0, sizeof wr124);
	wr124[0] = 0x10; wr124[4] = 124; wr124[5] = 248;
	pdu_expect_ex(wr124, sizeof wr124, 0x03);
	CHECK(M.read_calls == 0 && M.write_calls == 0);
	CHECK(M.violations == 0);
}

TEST(exc_maximum_quantities_are_accepted)
{
	model_reset();
	uint8_t o[256];
	uint8_t c2000[5] = { 0x01, 0, 0, 0x07, 0xD0 };
	CHECK(pdu(c2000, 5, o, 253) == 2 + 250 && o[0] == 0x01 && o[1] == 250);
	uint8_t r125[5] = { 0x03, 0, 0, 0, 125 };
	CHECK(pdu(r125, 5, o, 253) == 2 + 250 && o[0] == 0x03 && o[1] == 250);
	uint8_t w123[6 + 246]; memset(w123, 0, sizeof w123);
	w123[0] = 0x10; w123[4] = 123; w123[5] = 246;
	CHECK(pdu(w123, sizeof w123, o, 253) == 5 && o[0] == 0x10);
	uint8_t w1968[6 + 246]; memset(w1968, 0xFF, sizeof w1968);
	w1968[0] = 0x0F; w1968[1] = 0; w1968[2] = 0; w1968[3] = 0x07; w1968[4] = 0xB0; w1968[5] = 246;
	CHECK(pdu(w1968, sizeof w1968, o, 253) == 5 && o[0] == 0x0F);
	CHECK(M.violations == 0);
}

TEST(exc_data_address_out_of_range)
{
	model_reset();
	uint8_t rc[5] = { 0x01, (NC - 4) >> 8, (NC - 4) & 0xFF, 0, 5 };	/* last 4 valid, asks 5 */
	pdu_expect_ex(rc, 5, 0x02);
	uint8_t rh[5] = { 0x03, 0, NR - 1, 0, 2 };
	pdu_expect_ex(rh, 5, 0x02);
	uint8_t wh[5] = { 0x06, 0x10, 0x00, 0, 1 };
	pdu_expect_ex(wh, 5, 0x02);
	uint8_t wc[5] = { 0x05, 0xFF, 0xFF, 0xFF, 0x00 };
	pdu_expect_ex(wc, 5, 0x02);
	/* start + count overflowing 16 bits must not wrap into a valid range */
	uint8_t wrap[5] = { 0x03, 0xFF, 0xFF, 0, 2 };
	pdu_expect_ex(wrap, 5, 0x02);
}

TEST(exc_fc05_value_must_be_0000_or_ff00)
{
	model_reset();
	for (unsigned v = 1; v < 0x10000; v += 257) {
		if (v == 0xFF00) continue;
		uint8_t rq[5] = { 0x05, 0, 1, (uint8_t)(v >> 8), (uint8_t)v };
		pdu_expect_ex(rq, 5, 0x03);
	}
	uint8_t rq[5] = { 0x05, 0, 1, 0xFF, 0x01 };
	pdu_expect_ex(rq, 5, 0x03);
	CHECK(M.write_calls == 0 && M.coil[1] == 0);
}

TEST(exc_wrong_request_length_is_illegal_data_value)
{
	model_reset();
	for (int fc = 1; fc <= 6; fc++) {
		for (size_t len = 1; len <= 8; len++) {
			if (len == 5) continue;
			uint8_t rq[8] = { (uint8_t)fc, 0, 0, 0, 1, 0, 0, 0 };
			pdu_expect_ex(rq, len, 0x03);
		}
	}
	/* FC 0F / 10: byte count inconsistent with quantity, or with the real length */
	uint8_t a[] = { 0x10, 0, 1, 0, 2, 3, 0, 1, 0, 2 };		/* byte count 3, should be 4 */
	pdu_expect_ex(a, sizeof a, 0x03);
	uint8_t b[] = { 0x10, 0, 1, 0, 2, 4, 0, 1, 0 };			/* declares 4 bytes, carries 3 */
	pdu_expect_ex(b, sizeof b, 0x03);
	uint8_t c[] = { 0x10, 0, 1, 0, 2, 4, 0, 1, 0, 2, 9 };		/* one byte too many */
	pdu_expect_ex(c, sizeof c, 0x03);
	uint8_t d[] = { 0x0F, 0, 1, 0, 9, 1, 0xFF };			/* 9 coils need 2 bytes */
	pdu_expect_ex(d, sizeof d, 0x03);
	uint8_t e[] = { 0x0F, 0, 1, 0, 8, 1 };				/* byte count but no data */
	pdu_expect_ex(e, sizeof e, 0x03);
	CHECK(M.write_calls == 0 && M.read_calls == 0);
}

TEST(exc_backend_failure_is_reported_as_04)
{
	model_reset();
	M.force_ex = MODBUS_EX_SLAVE_DEVICE_FAIL;
	uint8_t rq[5] = { 0x03, 0, 0, 0, 1 };
	pdu_expect_ex(rq, 5, 0x04);
	uint8_t w[5] = { 0x06, 0, 0, 0, 1 };
	pdu_expect_ex(w, 5, 0x04);
}

TEST(exc_missing_callback_is_illegal_function)
{
	modbus_backend_t empty;
	memset(&empty, 0, sizeof empty);
	static const uint8_t reqs[][8] = {
		{ 1, 0, 0, 0, 1 }, { 2, 0, 0, 0, 1 }, { 3, 0, 0, 0, 1 }, { 4, 0, 0, 0, 1 },
		{ 5, 0, 0, 0xFF, 0 }, { 6, 0, 0, 0, 1 },
		{ 0x0F, 0, 0, 0, 1, 1, 1 }, { 0x10, 0, 0, 0, 1, 2, 0, 1 } };
	static const size_t lens[] = { 5, 5, 5, 5, 5, 5, 7, 8 };
	for (int i = 0; i < 8; i++) {
		uint8_t o[256];
		size_t n = modbus_process_pdu(&empty, reqs[i], lens[i], o, 253);
		CHECK(n == 2 && o[0] == (uint8_t)(reqs[i][0] | 0x80) && o[1] == 0x01);
	}
}

/* ------------------------------------------------------------------ */
/* Memory safety                                                       */
/* ------------------------------------------------------------------ */

TEST(pdu_small_response_buffer_never_overflows)
{
	model_reset();
	/* The caller says how much room there is. A read that does not fit must be
	 * refused, not written past the end of the buffer. */
	static const uint8_t reqs[][5] = {
		{ 0x01, 0, 0, 0x07, 0xD0 }, { 0x02, 0, 0, 0x07, 0xD0 },
		{ 0x03, 0, 0, 0, 125 }, { 0x04, 0, 0, 0, 125 }, { 0x03, 0, 0, 0, 20 }, { 0x01, 0, 0, 0, 64 } };
	for (unsigned r = 0; r < 6; r++)
		for (size_t max = 2; max <= 12; max++) {
			uint8_t o[16];
			(void)pdu(reqs[r], 5, o, max);
		}
}

TEST(pdu_degenerate_arguments)
{
	uint8_t o[16], rq[5] = { 3, 0, 0, 0, 1 };
	CHECK(modbus_process_pdu(NULL, rq, 5, o, 16) == 0);
	CHECK(modbus_process_pdu(&BE, NULL, 5, o, 16) == 0);
	CHECK(modbus_process_pdu(&BE, rq, 5, NULL, 16) == 0);
	CHECK(modbus_process_pdu(&BE, rq, 0, o, 16) == 0);
	CHECK(modbus_process_pdu(&BE, rq, 5, o, 0) == 0);
	CHECK(modbus_process_pdu(&BE, rq, 5, o, 1) == 0);
}

TEST(pdu_random_requests_keep_invariants)
{
	srand(42);
	static const uint8_t fcs[] = { 1, 2, 3, 4, 5, 6, 0x0F, 0x10, 0, 7, 0x2B, 0x80, 0xFF };
	model_reset();
	for (int i = 0; i < NR; i++) { M.hr[i] = (uint16_t)rand(); M.ir[i] = (uint16_t)rand(); }
	for (int i = 0; i < NC; i++) { M.coil[i] = rand() & 1; M.di[i] = rand() & 1; }
	for (int it = 0; it < 300000; it++) {
		M.read_calls = M.write_calls = M.violations = 0;
		uint8_t rq[260];
		size_t len = (size_t)(rand() % 40);
		if (rand() % 8 == 0) len = (size_t)(rand() % 254);
		for (size_t i = 0; i < len; i++) rq[i] = (uint8_t)rand();
		if (len) rq[0] = (rand() % 10) ? fcs[rand() % 8] : fcs[rand() % 13];
		/* make a good share of them structurally plausible */
		if (len >= 5 && rand() % 2) { rq[3] = 0; rq[4] = (uint8_t)(1 + rand() % 130); rq[1] = 0; rq[2] = (uint8_t)(rand() % 250); }
		if (len >= 6 && (rq[0] == 0x10 || rq[0] == 0x0F) && rand() % 2) {
			unsigned n = rq[4], bytes = rq[0] == 0x10 ? 2u * n : (n + 7u) / 8u;
			rq[5] = (uint8_t)bytes; len = 6 + bytes > 253 ? 253 : 6 + bytes;
		}
		uint8_t o[256];
		size_t n = pdu(rq, len, o, 253);
		CHECK(M.violations == 0);
		if (len == 0) { CHECK(n == 0); continue; }
		CHECK(n >= 2);
		if (o[0] & 0x80) {
			CHECK(n == 2 && o[0] == (uint8_t)(rq[0] | 0x80) && o[1] >= 1 && o[1] <= 4);
			/* exceptions 01 and 03 are decided before the device is touched */
			if (o[1] == 1 || o[1] == 3) CHECK(M.read_calls == 0 && M.write_calls == 0);
		} else {
			CHECK(o[0] == rq[0]);
			switch (rq[0]) {
			case 1: case 2: CHECK(n == 2u + o[1] && o[1] == (uint8_t)((((rq[3] << 8) | rq[4]) + 7) / 8)); break;
			case 3: case 4: CHECK(n == 2u + o[1] && o[1] == 2 * ((rq[3] << 8) | rq[4])); break;
			case 5: case 6: CHECK(n == 5 && !memcmp(o, rq, 5)); break;
			case 0x0F: case 0x10: CHECK(n == 5 && !memcmp(&o[1], &rq[1], 4)); break;
			default: CHECK(0);
			}
		}
	}
}

/* ------------------------------------------------------------------ */
/* RTU frame layer                                                     */
/* ------------------------------------------------------------------ */

static size_t add_crc(uint8_t *f, size_t n)
{
	uint16_t c = crc_ref(f, n);
	f[n] = (uint8_t)c; f[n + 1] = (uint8_t)(c >> 8);
	return n + 2;
}

static size_t rtu(const modbus_rtu_t *r, const uint8_t *rx, size_t rxn, uint8_t *tx, size_t txmax)
{
	uint8_t *a = malloc(rxn ? rxn : 1), *b = malloc(txmax ? txmax : 1);
	memcpy(a, rx, rxn);
	size_t n = modbus_rtu_process_frame(r, a, rxn, b, txmax);
	CHECK(n <= txmax);
	memcpy(tx, b, n);
	free(a); free(b);
	return n;
}

TEST(rtu_modicon_guide_example)
{
	model_reset();
	/* Slave 17 (0x11): read holding registers 107..109 (40108-40110) */
	M.hr[0x6B] = 0xAE41; M.hr[0x6C] = 0x5652; M.hr[0x6D] = 0x4340;
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 0x11, &BE) == 0);
	const uint8_t rq[] = { 0x11, 0x03, 0x00, 0x6B, 0x00, 0x03, 0x76, 0x87 };
	const uint8_t rs[] = { 0x11, 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x49, 0xAD };
	uint8_t o[256];
	size_t n = rtu(&r, rq, sizeof rq, o, 256);
	CHECK(n == sizeof rs);
	CHECK_BYTES(o, rs, sizeof rs);
}

TEST(rtu_exception_response_frame)
{
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 1, &BE) == 0);
	uint8_t rq[8] = { 0x01, 0x2B, 0x00, 0x00 }, o[256];
	size_t n = add_crc(rq, 4);
	n = rtu(&r, rq, n, o, 256);
	const uint8_t want[] = { 0x01, 0xAB, 0x01, 0, 0 };
	uint8_t w[5]; memcpy(w, want, 3); add_crc(w, 3);
	CHECK(n == 5);
	CHECK_BYTES(o, w, 5);
}

TEST(rtu_any_single_bit_error_is_dropped_without_side_effects)
{
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 5, &BE) == 0);
	uint8_t f[16] = { 0x05, 0x06, 0x00, 0x03, 0x12, 0x34 };	/* write register 3 */
	size_t n = add_crc(f, 6);
	uint8_t o[256];
	for (size_t byte = 0; byte < n; byte++)
		for (int b = 0; b < 8; b++) {
			uint8_t g[16]; memcpy(g, f, n);
			g[byte] ^= (uint8_t)(1u << b);
			size_t got = rtu(&r, g, n, o, 256);
			/* flipping an address bit sends it to another slave; either way no write */
			CHECK(got == 0);
			CHECK(M.write_calls == 0);
		}
	CHECK(rtu(&r, f, n, o, 256) == n);		/* the intact frame still works */
	CHECK(M.hr[3] == 0x1234);
}

TEST(rtu_address_filtering)
{
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 7, &BE) == 0);
	for (int a = 0; a < 256; a++) {
		uint8_t f[16] = { (uint8_t)a, 0x03, 0x00, 0x00, 0x00, 0x01 }, o[256];
		size_t n = add_crc(f, 6);
		size_t got = rtu(&r, f, n, o, 256);
		if (a == 7) CHECK(got == 7 && o[0] == 7);
		else CHECK(got == 0);
	}
}

TEST(rtu_broadcast_write_is_executed_but_never_answered)
{
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 9, &BE) == 0);
	uint8_t f[16] = { 0x00, 0x06, 0x00, 0x02, 0xBE, 0xEF }, o[256];
	size_t n = add_crc(f, 6);
	CHECK(rtu(&r, f, n, o, 256) == 0);
	CHECK(M.hr[2] == 0xBEEF);
	uint8_t g[16] = { 0x00, 0x05, 0x00, 0x04, 0xFF, 0x00 };
	n = add_crc(g, 6);
	CHECK(rtu(&r, g, n, o, 256) == 0);
	CHECK(M.coil[4] == 1);
}

TEST(rtu_broadcast_read_is_ignored)
{
	/* Spec: broadcasts are necessarily writing commands. A read has no meaning,
	 * so the device should not touch its hardware for it. */
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 9, &BE) == 0);
	uint8_t f[16] = { 0x00, 0x03, 0x00, 0x00, 0x00, 0x01 }, o[256];
	size_t n = add_crc(f, 6);
	CHECK(rtu(&r, f, n, o, 256) == 0);
	CHECK(M.read_calls == 0);
}

TEST(rtu_frame_length_limits)
{
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 1, &BE) == 0);
	uint8_t f[300], o[300];
	for (size_t n = 0; n <= 3; n++) {
		memset(f, 1, sizeof f);
		CHECK(rtu(&r, f, n, o, 256) == 0);
	}
	/* Spec: the maximum RTU frame is 256 bytes. A 256-byte frame with an unknown
	 * function code is answered with exception 01, a 257-byte one is ignored. */
	for (size_t n = 256; n <= 258; n++) {
		memset(f, 0x5A, sizeof f);
		f[0] = 1; f[1] = 0x41;
		add_crc(f, n - 2);
		size_t got = rtu(&r, f, n, o, 256);
		if (n == 256) { CHECK(got == 5 && o[1] == 0xC1 && o[2] == 0x01); }
		else CHECK(got == 0);
	}
}

TEST(rtu_small_transmit_buffer_never_overflows)
{
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 1, &BE) == 0);
	static const uint8_t pdus[][5] = { { 3, 0, 0, 0, 125 }, { 1, 0, 0, 7, 0xD0 }, { 4, 0, 0, 0, 10 } };
	for (unsigned p = 0; p < 3; p++)
		for (size_t max = 0; max <= 16; max++) {
			uint8_t f[16] = { 1 }; memcpy(&f[1], pdus[p], 5);
			size_t n = add_crc(f, 6), got;
			uint8_t o[16];
			got = rtu(&r, f, n, o, max);
			CHECK(got <= max);
		}
}

TEST(rtu_degenerate_arguments)
{
	model_reset();
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 1, &BE) == 0);
	uint8_t f[16] = { 1, 3, 0, 0, 0, 1 }, o[64];
	size_t n = add_crc(f, 6);
	CHECK(modbus_rtu_process_frame(NULL, f, n, o, 64) == 0);
	CHECK(modbus_rtu_process_frame(&r, NULL, n, o, 64) == 0);
	CHECK(modbus_rtu_process_frame(&r, f, n, NULL, 64) == 0);
	modbus_rtu_t nb = { 1, NULL };
	CHECK(modbus_rtu_process_frame(&nb, f, n, o, 64) == 0);
}

TEST(rtu_init_rejects_reserved_addresses)
{
	/* Spec: 0 is broadcast, 1..247 are slaves, 248..255 are reserved. */
	modbus_rtu_t r;
	CHECK(mb_rtu_init(&r, 1, &BE) == 0);
	CHECK(mb_rtu_init(&r, 247, &BE) == 0);
	CHECK(mb_rtu_init(&r, 0, &BE) != 0);
	for (int a = 248; a < 256; a++) CHECK(mb_rtu_init(&r, (uint8_t)a, &BE) != 0);
	CHECK(mb_rtu_init(&r, 1, NULL) != 0);
}

TEST(rtu_random_frames_keep_invariants)
{
	srand(7);
	modbus_rtu_t r; CHECK(mb_rtu_init(&r, 3, &BE) == 0);
	for (int it = 0; it < 200000; it++) {
		model_reset();
		uint8_t f[260];
		size_t len = (size_t)(rand() % 40);
		for (size_t i = 0; i < len; i++) f[i] = (uint8_t)rand();
		if (len >= 4 && rand() % 3) { f[0] = (rand() % 3) ? 3 : 0; len = add_crc(f, len - 2); }
		uint8_t o[256];
		size_t n = rtu(&r, f, len, o, 256);
		CHECK(M.violations == 0);
		if (n) {
			CHECK(n >= 5 && o[0] == 3);
			CHECK(modbus_rtu_crc16(o, n) == 0);	/* a valid frame has a zero residue */
		}
	}
}

TEST(t35_follows_the_spec)
{
	/* <= 19200 Bd: 3.5 * 11 bit times; above: fixed 1750 us (spec: 1.750 ms) */
	CHECK(modbus_rtu_t35_us(1200) == 32084);	/* 38.5 / 1200 s = 32083.3 us, rounded up */
	CHECK(modbus_rtu_t35_us(9600) == 4011);
	CHECK(modbus_rtu_t35_us(19200) == 2006);
	CHECK(modbus_rtu_t35_us(19201) == 1750);
	CHECK(modbus_rtu_t35_us(38400) == 1750);
	CHECK(modbus_rtu_t35_us(115200) == 1750);
	CHECK(modbus_rtu_t35_us(0) == 1750);
}

int main(int argc, char **argv) { return mbtest_main(argc, argv); }
