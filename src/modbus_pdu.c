#include <modbus/modbus_pdu.h>

static inline uint16_t read_be16(const uint8_t *buf)
{
	return (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

static inline void write_be16(uint8_t *buf, uint16_t val)
{
	buf[0] = (uint8_t)(val >> 8);
	buf[1] = (uint8_t)(val & 0xFF);
}

/*
 * Handlers get the payload (after the function code) of the request and a
 * response area of resp_cap bytes (also after the function code). They check
 * that the response fits BEFORE calling into the device, so a device callback
 * can never be asked to write past the end of the caller's buffer.
 *
 * Order of checks follows the spec: function supported (01), then quantity or
 * value (03), then address (02, decided by the device), then device failure
 * (04).
 */

typedef modbus_exception_t (*read_bits_fn)(uint16_t, uint16_t, uint8_t *);
typedef modbus_exception_t (*read_regs_fn)(uint16_t, uint16_t, uint16_t *);

/* FC 01 / 02 */
static modbus_exception_t
handle_read_bits(read_bits_fn fn, const uint8_t *req, size_t req_len,
		 uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
	if (fn == NULL) return MODBUS_EX_ILLEGAL_FUNCTION;
	if (req_len != 4) return MODBUS_EX_ILLEGAL_DATA_VAL;

	uint16_t addr = read_be16(&req[0]);
	uint16_t count = read_be16(&req[2]);
	if (count < 1 || count > MODBUS_MAX_READ_BITS) return MODBUS_EX_ILLEGAL_DATA_VAL;

	size_t byte_count = ((size_t)count + 7) / 8;
	if (1 + byte_count > resp_cap) return MODBUS_EX_SLAVE_DEVICE_FAIL;

	resp[0] = (uint8_t)byte_count;
	modbus_exception_t ex = fn(addr, count, &resp[1]);
	if (ex != MODBUS_EX_NONE) return ex;

	*resp_len = 1 + byte_count;
	return MODBUS_EX_NONE;
}

/* FC 03 / 04 */
static modbus_exception_t
handle_read_regs(read_regs_fn fn, const uint8_t *req, size_t req_len,
		 uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
	if (fn == NULL) return MODBUS_EX_ILLEGAL_FUNCTION;
	if (req_len != 4) return MODBUS_EX_ILLEGAL_DATA_VAL;

	uint16_t addr = read_be16(&req[0]);
	uint16_t count = read_be16(&req[2]);
	if (count < 1 || count > MODBUS_MAX_READ_REGS) return MODBUS_EX_ILLEGAL_DATA_VAL;

	size_t byte_count = (size_t)count * 2;
	if (1 + byte_count > resp_cap) return MODBUS_EX_SLAVE_DEVICE_FAIL;

	uint16_t regs[MODBUS_MAX_READ_REGS];
	modbus_exception_t ex = fn(addr, count, regs);
	if (ex != MODBUS_EX_NONE) return ex;

	resp[0] = (uint8_t)byte_count;
	for (uint16_t i = 0; i < count; i++)
		write_be16(&resp[1 + 2 * (size_t)i], regs[i]);

	*resp_len = 1 + byte_count;
	return MODBUS_EX_NONE;
}

/* FC 05 */
static modbus_exception_t
handle_write_single_coil(const modbus_backend_t *be, const uint8_t *req, size_t req_len,
			 uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
	if (be->write_coil == NULL) return MODBUS_EX_ILLEGAL_FUNCTION;
	if (req_len != 4) return MODBUS_EX_ILLEGAL_DATA_VAL;
	if (resp_cap < 4) return MODBUS_EX_SLAVE_DEVICE_FAIL;

	uint16_t addr = read_be16(&req[0]);
	uint16_t value = read_be16(&req[2]);
	if (value != 0xFF00 && value != 0x0000) return MODBUS_EX_ILLEGAL_DATA_VAL;

	modbus_exception_t ex = be->write_coil(addr, value == 0xFF00);
	if (ex != MODBUS_EX_NONE) return ex;

	for (int i = 0; i < 4; i++) resp[i] = req[i];	/* response echoes the request */
	*resp_len = 4;
	return MODBUS_EX_NONE;
}

/* FC 06 */
static modbus_exception_t
handle_write_single_reg(const modbus_backend_t *be, const uint8_t *req, size_t req_len,
			uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
	if (be->write_single_holding_register == NULL) return MODBUS_EX_ILLEGAL_FUNCTION;
	if (req_len != 4) return MODBUS_EX_ILLEGAL_DATA_VAL;
	if (resp_cap < 4) return MODBUS_EX_SLAVE_DEVICE_FAIL;

	modbus_exception_t ex = be->write_single_holding_register(read_be16(&req[0]), read_be16(&req[2]));
	if (ex != MODBUS_EX_NONE) return ex;

	for (int i = 0; i < 4; i++) resp[i] = req[i];
	*resp_len = 4;
	return MODBUS_EX_NONE;
}

/* FC 0F */
static modbus_exception_t
handle_write_multi_coils(const modbus_backend_t *be, const uint8_t *req, size_t req_len,
			 uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
	if (be->write_multi_coils == NULL) return MODBUS_EX_ILLEGAL_FUNCTION;
	if (req_len < 5) return MODBUS_EX_ILLEGAL_DATA_VAL;
	if (resp_cap < 4) return MODBUS_EX_SLAVE_DEVICE_FAIL;

	uint16_t addr = read_be16(&req[0]);
	uint16_t count = read_be16(&req[2]);
	size_t byte_count = req[4];

	if (count < 1 || count > MODBUS_MAX_WRITE_BITS) return MODBUS_EX_ILLEGAL_DATA_VAL;
	if (byte_count != ((size_t)count + 7) / 8 || req_len != 5 + byte_count)
		return MODBUS_EX_ILLEGAL_DATA_VAL;

	modbus_exception_t ex = be->write_multi_coils(addr, count, &req[5]);
	if (ex != MODBUS_EX_NONE) return ex;

	write_be16(&resp[0], addr);
	write_be16(&resp[2], count);
	*resp_len = 4;
	return MODBUS_EX_NONE;
}

/* FC 10 */
static modbus_exception_t
handle_write_multi_regs(const modbus_backend_t *be, const uint8_t *req, size_t req_len,
			uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
	if (be->write_multi_holding_registers == NULL) return MODBUS_EX_ILLEGAL_FUNCTION;
	if (req_len < 5) return MODBUS_EX_ILLEGAL_DATA_VAL;
	if (resp_cap < 4) return MODBUS_EX_SLAVE_DEVICE_FAIL;

	uint16_t addr = read_be16(&req[0]);
	uint16_t count = read_be16(&req[2]);
	size_t byte_count = req[4];

	if (count < 1 || count > MODBUS_MAX_WRITE_REGS) return MODBUS_EX_ILLEGAL_DATA_VAL;
	if (byte_count != (size_t)count * 2 || req_len != 5 + byte_count)
		return MODBUS_EX_ILLEGAL_DATA_VAL;

	uint16_t regs[MODBUS_MAX_WRITE_REGS];
	for (uint16_t i = 0; i < count; i++)
		regs[i] = read_be16(&req[5 + 2 * (size_t)i]);

	modbus_exception_t ex = be->write_multi_holding_registers(addr, count, regs);
	if (ex != MODBUS_EX_NONE) return ex;

	write_be16(&resp[0], addr);
	write_be16(&resp[2], count);
	*resp_len = 4;
	return MODBUS_EX_NONE;
}

size_t
modbus_process_pdu(const modbus_backend_t *backend, const uint8_t *req_pdu, size_t req_len,
		   uint8_t *resp_pdu, size_t max_resp_len)
{
	/* Even an exception response needs 2 bytes */
	if (backend == NULL || req_pdu == NULL || resp_pdu == NULL || req_len < 1 || max_resp_len < 2)
		return 0;

	uint8_t fc = req_pdu[0];
	const uint8_t *req = &req_pdu[1];
	size_t req_n = req_len - 1;
	uint8_t *resp = &resp_pdu[1];
	size_t cap = max_resp_len - 1;
	size_t resp_n = 0;
	modbus_exception_t ex;

	switch (fc) {
	case MODBUS_FC_READ_COILS:
		ex = handle_read_bits(backend->read_coils, req, req_n, resp, cap, &resp_n);
		break;
	case MODBUS_FC_READ_DISCRETE_INPUTS:
		ex = handle_read_bits(backend->read_discrete_inputs, req, req_n, resp, cap, &resp_n);
		break;
	case MODBUS_FC_READ_MULTI_HOLDING_REGISTERS:
		ex = handle_read_regs(backend->read_multi_holding_registers, req, req_n, resp, cap, &resp_n);
		break;
	case MODBUS_FC_READ_INPUT_REGISTERS:
		ex = handle_read_regs(backend->read_input_registers, req, req_n, resp, cap, &resp_n);
		break;
	case MODBUS_FC_WRITE_SINGLE_COIL:
		ex = handle_write_single_coil(backend, req, req_n, resp, cap, &resp_n);
		break;
	case MODBUS_FC_WRITE_SINGLE_HOLDING_REGISTER:
		ex = handle_write_single_reg(backend, req, req_n, resp, cap, &resp_n);
		break;
	case MODBUS_FC_WRITE_MULTI_COILS:
		ex = handle_write_multi_coils(backend, req, req_n, resp, cap, &resp_n);
		break;
	case MODBUS_FC_WRITE_MULTI_HOLDING_REGISTERS:
		ex = handle_write_multi_regs(backend, req, req_n, resp, cap, &resp_n);
		break;
	default:
		ex = MODBUS_EX_ILLEGAL_FUNCTION;
		break;
	}

	if (ex != MODBUS_EX_NONE) {
		resp_pdu[0] = (uint8_t)(fc | 0x80);
		resp_pdu[1] = (uint8_t)ex;
		return 2;
	}

	resp_pdu[0] = fc;
	return 1 + resp_n;
}
