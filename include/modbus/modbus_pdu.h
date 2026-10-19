/*
 * Modbus PDU layer: function code + data, independent of the transport.
 *
 * Follows "MODBUS Application Protocol Specification V1.1b3". Supported
 * functions: 01 02 03 04 05 06 0F 10. Everything else is answered with
 * exception 01 (illegal function).
 *
 * This layer knows nothing about slave addresses, CRCs, UARTs or the SDK, it
 * only needs <stdint.h>, so it can also be built and tested on a host.
 */
#ifndef MODBUS_PDU_H
#define MODBUS_PDU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MODBUS_MAX_PDU_LEN	253

/* Quantity limits from the spec */
#define MODBUS_MAX_READ_BITS	2000	/* FC 01, 02 */
#define MODBUS_MAX_READ_REGS	125	/* FC 03, 04 */
#define MODBUS_MAX_WRITE_BITS	1968	/* FC 0F */
#define MODBUS_MAX_WRITE_REGS	123	/* FC 10 */

/* Function codes */
#define MODBUS_FC_READ_COILS                    0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS          0x02
#define MODBUS_FC_READ_MULTI_HOLDING_REGISTERS  0x03
#define MODBUS_FC_READ_INPUT_REGISTERS          0x04
#define MODBUS_FC_WRITE_SINGLE_COIL             0x05
#define MODBUS_FC_WRITE_SINGLE_HOLDING_REGISTER 0x06
#define MODBUS_FC_WRITE_MULTI_COILS             0x0F
#define MODBUS_FC_WRITE_MULTI_HOLDING_REGISTERS 0x10

/* True for the function codes that change the device (the only ones a
 * broadcast request may carry). */
#define MODBUS_FC_IS_WRITE(fc) ((fc) == MODBUS_FC_WRITE_SINGLE_COIL || \
	(fc) == MODBUS_FC_WRITE_SINGLE_HOLDING_REGISTER || \
	(fc) == MODBUS_FC_WRITE_MULTI_COILS || \
	(fc) == MODBUS_FC_WRITE_MULTI_HOLDING_REGISTERS)

/* Exception codes (an exception response is: function code | 0x80, code) */
typedef enum {
	MODBUS_EX_NONE              = 0x00,
	MODBUS_EX_ILLEGAL_FUNCTION  = 0x01,
	MODBUS_EX_ILLEGAL_DATA_ADDR = 0x02,
	MODBUS_EX_ILLEGAL_DATA_VAL  = 0x03,
	MODBUS_EX_SLAVE_DEVICE_FAIL = 0x04,
	MODBUS_EX_ACKNOWLEDGE       = 0x05,
	MODBUS_EX_SLAVE_BUSY        = 0x06,
	MODBUS_EX_MEMORY_PARITY     = 0x08
} modbus_exception_t;

/*
 * The device behind the protocol. A callback may be NULL, the matching
 * function is then answered with exception 01. The PDU layer validates the
 * quantity (1..max for the function) before calling, so callbacks only have
 * to check that the address range exists (and return MODBUS_EX_ILLEGAL_DATA_ADDR
 * if it does not). Registers are host-endian, bits are packed LSB first.
 */
typedef struct {
	modbus_exception_t (*read_coils)(uint16_t addr, uint16_t count, uint8_t *dest_packed);
	modbus_exception_t (*read_discrete_inputs)(uint16_t addr, uint16_t count, uint8_t *dest_packed);
	modbus_exception_t (*read_multi_holding_registers)(uint16_t addr, uint16_t count, uint16_t *dest_regs);
	modbus_exception_t (*read_input_registers)(uint16_t addr, uint16_t count, uint16_t *dest_regs);
	modbus_exception_t (*write_coil)(uint16_t addr, bool state);
	modbus_exception_t (*write_single_holding_register)(uint16_t addr, uint16_t value);
	modbus_exception_t (*write_multi_coils)(uint16_t addr, uint16_t count, const uint8_t *src_packed);
	modbus_exception_t (*write_multi_holding_registers)(uint16_t addr, uint16_t count, const uint16_t *src_regs);
} modbus_backend_t;

/*
 * Handles one request PDU and builds the response PDU.
 *
 * Returns the response length, or 0 when nothing can be answered (bad
 * arguments, empty request, or max_resp_len < 2). Never writes more than
 * max_resp_len bytes; a response that would not fit becomes exception 04.
 */
size_t modbus_process_pdu(const modbus_backend_t *backend, const uint8_t *req_pdu, size_t req_len,
			  uint8_t *resp_pdu, size_t max_resp_len);

#endif /* MODBUS_PDU_H */
