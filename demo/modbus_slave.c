/*
 * Modbus RTU slave demo: a small heater "plant" exposed over a serial line.
 *
 *   coil 0            heater enable            (RW)
 *   discrete in 0/1   alarm / heater on        (RO)
 *   holding reg 0/1   setpoint / alarm limit   (RW), 1/10 degC
 *   input reg 0       temperature              (RO), 1/10 degC
 *
 * Build options (add to CFLAGS in the Makefile):
 *   -DMODBUS_DEMO_ADDRESS=n    slave address, 1..247 (default 1)
 *   -DMODBUS_DEMO_8N2        8N2 instead of the spec default 8E1
 *   (default)                   RS-485 2-wire half duplex, driver enable on UART RTS
 *   -DMODBUS_DEMO_RS485_4W      RS-485 with separate TX/RX pairs
 *   -DMODBUS_DEMO_RS232         plain RS-232, no driver control
 *   -DMODBUS_DEMO_DE_LEAD_US=n  -DMODBUS_DEMO_DE_TRAIL_US=n   transceiver switching times
 *   -DMODBUS_DEMO_STRICT_T15         drop frames with a character gap above t1.5
 */
#include <platform/interfaces/uart.h>
#include <modbus/modbus_rtu_slave.h>
#include <target_config.h>	/* PLAT_UART_BAUD_RATE */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#ifndef MODBUS_DEMO_ADDRESS
#define MODBUS_DEMO_ADDRESS	1
#endif
#ifndef MODBUS_DEMO_RS232
#define MODBUS_DEMO_RS485	/* RS-485 is the default */
#endif
#ifndef MODBUS_DEMO_DE_LEAD_US
#define MODBUS_DEMO_DE_LEAD_US	0
#endif
#ifndef MODBUS_DEMO_DE_TRAIL_US
#define MODBUS_DEMO_DE_TRAIL_US	0
#endif

#define N_COILS		8
#define N_DISCRETE	8
#define N_HOLDING	4
#define N_INPUT		4

#define COIL_HEATER_EN	0
#define DI_ALARM	0
#define DI_HEATER_ON	1
#define HR_SETPOINT	0
#define HR_ALARM_LIMIT	1
#define IR_TEMPERATURE	0

#define AMBIENT_DECI_C	200
#define STEP_US		100000ull	/* plant update every 100 ms */

static struct {
	uint8_t  coils[(N_COILS + 7) / 8];
	uint8_t  discrete[(N_DISCRETE + 7) / 8];
	uint16_t holding[N_HOLDING];
	uint16_t input[N_INPUT];
} plant;

static inline bool bit_get(const uint8_t *a, unsigned i) { return (a[i / 8] >> (i % 8)) & 1u; }
static inline void bit_set(uint8_t *a, unsigned i, bool v)
{
	if (v) a[i / 8] |= (uint8_t)(1u << (i % 8));
	else   a[i / 8] &= (uint8_t)~(1u << (i % 8));
}

/* ---------------- Simulated process ---------------- */

static void plant_init(void)
{
	memset(&plant, 0, sizeof(plant));
	plant.holding[HR_SETPOINT]    = 600;
	plant.holding[HR_ALARM_LIMIT] = 800;
	plant.input[IR_TEMPERATURE]   = AMBIENT_DECI_C;
}

static void plant_step(void)
{
	uint16_t temp   = plant.input[IR_TEMPERATURE];
	bool     alarm  = temp > plant.holding[HR_ALARM_LIMIT];
	bool     heater = bit_get(plant.coils, COIL_HEATER_EN) && !alarm &&
			  temp < plant.holding[HR_SETPOINT];

	if (heater)                     temp += 5;
	else if (temp > AMBIENT_DECI_C) temp -= 2;

	plant.input[IR_TEMPERATURE] = temp;
	bit_set(plant.discrete, DI_ALARM, alarm);
	bit_set(plant.discrete, DI_HEATER_ON, heater);
}

/* ---------------- Modbus backend ---------------- */

static modbus_exception_t read_bits(const uint8_t *src, unsigned n, uint16_t addr, uint16_t count, uint8_t *dst)
{
	if ((unsigned)addr + count > n) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	memset(dst, 0, (count + 7u) / 8u);
	for (uint16_t i = 0; i < count; i++)
		if (bit_get(src, addr + i)) bit_set(dst, i, true);
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_read_coils(uint16_t a, uint16_t c, uint8_t *d) { return read_bits(plant.coils, N_COILS, a, c, d); }
static modbus_exception_t be_read_discrete(uint16_t a, uint16_t c, uint8_t *d) { return read_bits(plant.discrete, N_DISCRETE, a, c, d); }
static modbus_exception_t be_read_holding(uint16_t a, uint16_t c, uint16_t *d)
{
	if ((unsigned)a + c > N_HOLDING) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (uint16_t i = 0; i < c; i++) d[i] = plant.holding[a + i];
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_read_input(uint16_t a, uint16_t c, uint16_t *d)
{
	if ((unsigned)a + c > N_INPUT) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (uint16_t i = 0; i < c; i++) d[i] = plant.input[a + i];
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_write_coil(uint16_t a, bool s)
{
	if (a >= N_COILS) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	bit_set(plant.coils, a, s);
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_write_holding(uint16_t a, uint16_t v)
{
	if (a >= N_HOLDING) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	plant.holding[a] = v;
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_write_coils(uint16_t a, uint16_t c, const uint8_t *s)
{
	if ((unsigned)a + c > N_COILS) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (uint16_t i = 0; i < c; i++) bit_set(plant.coils, a + i, bit_get(s, i));
	return MODBUS_EX_NONE;
}
static modbus_exception_t be_write_holdings(uint16_t a, uint16_t c, const uint16_t *s)
{
	if ((unsigned)a + c > N_HOLDING) return MODBUS_EX_ILLEGAL_DATA_ADDR;
	for (uint16_t i = 0; i < c; i++) plant.holding[a + i] = s[i];
	return MODBUS_EX_NONE;
}

static const modbus_backend_t backend = {
	.read_coils                    = be_read_coils,
	.read_discrete_inputs          = be_read_discrete,
	.read_multi_holding_registers  = be_read_holding,
	.read_input_registers          = be_read_input,
	.write_coil                    = be_write_coil,
	.write_single_holding_register = be_write_holding,
	.write_multi_coils             = be_write_coils,
	.write_multi_holding_registers = be_write_holdings
};

/* ---------------- Serial port glue ---------------- */

static uint64_t port_now_us(void *ctx)
{
	(void)ctx;
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000u;
}
static int  port_getc(void *ctx) { (void)ctx; return uart_getc(); }
static void port_putc(void *ctx, uint8_t b) { (void)ctx; uart_putc_raw(b); }
static void port_flush(void *ctx) { (void)ctx; uart_tx_flush(); }
#ifdef MODBUS_DEMO_RS485
/* Driver enable on the UART RTS pin; boards that wire DE/RE to a GPIO replace this */
static void port_set_tx(void *ctx, bool en) { (void)ctx; uart_set_rts(en); }
#endif

static const modbus_serial_ops_t port_ops = {
	.getc     = port_getc,
	.putc     = port_putc,
	.now_us   = port_now_us,
	.tx_flush = port_flush,
#ifdef MODBUS_DEMO_RS485
	.set_tx   = port_set_tx,
#endif
};

void main(void)
{
	static modbus_slave_t slave;
	modbus_slave_cfg_t cfg = {
		.baud        = PLAT_UART_BAUD_RATE,
#ifdef MODBUS_DEMO_8N2
		.parity      = MODBUS_PARITY_NONE,
		.stop_bits   = 2,
#else
		.parity      = MODBUS_PARITY_EVEN,
		.stop_bits   = 1,
#endif
#if defined(MODBUS_DEMO_RS485_4W)
		.link        = MODBUS_LINK_RS485_4W,
#elif defined(MODBUS_DEMO_RS485)
		.link        = MODBUS_LINK_RS485_2W,
#else
		.link        = MODBUS_LINK_RS232,
#endif
		.de_lead_us  = MODBUS_DEMO_DE_LEAD_US,
		.de_trail_us = MODBUS_DEMO_DE_TRAIL_US,
#ifdef MODBUS_DEMO_STRICT_T15
		.strict_t15  = true,
#endif
	};

	/* The console UART is used as the Modbus port: no printing from here on */
	uart_set_format(cfg.parity == MODBUS_PARITY_NONE ? UART_PARITY_NONE : UART_PARITY_EVEN, cfg.stop_bits);

	plant_init();
	if (modbus_slave_init(&slave, MODBUS_DEMO_ADDRESS, &backend, &port_ops, &cfg) != 0)
		for (;;) { }	/* bad configuration: nothing sensible to do */

	uint64_t last_step = port_now_us(NULL);
	for (;;) {
		modbus_slave_poll(&slave);

		uint64_t now = port_now_us(NULL);
		if (now - last_step >= STEP_US) {
			plant_step();
			last_step = now;
		}
	}
}
