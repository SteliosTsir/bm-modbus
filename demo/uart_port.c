#include "uart_port.h"
#include <target_config.h>		/* PLAT_UART_* */
#include <platform/riscv/mmio.h>	/* read8/write8/... */
#include <platform/interfaces/uart.h>	/* uart_init(), uart_getc() of the SDK console UART */
#include <errno.h>

/* True when the Modbus bus is on the SDK's own console UART: then the SDK driver can be reused
 * for initialisation and for receiving (it is the same code we would write). */
#ifndef MODBUS_UART_BASE
#define USING_SDK_UART	1
#endif

#ifndef MODBUS_UART_BASE
#define MODBUS_UART_BASE	PLAT_UART_BASE
#endif
#ifndef MODBUS_UART_REG_SHIFT
#define MODBUS_UART_REG_SHIFT	PLAT_UART_REG_SHIFT
#endif
#ifndef MODBUS_UART_SHIFTED_IO
#define MODBUS_UART_SHIFTED_IO	PLAT_UART_SHIFTED_IO
#endif
#ifndef MODBUS_UART_CLOCK_HZ
#define MODBUS_UART_CLOCK_HZ	PLAT_UART_CLOCK_HZ
#endif

#define UART_THR	0x0	/* also RBR, and DLL with DLAB set */
#define UART_IER	0x1	/* also DLM with DLAB set */
#define UART_FCR	0x2
#define UART_LCR	0x3
#define UART_MCR	0x4
#define UART_LSR	0x5
#define LSR_DR		0x01
#define LSR_ERRORS	0x1E	/* overrun, parity, framing, break */
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#ifdef PLAT_UART_DW_APB
#define UART_DLF	0x30	/* DesignWare divisor latch fraction (register index, as in sdk uart.c) */
#endif

static inline void reg_write(int reg, uint8_t val)
{
	uintptr_t addr = (uintptr_t)(MODBUS_UART_BASE + (reg << MODBUS_UART_REG_SHIFT));
#if MODBUS_UART_SHIFTED_IO && (MODBUS_UART_REG_SHIFT == 4)
	write64((uint64_t *)addr, val);
#elif MODBUS_UART_SHIFTED_IO && (MODBUS_UART_REG_SHIFT == 2)
	write32((uint32_t *)addr, val);
#else
	write8((uint8_t *)addr, val);
#endif
}

static inline uint8_t reg_read(int reg)
{
	uintptr_t addr = (uintptr_t)(MODBUS_UART_BASE + (reg << MODBUS_UART_REG_SHIFT));
#if MODBUS_UART_SHIFTED_IO && (MODBUS_UART_REG_SHIFT == 4)
	return (uint8_t)read64((uint64_t *)addr);
#elif MODBUS_UART_SHIFTED_IO && (MODBUS_UART_REG_SHIFT == 2)
	return (uint8_t)read32((uint32_t *)addr);
#else
	return read8((uint8_t *)addr);
#endif
}

static uint8_t lcr_value(bool parity_even, unsigned stop_bits)
{
	/* bits 1:0 = 8 data bits, bit 2 = 2 stop bits, bit 3 = parity enable, bit 4 = even parity, bit 7 (DLAB) = 0 */
	uint8_t lcr = 0x3;
	if (stop_bits == 2) lcr |= 0x04;
	if (parity_even) lcr |= 0x18;
	return lcr;
}

void uart_port_init(uint32_t baud, bool parity_even, unsigned stop_bits)
{
	uint32_t bd16 = baud << 4;
	uint16_t div = (uint16_t)(MODBUS_UART_CLOCK_HZ / bd16);

#ifdef USING_SDK_UART
	/* Let the SDK bring the console UART up first: on a DesignWare UART (Milk-V Duo S) it quiesces
	 * and soft-resets the controller, without which our LCR writes below can be silently dropped. */
	uart_init();
#endif

	reg_write(UART_IER, 0);			/* no interrupts */
	reg_write(UART_LCR, 0x80);		/* DLAB: divisor latch access */
	reg_write(UART_THR, div & 0xFF);	/* DLL */
	reg_write(UART_IER, div >> 8);		/* DLM */
#ifdef PLAT_UART_DW_APB
	/* Fractional part in 1/16 steps, same formula as sdk uart.c. uart_init() left the value for
	 * 115200 baud here, which would be wrong for any other rate. */
	uint32_t rem = MODBUS_UART_CLOCK_HZ % bd16;
	uint32_t dlf = ((rem << 4) + (bd16 >> 1)) / bd16;
	if (dlf >> 4) { dlf = 0; div++; reg_write(UART_THR, div & 0xFF); reg_write(UART_IER, div >> 8); }
	reg_write(UART_DLF, (uint8_t)dlf);
#endif
	reg_write(UART_LCR, lcr_value(parity_even, stop_bits));
	reg_write(UART_FCR, 0x07);		/* FIFOs on, cleared */
	reg_write(UART_MCR, 0x0);
}

int uart_port_getc(void)
{
#ifdef USING_SDK_UART
	return uart_getc();			/* identical behaviour: byte, -EAGAIN or -EIO */
#else
	uint8_t lsr = reg_read(UART_LSR);
	if (lsr & LSR_ERRORS) return -EIO;
	if (lsr & LSR_DR) return reg_read(UART_THR);
	return -EAGAIN;
#endif
}

void uart_port_putc_raw(uint8_t c)
{
	while (!(reg_read(UART_LSR) & LSR_THRE)) { }
	reg_write(UART_THR, c);
}

void uart_port_tx_flush(void)
{
	while (!(reg_read(UART_LSR) & LSR_TEMT)) { }
}

void uart_port_set_format(bool parity_even, unsigned stop_bits)
{
	reg_write(UART_LCR, lcr_value(parity_even, stop_bits));
}

void uart_port_set_rts(bool asserted)
{
	reg_write(UART_MCR, asserted ? 0x2 : 0x0);
}