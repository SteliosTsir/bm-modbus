/*
 * The UART the Modbus bus is attached to. By default it is the SDK's console UART
 * (PLAT_UART_*). To use another 16550-compatible UART, define at build time
 *   -DMODBUS_UART_BASE=0x...  [-DMODBUS_UART_REG_SHIFT=n] [-DMODBUS_UART_SHIFTED_IO=0|1]
 *   [-DMODBUS_UART_CLOCK_HZ=n]
 * (the last three default to the PLAT_UART_* values).
 *
 * Reused from the SDK: on the console UART, uart_init() (incl. the DesignWare reset) and
 * uart_getc(); register access through platform/riscv/mmio.h; PLAT_UART_* from target_config.h.
 * Written here because the SDK has no equivalent: uart_putc() turns '\n' into "\r\n" (that would
 * corrupt binary frames), and there is no API for parity, stop bits, baud, TX flush or RTS.
 */
#ifndef UART_PORT_H
#define UART_PORT_H
#include <stdint.h>
#include <stdbool.h>

/* Full setup of a UART that nobody initialised: baud, 8 data bits, parity, stop bits, FIFOs on.
 * Not needed for the SDK console UART, which uart_init() already set up. */
void uart_port_init(uint32_t baud, bool parity_even, unsigned stop_bits);

/* Like uart_getc(): a byte 0..255, -EAGAIN when none waits, -EIO on a line error (the damaged
 * byte stays in the FIFO and is returned by the next call). */
int uart_port_getc(void);
void uart_port_putc_raw(uint8_t c);		/* no '\n' -> "\r\n" translation */
void uart_port_tx_flush(void);			/* until the last stop bit has left (LSR.TEMT) */
void uart_port_set_format(bool parity_even, unsigned stop_bits);	/* 8 data bits; parity none/even only */
void uart_port_set_rts(bool asserted);		/* MCR.RTS, used as RS-485 DE/RE */

#endif