# bm-modbus

Modbus RTU (PDU + RTU framing + demo) on top of the
[BareMetal SDK](https://github.com/CARV-ICS-FORTH/BareMetal), which is the
`sdk/` submodule, pinned to a known commit.

## Setup

    git clone --recurse-submodules <this repo>
    # or, after a plain clone:
    git submodule update --init

    make TARGET=qemu            # applies patches/ to sdk/, builds, links
    make TARGET=qemu run

## What is in here

| file | job |
|---|---|
| `include/modbus/modbus_pdu.h`, `src/modbus_pdu.c` | function codes 01 02 03 04 05 06 0F 10, exceptions, quantity limits; no SDK dependency |
| `modbus_rtu.h`, `src/modbus_rtu.c` | RTU frame: address, CRC-16, broadcast rules, t1.5/t3.5 values |
| `modbus_rtu_slave.h`, `src/modbus_rtu_slave.c` | serial-line slave state machine (INIT / IDLE / RECEPTION), RS-232 and RS-485 (2-wire and 4-wire) |
| `demo/modbus_slave.c` | heater example on the console UART |

The engine talks to the hardware through `modbus_serial_ops_t` (getc, putc,
now_us, tx_flush, set_tx, delay_us), so it runs unchanged on the board and in
the host tests.

## Demo options (`make DEMO_FLAGS="..."`)

    -DMODBUS_DEMO_ADDRESS=n          slave address 1..247 (default 1)
    -DMODBUS_DEMO_8N2                8N2 instead of the spec default 8E1
    (default)                        RS-485 2-wire half duplex, DE/RE on the UART RTS pin
    -DMODBUS_DEMO_RS485_4W           RS-485 with separate TX and RX pairs
    -DMODBUS_DEMO_RS232              plain RS-232, no driver control
    -DMODBUS_DEMO_DE_LEAD_US=n  -DMODBUS_DEMO_DE_TRAIL_US=n
    -DMODBUS_DEMO_STRICT_T15         drop frames with a gap above t1.5 between characters

The master must use the same baud rate and framing (default: PLAT_UART_BAUD_RATE, 8E1).
The demo takes over the console UART, so nothing else may print on it.

## Tests

    make test      # no SDK needed: ASan+UBSan unit tests, engine tests with a simulated
                   # serial port and clock, end-to-end over a pty with a Python master

## SDK patches

None are needed. The UART operations Modbus needs beyond `uart_getc()` (raw
transmit, wait for the shifter, 8E1/8N2 line format, RTS) are in `demo/uart_port.c`,
and the code avoids `memset()` because the SDK's version can overrun short buffers.

## Layout

    include/modbus/   public headers      src/    PDU, RTU, slave engine
    demo/             example slave + uart_port   tests/  host tests
