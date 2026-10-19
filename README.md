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

## SDK patches

`patches/` holds small generic fixes the SDK does not have yet (memset bound,
global trap handlers for LTO, uart_putc_raw). `make` applies them to the
submodule working tree (skipping any already applied). Because of that the
submodule shows as modified; `.gitmodules` sets `ignore = dirty` to hide it.
When the fixes are merged upstream, bump the submodule and delete the patch.

## Layout

    include/modbus/   public headers      src/    modbus_pdu.c, modbus_rtu.c
    demo/             example slave       tests/  tests
    patches/          fixes for sdk/      scripts/apply-sdk-patches.sh
