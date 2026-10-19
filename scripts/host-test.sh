#!/bin/bash
# Host tests (no SDK, no QEMU): AddressSanitizer + UBSan, plus an end-to-end run
# of the slave engine on a pseudo-terminal against an independent Python master.
set -e
cd "$(dirname "$0")/.."
OUT=build/host; mkdir -p $OUT
CC=${HOSTCC:-gcc}
FL="-O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all -I include -I tests"
export ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=0
rc=0
$CC $FL tests/test_core.c  src/modbus_pdu.c src/modbus_rtu.c -o $OUT/test_core
$CC $FL tests/test_slave.c src/*.c -o $OUT/test_slave
$CC $FL tests/host/pty_slave.c src/*.c -o $OUT/pty_slave
echo "== PDU + RTU framing";  $OUT/test_core  || rc=1
echo "== serial slave engine"; $OUT/test_slave || rc=1
if command -v python3 >/dev/null; then
	for m in rs232 rs485; do echo "== end to end over a pty ($m)"; python3 tests/host/pty_master.py $OUT/pty_slave $m || rc=1; done
fi
exit $rc
