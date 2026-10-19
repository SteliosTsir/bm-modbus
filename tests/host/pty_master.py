#!/usr/bin/env python3
"""Independent Modbus RTU master (written from the spec, own CRC) driving pty_slave."""
import os, sys, time, struct, subprocess, select, termios, tty

def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def adu(addr, pdu):
    body = bytes([addr]) + pdu
    return body + struct.pack('<H', crc16(body))

def main(exe, mode):
    p = subprocess.Popen([exe, '5', '115200', mode], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    path = p.stdout.readline().strip()
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    time.sleep(0.05)                       # let the slave leave its initial t3.5 silence

    def drain():
        while select.select([fd], [], [], 0)[0]:
            os.read(fd, 4096)

    def txrx(frame, timeout=0.05):
        drain()
        os.write(fd, frame)
        buf = b''
        end = time.time() + timeout
        while time.time() < end:
            r = select.select([fd], [], [], 0.002)[0]
            if r:
                buf += os.read(fd, 4096)
                end = time.time() + 0.01   # reply ends after silence
        return buf

    fails = 0
    def check(name, ok, info=''):
        nonlocal fails
        print(('PASS ' if ok else 'FAIL ') + name + (' ' + info if info and not ok else ''))
        fails += (not ok)

    # write registers 0..2 with FC10, read back with FC03
    r = txrx(adu(5, bytes([0x10, 0, 0, 0, 3, 6, 0x00, 0x0A, 0x01, 0x02, 0xFF, 0xFF])))
    check('fc10 reply', r == adu(5, bytes([0x10, 0, 0, 0, 3])), r.hex())
    r = txrx(adu(5, bytes([0x03, 0, 0, 0, 3])))
    check('fc03 readback', r == adu(5, bytes([0x03, 6, 0, 0x0A, 1, 2, 0xFF, 0xFF])), r.hex())
    r = txrx(adu(5, bytes([0x06, 0, 7, 0x12, 0x34])))
    check('fc06 echo', r == adu(5, bytes([0x06, 0, 7, 0x12, 0x34])), r.hex())
    r = txrx(adu(5, bytes([0x05, 0, 2, 0xFF, 0x00])))
    check('fc05 echo', r == adu(5, bytes([0x05, 0, 2, 0xFF, 0x00])), r.hex())
    r = txrx(adu(5, bytes([0x0F, 0, 4, 0, 10, 2, 0xCD, 0x01])))
    check('fc0f reply', r == adu(5, bytes([0x0F, 0, 4, 0, 10])), r.hex())
    r = txrx(adu(5, bytes([0x01, 0, 0, 0, 16])))
    # coil2=1 then 0F wrote coils 4..13 = CD 01 -> bits 4,5(0xCD=1100 1101: b0,b2,b3,b6,b7) + 0x01 b0
    exp = [0]*16; exp[2] = 1
    for i in range(10): exp[4+i] = ((bytes([0xCD, 0x01])[i//8]) >> (i % 8)) & 1
    pk = bytes([sum(exp[8*k+j] << j for j in range(8)) for k in range(2)])
    check('fc01 readback', r == adu(5, bytes([0x01, 2]) + pk), r.hex())
    r = txrx(adu(5, bytes([0x03, 0, 20, 0, 1])))
    check('exception 02', r == adu(5, bytes([0x83, 2])), r.hex())
    r = txrx(adu(5, bytes([0x03, 0, 0, 0, 126])))
    check('exception 03 (qty 126)', r == adu(5, bytes([0x83, 3])), r.hex())
    r = txrx(adu(5, bytes([0x2B, 0x0E, 1, 0])))
    check('exception 01 (FC 0x2B)', r == adu(5, bytes([0xAB, 1])), r.hex())
    r = txrx(adu(6, bytes([0x03, 0, 0, 0, 1])))
    check('other slave: silence', r == b'', r.hex())
    bad = bytearray(adu(5, bytes([0x03, 0, 0, 0, 1]))); bad[-1] ^= 0x40
    r = txrx(bytes(bad))
    check('bad CRC: silence', r == b'', r.hex())
    r = txrx(adu(0, bytes([0x06, 0, 9, 0x00, 0x2A])))
    check('broadcast write: silence', r == b'')
    r = txrx(adu(5, bytes([0x03, 0, 9, 0, 1])))
    check('broadcast write took effect', r == adu(5, bytes([0x03, 2, 0x00, 0x2A])), r.hex())
    r = txrx(adu(0, bytes([0x03, 0, 0, 0, 1])))
    check('broadcast read: silence', r == b'')
    # 100 back-to-back request/response cycles
    ok = 0
    for i in range(100):
        v = (i * 257) & 0xFFFF
        w = txrx(adu(5, bytes([0x06, 0, 1, v >> 8, v & 255])), 0.02)
        ok += (w == adu(5, bytes([0x06, 0, 1, v >> 8, v & 255])))
    check('100 write cycles', ok == 100, str(ok))

    p.terminate(); err = p.stderr.read(); p.wait()
    print('slave stats:', err.strip())
    os.close(fd)
    return fails

sys.exit(1 if main(sys.argv[1], sys.argv[2]) else 0)
