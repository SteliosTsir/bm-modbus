#!/usr/bin/env python3
"""Simulated RS-485 temperature/humidity sensor (XY-MD02 style), written from the Modbus spec.

  sensor_sim.py                    creates a pty and prints its path (for host tests)
  sensor_sim.py --unix /tmp/mb.sock   connects to a QEMU serial socket
                                      (qemu ... -serial unix:/tmp/mb.sock,server,wait)

Slave address 1. Input registers: 1 = temperature, 2 = humidity (1/10 units, signed 16 bit);
holding registers 0..7 are read/write. A request is a frame followed by >= 2 ms of silence;
bad CRC, other addresses and unknown functions are handled as the spec says.
  --flaky      answers only every second request (the master must retry)
  --damage     corrupts the CRC of every first answer (the master must retry)
  --echo       repeats every request frame before answering (an adapter that hears itself)
  --temp N     temperature in 1/10 degC (default 235), --hum N (default 456)
"""
import os, sys, time, select, struct, argparse, socket, tty

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--unix'); ap.add_argument('--flaky', action='store_true'); ap.add_argument('--damage', action='store_true')
    ap.add_argument('--temp', type=int, default=235); ap.add_argument('--hum', type=int, default=456)
    ap.add_argument('--addr', type=int, default=1); ap.add_argument('--delay', type=float, default=0.010)
    ap.add_argument('--quiet', action='store_true'); ap.add_argument('--echo', action='store_true')
    o = ap.parse_args()

    if o.unix:
        s = socket.socket(socket.AF_UNIX); s.connect(o.unix)
        rd = lambda: s.recv(4096); wr = lambda b: s.sendall(b); h = s
        print('connected to', o.unix, flush=True)
    else:
        master, slave = os.openpty(); tty.setraw(master); tty.setraw(slave)
        print(os.ttyname(slave), flush=True)
        rd = lambda: os.read(master, 4096); wr = lambda b: os.write(master, b); h = master

    ir = {1: o.temp & 0xFFFF, 2: o.hum & 0xFFFF}
    hr = [0] * 8
    n_req = 0
    buf = b''; last = time.time()
    while True:
        r = select.select([h], [], [], 0.002)[0]
        if r:
            d = rd()
            if not d: break
            buf += d; last = time.time(); continue
        if not buf or time.time() - last < 0.002:
            continue
        frame, buf = buf, b''
        if o.echo: wr(frame)
        if len(frame) < 4 or crc16(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
            if not o.quiet: print('ignored (bad CRC/short):', frame.hex(' '), flush=True)
            continue
        a, fc, body = frame[0], frame[1], frame[2:-2]
        if a not in (o.addr, 0):
            continue
        n_req += 1
        resp = None
        if fc in (3, 4) and len(body) == 4:
            addr, cnt = struct.unpack('>HH', body)
            if not 1 <= cnt <= 125: resp = bytes([fc | 0x80, 3])
            else:
                src = ir if fc == 4 else {i: hr[i] for i in range(8)}
                if all(addr + i in src for i in range(cnt)):
                    resp = bytes([fc, 2 * cnt]) + b''.join(struct.pack('>H', src[addr + i]) for i in range(cnt))
                else: resp = bytes([fc | 0x80, 2])
        elif fc == 6 and len(body) == 4:
            addr, val = struct.unpack('>HH', body)
            if addr < 8: hr[addr] = val; resp = bytes([6]) + body
            else: resp = bytes([0x86, 2])
        elif fc == 0x10 and len(body) >= 5:
            addr, cnt, bc = struct.unpack('>HHB', body[:5])
            if cnt < 1 or cnt > 123 or bc != 2 * cnt or len(body) != 5 + bc: resp = bytes([0x90, 3])
            elif addr + cnt > 8: resp = bytes([0x90, 2])
            else:
                for i in range(cnt): hr[addr + i] = struct.unpack('>H', body[5 + 2 * i:7 + 2 * i])[0]
                resp = bytes([0x10]) + body[:4]
        else:
            resp = bytes([fc | 0x80, 1])
        if not o.quiet: print('request #%d: %s' % (n_req, frame.hex(' ')), flush=True)
        if a == 0: continue                       # broadcast: never answered
        if o.flaky and n_req % 2 == 1: continue
        out = bytes([o.addr]) + resp
        out += struct.pack('<H', crc16(out))
        if o.damage and n_req % 2 == 1: out = out[:-1] + bytes([out[-1] ^ 0x01])
        time.sleep(o.delay)
        wr(out)

main()