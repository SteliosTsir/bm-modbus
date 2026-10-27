#!/usr/bin/env python3
"""Read-only probe: does this sensor answer a read of the baud-rate register (0x0102) in ANY combination?

Tries every combination of: baud rate, parity, stop bits, function code (3 and 4), register address
(0x0102, plus the neighbours 0x0101 and 0x0103 with --neighbors, in case of an off-by-one numbering)
and quantity (1 and 2 registers). It only sends READ requests, nothing is ever written.

  python3 mbhold.py /dev/ttyUSB0                       # bauds 9600,14400,19200, parity n/e/o, 1-2 stop bits
  python3 mbhold.py /dev/ttyUSB0 -b 9600 --parity n --stops 1      # only the settings the sensor really uses
  python3 mbhold.py /dev/ttyUSB0 --neighbors --all     # widest search (several minutes)

Each request is classified as:
  VALID      a reply with a good CRC and a normal function code (data came back)
  EXCEPTION  a frame with a good CRC and an exception code (the sensor answered "no")
  JUNK       bytes came back but they are not a valid frame (for example 80 81 82 ... 87)
  nothing    no bytes
A summary at the end lists every combination that was VALID or EXCEPTION, and the JUNK patterns seen.
"""
import os, sys, time, argparse, select, termios

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def open_serial(path, baud, parity, stop):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    name = 'B%d' % baud
    if not hasattr(termios, name): sys.exit('unsupported baud rate %d' % baud)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    c = termios.CS8 | termios.CLOCAL | termios.CREAD
    if parity in ('e', 'o'): c |= termios.PARENB
    if parity == 'o': c |= termios.PARODD
    if stop == 2: c |= termios.CSTOPB
    a[2] = c; a[4] = a[5] = getattr(termios, name)
    a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd

def xfer(fd, data, wait):
    termios.tcflush(fd, termios.TCIOFLUSH)
    os.write(fd, data); termios.tcdrain(fd)
    out = b''; end = time.time() + wait
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.02)
        if r: out += os.read(fd, 256)
    return out

def classify(rx, addr, fc, qty):
    """-> (kind, detail)"""
    if not rx: return 'nothing', ''
    for i in range(len(rx)):
        if rx[i] != addr: continue
        if i + 5 <= len(rx) and crc16(rx[i:i + 5]) == 0 and rx[i + 1] != fc:
            return 'EXCEPTION', 'function byte %02x, code %02x: %s' % (rx[i + 1], rx[i + 2], rx[i:i + 5].hex(' '))
        if i + 2 < len(rx) and rx[i + 1] == fc:
            n = rx[i + 2]; end = i + 3 + n + 2
            if n == 2 * qty and end <= len(rx) and crc16(rx[i:end]) == 0:
                regs = [int.from_bytes(rx[i + 3 + 2 * k:i + 5 + 2 * k], 'big') for k in range(qty)]
                return 'VALID', 'registers %s: %s' % (regs, rx[i:end].hex(' '))
    return 'JUNK', rx.hex(' ')

def ints(s): return [int(x, 0) for x in s.split(',')]

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port')
    ap.add_argument('-b', '--bauds', default='9600,14400,19200')
    ap.add_argument('--parity', default='neo'); ap.add_argument('--stops', default='1,2')
    ap.add_argument('--fcs', default='3,4'); ap.add_argument('--qty', default='1,2')
    ap.add_argument('--reg', default='0x102'); ap.add_argument('--neighbors', action='store_true')
    ap.add_argument('--all', action='store_true', help='also try 1200,2400,4800,38400,57600,115200 baud')
    ap.add_argument('-a', '--addr', type=int, default=1)
    ap.add_argument('-t', '--wait', type=float, default=1.0, help='seconds to wait for each reply (this sensor needs ~0.6)')
    o = ap.parse_args()
    bauds = ints(o.bauds)
    if o.all: bauds += [x for x in (4800, 2400, 1200, 38400, 57600, 115200) if x not in bauds]
    regs = ints(o.reg)
    if o.neighbors: regs = sorted({r + d for r in regs for d in (-1, 0, 1) if r + d >= 0})
    stops = [int(x) for x in o.stops.split(',')]; fcs = ints(o.fcs); qtys = ints(o.qty)
    total = len(bauds) * len(o.parity) * len(stops) * len(fcs) * len(regs) * len(qtys)
    print('%d requests, about %.1f minutes. Read-only. Ctrl-C stops and prints what was found.' % (total, total * (o.wait + 0.05) / 60))
    found = []; junk = {}; n = 0
    try:
        for baud in bauds:
            for p in o.parity:
                for stop in stops:
                    fd = open_serial(o.port, baud, p, stop)
                    for fc in fcs:
                        for reg in regs:
                            for qty in qtys:
                                n += 1
                                req = bytes([o.addr, fc, reg >> 8, reg & 255, qty >> 8, qty & 255]); c = crc16(req)
                                rx = xfer(fd, req + bytes([c & 255, c >> 8]), o.wait)
                                kind, detail = classify(rx, o.addr, fc, qty)
                                tag = '%d 8%s%d fc%d reg 0x%04x qty %d' % (baud, p.upper(), stop, fc, reg, qty)
                                if kind in ('VALID', 'EXCEPTION'):
                                    found.append((tag, kind, detail)); print('%s -> %s  %s' % (tag, kind, detail), flush=True)
                                elif kind == 'JUNK':
                                    junk.setdefault(detail, []).append(tag)
                                    if len(junk[detail]) == 1: print('%s -> JUNK  %s' % (tag, detail), flush=True)
                    os.close(fd)
                    print('[%d/%d] %d 8%s%d done' % (n, total, baud, p.upper(), stop), flush=True)
    except KeyboardInterrupt:
        print('\n(stopped)')
    print('\n==== summary ====')
    if found:
        for tag, kind, detail in found: print('%-9s %s  %s' % (kind, tag, detail))
    else:
        print('No VALID reply and no EXCEPTION for any combination.')
    for d, tags in junk.items():
        print('JUNK pattern %s seen %d times (first: %s)' % (d, len(tags), tags[0]))
    if not found and not junk: print('Nothing came back at all.')

main()