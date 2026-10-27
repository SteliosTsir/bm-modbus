#!/usr/bin/env python3
"""Connects the UART of a QEMU guest to a real serial port (for example a USB-RS485 adapter).

  1. start QEMU with its UART on a socket that waits for us:
       -serial unix:/tmp/mb.sock,server,wait
  2. start this bridge:
       python3 qemu_serial_bridge.py /dev/ttyUSB0 -b 9600 -p n -s 1
     (it connects to QEMU, which then starts the guest)

Why a bridge and not  -serial /dev/ttyUSB0 ? QEMU does not pace the emulated UART at the guest's baud
rate and works out the host baud rate from its own clock settings, so the real port could end up at the
wrong speed. Here the real port gets exactly the settings you give (-b/-p/-s).

What the bridge does:
  * guest -> adapter: bytes are written to the adapter, except for log lines. A log line is
    0xFF "LOG:" text "\\n" (what the guest's demo sends when built with -DSENSOR_SHARED_UART): it is
    printed here on the terminal and never reaches the bus. Everything the guest prints before its
    first log line (the SDK boot messages) is shown as "[boot]" and also kept off the bus.
  * adapter -> guest: bytes that arrive close together are handed to the guest together (a short
    reply then never looks to the guest like several frames, whatever the USB latency was).
  * --dashboard PORT: the guest's "DATA {json}" log lines (one per poll) are not printed but served as a live
    web page on http://localhost:PORT/ (tools/mb_dashboard.py, no internet or extra packages needed).
  * --drop-echo: only for an adapter that hears its own transmission (you see your own request come
    back before the answer). The copy of the frame that comes straight back is discarded. Do not use
    it otherwise: an answer that starts with the same bytes as the request would lose them.
"""
import os, sys, time, socket, select, termios, argparse, stat

MARK = b'\xffLOG:'

def open_serial(path, baud, parity, stop):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    name = 'B%d' % baud
    if not hasattr(termios, name):
        sys.exit('unsupported baud rate %d' % baud)
    a = termios.tcgetattr(fd)
    a[0] = 0                                  # iflag: no translation, parity errors delivered as data
    a[1] = 0                                  # oflag
    a[3] = 0                                  # lflag: raw
    cflag = termios.CS8 | termios.CLOCAL | termios.CREAD
    if parity in ('e', 'o'): cflag |= termios.PARENB
    if parity == 'o': cflag |= termios.PARODD
    if stop == 2: cflag |= termios.CSTOPB
    a[2] = cflag
    a[4] = a[5] = getattr(termios, name)
    a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
    try:
        termios.tcsetattr(fd, termios.TCSANOW, a)
    except termios.error as e:
        print('(could not set serial parameters: %s)' % e)
    return fd

class Demux:
    """Splits the guest's byte stream into bus bytes and log lines."""
    def __init__(self, out_bus, out_text, on_data=None):
        self.bus, self.text = out_bus, out_text
        self.on_data = on_data         # called with the JSON of every "DATA {json}" log line
        self.state = 'boot'            # boot -> bus <-> log
        self.hold = b''                # bytes that might be the start of MARK
        self.line = b''
    def feed(self, data):
        for b in data:
            c = bytes([b])
            if self.state == 'log':
                if c == b'\n':
                    txt = self.line.decode(errors='replace').rstrip('\r')
                    if txt.startswith('DATA ') and self.on_data:
                        self.on_data(txt[5:])      # telemetry: goes to the dashboard, not to the terminal
                    else:
                        self.text('[guest] ' + txt + '\n')
                    self.line = b''; self.state = 'bus'
                else:
                    self.line += c
                continue
            if self.state == 'boot':
                if (self.hold + c) == MARK:
                    self.hold = b''; self.flush_boot(); self.state = 'log'
                elif MARK.startswith(self.hold + c):
                    self.hold += c
                else:
                    self.line += self.hold + c; self.hold = b''
                    if c == b'\n': self.flush_boot()
                continue
            # bus state
            if (self.hold + c) == MARK:
                self.hold = b''; self.state = 'log'; self.line = b''
            elif MARK.startswith(self.hold + c):
                self.hold += c
            else:
                self.bus(self.hold + c); self.hold = b''
    def flush_boot(self):
        if self.line:
            self.text('[boot] ' + self.line.decode(errors='replace').rstrip('\r\n') + '\n'); self.line = b''

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('serial', help='real serial port, e.g. /dev/ttyUSB0')
    ap.add_argument('--qemu', default='/tmp/mb.sock', help='QEMU unix socket (default /tmp/mb.sock)')
    ap.add_argument('-b', '--baud', type=int, default=9600)
    ap.add_argument('-p', '--parity', choices=['n', 'e', 'o'], default='n')
    ap.add_argument('-s', '--stop', type=int, choices=[1, 2], default=1)
    ap.add_argument('--idle-ms', type=float, default=3.0, help='hand reply bytes to the guest after this much silence (default 3)')
    ap.add_argument('--drop-echo', action='store_true')
    ap.add_argument('--dashboard', type=int, metavar='PORT', help='serve a live web dashboard on this port from the guest\'s DATA lines (http://localhost:PORT/)')
    ap.add_argument('--image', help='firmware file whose size is shown on the dashboard')
    ap.add_argument('--verbose', '-v', action='store_true', help='also show the bytes in both directions')
    o = ap.parse_args()

    ser = open_serial(o.serial, o.baud, o.parity, o.stop)
    sock = socket.socket(socket.AF_UNIX)
    for _ in range(100):                       # QEMU may need a moment to create the socket
        try: sock.connect(o.qemu); break
        except (FileNotFoundError, ConnectionRefusedError): time.sleep(0.1)
    else:
        sys.exit('could not connect to %s (is QEMU running with -serial unix:%s,server,wait ?)' % (o.qemu, o.qemu))
    print('bridge: %s <-> %s  (%d baud, 8%s%d)' % (o.qemu, o.serial, o.baud, o.parity.upper(), o.stop), flush=True)

    tx_recent = b''                            # what we sent to the bus, to recognise an echo
    txbuf = b''; last_tx = 0.0                 # guest -> bus bytes, written as ONE block once the guest pauses
    def say(t): sys.stdout.write(t); sys.stdout.flush()
    def to_bus(data):
        # Do not write byte by byte: the guest's bytes may reach us in several pieces, and a gap inside a
        # frame can make the sensor (or an auto-direction adapter) drop it. Collect, then send in one write.
        nonlocal txbuf, last_tx
        txbuf += data; last_tx = time.time()
    def flush_tx():
        nonlocal txbuf, tx_recent
        if not txbuf: return
        data, txbuf = txbuf, b''
        os.write(ser, data); termios.tcdrain(ser)
        tx_recent += data
        if o.verbose: say('  >> bus   %s\n' % ' '.join('%02x' % x for x in data))
    on_data = None
    if o.dashboard:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from mb_dashboard import Dashboard
        dash = Dashboard(o.dashboard, o.image)
        on_data = dash.push
        print('dashboard: http://localhost:%d/' % o.dashboard, flush=True)
    demux = Demux(to_bus, say, on_data)
    pending = b''; last_rx = 0.0
    while True:
        timeout = None
        if pending: timeout = max(0.0, o.idle_ms / 1000.0 - (time.time() - last_rx))
        if txbuf:
            t = max(0.0, o.idle_ms / 1000.0 - (time.time() - last_tx))
            timeout = t if timeout is None else min(timeout, t)
        r, _, _ = select.select([sock, ser], [], [], timeout)
        if sock in r:
            d = sock.recv(4096)
            if not d: print('QEMU closed the connection'); break
            demux.feed(d)
        if txbuf and time.time() - last_tx >= o.idle_ms / 1000.0:
            flush_tx()
        if ser in r:
            try: d = os.read(ser, 4096)
            except BlockingIOError: d = b''
            if d and o.drop_echo and tx_recent:
                n = 0
                while n < len(d) and tx_recent and d[n] == tx_recent[0]:
                    n += 1; tx_recent = tx_recent[1:]
                d = d[n:]
            if d:
                pending += d; last_rx = time.time()
        if pending and time.time() - last_rx >= o.idle_ms / 1000.0:
            if o.verbose: say('  << bus   %s\n' % ' '.join('%02x' % x for x in pending))
            sock.sendall(pending); pending = b''
            tx_recent = b''

if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: print()