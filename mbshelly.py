#!/usr/bin/env python3
"""Interactive Modbus RTU master shell for testing a slave over a QEMU socket, a pty or a serial port.

  python3 mbshell.py                     # /tmp/mb.sock (qemu -serial unix:/tmp/mb.sock,server,wait)
  python3 mbshell.py /dev/pts/3          # a pty or a real serial device (raw, no pyserial needed)
  python3 mbshell.py localhost:5555      # TCP (qemu -serial tcp::5555,server,wait)
  options: -a ADDR (slave, default 1)   -t SECONDS (reply timeout, default 0.5)
           -b BAUD (default 115200)  -p e|o|n (parity, default e)  -s 1|2 (stop bits, default 1)
           -b/-p/-s are applied to real serial devices (/dev/ttyUSB0 ...) and must match the slave;
           they are ignored for sockets.

Type 'help' for commands. Every frame sent and received is shown in hex with CRC check.
"""
import os, sys, stat, time, socket, select, struct, tty, termios, argparse, shlex
try:
    import readline  # noqa: F401  (history and line editing)
except ImportError:
    pass

def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def hexs(b): return ' '.join('%02x' % x for x in b)

class Link:
    def __init__(self, target, baud=115200, parity='e', stop=1):
        self.sock = None; self.fd = None
        if ':' in target and not target.startswith('/'):
            host, port = target.rsplit(':', 1)
            self.sock = socket.create_connection((host, int(port)))
        elif os.path.exists(target) and stat.S_ISSOCK(os.stat(target).st_mode):
            self.sock = socket.socket(socket.AF_UNIX); self.sock.connect(target)
        else:
            self.fd = os.open(target, os.O_RDWR | os.O_NOCTTY); tty.setraw(self.fd)
            self.configure(baud, parity, stop)
    def configure(self, baud, parity, stop):
        name = 'B%d' % baud
        if not hasattr(termios, name):
            sys.exit('unsupported baud rate %d' % baud)
        try:
            attr = termios.tcgetattr(self.fd)
            attr[4] = attr[5] = getattr(termios, name)          # ispeed, ospeed
            cflag = attr[2] & ~(termios.CSIZE | termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CRTSCTS)
            cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
            if parity in ('e', 'o'): cflag |= termios.PARENB
            if parity == 'o': cflag |= termios.PARODD
            if stop == 2: cflag |= termios.CSTOPB
            attr[2] = cflag
            attr[0] &= ~(termios.INPCK | termios.IGNPAR | termios.PARMRK | termios.ISTRIP)  # deliver parity-error bytes as plain data
            termios.tcsetattr(self.fd, termios.TCSANOW, attr)
            print('serial port set to %d baud, 8%s%d' % (baud, parity.upper(), stop))
        except termios.error as e:
            print('(could not set serial parameters: %s)' % e)
    def _h(self): return self.sock if self.sock else self.fd
    def write(self, b):
        if self.sock: self.sock.sendall(b)
        else: os.write(self.fd, b)
    def read(self):
        if self.sock: return self.sock.recv(4096)
        return os.read(self.fd, 4096)
    def readable(self, t):
        return bool(select.select([self._h()], [], [], t)[0])
    def drain(self, t=0.05):
        out = b''
        while self.readable(t):
            d = self.read()
            if not d: break
            out += d
        return out
    def transact(self, frame, timeout):
        self.drain(0.0)
        self.write(frame)
        out = b''
        end = time.time() + timeout
        while time.time() < end:
            if self.readable(0.01):
                d = self.read()
                if not d: break
                out += d
                end = time.time() + 0.05       # frame ends after a short silence
        return out

EXC = {1: 'illegal function', 2: 'illegal data address', 3: 'illegal data value',
       4: 'slave device failure', 5: 'acknowledge', 6: 'slave busy', 8: 'memory parity error'}

def num(s): return int(s, 0)
def be16(v): return struct.pack('>H', v & 0xFFFF)

class Shell:
    def __init__(self, link, addr, timeout):
        self.link, self.addr, self.timeout = link, addr, timeout
        self.corrupt = False

    def send(self, pdu, corrupt=False, expect=True):
        corrupt = corrupt or self.corrupt
        body = bytes([self.addr]) + pdu
        frame = body + struct.pack('<H', crc16(body))
        if corrupt:
            frame = frame[:-1] + bytes([frame[-1] ^ 0x01])
        print('  TX  ' + hexs(frame) + ('   (CRC deliberately wrong)' if corrupt else ''))
        rx = self.link.transact(frame, self.timeout)
        if not rx:
            print('  RX  (nothing)' + ('' if self.addr != 0 else '   (broadcast: no reply is correct)'))
            return None
        crc_ok = len(rx) >= 4 and crc16(rx[:-2]) == struct.unpack('<H', rx[-2:])[0]
        print('  RX  ' + hexs(rx) + ('' if crc_ok else '   <-- BAD CRC / not a clean frame'))
        if not crc_ok: return None
        if rx[0] != self.addr: print('  note: reply from address %d' % rx[0])
        return rx[1:-2]

    def decode_exc(self, pdu):
        if pdu and pdu[0] & 0x80:
            print('  => EXCEPTION %02x: %s' % (pdu[1], EXC.get(pdu[1], '?')))
            return True
        return False

    def bits(self, fc, a, n):
        r = self.send(bytes([fc]) + be16(a) + be16(n))
        if r is None or self.decode_exc(r): return
        vals = [(r[2 + i // 8] >> (i % 8)) & 1 for i in range(n)]
        print('  => ' + ', '.join('%d:%d' % (a + i, v) for i, v in enumerate(vals)))

    def regs(self, fc, a, n):
        r = self.send(bytes([fc]) + be16(a) + be16(n))
        if r is None or self.decode_exc(r): return
        vals = struct.unpack('>%dH' % (len(r[2:]) // 2), r[2:])
        print('  => ' + ', '.join('%d: %d (0x%04x)' % (a + i, v, v) for i, v in enumerate(vals)))

    def echo_check(self, r, what):
        if r is None or self.decode_exc(r): return
        print('  => ' + what)

    def run(self, line):
        corrupt = False
        t = shlex.split(line)
        if not t: return True
        if t[0] == 'bad': corrupt = True; t = t[1:]
        self.corrupt = corrupt
        if not t: return True
        c, a = t[0].lower(), t[1:]
        if c in ('quit', 'exit', 'q'): return False
        elif c in ('help', '?'): print(HELP)
        elif c == 'rc': self.bits(1, num(a[0]), num(a[1]) if len(a) > 1 else 1)
        elif c == 'rd': self.bits(2, num(a[0]), num(a[1]) if len(a) > 1 else 1)
        elif c == 'rh': self.regs(3, num(a[0]), num(a[1]) if len(a) > 1 else 1)
        elif c == 'ri': self.regs(4, num(a[0]), num(a[1]) if len(a) > 1 else 1)
        elif c == 'wc':
            on = a[1].lower() in ('1', 'on', 'true')
            r = self.send(bytes([5]) + be16(num(a[0])) + (b'\xff\x00' if on else b'\x00\x00'), corrupt)
            self.echo_check(r, 'coil %d = %s' % (num(a[0]), 'ON' if on else 'OFF'))
        elif c == 'wr':
            r = self.send(bytes([6]) + be16(num(a[0])) + be16(num(a[1])), corrupt)
            self.echo_check(r, 'register %d = %d' % (num(a[0]), num(a[1])))
        elif c == 'wrs':
            vals = [num(x) for x in a[1:]]
            r = self.send(bytes([0x10]) + be16(num(a[0])) + be16(len(vals)) + bytes([2 * len(vals)]) +
                          b''.join(be16(v) for v in vals), corrupt)
            self.echo_check(r, 'wrote %d registers from %d' % (len(vals), num(a[0])))
        elif c == 'wcs':
            bits = [1 if x in ('1', 'on') else 0 for x in a[1:]]
            packed = bytes(sum(b << j for j, b in enumerate(bits[i:i + 8])) for i in range(0, len(bits), 8))
            r = self.send(bytes([0x0F]) + be16(num(a[0])) + be16(len(bits)) + bytes([len(packed)]) + packed, corrupt)
            self.echo_check(r, 'wrote %d coils from %d' % (len(bits), num(a[0])))
        elif c == 'raw':      # PDU bytes in hex, address and CRC added
            self.send(bytes(int(x, 16) for x in a), corrupt)
        elif c == 'rawframe': # complete frame as given, nothing added
            frame = bytes(int(x, 16) for x in a)
            print('  TX  ' + hexs(frame))
            rx = self.link.transact(frame, self.timeout)
            print('  RX  ' + (hexs(rx) if rx else '(nothing)'))
        elif c == 'addr':
            if a: self.addr = num(a[0])
            print('  slave address = %d' % self.addr)
        elif c == 'timeout':
            if a: self.timeout = float(a[0])
            print('  reply timeout = %.2f s' % self.timeout)
        else:
            print('  unknown command, type help')
        return True

HELP = '''commands (numbers can be decimal or 0x hex):
  rc  ADDR [N]            read coils (01)            rd  ADDR [N]   read discrete inputs (02)
  rh  ADDR [N]            read holding regs (03)     ri  ADDR [N]   read input regs (04)
  wc  ADDR on|off         write single coil (05)     wr  ADDR VAL   write single register (06)
  wcs ADDR b b b ...      write coils (0F)           wrs ADDR v v ...  write registers (10)
  raw HEX...              send address + these PDU bytes + CRC   (e.g. raw 2b 0e 01 00)
  rawframe HEX...         send exactly these bytes, nothing added
  bad <command>           same, with a deliberately wrong CRC (the slave must stay silent)
  addr N                  talk to slave N (0 = broadcast, no reply expected)
  timeout S               how long to wait for a reply
  quit
demo map: coil 0 heater enable | discrete 0 alarm, 1 heater on | holding 0 setpoint, 1 alarm limit
          input 0 temperature (1/10 degC)'''

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('target', nargs='?', default='/tmp/mb.sock')
    p.add_argument('-a', '--addr', type=lambda x: int(x, 0), default=1)
    p.add_argument('-t', '--timeout', type=float, default=0.5)
    p.add_argument('-b', '--baud', type=int, default=115200)
    p.add_argument('-p', '--parity', choices=['e', 'o', 'n'], default='e')
    p.add_argument('-s', '--stop', type=int, choices=[1, 2], default=1)
    o = p.parse_args()
    link = Link(o.target, o.baud, o.parity, o.stop)
    boot = link.drain(1.0)
    if boot: print('(discarded %d bytes of boot output)' % len(boot))
    time.sleep(0.3)                      # let the slave see a quiet line (t3.5)
    sh = Shell(link, o.addr, o.timeout)
    print('connected to %s, slave %d. Type help.' % (o.target, o.addr))
    interactive = sys.stdin.isatty()
    while True:
        try:
            line = input('modbus> ' if interactive else '')
        except (EOFError, KeyboardInterrupt):
            print(); break
        if not interactive: print('modbus> ' + line)
        try:
            if not sh.run(line): break
        except (IndexError, ValueError):
            print('  bad arguments, type help')
        except OSError as e:
            print('  I/O error:', e); break

if __name__ == '__main__':
    main()