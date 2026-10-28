"""
Smart Trashbin Controller - M5Stack Tab5 (UIFlow2.0 / MicroPython)
====================================================================
PD42S1 smart stepper motor x1 (ALIENTEK) | RS485 bus | addr 0x01

RS485 interface & protocol frames follow CPDR.py in this folder
(already verified on Tab5). This project adds:
  - Absolute position mode (0xF2) -> open / close lid
  - Torque mode (0xF0)           -> compress / capacity check
  - Read commands: 0x2A position / 0x2D stall / 0x30 arrived

Buttons:
  Open     : absolute pos mode, dir=0 (CW), acc=150, speed=10RPM,
             pos=37000
  Close    : absolute pos mode, dir=0 (CW), acc=150, speed=10RPM,
             pos=25000
  (Open/Close completion = same stationary judgement as Check:
   position stops changing AND is within TARGET_TOL of the target)
  Compress : torque mode, dir=1 (CCW), 600mA, motor keeps pressing,
             no auto-brake (exit by Open/Close command)
  Check    : torque mode, dir=1 (CCW), 600mA; sample position every
             200ms, when position change inside the observation
             window is small (<= CHECK_DELTA pulses) -> judged
             stationary -> compute capacity from current position,
             no auto-brake (motor keeps pressing)
             pos 25000 = 0% , pos 13000 = 100%

IR distance sensor: DTOF-2510 ToF on a second UART. It reports
ASCII lines 'distance_mm,confidence\n' (distance = float mm,
confidence = int 0~1000). For others modify SensorParser.parse().

Usage: copy the whole file into UIFlow2 Python mode and run.
NOTE: keep this file pure ASCII - UIFlow2 web editor can corrupt
non-ASCII characters on paste. All lines are kept <= 72 chars so
any editor line-wrapping cannot break the code.
"""


import machine, time
import M5
from M5 import *
import m5ui
import lvgl as lv

# ===================== protocol constants =====================
_FHEAD     = 0xC5    # frame head
_FTAIL     = 0x5C    # frame tail
_F_POS     = 0xF2    # absolute position mode move
_F_TORQUE  = 0xF0    # torque mode move
_F_POS_TQ  = 0x64    # set position-loop max current
_F_EN      = 0xFA    # motor enable
_F_ECHO    = 0x6F    # set command echo (NOT used, see warning)
_F_STOP    = 0xFC    # immediate stop (brake)
_F_CLEAR   = 0xFB    # clear state (stall/fail/brake)
_F_RD_POS  = 0x2A    # read realtime position
_F_RD_CLOG = 0x2D    # read stall flag
_F_RD_ARV  = 0x30    # read arrived flag

ADDR = 0x01          # motor slave address

# ===================== RS485 pins (Tab5) =====================
_UART = 1            # UART id
_TX   = 20
_RX   = 21
_DIR  = 34           # DE/RE direction pin
CMD_GAP_MS = 50      # command gap (ms), verified value from CPDR.py

# ===================== motion parameters =====================
SPEED = 10           # speed (RPM)
ACC   = 150          # accel/decel (0~200, 0=no ramp)
POS_OPEN   = 37000   # open lid absolute position
POS_CLOSE  = 25000   # close lid absolute position
TORQUE_MA  = 600     # compress/check torque (mA, 0~3000)
POS_TQ_MA  = 600     # open/close force: position-loop max current
                     # (100~3000mA), raise if lid is too weak
POS_FULL   = 13000   # position for 100% capacity
POS_EMPTY  = 25000   # position for 0% capacity

# direction: 0=CW 1=CCW, flip these if motion direction is wrong
DIR_OPEN     = 0     # open lid: CW
DIR_CLOSE    = 0     # close lid: CW
DIR_COMPRESS = 1     # compress/check: CCW

# ===================== IR distance sensor UART =====================
_SENSOR_UART = 2     # UART id (change to 0 if init fails)
_SENSOR_TX   = 37    # Tab5 M5-Bus UART0 TXD
_SENSOR_RX   = 38    # Tab5 M5-Bus UART0 RXD (cross to sensor)
_SENSOR_BAUD = 921600
AUTO_OPEN_MM  = 200    # auto-open when distance <= 200mm (20cm)
AUTO_CLOSE_MS = 3000   # auto-close 3s after hand leaves

POLL_MS = 200            # motor status poll interval
TASK_TIMEOUT_MS  = 60000 # task timeout protection
COMM_TIMEOUT_MS  = 5000  # motor no-response timeout
                         # (motor cmd echo must be enabled)
                         # set 0 to disable this check
DEBUG_RX = True          # print raw RX bytes / parsed frames
                         # set False once everything works

CHECK_SAMPLES = 5        # stationary window: 5 samples x POLL_MS
                         # = 1s of observation
CHECK_DELTA   = 10       # max position change (pulses) inside the
                         # window, <= this -> judged stationary
TARGET_TOL    = 300      # success band (pulses) around the target
                         # position: stationary within this band
                         # = open/close succeeded

# ===================== UI texts / fonts =====================
# UI is fixed to English: Montserrat fonts have no CJK glyphs
T = {
    'title': 'Smart Trashbin',
    'lid': 'Lid', 'cap': 'Capacity', 'dist': 'Distance',
    'btn_open': 'Open', 'btn_close': 'Close',
    'btn_compress': 'Compress', 'btn_check': 'Check',
    'lid_unknown': 'Unknown', 'lid_opening': 'Opening...',
    'lid_closing': 'Closing...', 'lid_open': 'Opened',
    'lid_close': 'Closed', 'lid_err': 'Error',
    'cap_na': '-- %', 'cap_checking': 'Checking...',
    'dist_na': '-- mm',
}
FONT_S   = lv.font_montserrat_24   # normal size
FONT_BIG = lv.font_montserrat_36   # big size (title / capacity)

# ===================== globals =====================
_uart   = None
_dpin   = None
_sensor = None
_rxbuf  = bytearray()    # motor ack byte stream buffer
_rx_cnt = 0              # valid ack frame counter

m = {'pos': None, 'clog': None, 'arrived': None, 'last_rx': 0}
g = {
    'task': None,        # None/'open'/'close'/'compress'/'check'
    'phase': 0,          # check task: 0=sampling positions
    't0': 0, 'last_poll': 0, 'phase_t0': 0,
    'hist': [],          # check task: position samples
    'wait_pos': False,   # check task: position ack pending
    'pending': None,     # task registered by a button press
}
s = {'last': 0, 'near': False, 'left_t': 0}  # sensor poll + auto lid

g_lid  = None            # lid status label
g_cap  = None            # capacity label
g_dist = None            # IR distance label


# ===================== RS485 / protocol =====================
def _checksum(data):
    s = 0
    for b in data:
        s += b
    return s & 0xFF


def _rs485_send(data):
    """Half-duplex send: DIR=1 -> send -> wait last byte on wire
    -> DIR=0 -> wait CMD_GAP_MS."""
    _dpin.value(1)
    time.sleep_us(200)              # DE setup time
    _uart.write(bytes(data))        # non-blocking write (TX buffer)
    if hasattr(_uart, 'flush'):
        _uart.flush()               # block until last byte on wire
        # tiny margin, then release bus ASAP (late release =
        # motor reply collides with our transmitter)
        time.sleep_us(100)
    else:
        # no flush: estimate shift-out time by baud rate
        time.sleep_ms((len(data) * 10 * 1000) // 115200 + 1)
    _dpin.value(0)                  # back to receive
    time.sleep_ms(CMD_GAP_MS)       # command gap


def _build_pos(addr, direction, acc, speed, pulses):
    """Absolute position move frame (14 bytes, 0xF2)."""
    b = bytearray(14)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_POS
    b[3] = direction                 # 0=CW, 1=CCW
    b[4] = acc
    b[5] = (speed >> 8) & 0xFF
    b[6] = speed & 0xFF
    b[7] = (pulses >> 24) & 0xFF
    b[8] = (pulses >> 16) & 0xFF
    b[9] = (pulses >> 8) & 0xFF
    b[10] = pulses & 0xFF
    b[11] = 0                        # reserved, checksum covers it
    b[12] = _checksum(b[:12])
    b[13] = _FTAIL
    return b


def _build_torque(addr, direction, ma):
    """Torque move frame (8 bytes, 0xF0)."""
    b = bytearray(8)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_TORQUE
    b[3] = direction
    b[4] = (ma >> 8) & 0xFF
    b[5] = ma & 0xFF
    b[6] = _checksum(b[:6])
    b[7] = _FTAIL
    return b


def _build_read(addr, func):
    """Read command frame (5 bytes)."""
    b = bytearray(5)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = func
    b[3] = _checksum(b[:3])
    b[4] = _FTAIL
    return b


def _build_enable(addr, en):
    """Enable frame (6 bytes): en=0 enable, en=1 disable."""
    b = bytearray(6)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_EN
    b[3] = en
    b[4] = _checksum(b[:4])
    b[5] = _FTAIL
    return b


def _build_set_pos_torque(addr, ma):
    """Set position-loop max current frame (7 bytes, 0x64).
    This is the force limit of absolute-position moves,
    range 100~3000mA."""
    b = bytearray(7)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_POS_TQ
    b[3] = (ma >> 8) & 0xFF
    b[4] = ma & 0xFF
    b[5] = _checksum(b[:5])
    b[6] = _FTAIL
    return b


def _build_set_echo(addr, echo):
    """Set command echo frame (6 bytes, 0x6F).
    NOTE (verified on this unit): data=0 turns echo ON (respond),
    data=1 turns it OFF (silent) - inverted vs the STM32 example
    comment. The OFF state persists across reboots, so echo must
    be re-enabled at every boot."""
    b = bytearray(6)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_ECHO
    b[3] = echo
    b[4] = _checksum(b[:4])
    b[5] = _FTAIL
    return b


def _build_stop(addr):
    """Immediate stop frame (5 bytes)."""
    return _build_read(addr, _F_STOP)


def _build_clear(addr):
    """Clear state frame (5 bytes)."""
    return _build_read(addr, _F_CLEAR)


# ===================== ack frame parsing =====================
def _feed_rx():
    n = _uart.any()
    if n:
        d = _uart.read(n)
        if d:
            _rxbuf.extend(d)
            if DEBUG_RX:
                print('RX raw: '
                      + ' '.join('%02X' % b for b in d))


def _next_frame():
    """Cut one valid ack frame from _rxbuf, or None.
    Ack format: C5 + addr + func + err(0x01=ok) + data + cksum + 5C
    NOTE: MicroPython bytearray has no slice deletion,
    use re-assignment instead."""
    global _rxbuf
    while True:
        i = _rxbuf.find(b'\xC5')
        if i < 0:
            _rxbuf = bytearray()
            return None
        if i > 0:
            _rxbuf = _rxbuf[i:]
        t = _rxbuf.find(b'\x5C', 1)
        if t < 0:
            # garbage at head, drop and keep scanning
            if len(_rxbuf) > 64:
                _rxbuf = _rxbuf[1:]
                continue
            return None
        f = bytes(_rxbuf[:t + 1])
        _rxbuf = _rxbuf[t + 1:]
        if len(f) >= 6 and f[-2] == _checksum(f[:-2]):
            return f


def _proc_frames():
    global _rx_cnt
    while True:
        f = _next_frame()
        if f is None:
            break
        if DEBUG_RX:
            print('frame: ' + ' '.join('%02X' % b for b in f))
        if f[3] != 0x01:
            print('motor ack error: 0x%02X' % f[3])
            continue
        _rx_cnt += 1
        m['last_rx'] = time.ticks_ms()
        func, data = f[2], f[4:-2]
        if func == _F_RD_POS and len(data) >= 4:
            p = (data[0] << 24) | (data[1] << 16)
            p = p | (data[2] << 8) | data[3]
            if p >= 0x80000000:
                p -= 0x100000000      # signed int32
            m['pos'] = p
        elif func == _F_RD_CLOG and len(data) >= 1:
            m['clog'] = data[0]
        elif func == _F_RD_ARV and len(data) >= 1:
            m['arrived'] = data[0]


# ============ IR distance sensor (DTOF-2510 ToF) ============
class SensorParser:
    """DTOF-2510 reports ASCII lines 'distance_mm,confidence\n'.
    distance = float mm, confidence = int 0~1000. parse() returns
    distance in mm. For other sensors just modify parse()."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)

    def parse(self):
        while True:
            i = self.buf.find(b'\n')
            if i < 0:
                if len(self.buf) > 64:
                    self.buf = bytearray()  # junk with no newline
                return None
            line = bytes(self.buf[:i])
            self.buf = self.buf[i + 1:]
            j = line.find(b',')
            if j <= 0:
                continue
            try:
                dist_mm = int(float(line[:j].decode().strip()) + 0.5)
            except ValueError:
                continue
            if dist_mm < 20 or dist_mm > 10000:
                continue
            return dist_mm


# ===================== capacity conversion =====================
def pos_to_pct(pos):
    """Position to capacity: 25000=0%, 13000=100%"""
    pct = (POS_EMPTY - pos) * 100 // (POS_EMPTY - POS_FULL)
    return max(0, min(100, pct))


# ===================== task state machine =====================
def _begin_task(name):
    t = time.ticks_ms()
    g.update(task=name, phase=0, t0=t, last_poll=t, phase_t0=t)
    m['pos'] = m['clog'] = m['arrived'] = None
    m['last_rx'] = t
    g['hist'] = []              # reset position sample window
    g['wait_pos'] = False

    # clear old stall/brake state
    _rs485_send(_build_clear(ADDR))
    if name == 'open':
        # set open force, then move
        _rs485_send(_build_set_pos_torque(ADDR, POS_TQ_MA))
        _rs485_send(_build_pos(ADDR, DIR_OPEN, ACC, SPEED, POS_OPEN))
        g_lid.set_text(T['lid_opening'])
    elif name == 'close':
        # set close force, then move
        _rs485_send(_build_set_pos_torque(ADDR, POS_TQ_MA))
        _rs485_send(_build_pos(ADDR, DIR_CLOSE, ACC, SPEED, POS_CLOSE))
        g_lid.set_text(T['lid_closing'])
    elif name == 'compress':
        # send torque once, motor keeps pressing, no brake
        # (exit by Open/Close command)
        _rs485_send(_build_torque(ADDR, DIR_COMPRESS, TORQUE_MA))
        print('compress: torque %dmA hold, exit by open/close'
              % TORQUE_MA)
        g['task'] = None
    else:
        # check
        _rs485_send(_build_torque(ADDR, DIR_COMPRESS, TORQUE_MA))
        g_cap.set_text(T['cap_checking'])


def _abort_task(reason):
    """Force brake only for error protection (timeout / motor no
    response). Normal compress/check never reaches here."""
    _rs485_send(_build_stop(ADDR))
    _rs485_send(_build_clear(ADDR))
    if g['task'] in ('open', 'close'):
        g_lid.set_text(T['lid_err'])
    elif g['task'] == 'check':
        g_cap.set_text(T['cap_na'])
    print('task aborted [%s]: %s' % (g['task'], reason))
    g['task'] = None


def _task_machine(now):
    if g['task'] is None:
        return

    # comm watchdog: no motor ack while a task is running
    if COMM_TIMEOUT_MS and time.ticks_diff(now, m['last_rx']) \
            > COMM_TIMEOUT_MS:
        _abort_task('motor no response, check 485 wiring/'
                    'addr/cmd echo setting')
        return

    if time.ticks_diff(now, g['t0']) > TASK_TIMEOUT_MS:
        _abort_task('timeout')
        return

    task = g['task']

    # ---- open/close ----
    # Same stationary judgement as capacity check: sample position,
    # when stationary AND within TARGET_TOL of the target position
    # -> success.
    if task in ('open', 'close'):
        target = POS_OPEN if task == 'open' else POS_CLOSE
        if time.ticks_diff(now, g['last_poll']) >= POLL_MS:
            g['last_poll'] = now
            g['wait_pos'] = True
            _rs485_send(_build_read(ADDR, _F_RD_POS))
        if g['wait_pos'] and m['pos'] is not None:
            g['wait_pos'] = False
            hist = g['hist']
            hist.append(m['pos'])
            if len(hist) >= CHECK_SAMPLES:
                win = hist[-CHECK_SAMPLES:]
                # stationary: max-min within window <= CHECK_DELTA
                if max(win) - min(win) <= CHECK_DELTA:
                    if abs(win[-1] - target) <= TARGET_TOL:
                        # stationary near target -> success
                        if task == 'open':
                            g_lid.set_text(T['lid_open'])
                        else:
                            g_lid.set_text(T['lid_close'])
                        g['task'] = None
                    else:
                        # stationary but far from target -> blocked
                        _abort_task('stalled away from target')
        return

    # ---- capacity check ----
    # Sample position while pressing; when the change inside the
    # window is small -> judged stationary -> compute capacity.
    # No brake, motor keeps pressing.
    if time.ticks_diff(now, g['last_poll']) >= POLL_MS:
        g['last_poll'] = now
        g['wait_pos'] = True
        _rs485_send(_build_read(ADDR, _F_RD_POS))
    if g['wait_pos'] and m['pos'] is not None:
        g['wait_pos'] = False
        hist = g['hist']
        hist.append(m['pos'])
        if len(hist) >= CHECK_SAMPLES:
            win = hist[-CHECK_SAMPLES:]
            # stationary: max-min within window <= CHECK_DELTA
            if max(win) - min(win) <= CHECK_DELTA:
                pct = pos_to_pct(win[-1])
                g_cap.set_text('%d %%' % pct)
                print('check done: pos=%d, capacity=%d%%'
                      % (win[-1], pct))
                g['task'] = None


# ---------------- IR sensor polling ----------------
def _sensor_poll(now):
    if _sensor is None:
        return
    # auto-close: hand left and stayed away for AUTO_CLOSE_MS
    if s['left_t'] and time.ticks_diff(now, s['left_t']) \
            >= AUTO_CLOSE_MS:
        if g['task'] is None and g['pending'] is None:
            g['pending'] = 'close'
        s['left_t'] = 0
    if time.ticks_diff(now, s['last']) < 200:
        return
    s['last'] = now
    n = _sensor.any()
    if n:
        d = _sensor.read(n)
        if d:
            _sparser.feed(d)
    dist = None
    while True:
        d = _sparser.parse()
        if d is None:
            break
        dist = d            # keep the newest distance
    if dist is not None:
        g_dist.set_text('%d mm' % dist)
        near = dist <= AUTO_OPEN_MM
        if near and not s['near']:
            # object just entered range -> auto open
            s['left_t'] = 0
            if g['task'] is None and g['pending'] is None:
                g['pending'] = 'open'
        elif not near and s['near']:
            # object just left range -> arm auto-close timer
            s['left_t'] = now
        s['near'] = near


# ===================== UI callbacks =====================
def _make_task_cb(name):
    def cb(e):
        if e.code == lv.EVENT.CLICKED:
            if g['task'] is None and g['pending'] is None:
                g['pending'] = name  # register only, loop executes
    return cb


# ===================== entry =====================
def setup():
    global _uart, _dpin, _sensor, _sparser
    global g_lid, g_cap, g_dist

    M5.begin()
    Widgets.setRotation(0)    # portrait 720x1280 (counterclockwise 90)
    m5ui.init()

    page = m5ui.M5Page(bg_c=0x1A1A2E)

    # ---- title (720px portrait width, roughly centered) ----
    m5ui.M5Label(text=T['title'], x=230, y=15,
                 text_c=0xFFFFFF, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_BIG, parent=page)

    # ---- status rows: label left, value right ----
    # lid status
    m5ui.M5Label(text=T['lid'], x=40, y=80,
                 text_c=0xAAAAAA, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_S, parent=page)
    g_lid = m5ui.M5Label(text=T['lid_unknown'], x=200, y=74,
                         text_c=0x00E676, bg_c=0x2A2A3E,
                         bg_opa=255, font=FONT_S, parent=page)
    g_lid.set_size(480, 44)

    # capacity
    m5ui.M5Label(text=T['cap'], x=40, y=140,
                 text_c=0xAAAAAA, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_S, parent=page)
    g_cap = m5ui.M5Label(text=T['cap_na'], x=200, y=128,
                         text_c=0xFFD600, bg_c=0x2A2A3E,
                         bg_opa=255, font=FONT_BIG, parent=page)
    g_cap.set_size(480, 60)

    # IR distance
    m5ui.M5Label(text=T['dist'], x=40, y=215,
                 text_c=0xAAAAAA, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_S, parent=page)
    g_dist = m5ui.M5Label(text=T['dist_na'], x=200, y=209,
                          text_c=0xFFFFFF, bg_c=0x2A2A3E,
                          bg_opa=255, font=FONT_S, parent=page)
    g_dist.set_size(480, 44)

    # ---- 2x2 function buttons (all inside upper half: y <= 640) ----
    btns = [
        (T['btn_open'], 40, 300, 0x0E6655, 'open'),
        (T['btn_close'], 370, 300, 0x1A5276, 'close'),
        (T['btn_compress'], 40, 470, 0x7D6608, 'compress'),
        (T['btn_check'], 370, 470, 0x6C3483, 'check'),
    ]
    for text, x, y, color, name in btns:
        btn = m5ui.M5Button(text=text, x=x, y=y, w=310, h=150,
                            text_c=0xFFFFFF, bg_c=color,
                            font=FONT_S, parent=page)
        btn.add_event_cb(_make_task_cb(name), lv.EVENT.ALL, None)

    page.screen_load()

    # ---- RS485 init + enable motor on boot ----
    _uart = machine.UART(_UART, tx=_TX, rx=_RX,
                         baudrate=115200, bits=8,
                         parity=None, stop=1)
    _dpin = machine.Pin(_DIR, machine.Pin.OUT, value=0)
    _rs485_send(_build_enable(ADDR, 0))     # en=0 enable
    # this unit: 0x6F data=0 turns echo ON, data=1 turns it OFF
    # (state persists across reboots, must re-enable every boot)
    _rs485_send(_build_set_echo(ADDR, 0))

    # quick comm check: read position once, expect a reply
    m['pos'] = None
    _rs485_send(_build_read(ADDR, _F_RD_POS))
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 500:
        _feed_rx()
        _proc_frames()
        if m['pos'] is not None:
            break
        time.sleep_ms(10)
    if m['pos'] is not None:
        print('motor comm OK, pos=%d' % m['pos'])
    else:
        print('motor comm FAILED - no reply to read position')

    # ---- IR sensor UART (failure must not block the program) ----
    try:
        _sensor = machine.UART(_SENSOR_UART, tx=_SENSOR_TX,
                               rx=_SENSOR_RX, baudrate=_SENSOR_BAUD,
                               bits=8, parity=None, stop=1)
        _sparser = SensorParser()
    except Exception as e:
        print('sensor UART init failed (change _SENSOR_UART):', e)
        _sensor = None

    print('SmartTrashbin ready. SPEED=%dRPM ACC=%d'
          % (SPEED, ACC))


def loop():
    M5.update()
    now = time.ticks_ms()

    _feed_rx()
    _proc_frames()

    if g['pending'] is not None and g['task'] is None:
        _begin_task(g['pending'])
        g['pending'] = None

    _task_machine(now)
    _sensor_poll(now)

    time.sleep_ms(10)


if __name__ == "__main__":
    try:
        setup()
        while True:
            loop()
    except (Exception, KeyboardInterrupt) as e:
        try:
            from utility import print_error_msg
            print_error_msg(e)
        except ImportError:
            print("error:", e)
