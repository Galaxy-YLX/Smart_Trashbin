"""
智能垃圾桶控制 — M5Stack Tab5 (UIFlow2.0 / MicroPython)
============================================================
正点原子 PD42S1 | RS485 总线 | 单电机(从机地址 0x01)

485 接口与协议帧参考自本目录 CPDR.py(已在 Tab5 上验证), 本项目补充:
  - 绝对位置模式(0xF2)  -> 打开/关闭盖子
  - 力矩模式(0xF0)      -> 压缩垃圾/容量检测
  - 读取指令: 0x2A实时位置 / 0x2D堵转标志 / 0x30到位状态

按键功能:
  打开盖子 : 绝对位置模式 正转, 加减速150, 速度10RPM, 绝对位置37000
  关闭盖子 : 绝对位置模式 正转, 加减速150, 速度10RPM, 绝对位置25000
  压缩垃圾 : 力矩模式 反转, 力矩600mA, 电机保持力矩压紧, 不自动刹车
  容量检测 : 力矩模式 反转, 力矩600mA, 检测到堵转(压不动)时读当前位置换算
             容量百分比(不刹车, 电机继续保压), 位置25000=0% 位置13000=100%

红外距离传感器: 另开一路串口读取, 默认按 TFmini/TF-Luna 帧解析
  (59 59 distL distH strL strH mode 00 sum), 其它协议改 SensorParser.parse()

使用: 复制本文件全部内容到 UIFlow2 的 Python 模式运行。
"""


import machine, time
import M5
from M5 import *
import m5ui
import lvgl as lv

# ===================== 协议常量 =====================
_FHEAD     = 0xC5    # 帧头
_FTAIL     = 0x5C    # 帧尾
_F_POS     = 0xF2    # 绝对位置模式运动
_F_TORQUE  = 0xF0    # 力矩模式运动
_F_EN      = 0xFA    # 电机使能
_F_STOP    = 0xFC    # 立即停止
_F_CLEAR   = 0xFB    # 清除状态(堵转/失败/刹车)
_F_RD_POS  = 0x2A    # 读实时位置(应答data[0..3]=int32大端)
_F_RD_CLOG = 0x2D    # 读堵转标志(应答data[0]: 0=未堵转 1=堵转)
_F_RD_ARV  = 0x30    # 读到位状态(应答data[0]: 0=未到位 1=到位)

ADDR = 0x01          # 电机从机地址

# ===================== RS485 引脚 (Tab5, 同CPDR.py) =====================
_UART = 1            # 串口编号
_TX   = 20
_RX   = 21
_DIR  = 34           # DE/RE 方向脚
CMD_GAP_MS = 50      # 指令间隔(ms), CPDR.py中验证过的最小稳定值

# ===================== 运动参数 =====================
SPEED = 10           # 速度 (RPM)
ACC   = 150          # 加减速 (0~200, 0=不加速)
POS_OPEN   = 37000   # 开盖绝对位置
POS_CLOSE  = 25000   # 关盖绝对位置
TORQUE_MA  = 600     # 压缩/容量检测力矩 (mA, 0~3000)
POS_FULL   = 13000   # 100%容量对应位置
POS_EMPTY  = 25000   # 0%容量对应位置

# 方向: 0=CW(正转) 1=CCW(反转), 调试时翻转即可
DIR_OPEN     = 0     # 打开盖子: 正转
DIR_CLOSE    = 0     # 关闭盖子: 正转
DIR_COMPRESS = 1     # 压缩/容量检测: 反转

# ===================== 红外距离传感器串口 =====================
_SENSOR_UART = 2     # 串口编号(初始化失败可改为0)
_SENSOR_TX   = 37    # Tab5 M5-Bus UART0 TXD
_SENSOR_RX   = 38    # Tab5 M5-Bus UART0 RXD (TXD<->RXD 与传感器交叉)
_SENSOR_BAUD = 115200

POLL_MS = 200            # 查询电机状态的间隔
TASK_TIMEOUT_MS  = 60000 # 任务超时保护
COMM_TIMEOUT_MS  = 5000  # 电机无应答超时(电机需开启指令应答; 0=关闭此检测)

# ===================== 界面文本/字体 =====================
# 界面固定英文: Montserrat内置字体不含中文字形
T = {
    'title': 'Smart Trashbin',
    'lid': 'Lid', 'cap': 'Capacity', 'dist': 'Distance',
    'btn_open': 'Open', 'btn_close': 'Close',
    'btn_compress': 'Compress', 'btn_check': 'Check',
    'lid_unknown': 'Unknown', 'lid_opening': 'Opening...',
    'lid_closing': 'Closing...', 'lid_open': 'Opened',
    'lid_close': 'Closed', 'lid_err': 'Error',
    'cap_na': '-- %', 'cap_checking': 'Checking...', 'dist_na': '-- cm',
}
FONT_S   = lv.font_montserrat_24   # 普通字号
FONT_BIG = lv.font_montserrat_36   # 大字号(标题/容量)

# ===================== 全局 =====================
_uart   = None
_dpin   = None
_sensor = None
_rxbuf  = bytearray()    # 电机应答字节流缓冲
_rx_cnt = 0              # 收到的合法应答帧计数(用于通信诊断)

m = {'pos': None, 'clog': None, 'arrived': None, 'last_rx': 0}   # 电机最新反馈
g = {'task': None,       # None/'open'/'close'/'compress'/'check'
     'phase': 0,         # 0=等堵转 1=等位置应答
     't0': 0, 'last_poll': 0, 'phase_t0': 0,
     'pending': None}    # 待执行任务(按键只登记, 主循环执行)
s = {'last': 0}          # 传感器轮询计时

g_lid  = None            # 开盖状态标签
g_cap  = None            # 当前容量标签
g_dist = None            # 红外距离标签


# ===================== RS485 / 协议(同CPDR.py) =====================
def _checksum(data):
    s = 0
    for b in data:
        s += b
    return s & 0xFF


def _rs485_send(data):
    """半双工发送: DIR=1 -> 发 -> 等末字节上线 -> DIR=0 -> 间隔 CMD_GAP_MS"""
    _dpin.value(1)
    time.sleep_us(200)                              # DE 建立时间
    _uart.write(bytes(data))                        # 非阻塞写入(只进 TX 缓冲)
    if hasattr(_uart, 'flush'):
        _uart.flush()                               # 阻塞到真正发完
        time.sleep_ms(2)                            # 末字节(0x5C)上线余量
    else:
        time.sleep_ms((len(data) * 10 * 1000) // 115200 + 3)  # 无 flush 时按波特率估算
    _dpin.value(0)                                  # 发完切接收
    time.sleep_ms(CMD_GAP_MS)                       # 指令间隔


def _build_pos(addr, direction, acc, speed, pulses):
    """绝对位置运动帧 (14 字节, 0xF2)"""
    b = bytearray(14)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_POS
    b[3] = direction                      # 0=CW, 1=CCW
    b[4] = acc
    b[5] = (speed >> 8) & 0xFF
    b[6] = speed & 0xFF
    b[7] = (pulses >> 24) & 0xFF
    b[8] = (pulses >> 16) & 0xFF
    b[9] = (pulses >> 8) & 0xFF
    b[10] = pulses & 0xFF
    b[11] = 0                             # 保留字节, 校验和覆盖它
    b[12] = _checksum(b[:12])
    b[13] = _FTAIL
    return b


def _build_torque(addr, direction, ma):
    """力矩运动帧 (8 字节, 0xF0)"""
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
    """读指令帧 (5 字节)"""
    b = bytearray(5)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = func
    b[3] = _checksum(b[:3])
    b[4] = _FTAIL
    return b


def _build_enable(addr, en):
    """使能帧 (6 字节): en=0 使能, en=1 失能"""
    b = bytearray(6)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_EN
    b[3] = en
    b[4] = _checksum(b[:4])
    b[5] = _FTAIL
    return b


def _build_stop(addr):
    """立即停止帧 (5 字节)"""
    return _build_read(addr, _F_STOP)


def _build_clear(addr):
    """清除状态帧 (5 字节)"""
    return _build_read(addr, _F_CLEAR)


# ===================== 应答帧解析 =====================
def _feed_rx():
    n = _uart.any()
    if n:
        d = _uart.read(n)
        if d:
            _rxbuf.extend(d)


def _next_frame():
    """从 _rxbuf 切出一帧合法应答, 无则返回 None。
    应答格式: C5 + 地址 + 功能码 + 错误码(0x01=成功) + 数据 + 校验和 + 5C
    注意: MicroPython的bytearray不支持del切片, 用切片重新赋值代替。"""
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
            if len(_rxbuf) > 64:            # 头部垃圾数据, 丢掉继续找
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
        if f[3] != 0x01:
            print('电机应答错误码: 0x%02X' % f[3])
            continue
        _rx_cnt += 1
        m['last_rx'] = time.ticks_ms()
        func, data = f[2], f[4:-2]
        if func == _F_RD_POS and len(data) >= 4:
            m['pos'] = int.from_bytes(data[:4], 'big', True)
        elif func == _F_RD_CLOG and len(data) >= 1:
            m['clog'] = data[0]
        elif func == _F_RD_ARV and len(data) >= 1:
            m['arrived'] = data[0]


# ===================== 红外距离传感器(默认TFmini协议) =====================
class SensorParser:
    """TFmini/TF-Luna: 59 59 distL distH strL strH mode 00 sum(前8字节累加低8位)
    其它协议的传感器只需修改 parse()。"""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)

    def parse(self):
        while True:
            i = self.buf.find(b'\x59\x59')
            if i < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]    # 只留最后1字节(可能是帧头的一半)
                return None
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < 9:
                return None
            f = self.buf[:9]
            if f[8] == _checksum(f[:8]):
                self.buf = self.buf[9:]
                return f[2] | (f[3] << 8)       # 距离, 单位cm
            self.buf = self.buf[1:]


# ===================== 容量换算 =====================
def pos_to_pct(pos):
    """位置换算容量: 25000=0%, 13000=100%"""
    pct = (POS_EMPTY - pos) * 100 // (POS_EMPTY - POS_FULL)
    return max(0, min(100, pct))


# ===================== 任务状态机(不阻塞UI) =====================
def _begin_task(name):
    t = time.ticks_ms()
    g.update(task=name, phase=0, t0=t, last_poll=t, phase_t0=t)
    m['pos'] = m['clog'] = m['arrived'] = None
    m['last_rx'] = t

    _rs485_send(_build_clear(ADDR))          # 清掉旧的堵转/刹车状态
    if name == 'open':
        _rs485_send(_build_pos(ADDR, DIR_OPEN, ACC, SPEED, POS_OPEN))
        g_lid.set_text(T['lid_opening'])
    elif name == 'close':
        _rs485_send(_build_pos(ADDR, DIR_CLOSE, ACC, SPEED, POS_CLOSE))
        g_lid.set_text(T['lid_closing'])
    elif name == 'compress':
        # 压缩: 只发一次力矩指令, 电机保持力矩压紧, 不刹车(由开关盖指令退出)
        _rs485_send(_build_torque(ADDR, DIR_COMPRESS, TORQUE_MA))
        print('compress: torque %dmA hold, exit by open/close' % TORQUE_MA)
        g['task'] = None
    else:  # check
        _rs485_send(_build_torque(ADDR, DIR_COMPRESS, TORQUE_MA))
        g_cap.set_text(T['cap_checking'])


def _abort_task(reason):
    """仅异常保护(超时/电机无应答)时强制刹车; 正常压缩/容量检测不会走到这里。"""
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

    # 通信看门狗: 任务中一直收不到电机应答
    if COMM_TIMEOUT_MS and time.ticks_diff(now, m['last_rx']) > COMM_TIMEOUT_MS:
        _abort_task('电机无应答, 检查485接线/从机地址/是否开启指令应答(0x6F=1)')
        return

    if time.ticks_diff(now, g['t0']) > TASK_TIMEOUT_MS:
        _abort_task('超时')
        return

    task = g['task']

    # ---- 开盖/关盖: 等到位 ----
    if task in ('open', 'close'):
        if time.ticks_diff(now, g['last_poll']) >= POLL_MS:
            g['last_poll'] = now
            _rs485_send(_build_read(ADDR, _F_RD_ARV))
        if m['arrived'] == 1:
            g_lid.set_text(T['lid_open'] if task == 'open' else T['lid_close'])
            g['task'] = None
        return

    # ---- 容量检测: 等堵转 -> 读位置(不刹车, 电机继续保压) ----
    if g['phase'] == 0:
        if time.ticks_diff(now, g['last_poll']) >= POLL_MS:
            g['last_poll'] = now
            _rs485_send(_build_read(ADDR, _F_RD_CLOG))
        if m['clog'] == 1:                     # 压不动了, 直接读位置
            g['phase'] = 1
            g['last_poll'] = now
            m['pos'] = None
            _rs485_send(_build_read(ADDR, _F_RD_POS))

    elif g['phase'] == 1:                      # 等位置应答
        if m['pos'] is None and time.ticks_diff(now, g['last_poll']) >= POLL_MS:
            g['last_poll'] = now
            _rs485_send(_build_read(ADDR, _F_RD_POS))   # 超时重发
        if m['pos'] is not None:
            pct = pos_to_pct(m['pos'])
            g_cap.set_text('%d %%' % pct)
            print('check done: pos=%d, capacity=%d%%' % (m['pos'], pct))
            g['task'] = None


# ---------------- 红外传感器轮询 ----------------
def _sensor_poll(now):
    if _sensor is None:
        return
    if time.ticks_diff(now, s['last']) < 200:
        return
    s['last'] = now
    n = _sensor.any()
    if n:
        d = _sensor.read(n)
        if d:
            _sparser.feed(d)
    dist = _sparser.parse()
    if dist is not None:
        g_dist.set_text('%d cm' % dist)


# ===================== UI 回调 =====================
def _make_task_cb(name):
    def cb(e):
        if e.code == lv.EVENT.CLICKED:
            if g['task'] is None and g['pending'] is None:
                g['pending'] = name          # 只登记, 主循环里执行
    return cb


# ===================== 入口 =====================
def setup():
    global _uart, _dpin, _sensor, _sparser, g_lid, g_cap, g_dist

    M5.begin()
    Widgets.setRotation(1)
    m5ui.init()

    page = m5ui.M5Page(bg_c=0x1A1A2E)

    # 标题
    m5ui.M5Label(text=T['title'], x=430, y=25,
                 text_c=0xFFFFFF, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_BIG, parent=page)

    # 开盖状态
    m5ui.M5Label(text=T['lid'], x=120, y=110,
                 text_c=0xAAAAAA, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_S, parent=page)
    g_lid = m5ui.M5Label(text=T['lid_unknown'], x=320, y=104,
                         text_c=0x00E676, bg_c=0x2A2A3E, bg_opa=255,
                         font=FONT_S, parent=page)
    g_lid.set_size(300, 46)

    # 当前容量
    m5ui.M5Label(text=T['cap'], x=120, y=190,
                 text_c=0xAAAAAA, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_S, parent=page)
    g_cap = m5ui.M5Label(text=T['cap_na'], x=320, y=178,
                         text_c=0xFFD600, bg_c=0x2A2A3E, bg_opa=255,
                         font=FONT_BIG, parent=page)
    g_cap.set_size(300, 60)

    # 红外距离
    m5ui.M5Label(text=T['dist'], x=120, y=280,
                 text_c=0xAAAAAA, bg_c=0x1A1A2E, bg_opa=0,
                 font=FONT_S, parent=page)
    g_dist = m5ui.M5Label(text=T['dist_na'], x=320, y=274,
                          text_c=0xFFFFFF, bg_c=0x2A2A3E, bg_opa=255,
                          font=FONT_S, parent=page)
    g_dist.set_size(300, 46)

    # ---- 2x2 功能按键 ----
    btns = [
        (T['btn_open'],     100, 380, 0x0E6655, _make_task_cb('open')),
        (T['btn_close'],    660, 380, 0x1A5276, _make_task_cb('close')),
        (T['btn_compress'], 100, 560, 0x7D6608, _make_task_cb('compress')),
        (T['btn_check'],    660, 560, 0x6C3483, _make_task_cb('check')),
    ]
    for text, x, y, color, cb in btns:
        btn = m5ui.M5Button(text=text, x=x, y=y, w=520, h=150,
                            text_c=0xFFFFFF, bg_c=color,
                            font=FONT_S, parent=page)
        btn.add_event_cb(cb, lv.EVENT.ALL, None)

    page.screen_load()

    # ---- RS485 初始化 + 上电使能电机 ----
    _uart = machine.UART(_UART, tx=_TX, rx=_RX,
                         baudrate=115200, bits=8, parity=None, stop=1)
    _dpin = machine.Pin(_DIR, machine.Pin.OUT, value=0)
    _rs485_send(_build_enable(ADDR, 0))     # en=0 使能

    # ---- 红外传感器串口(初始化失败不阻塞主程序) ----
    try:
        _sensor = machine.UART(_SENSOR_UART, tx=_SENSOR_TX, rx=_SENSOR_RX,
                               baudrate=_SENSOR_BAUD, bits=8, parity=None, stop=1)
        _sparser = SensorParser()
    except Exception as e:
        print('传感器串口初始化失败(可改 _SENSOR_UART 编号):', e)
        _sensor = None

    print('SmartTrashbin ready. SPEED=%dRPM ACC=%d' % (SPEED, ACC))


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
