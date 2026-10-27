"""
CoreXY 双电机控制 — M5Stack Tab5 (UIFlow2.0 / MicroPython)
============================================================
正点原子 PD42S1 × 2 | RS485 总线 | 相对位置模式 (0xF3)
A 电机(左)=0x02   B 电机(右)=0x01

功能:
  - 上/下/左/右 4 个方向按键，对应 Y+ / Y- / X- / X+ 移动
  - 步距选择: 1 / 10 / 30 / 50 / 100 mm
  - 速度 100 RPM, 加减速 10 (见下方常量)

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
_F_POS_REL = 0xF3    # 相对位置模式运动
_F_EN      = 0xFA    # 电机使能
_F_STOP    = 0xFC    # 立即停止

# ===================== 电机地址 =====================
ADDR_A = 0x02   # A 电机 = 左
ADDR_B = 0x01   # B 电机 = 右

# ===================== RS485 引脚 (Tab5) =====================
_UART = 1       # 串口编号
_TX   = 20
_RX   = 21
_DIR  = 34      # DE/RE 方向脚

# ===================== 运动参数 =====================
SPEED = 100          # 速度 (RPM)
ACC   = 10           # 加减速 (0~200, 0=不加速)
PULSE_PER_REV = 51200
MM_PER_REV    = 40
PULSE_PER_MM  = PULSE_PER_REV // MM_PER_REV   # 1280 脉冲/mm
CMD_GAP_MS    = 50        # 指令间隔(ms)，唯一需要调优的项。2ms 不稳、30ms 太保守，二分 8~20 取最小

# ===================== 方向/正反转常量 (调试时翻转 ±1 即可) =====================
DIR_X = -1     # X 轴正方向系数（已反向：左右）
DIR_Y = -1     # Y 轴正方向系数（已反向：上下）
INV_A = 1      # A 电机(左,0x02)方向系数
INV_B = 1      # B 电机(右,0x01)方向系数

# ===================== 步距选项 (mm) =====================
DISTANCES = [1, 10, 30, 50, 100]

# ===================== 全局 =====================
_uart  = None
_dpin  = None
g_dist = 10              # 当前步距 (mm)
g_lbl  = None            # 状态标签


# ===================== RS485 / 协议 =====================
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
    time.sleep_ms(CMD_GAP_MS)                       # 指令间隔(唯一调优项)


def _build_pos_rel(addr, direction, acc, speed, pulses):
    """相对位置运动帧 (14 字节)"""
    b = bytearray(14)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_POS_REL
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
    b = bytearray(5)
    b[0] = _FHEAD
    b[1] = addr
    b[2] = _F_STOP
    b[3] = _checksum(b[:3])
    b[4] = _FTAIL
    return b


# ===================== 运动 =====================
def _do_move(mx, my):
    """按名义方向 (mx,my ∈ {-1,0,1}) 移动一个步距"""
    dx = DIR_X * mx * g_dist      # 期望 X 位移 (mm)
    dy = DIR_Y * my * g_dist      # 期望 Y 位移 (mm)

    # CoreXY 逆运动学: A = X+Y, B = X-Y
    a_mm = (dx + dy) * INV_A
    b_mm = (dx - dy) * INV_B

    # mm -> 脉冲
    a_p = round(a_mm * PULSE_PER_MM)
    b_p = round(b_mm * PULSE_PER_MM)

    # 有符号脉冲 -> 方向 + 绝对值
    a_dir = 0 if a_p >= 0 else 1
    b_dir = 0 if b_p >= 0 else 1

    _rs485_send(_build_pos_rel(ADDR_A, a_dir, ACC, SPEED, abs(a_p)))
    _rs485_send(_build_pos_rel(ADDR_B, b_dir, ACC, SPEED, abs(b_p)))
    print("move X%+d Y%+d mm | A=%d B=%d pulses" % (dx, dy, abs(a_p), abs(b_p)))


def _do_stop():
    _rs485_send(_build_stop(ADDR_A))
    _rs485_send(_build_stop(ADDR_B))
    print("stop")


# ===================== UI 回调 =====================
def _make_move_cb(mx, my):
    def cb(e):
        if e.code == lv.EVENT.CLICKED:
            _do_move(mx, my)
    return cb


def _make_dist_cb(mm):
    def cb(e):
        global g_dist
        if e.code == lv.EVENT.CLICKED:
            g_dist = mm
            if g_lbl:
                g_lbl.set_text("STEP %d mm" % g_dist)
    return cb


def _make_stop_cb():
    def cb(e):
        if e.code == lv.EVENT.CLICKED:
            _do_stop()
            if g_lbl:
                g_lbl.set_text("STOPPED")
    return cb


# ===================== 入口 =====================
def setup():
    global _uart, _dpin, g_lbl

    M5.begin()
    Widgets.setRotation(1)
    m5ui.init()

    page = m5ui.M5Page(bg_c=0x1A1A2E)

    # 标题
    m5ui.M5Label(text="CoreXY Control", x=60, y=30,
                 text_c=0xFFFFFF, bg_c=0x1A1A2E, bg_opa=0,
                 font=lv.font_montserrat_36, parent=page)

    # 状态标签
    g_lbl = m5ui.M5Label(text="STEP %d mm" % g_dist, x=60, y=110,
                         text_c=0x00E676, bg_c=0x2A2A3E, bg_opa=255,
                         font=lv.font_montserrat_24, parent=page)
    g_lbl.set_size(260, 46)

    # ---- 方向按键 (D-pad) ----
    cx, cy = 360, 460
    bw, bh = 160, 120
    gap = 80
    dbtn = {
        'UP':    (cx - bw // 2, cy - bh - gap, _make_move_cb(0, 1)),
        'DOWN':  (cx - bw // 2, cy + gap,      _make_move_cb(0, -1)),
        'LEFT':  (cx - bw - gap, cy - bh // 2, _make_move_cb(-1, 0)),
        'RIGHT': (cx + gap,      cy - bh // 2, _make_move_cb(1, 0)),
    }
    for text, (x, y, cb) in dbtn.items():
        btn = m5ui.M5Button(text=text, x=x, y=y, w=bw, h=bh,
                            text_c=0xFFFFFF, bg_c=0x2196F3,
                            font=lv.font_montserrat_24, parent=page)
        btn.add_event_cb(cb, lv.EVENT.ALL, None)

    # ---- 步距选择 ----
    m5ui.M5Label(text="STEP (mm)", x=860, y=150,
                 text_c=0xFFFFFF, bg_c=0x1A1A2E, bg_opa=0,
                 font=lv.font_montserrat_24, parent=page)

    x, w, h = 860, 240, 64
    y = 200
    for mm in DISTANCES:
        btn = m5ui.M5Button(text=str(mm), x=x, y=y, w=w, h=h,
                            text_c=0xFFFFFF, bg_c=0x3F51B5,
                            font=lv.font_montserrat_24, parent=page)
        btn.add_event_cb(_make_dist_cb(mm), lv.EVENT.ALL, None)
        y += 78

    # ---- 停止按键 ----
    sbtn = m5ui.M5Button(text="STOP", x=x, y=y + 6, w=w, h=h,
                         text_c=0xFFFFFF, bg_c=0xE53935,
                         font=lv.font_montserrat_24, parent=page)
    sbtn.add_event_cb(_make_stop_cb(), lv.EVENT.ALL, None)

    page.screen_load()

    # ---- RS485 初始化 + 上电使能两个电机 ----
    _uart = machine.UART(_UART, tx=_TX, rx=_RX,
                         baudrate=115200, bits=8, parity=None, stop=1)
    _dpin = machine.Pin(_DIR, machine.Pin.OUT, value=0)
    _rs485_send(_build_enable(ADDR_A, 0))   # en=0 使能
    _rs485_send(_build_enable(ADDR_B, 0))

    print("CoreXY ready. STEP=%dmm SPEED=%dRPM ACC=%d" % (g_dist, SPEED, ACC))


def loop():
    M5.update()
    time.sleep_ms(20)


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
