"""WL1606 CW/packet TX and short-packet RX test through CH347.

Examples::
    python wl160x_test_mode.py --tx_test_mode 0 --chan 40 --tx_pwr 1
    python wl160x_test_mode.py --len 20 --pkt_type 0x55 --tx_cnt 100
    python wl160x_test_mode.py --mode 1 --pkt_type 0x55 --len 10 --out_rx_result_time 1000
    python wl160x_test_mode.py --dry-run --tx_cnt 2 --verbose

References: source/Dri_Wl1606.c (Init/TxLoop), both WL1606 TX register
sequences, wl1606_脚本参数说明.txt and wl1606功率档位映射.png.
The packet sequence defines native WL1606 1-Mbit/s, five-byte sync, no
whitening/auto-ACK. HS6220 software guards/CRC in main.c are not enabled.
Payload bits are reversed before FIFO writes, as in the packet trace
(0x20..0x26 -> 0x04,0x84,0x44,0xC4,0x24,0xA4,0x64).

0..63 bytes use fixed length. 64..255 bytes follow LONG_PAYLOAD_EN:
length byte (not bit-reversed), initial FIFO fill, then threshold refills.
Long packets require hardware validation: the C source calls for 10-Mbit/s
SPI; even at 15 MHz, Python call overhead and the 2-us byte interval
may underrun the FIFO.
--dry-run validates logic with simulated flags; it does not validate RF.
"""

import argparse
import ctypes
import math
import random
import sys
import time
from ctypes import POINTER, Structure, c_bool, c_ubyte, c_ulong, c_ushort
from pathlib import Path
import re
from datetime import datetime

# Descending output power, exactly as shown in the supplied mapping image.
POWER_TABLE = (
    (0x0C, 8), (0x08, 3), (0x18, 1.7), (0x28, 0.3), (0x38, -0.8),
    (0x48, -1.7), (0x58, -2.7), (0x68, -3.8), (0x78, -5.2),
    (0x88, -6.8), (0x98, -7.8), (0xA8, -9), (0xB8, -10),
    (0xC8, -18), (0xD8, -27),
)
RF_INIT = (
    (0x04, 0xC2), (0x11, 0x3A), (0x12, 0x0C), (0x22, 0x08),
    (0x23, 0x08), (0x2D, 0x00), (0x2E, 0x09), (0x34, 0x1A),
    (0x35, 0x40), (0x50, 0xF8),
)
CW_INIT = (
    (0x06, 0x7A), (0x02, 0x57), (0x04, 0xC2), (0x11, 0x3A),
    (0x12, 0x0C), (0x13, 0x00), (0x22, 0x08), (0x23, 0x08),
    (0x2D, 0x00), (0x2E, 0x09), (0x32, 0x1F), (0x34, 0x1A),
    (0x35, 0x40), (0x37, 0x00), (0x50, 0xF8), (0x01, 0xF9),
    (0x46, 0x80), (0x44, 0xB0), (0x16, 0x83),
)

SPI_CLK_MHz = {0:"60MHz", 
               1:"30MHz", 
               2:"15MHz", 
               3:"7.5MHz", 
               4:"3.75MHz", 
               5:"1.875MHz", 
               6:"937.5kHz", 
               7:"468.75kHz"}

BIT_REVERSE = bytes(int(f"{value:08b}"[::-1], 2) for value in range(256))
FIFO_SIZE = 63
FIFO_FULL_THRESHOLD = 33
FIFO_EMPTY_THRESHOLD = 13
PKT_FLAG = 0x40
FIFO_FLAG = 0x20
RX_TIMEOUT_SECONDS = 1.0


class SPI_CONFIG(Structure):
    # Same ABI/layout as the working wl160x_cfg_reg_ok.py, no packing override.
    _fields_ = [
        ("mMode", c_ubyte), ("mClock", c_ubyte), ("mByteOrder", c_ubyte),
        ("mSPIWriteReadInterval", c_ushort), ("mSpiOutDefaultData", c_ubyte),
        ("mChipSelect", c_ulong), ("CS1Polarity", c_ubyte),
        ("CS2Polarity", c_ubyte), ("mIsAutoDeactiveCS", c_ushort),
        ("mActiveDelayNum", c_ushort), ("mDelayNum", c_ulong),
    ]


class CH347SPI:
    def __init__(self, index=0, dll_path=None):
        self.index = index
        self.opened = False
        self.dll_path = Path(dll_path) if dll_path else Path(__file__).with_name("CH347DLLA64.dll")
        self.dll = None

    def open(self):
        if sys.platform != "win32" or ctypes.sizeof(ctypes.c_void_p) != 8:
            raise RuntimeError("CH347DLLA64.dll 需要 Windows 64 位 Python")
        self.dll = ctypes.WinDLL(str(self.dll_path.resolve()))
        signatures = {
            # HANDLE is pointer-sized; DWORD is 32-bit on Windows.
            "CH347OpenDevice": ([c_ulong], ctypes.c_void_p),
            "CH347CloseDevice": ([c_ulong], c_bool),
            "CH347SPI_Init": ([c_ulong, POINTER(SPI_CONFIG)], c_bool),
            "CH347SPI_ChangeCS": ([c_ulong, c_ubyte], c_bool),
            "CH347SPI_WriteRead": ([c_ulong, c_ulong, c_ulong, POINTER(c_ubyte)], c_bool),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.dll, name)
            function.argtypes, function.restype = args, result
        handle = self.dll.CH347OpenDevice(self.index)
        if handle in (None, 0xFFFFFFFF, ctypes.c_void_p(-1).value):
            raise RuntimeError(f"无法打开 CH347，设备索引 {self.index}")
        self.opened = True
        cfg = SPI_CONFIG()
        cfg.mMode = 1
        cfg.mClock = 5  # CH347 clock. 2: 60 MHz / 2**2 = 15 MHz
        cfg.mByteOrder = 1
        # 0：transfer_fifo时，每个tx_data之间连续发送，无间隔； 
        # 1：transfer_fifo时，每个tx_data之间间隔257us发送； 
        # 2：transfer_fifo时，每个tx_data之间间隔513us发送；
        cfg.mSPIWriteReadInterval = 0
        cfg.mSpiOutDefaultData = 0x00
        cfg.mChipSelect = 0x80
        cfg.mIsAutoDeactiveCS = 1

        # # JXX: add
        # cfg.mActiveDelayNum = 0
        # cfg.mDelayNum = 0

        self.spi_clk = float(re.search(r"[+-]?\d*\.?\d+", SPI_CLK_MHz[cfg.mClock]).group())
        
        try:
            if not self.dll.CH347SPI_Init(self.index, ctypes.byref(cfg)):
                raise RuntimeError("CH347SPI_Init 失败")
        except BaseException:
            self.close()
            raise
        print(f"CH347 SPI: Mode 1, SPI CLK: {SPI_CLK_MHz[cfg.mClock]}, MSB first, CS0")

    def transfer(self, address, value=0, read=False):
        if not self.opened:
            raise RuntimeError("CH347 尚未打开")
        if not 0 <= address <= 0x7F or not 0 <= value <= 0xFF:
            raise ValueError("寄存器地址/数据越界")
        buf = (c_ubyte * 2)(address | (0x80 if read else 0), 0xFF if read else value)
        if not self.dll.CH347SPI_ChangeCS(self.index, 0):
            raise RuntimeError("CH347 拉低 CS 失败")
        try:
            success = self.dll.CH347SPI_WriteRead(self.index, 0x80, 2, buf)
        finally:
            released = self.dll.CH347SPI_ChangeCS(self.index, 1)
        if not success or not released:
            raise RuntimeError(f"SPI {'read' if read else 'write'} 0x{address:02X} 失败")
        return int(buf[1]) if read else value

    def close(self):
        if self.opened:
            self.opened = False
            if not self.dll.CH347CloseDevice(self.index):
                raise RuntimeError("CH347CloseDevice 失败")

    def read_fifo(self, length):
        """Read reg100 continuously under one CS (manual section 7, Fig.7)."""
        if not self.opened:
            raise RuntimeError("CH347 尚未打开")
        if not 0 <= length <= FIFO_SIZE:
            raise ValueError("RX FIFO 单次读取长度必须在 0..63 之间")
        if not length:
            return b""
        buf = (c_ubyte * (length + 1))(0xE4, *([0xFF] * length))
        if not self.dll.CH347SPI_ChangeCS(self.index, 0):
            raise RuntimeError("CH347 拉低 CS 失败")
        try:
            success = self.dll.CH347SPI_WriteRead(self.index, 0x80, len(buf), buf)
        finally:
            released = self.dll.CH347SPI_ChangeCS(self.index, 1)
        if not success or not released:
            raise RuntimeError(f"SPI FIFO 连续读取失败 ({length} 字节)")
        return bytes(buf[1:])

    def transfer_fifo(self, data):
        """Write FIFO per WL1606A manual Fig.7: one 0x64 then N bytes.

        Keep CS active for the entire WriteRead call. This interface sends
        the supplied wire bytes unchanged; payload bit reversal is done by
        send_packet. An empty payload does not generate an SPI transaction.
        The caller is responsible for the available FIFO capacity.
        """
        if not self.opened:
            raise RuntimeError("CH347 尚未打开")
        data = bytes(data)
        if not data:
            return 0
        buf = (c_ubyte * (len(data) + 1))(0x64, *data)
        if not self.dll.CH347SPI_ChangeCS(self.index, 0):
            raise RuntimeError("CH347 拉低 CS 失败")
        try:
            success = self.dll.CH347SPI_WriteRead(self.index, 0x80, len(buf), buf)
        finally:
            released = self.dll.CH347SPI_ChangeCS(self.index, 1)
        if not success or not released:
            raise RuntimeError(f"SPI FIFO 连续写入失败 ({len(data)} 字节)")
        return len(data)


class DryRunSPI:
    """No DLL/device access; flags emulate immediate progress, not real timing."""
    def __init__(self):
        self.registers = {0x29: 0x10, 0x40: 0x88, 0x52: 0x90}
        self.fifo = []
        self.expected = 0
        self.rx_mode = False

    def open(self):
        print("[dry-run] 模拟寄存器及标志，不连接 CH347；RX 默认无输入包")

    def transfer(self, address, value=0, read=False):
        if read:
            if address == 0x61:
                if self.rx_mode:
                    return 0
                return PKT_FLAG if len(self.fifo) >= self.expected else FIFO_FLAG
            return self.registers.get(address, 0)
        self.registers[address] = value
        if address == 0x68 and value & 0x80:
            self.fifo = []
        elif address == 0x64:
            self.fifo.append(value)
        elif address == 0x0E and value & 1 and not value & 4 and not self.rx_mode:
            length_reg = self.registers.get(0x5F, 0)
            self.expected = (length_reg & 0x7F) if length_reg & 0x80 else self.fifo[0] + 1
        return value

    def close(self):
        pass

    def read_fifo(self, length):
        data = bytes(self.fifo[:length])
        del self.fifo[:length]
        return data

    def transfer_fifo(self, data):
        data = bytes(data)
        self.fifo.extend(data)
        return len(data)


class WL1606:
    def __init__(self, spi, verbose=False, sleep=time.sleep):
        self.spi, self.verbose, self.sleep = spi, verbose, sleep

    def write(self, address, value):
        self.spi.transfer(address, value)
        if self.verbose:
            print(f"write={address:02X} {value:02X}")

    def read(self, address):
        value = self.spi.transfer(address, read=True)
        if self.verbose:
            print(f"read={address:02X} {value:02X}")
        return value

    def update(self, address, clear=0, set_bits=0):
        self.write(address, (self.read(address) & (~clear & 0xFF)) | set_bits)

    def write_sequence(self, sequence):
        for address, value in sequence:
            self.write(address, value)

    def transfer_fifo(self, data):
        """Write one contiguous FIFO block; log one line per CS transaction."""
        data = bytes(data)
        count = self.spi.transfer_fifo(data)
        if self.verbose and data:
            print(f"write_fifo=64 {data.hex(' ').upper()} ({count} bytes)")
        return count

    def set_power(self, level):
        self.write(0x12, POWER_TABLE[level - 1][0])

    # CW模式即direct mode，其频点设置与正常模式不同，频率范围为2402~2480MHz
    # 见<WL1606A Engineering Spec.241123.pdf>的12.8.1
    def set_cw_freq(self, channel):
        # 根据传入信道，计算Reg14和Reg15的配置
        if not 2 <= channel <= 80:
            raise ValueError("CW channel must be between 2 ~ 80")

        self.read(0x0F)
        self.write(0x0F, 0x4c)
        self.read(0x0E)
        self.write(0x0E, 0x00)
        self.sleep(0.001)
        
        reg_14 = (channel % 32) * 2
        reg_15 = 0x4B + (channel // 32)
        self.write(0x0F, reg_15)
        self.write(0x0E, reg_14 | 0x01)

    def init_cw(self, channel, power):
            self.write_sequence(CW_INIT) 
            self.set_power(power)
            self.set_cw_freq(channel)
            

    def init_packet(self, length, channel, power=None):
        self.write(0x20, 0x80)
        self.sleep(0.010)
        self.write(0x06, 0x7A)
        self.write(0x5E, 0x80)
        self.sleep(0.010)
        for address, value in RF_INIT:
            self.write(address, value)
            # Wl16xx_Default_Cfg skips readback of RF table index 6 (0x2E).
            if address != 0x2E:
                actual = self.read(address)
                if actual != value:
                    raise RuntimeError(
                        f"初始化回读失败: 0x{address:02X}, 写 {value:02X}, 读 {actual:02X}"
                    )
        self.write_sequence(((0x41, 0x03), (0x49, 0x71), (0x48, 0x91),
                             (0x4B, 0x7D), (0x4A, 0x6B), (0x4D, 0x5C)))
        self.update(0x52, clear=0x20)
        # Bit7 selects fixed length (7-bit length field). LONG_PAYLOAD_EN
        # instead uses length in FIFO. Explicit 0 restores that reset mode.
        self.write(0x5F, (0x80 | length) if length <= FIFO_SIZE else 0x00)
        self.write_sequence(((0x28, 0xFF), (0x53, 0xFF)))
        self.update(0x29, clear=0x10, set_bits=0x10)
        # Matches supplied packet trace (0x40=0x88), no HS6220 framing.
        self.write(0x40, (self.read(0x40) & 0x03) | 0x88)
        self.write_sequence(((0x50, 0x68), (0x51, 0x44),
                             (0x0F, channel), (0x46, 0x03)))
        if power is not None:
            self.set_power(power)
        self.update(0x68, set_bits=0x80)

    def arm_rx(self):
        # Clear RX_EN before toggling it, as in Wl16xx_RX_Mode. Clear the
        # read pointer and stale IRQs before enabling reception again.
        self.update(0x0F, clear=0x80)
        self.update(0x0E, clear=0x01)
        self.update(0x69, set_bits=0x80)
        self.update(0x25, set_bits=0x03)
        self.sleep(0.000010)
        self.update(0x0F, set_bits=0x80)

    def init_rx(self, length, channel):
        # Same fixed-length, CRC-enabled, no-length-prefix wire format as
        # the supplied RX sequence and the existing short-packet TX.
        self.init_packet(length, channel)
        self.arm_rx()

    def receive_packet(self, expect_len):
        """Return None while waiting, or (CRC error, decoded payload).

        Check CRC before reading FIFO. Fixed-length mode has no length
        prefix; read expect_len bytes supplied by --len, as in the C driver.
        Every completed packet, including a bad one, rearms the receiver.
        """
        if not self.read(0x61) & PKT_FLAG:
            return None
        try:
            crc_error = bool(self.read(0x60) & 0x80)
            if crc_error:
                return True, b""
            wire_payload = self.spi.read_fifo(expect_len)
            if self.verbose:
                print(f"read_fifo=E4 {wire_payload.hex(' ').upper()} ({len(wire_payload)} bytes)")
            return False, wire_payload.translate(BIT_REVERSE)
        finally:
            self.arm_rx()

    def send_packet(self, payload, timeout_ms):
        self.update(0x68, set_bits=0x80)
        wire_payload = payload.translate(BIT_REVERSE)
        # For long packets, the unmodified length byte consumes one FIFO slot.
        stream = bytes([len(payload)]) + wire_payload if len(payload) > FIFO_SIZE else wire_payload
        sent = min(len(stream), FIFO_SIZE)
        self.transfer_fifo(stream[:sent])
        # Clear stale packet/FIFO IRQ before a new TX trigger.
        #self.update(0x25, set_bits=0x03)
        self.update(0x0F, clear=0x80)
        self.update(0x0E, clear=0x01)
        # Note: 实际两条spi cmd+data之间存在200us左右的间隔，
        # 所以这里不加sleep也可以，不过为了避免太过极限，还是加上
        self.sleep(0.000010)
        self.update(0x0E, set_bits=0x01)
        #deadline = time.monotonic() + timeout_ms / 1000.0
        # while True:
        #     flag = self.read(0x61)
        #     if flag & PKT_FLAG:
        #         if sent == len(stream):
        #             self.update(0x25, set_bits=0x02)
        #             return
        #         raise RuntimeError(f"TX 提前结束/疑似 FIFO 欠载: 已装载 {sent}/{len(stream)} 字节")
        #     if flag & FIFO_FLAG:
        #         end = min(sent + FIFO_SIZE - FIFO_FULL_THRESHOLD, len(stream))
        #         self.transfer_fifo(stream[sent:end])
        #         sent = end
        #         self.update(0x25, set_bits=0x01)
        #     if time.monotonic() >= deadline:
        #         raise TimeoutError(f"等待 TX 完成超时 ({timeout_ms:g} ms)，flag=0x{flag:02X}")
        #     if len(payload) <= FIFO_SIZE:
        #         self.sleep(0.0001)
        #self.sleep(0.1)

    def stop(self):
        # Clearing TX_EN also turns off the continuous carrier (0x05 -> 0x04).
        self.update(0x0E, clear=0x01)
        self.update(0x0F, clear=0x80)
        self.write(0x20, 0x80)
        self.sleep(0.010)


def integer(text):
    try:
        return int(text, 16) if text.lower().startswith(("0x", "+0x", "-0x")) else int(text, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("请输入十进制或 0x 开头的十六进制整数") from exc


def packet_type(text):
    aliases = {"all1": "0xff", "all0": "0x00", "ones": "0xff", "zeros": "0x00", "055": "0x55"}
    text = aliases.get(text.lower(), text.lower())
    if text == "random":
        return text
    value = integer(text)
    if value not in (0xFF, 0, 0x55, 0xAA, 0x0F, 0xF0):
        raise argparse.ArgumentTypeError("仅支持 all1/all0/0xff/0x00/0x55/0xaa/0x0f/0xf0/random")
    return f"0x{value:02x}"


def make_payload(length, kind):
    if kind == "random":
        return bytes(random.getrandbits(8) for _ in range(length))
    return bytes([int(kind, 16)]) * length


class RxStats:
    def __init__(self, expected_payload):
        self.expected_payload = expected_payload
        self.rx_cnt = self.crc_err_cnt = self.invalid_pkt_cnt = 0

    def record(self, crc_error, payload):
        if crc_error:
            self.crc_err_cnt += 1
        elif payload != self.expected_payload:
            self.invalid_pkt_cnt += 1
        else:
            self.rx_cnt += 1
            #JXX
            while True:
                pass

    def report(self):
        print(f"Rx cnt: {self.rx_cnt}, Crc_err_cnt: {self.crc_err_cnt},  Invalid pkt cnt:{self.invalid_pkt_cnt}", flush=True)


def run_rx(radio, args, clock=time.monotonic, sleep=time.sleep):
    stats = RxStats(make_payload(args.len, args.pkt_type))
    interval = args.out_rx_result_time / 1000.0
    next_report = clock() + interval
    receive_deadline = clock() + RX_TIMEOUT_SECONDS
    if args.dry_run:
        stats.report()
        return stats

    while True:
        packet = radio.receive_packet(expect_len=args.len)
        now = clock()
        if packet is not None:
            crc_error, payload = packet
            if not crc_error:
                # print(datetime.now().strftime("%H:%M:%S"))
                print(f"time: {datetime.now()}, Rx data: {payload.hex(' ').upper()} ({len(payload)} bytes)", flush=True)
            stats.record(*packet)
            receive_deadline = now + RX_TIMEOUT_SECONDS
        elif now >= receive_deadline:
            # Manual 8.3: exit RX on timeout, then resume waiting. This
            # recovery timer is independent of the statistics interval.
            radio.arm_rx()
            receive_deadline = clock() + RX_TIMEOUT_SECONDS
        if now >= next_report:
            stats.report()
            next_report += (int((now - next_report) / interval) + 1) * interval
        if packet is None:
            sleep(min(0.0001, max(0.0, next_report - clock())))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", type=integer, choices=(0, 1), default=0, help="0=TX (默认)，1=RX")
    parser.add_argument("--len", "--tx_len", dest="len", type=integer, default=20, help="payload 字节数，TX 0..255，RX 短包 1..63，默认 20")
    parser.add_argument("--pkt_type", "--tx_pkt_type", dest="pkt_type", type=packet_type, default="0xff", help="默认全 1；random 仅限 TX")
    parser.add_argument("--chan", "--tx_chan", dest="chan", type=integer, default=40, help="0..80，频率=2400+chan MHz，默认40")
    parser.add_argument("--tx_cnt", default=0xFFFF, help="1..65534 次；65535=持续发送；默认：0xFFFF；仅 TX")
    parser.add_argument("--tx_interval", default=20, help="两包之间 sleep 毫秒，默认 20；仅 TX")
    parser.add_argument("--tx_pwr", default=1, help="1..15，默认 1 (8 dBm，最大)；仅 TX")
    parser.add_argument("--tx_test_mode", default=1, help="0=CW，1=packet (默认)；仅 TX")
    parser.add_argument("--out_rx_result_time", default=1000, help="RX 累计统计输出间隔，正毫秒数，默认 1000；仅 RX")
    parser.add_argument("--dev_index", type=integer, default=0, help="CH347 设备索引，默认 0")
    parser.add_argument("--dll", help="CH347DLLA64.dll 路径，默认脚本同目录")
    parser.add_argument("--tx_timeout", default=1000, help="每包完成超时毫秒，默认 1000；仅 TX（当前 TX 未启用完成轮询）")
    parser.add_argument("--verbose", action="store_true", help="打印每笔寄存器读写（会减慢长包补充 FIFO）")
    parser.add_argument("--dry-run", action="store_true", help="模拟 SPI；TX 无限次数时模拟 1 包；RX 输出一次零计数后退出")
    args = parser.parse_args(argv)
    # Convert/validate only parameters used by the selected direction.
    active = {"out_rx_result_time": float} if args.mode else {
        "tx_cnt": integer, "tx_pwr": integer, "tx_test_mode": integer,
        "tx_interval": float, "tx_timeout": float,
    }
    for name, convert in active.items():
        try:
            setattr(args, name, convert(str(getattr(args, name))))
        except (ValueError, argparse.ArgumentTypeError) as exc:
            parser.error(f"--{name}: {exc}")
    ranges = [("len", 1 if args.mode else 0, FIFO_SIZE if args.mode else 255),
              ("chan", 0, 80), ("dev_index", 0, 0xFFFFFFFF)]
    if not args.mode:
        ranges.extend((("tx_cnt", 1, 65535), ("tx_pwr", 1, 15), ("tx_test_mode", 0, 1)))
    for name, low, high in ranges:
        if not low <= getattr(args, name) <= high:
            parser.error(f"--{name} 必须在 {low}..{high} 之间")
    if args.mode and args.pkt_type == "random":
        parser.error("RX mode 的 --pkt_type 不支持 random；请选择固定数据类型，例如 0xff 或 0x55")
    if args.mode and (not math.isfinite(args.out_rx_result_time) or args.out_rx_result_time <= 0):
        parser.error("--out_rx_result_time 必须是有限的正毫秒数")
    if not args.mode and (not math.isfinite(args.tx_interval) or args.tx_interval < 0):
        parser.error("--tx_interval 必须是有限的非负毫秒数")
    if not args.mode and (not math.isfinite(args.tx_timeout) or args.tx_timeout <= 0):
        parser.error("--tx_timeout 必须是有限的正毫秒数")
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.mode:
        print(f"模式=RX, 频率={2400 + args.chan} MHz, 预期 payload={args.len} 字节, "
              f"类型={args.pkt_type}, 统计间隔={args.out_rx_result_time:g} ms")
    else:
        reg, dbm = POWER_TABLE[args.tx_pwr - 1]
        print(f"模式=TX {'CW' if args.tx_test_mode == 0 else 'packet'}, 频率={2400 + args.chan} MHz, "
              f"功率档={args.tx_pwr} (约 {dbm:g} dBm, reg12=0x{reg:02X})")
    spi = DryRunSPI() if args.dry_run else CH347SPI(args.dev_index, args.dll)
    if args.dry_run:
        spi.rx_mode = bool(args.mode)
    radio = WL1606(spi, args.verbose, (lambda seconds: None) if args.dry_run else time.sleep)
    opened = False
    result = 0
    try:
        spi.open()
        opened = True
        if args.mode:
            radio.init_rx(args.len, args.chan)
            print("接收已开启，Ctrl+C 停止；以下三项均为本次运行的累计计数。")
            run_rx(radio, args)
        elif args.tx_test_mode == 0:
            radio.init_cw(args.chan, args.tx_pwr)
            print("单载波已开启，Ctrl+C 停止；CW 模式不使用包长、包型、包数和包间隔。")
            while not args.dry_run:
                time.sleep(0.2)
        else:
            print(f"payload={args.len} 字节, 类型={args.pkt_type}, "
                  f"包数={'持续' if args.tx_cnt == 0xFFFF else args.tx_cnt}, 间隔={args.tx_interval:g} ms")
            if args.len > FIFO_SIZE:
                print("注意：当前 TX 长包补充 FIFO 未启用，超过 63 字节不能完成完整发送。")
            radio.init_packet(args.len, args.chan, args.tx_pwr)
            limit = 1 if args.dry_run and args.tx_cnt == 0xFFFF else args.tx_cnt
            count = 0
            while args.tx_cnt == 0xFFFF or count < limit:
                radio.send_packet(make_payload(args.len, args.pkt_type), args.tx_timeout)
                count += 1
                if args.verbose or count == 1 or count % 100 == 0 or count == limit:
                    print(f"{'模拟发送' if args.dry_run else '发送完成'}: {count} 包")
                if args.dry_run and count >= limit:
                    break
                if args.tx_cnt == 0xFFFF or count < limit:
                    if args.tx_interval:
                        radio.sleep(args.tx_interval / 1000.0)
                    else:
                        # 如果设置的interval为0，也要手动加入100us的延时（实际测会延时700us左右），否则发送的空口数据包会有误码
                        radio.sleep(0.0001)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止接收。" if args.mode else "\n收到 Ctrl+C，停止发送。")
    except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        result = 1
    finally:
        if opened:
            try:
                radio.stop()
            except Exception as exc:
                print(f"停止 WL1606 失败: {exc}", file=sys.stderr)
                result = 1
            finally:
                try:
                    spi.close()
                    print("CH347 已关闭。")
                except Exception as exc:
                    print(f"关闭设备失败: {exc}", file=sys.stderr)
                    result = 1
    return result


if __name__ == "__main__":
    raise SystemExit(main())
