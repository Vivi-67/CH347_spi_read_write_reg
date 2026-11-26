"""CH347 控制 Ceres 1M 固定包长 TX/RX、单音和寄存器读写。

用法（64 位 Windows Python，DLL 默认位于本脚本目录）：
 Tx single tone:
    python Ceres_cfg_reg.py --tx_test_mode 0 --chan 40
 
 Tx packets:
    python Ceres_cfg_reg.py --mode 0 --tx_test_mode 1 --len 19 --pkt_type 0x55 --tx_cnt 100 --tx_interval 5 --chan 40

 Rx packets:
    python Ceres_cfg_reg.py --mode 1 --len 19 --pkt_type 0x55 --chan 40 --out_rx_result_time 1000

 Read/Write reg:
    python Ceres_cfg_reg.py read 0x4004
    python Ceres_cfg_reg.py write 0x4004 0x11223344

接线：CS0 -> Ceres CS，SCK -> CLK，共地，Ceres 需独立正常供电。
SDK 的 hal_spi.c 是三线 SDIO：MISO 接 SDIO，MOSI 必须经适当的隔离/
方向控制电路接 SDIO（例如经板级确认的串联限流电阻方案）。不要将 MOSI、
MISO、SDIO 三者直接短接：CH347SPI_WriteRead 在读阶段仍驱动 MOSI；
dummy FF 不代表高阻。四线板则分别连接 MOSI 和 MISO。

时序依据 MCU_SOP8_ctrl_Ceres_OK.dsl：Mode 0，MSB first，CS 低有效；
"""

import argparse
import ctypes
import math
import random
from pathlib import Path
import sys
import time


U8, U16, U32 = ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32
CS0 = 0x80

SPI_CLK_MHz = {0:"60MHz", 
               1:"30MHz", 
               2:"15MHz", 
               3:"7.5MHz", 
               4:"3.75MHz", 
               5:"1.875MHz", 
               6:"937.5kHz", 
               7:"468.75kHz"}


class SPI_CONFIG(ctypes.Structure):
    # WCH CH347DLL.H declares SPI_CONFIG inside #pragma pack(1).
    _pack_ = 1
    _fields_ = [
        ("mMode", U8), ("mClock", U8), ("mByteOrder", U8),
        ("mSPIWriteReadInterval", U16), ("mSpiOutDefaultData", U8),
        ("mChipSelect", U32), ("CS1Polarity", U8), ("CS2Polarity", U8),
        ("mIsAutoDeactiveCS", U16), ("mActiveDelayNum", U16),
        ("mDelayNum", U32),
    ]


class CH347SPI:
    """对应 drv_ceres_spi_init()/hal_ceres_spi_write()/read_reg()。"""

    def __init__(self, index=0, dll_path=None):
        if sys.platform != "win32" or ctypes.sizeof(ctypes.c_void_p) != 8:
            raise RuntimeError("CH347DLLA64.DLL 需要 64 位 Windows Python")
        self.index = index
        self.opened = False
        dll_path = Path(dll_path or Path(__file__).with_name("CH347DLLA64.DLL"))
        self.dll = ctypes.WinDLL(str(dll_path.resolve()))
        signatures = {
            "CH347OpenDevice": ([U32], ctypes.c_void_p),
            "CH347CloseDevice": ([U32], ctypes.c_int32),
            "CH347SPI_Init": ([U32, ctypes.POINTER(SPI_CONFIG)], ctypes.c_int32),
            "CH347SPI_WriteRead": ([U32, U32, U32, ctypes.POINTER(U8)], ctypes.c_int32),
            "CH347SPI_ChangeCS": ([U32, U8], ctypes.c_int32),
            "CH347SetTimeout": ([U32, U32, U32], ctypes.c_int32),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.dll, name)
            fn.argtypes, fn.restype = args, result
        handle = self.dll.CH347OpenDevice(index)
        if handle in (None, 0xFFFFFFFF, ctypes.c_void_p(-1).value):
            raise RuntimeError(f"CH347OpenDevice({index}) 失败")
        self.opened = True
        try:
            if not self.dll.CH347SetTimeout(index, 1000, 1000):
                raise RuntimeError("CH347SetTimeout 失败")
            cfg = SPI_CONFIG()
            cfg.mMode = 0              # DSL: CPOL=0, CPHA=0（WL160x参考脚本实际为1）
            cfg.mClock = 5             # 5:1.875 MHz
            cfg.mByteOrder = 1         # CH347: 1=MSB first
            cfg.mSPIWriteReadInterval = 1
            cfg.mSpiOutDefaultData = 0x00
            cfg.mChipSelect = CS0
            #cfg.CS1Polarity = cfg.CS2Polarity = 0
            cfg.mIsAutoDeactiveCS = 1
            cfg.mActiveDelayNum = cfg.mDelayNum = 0
            if not self.dll.CH347SPI_Init(index, ctypes.byref(cfg)):
                raise RuntimeError("CH347SPI_Init 失败")
            if not self.dll.CH347SPI_ChangeCS(index, 1):
                raise RuntimeError("CH347SPI_ChangeCS 释放片选失败")
            print(f"SPI Clock = {SPI_CLK_MHz[cfg.mClock]}")
        except BaseException:
            self.close()
            raise
        print("CH347: Mode 0, 1.875 MHz, MSB first, CS0低有效, 字节间隔2 us")

    def transfer(self, tx):
        self.dll.CH347SPI_ChangeCS(self.index, 0)
        buf = (U8 * len(tx)).from_buffer_copy(tx)
        # 一次调用覆盖完整帧，不能在命令与4字节数据之间释放CS。
        if not self.dll.CH347SPI_WriteRead(self.index, CS0, len(tx), buf):
            self.dll.CH347SPI_ChangeCS(self.index, 1)
            raise RuntimeError(f"SPI传输失败: {tx.hex(' ').upper()}")
        self.dll.CH347SPI_ChangeCS(self.index, 1)
        return bytes(buf)

    def close(self):
        if self.opened:
            self.opened = False
            try:
                self.dll.CH347SPI_ChangeCS(self.index, 1)
            finally:
                self.dll.CH347CloseDevice(self.index)


# drv_ceres.c 使用的寄存器地址
AON_CTRL, AON_LINK_CTRL, AON_PWR_CFG, AON_RSVD = 0x0000, 0x0010, 0x0020, 0x0038
COMM_1, AFC_0, AFC_1, AFC_5, AFC_6 = 0x4004, 0x4020, 0x4024, 0x4034, 0x4038
LNA, RXFLT0, PA, AGC_CTRL0, TXPWRDN = 0x403C, 0x4040, 0x4048, 0x8098, 0xC02C
AON_PKT_CTRL, AON_CRC_CTRL, AON_PKT_INIT = 0x0004, 0x0008, 0x000C
AON_ADDR_BASE0, AON_ADDR_BASE1, AON_ADDR_PREFIX = 0x0014, 0x0018, 0x001C
AON_GFSK_SMP_REF_CTRL, AON_GFSK_MOD_IDX_LE = 0x0024, 0x0028
AON_IRQ_STATUS, MODEM_COMMON_CTRL, PLINK_PKTCNTL0 = 0x0034, 0x8000, 0xC004
MODEM_RXFIFO_CTRL = 0x8058
RX_FIFO_THRESHOLD = 48  # drv_ceres.c: CERES_RX_ALMOST_FULL_THRES
PACKET_END, CRC_OK, CRC_ERROR = 1 << 15, 1 << 12, 1 << 13
TXFIFO_ALMOST_EMPTY, RXFIFO_ALMOST_FULL = 1, 2


def register_command(address, write=False):
    """对应 drv_ceres_write_reg/read_reg 的 mod/bank/cmd 计算。"""
    if not 0 <= address <= 0xFFFF:
        raise ValueError("地址必须在0x0000..0xFFFF范围内")
    mod, offset = address >> 14, address & 0x3FFF
    bank = ((offset >> 5) | (mod << 6)) & 0xFF
    return bank, (offset & 31) | (0x20 if write else 0x00)


class Ceres:
    def __init__(self, spi, timeout=2.0, verbose=True):
        self.spi, self.timeout = spi, timeout
        self.verbose = verbose
        self.static_length = 0
        self.rx_partial = bytearray()
        self.rx_fifo_recoveries = 0
        self.rx_fifo_fault_streak = 0
        self.last_bank = None          # 每次进程首次访问必选bank，避免依赖芯片旧状态

    def frame(self, tx, label):
        if self.verbose:
            print(f"SPI TX [{label}]: {tx.hex(' ').upper()}")
        return self.spi.transfer(tx)

    def select_bank(self, bank):
        if bank != self.last_bank:
            self.frame(bytes((0x41, bank)), f"WR_REG_BANK=0x{bank:02X}")
            self.last_bank = bank

    def write_reg(self, address, value):
        if not 0 <= value <= 0xFFFFFFFF:
            raise ValueError("寄存器值必须在0x00000000..0xFFFFFFFF范围内")
        bank, cmd = register_command(address, True)
        self.select_bank(bank)
        # ARM小端 value 的4个字节：字节内MSB first，字节间LSB byte first。
        self.frame(bytes((cmd,)) + value.to_bytes(4, "little"), f"WRITE 0x{address:04X}")
        if self.verbose:
            print(f"WRITE 0x{address:04X} <= 0x{value:08X}")

    def read_reg(self, address):
        bank, cmd = register_command(address)
        self.select_bank(bank)
        rx = self.frame(bytes((cmd, 0xFF, 0xFF, 0xFF, 0xFF)), f"READ 0x{address:04X}; 后4字节dummy")
        if self.verbose:
            print(f"SPI RX: {rx.hex(' ').upper()}")
        value = int.from_bytes(rx[1:5], "little")
        if self.verbose:
            print(f"READ 0x{address:04X} => 0x{value:08X}")
        return value

    def delay_ms(self, ms):
        print(f"DELAY {ms} ms")
        time.sleep(ms / 1000)

    def wait_bits(self, address, mask, label):
        deadline = time.monotonic() + self.timeout
        while True:
            value = self.read_reg(address)
            if value == 0xFFFFFFFF:
                raise RuntimeError(f"{label}: 读回全FF，请检查供电、MISO/SDIO及方向隔离")
            if value & mask:
                return value
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{label}超时: reg=0x{address:04X}, value=0x{value:08X}, mask=0x{mask:08X}")
            time.sleep(0.001)

    def init(self, channel=80, mode=0, test_mode=0):
        """drv_ceres_init(0, mode, CERES_24G_MODE, channel)。"""
        print("--- drv_ceres_aon_init() ---")
        self.select_bank(0)
        self.write_reg(AON_CTRL, 1 << 16)
        self.write_reg(AON_CTRL, 0)
        self.delay_ms(5)
        # SOFT_RESETN|BYPASS|TXFIFO_LIMIT(30)|RSTB_DBB_REG|XTAL_UP|DBB_UP
        #self.write_reg(AON_CTRL, (1 << 16) | (1 << 17) | (30 << 10) | 0x13)
        self.write_reg(AON_CTRL, (1 << 16) | (1 << 17) | (20 << 10) | 0x13
                    | ((1 << 8) if mode == 1 else 0))
        self.wait_bits(AON_CTRL, 1 << 26, "XTAL_READY")
        # XTAL_VAMP(0), BUF_DRV(4), DLY(1), FINE(0x10), COARSE(0xA),
        # VRTC(8), VDIG(3), VCORE(3), 两路LPMODE_B。
        self.write_reg(AON_PWR_CFG, (4 << 25) | (1 << 23) | (0x10 << 16)
                       | (0xA << 12) | (8 << 8) | (3 << 5) | 0x10 | (3 << 1) | 1)
        self.write_reg(AON_RSVD, 0x88005455)
        self.frame(b"\xD6", "CERES_CMD_CLR_CE")

        print("--- drv_ceres_rf_init(): drv_ceres_afc_auto_calibration() ---")
        link = (1 << 20) | (1 << 9) | (1 << 8) | channel
        self.write_reg(AON_LINK_CTRL, link)  # 2M校准；TX_MODE无自动重传字段
        self.write_reg(AFC_0, 0x06400020)
        self.write_reg(AFC_5 if channel < 64 else AFC_6, 1 << ((channel % 64) // 2))
        self.write_reg(AFC_0, 0x06400021)
        self.wait_bits(AFC_0, 1 << 6, "AUTO_AFC2_CAL_DONE")
        self.write_reg(AFC_0, 0x06400000)
        self.write_reg(AON_LINK_CTRL, link & ~(1 << 20))  # 回到默认1M
        self.write_reg(LNA, 0x0013944F)
        self.write_reg(RXFLT0, 0x618080A9)
        self.write_reg(PA, self.read_reg(PA))  # C源码 value |= DA_PADRV_IBIT(0)，为原值

        print("--- drv_ceres_core_init() ---")
        self.write_reg(AGC_CTRL0, self.read_reg(AGC_CTRL0) | (3 << 2))
        print("--- drv_ceres_link_init() ---")
        self.write_reg(TXPWRDN, (30 << 16) | (100 << 8) | 10)  # PEER_PLATFORM=0

    def update_reg(self, address, clear=0, set_bits=0):
        self.write_reg(address, (self.read_reg(address) & ~clear) | set_bits)

    def command(self, cmd, data=b''):
        return self.frame(bytes([cmd]) + data, f"CMD 0x{cmd:02X}")

    def read_data(self, cmd, length):
        return self.command(cmd, b'\xff' * length)[1:]

    def irq_status(self):
        value = self.read_reg(AON_IRQ_STATUS)
        if value == 0xFFFFFFFF:
            raise RuntimeError("IRQ读回全FF，请检查SPI连接和供电")
        return value

    def drv_ceres_set_rate(self, rate=0):
        """drv_ceres.c: CERES_RATE_1M=0, CERES_RATE_2M=1。"""
        if rate not in (0, 1):
            raise ValueError("rate必须为0或1")
        print("--- drv_ceres_set_rate() ---")
        self.update_reg(AON_LINK_CTRL, 1 << 20, rate << 20)

    def ceres_nordic_24g_mode_setup(self):
        """common.c: CERES_USING_STATIC_LENGTH=1, PEER_PLATFORM=0。"""
        print("--- ceres_nordic_24g_mode_setup(): drv_ceres_set_base_addr() ---")
        self.write_reg(AON_ADDR_BASE0, 0xF02C6A88)
        self.write_reg(AON_ADDR_BASE1, 0xC8889911)
        print("--- drv_ceres_set_addr_prefix() ---")
        self.write_reg(AON_ADDR_PREFIX, 0x70CC44C8)  # prefix C8/44/CC, TX pipe0, RX pipes0..2
        print("--- drv_ceres_set_crc() ---")
        self.write_reg(AON_CRC_CTRL, 0x02001021)  # CRC16, poly=1021, include address
        self.update_reg(AON_PKT_INIT, 0xFFFFFF, 0xFFFF)
        print("--- drv_ceres_set_packet_format() ---")
        self.write_reg(AON_PKT_CTRL, 0x001C0000)  # ACK_S1_HW_EN, BITENDIAN, BALEN(4)
        self.update_reg(AON_LINK_CTRL, 0xE00000)  # CERES_24G_MODE_NORMAL
        self.update_reg(MODEM_COMMON_CTRL, set_bits=1 << 10)
        self.write_reg(AON_GFSK_MOD_IDX_LE, (10 << 27) | (10 << 22) | (1 << 19) | (7 << 16) | 0x7400)
        self.write_reg(AON_GFSK_SMP_REF_CTRL, 0x02160118)

    def drv_ceres_set_static_length(self, length):
        if not 0 <= length <= 255:
            raise ValueError("固定包长必须在0..255")
        print("--- drv_ceres_set_static_length() ---")
        self.update_reg(PLINK_PKTCNTL0, 0xFF00, length << 8)
        self.static_length = length

    def ceres_tx_packet_init(self, length, kind):
        """固定包长分支只包含payload；不附加动态长度/PID/noack头。"""
        return make_payload(length, kind)

    def stop_packet_tx(self):
        self.command(0xD6)  # CERES_CMD_CLR_CE
        self.command(0xE1)  # CERES_CMD_FLUSH_TX_FIFO

    def drv_ceres_packet_handler(self, payload=None):
        """同步实现TX_CONT/W4_TXEND；无payload时处理一次RX状态。"""
        if payload is None:
            return self.receive_packet()
        deadline = time.monotonic() + self.timeout
        try:
            #self.command(0xD6)
            #self.command(0xE1)
            #self.write_reg(AON_IRQ_STATUS, self.irq_status())  # W1C，清除旧包完成
            sent = min(64, len(payload))  # CERES_TX_FIFO_SIZE
            self.command(0xB0, payload[:sent])  # CERES_CMD_WR_TX_NOACK
            self.command(0xD5)  # drv_ceres_trigger_start()
            while True:
                status = self.irq_status()
                if status & PACKET_END:
                    if sent != len(payload):
                        raise RuntimeError("TX提前结束：FIFO未及时补充")
                    self.write_reg(AON_IRQ_STATUS, status)
                    self.command(0xD6)
                    return
                if sent < len(payload) and status & TXFIFO_ALMOST_EMPTY:
                    end = min(sent + 40, len(payload))  # CERES_TX_CONT_SIZE
                    self.command(0xB0, payload[sent:end])
                    sent = end
                    self.write_reg(AON_IRQ_STATUS, TXFIFO_ALMOST_EMPTY)
                if time.monotonic() >= deadline:
                    raise TimeoutError("TX等待PACKET_END超时")
                # 长包不sleep，尽快服务FIFO；短包让出CPU。
                if len(payload) <= 64:
                    time.sleep(0.0001)
        except BaseException:
            try:
                self.stop_packet_tx()
            except (OSError, RuntimeError):
                pass
            raise

    def transmit_packets(self, length, kind, count, interval_ms):
        print("--- ceres_tx_packet_init() / drv_ceres_packet_handler() ---")
        sent = 0
        try:
            while count == 65535 or sent < count:
                self.drv_ceres_packet_handler(self.ceres_tx_packet_init(length, kind))
                sent += 1
                if count == 65535 or sent < count:
                    time.sleep(interval_ms / 1000)
            print(f"TX完成: {sent} 包，每包 {length} 字节，1M")
        finally:
            # 清理失败不能覆盖原始TX异常或Ctrl+C。
            active_error = sys.exc_info()[0] is not None
            try:
                self.stop_packet_tx()
            except (OSError, RuntimeError):
                if not active_error:
                    raise

    def drv_ceres_start_rx(self):
        print("--- drv_ceres_start_rx() ---")
        self.command(0xD6)
        self.command(0xE2)
        self.rx_partial.clear()
        self.write_reg(AON_IRQ_STATUS, self.irq_status())
        # reg_bt_core.h: RXFIFO_ALMOST_FULL_LIMIT[5:0]，与每次读取量一致。
        print("--- MODEM_RXFIFO_CTRL: RXFIFO_ALMOST_FULL_LIMIT(48) ---")
        self.update_reg(MODEM_RXFIFO_CTRL, 0x3F, RX_FIFO_THRESHOLD)
        threshold = self.read_reg(MODEM_RXFIFO_CTRL) & 0x3F
        if threshold != RX_FIFO_THRESHOLD:
            raise RuntimeError(f"RX FIFO阈值回读异常: expected=48, actual={threshold}")
        self.command(0xE3)  # SDK设置ACK payload；普通固定包长不请求ACK
        for cmd in (0xA8, 0xA9, 0xAA):
            self.command(cmd, b'\x01\x00\x55')
        self.command(0xD5)

    def rearm_rx(self):
        # 先停止和flush，再清状态。flush前清W1C会遗漏清理期间新产生的状态。
        self.command(0xD6)
        self.command(0xE2)
        self.rx_partial.clear()
        pending = self.irq_status()
        self.write_reg(AON_IRQ_STATUS, pending)
        self.command(0xD5)

    def receive_packet(self):
        """drv_ceres_state_rx_handler(): 返回(crc_ok, payload)或None。"""
        status = self.irq_status()
        if status & PACKET_END:
            try:
                crc_ok = bool(status & CRC_OK) and not bool(status & CRC_ERROR)
                if not crc_ok:
                    return False, b''
                # SDK: RD_RX_LENGTH + rx_header_len；固定包长时后者为static_length。
                size = self.read_data(0x60, 1)[0] + self.static_length
                if size != self.static_length or len(self.rx_partial) > size:
                    raise RuntimeError(f"RX长度状态异常: length={size}, fixed={self.static_length}")
                self.rx_partial.extend(self.read_data(0x61, size - len(self.rx_partial)))
                self.rx_fifo_fault_streak = 0
                return True, bytes(self.rx_partial)
            finally:
                self.rearm_rx()
        if status & RXFIFO_ALMOST_FULL:
            if len(self.rx_partial) + RX_FIFO_THRESHOLD > self.static_length:
                threshold = self.read_reg(MODEM_RXFIFO_CTRL) & 0x3F
                self.rx_fifo_recoveries += 1
                self.rx_fifo_fault_streak += 1
                print(f"RX FIFO异常: irq=0x{status:08X}, partial={len(self.rx_partial)}, "
                      f"static_length={self.static_length}, threshold={threshold}, "
                      f"recovery={self.rx_fifo_recoveries}; 丢弃当前不完整数据并重启RX", flush=True)
                if self.rx_fifo_fault_streak >= 3:
                    raise RuntimeError(f"RX FIFO连续3次异常，停止接收: irq=0x{status:08X}, "
                                       f"threshold={threshold}, fixed={self.static_length}")
                self.rearm_rx()
                return None
            self.rx_partial.extend(self.read_data(0x61, RX_FIFO_THRESHOLD))
            self.write_reg(AON_IRQ_STATUS, RXFIFO_ALMOST_FULL)
        return None

    def receive_packets(self, expected, report_ms):
        stats = RxStats(expected)
        try:
            self.drv_ceres_start_rx()
            next_report = time.monotonic() + report_ms / 1000
            while True:
                packet = self.drv_ceres_packet_handler()
                if packet is not None:
                    stats.record(*packet)
                now = time.monotonic()
                if now >= next_report:
                    stats.report()
                    next_report = now + report_ms / 1000
                if packet is None:
                    time.sleep(0.0001)
        finally:
            stats.report()
            active_error = sys.exc_info()[0] is not None
            try:
                self.command(0xD6)
                self.command(0xE2)
            except (OSError, RuntimeError):
                if not active_error:
                    raise

    def set_cw_tone_freq(self, freq_mhz):
        """对应 drv_ceres_set_cw_tone_freq()，32 MHz参考时钟。"""
        n, remainder = divmod(freq_mhz * 1_000_000, 32_000_000)
        k = (remainder << 24) // 32_000_000
        self.write_reg(AFC_1, (n << 24) | k)
        value = self.read_reg(AFC_0)
        value &= ~((1 << 15) | (1 << 16) | 0x7F00)
        value |= (1 << 15) | (1 << 16) | ((freq_mhz - 2400) << 8)
        self.write_reg(AFC_0, value)
        self.delay_ms(5)

    def start_single_tone(self, freq_mhz=2400):
        """对应 drv_ceres_start_single_tone()，保留源码中的重复写。"""
        print("--- drv_ceres_start_single_tone() ---")
        self.write_reg(COMM_1, 0x0012A634)  # FSM PLL idle
        self.write_reg(COMM_1, 0x0012A634)
        self.set_cw_tone_freq(freq_mhz)
        for _ in range(3):
            self.write_reg(COMM_1, 0x0012A632)  # FSM PLL AFC
        self.write_reg(PA, 0x0021FFC6)          # force max gain
        self.write_reg(COMM_1, 0x0012A636)     # FSM TX


def number_in_range(low, high):
    def parse(token):
        try:
            value = integer(token)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"无效整数: {token}") from exc
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(f"必须在{low}..{high}范围内")
        return value
    return parse


def integer(token):
    return int(token, 16) if token.lower().startswith(('0x', '+0x', '-0x')) else int(token, 10)


def packet_type(token):
    aliases = {'all1': '0xff', 'all0': '0x00', 'ones': '0xff', 'zeros': '0x00', '055': '0x55'}
    token = aliases.get(token.lower(), token.lower())
    if token == 'random':
        return token
    try:
        value = integer(token)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('无效pkt_type') from exc
    if value not in (0, 255, 0x55, 0xAA, 0x0F, 0xF0):
        raise argparse.ArgumentTypeError('支持all0/all1/0x00/0xff/0x55/0xaa/0x0f/0xf0/random')
    return f'0x{value:02x}'


def make_payload(length, kind):
    if kind == 'random':
        return bytes(random.getrandbits(8) for _ in range(length))
    return bytes([int(kind, 16)]) * length


class RxStats:
    def __init__(self, expected):
        self.expected = expected
        self.total = self.ok = self.crc_error = self.mismatch = 0
        self.byte_errors = self.bit_errors = 0

    def record(self, crc_ok, payload):
        self.total += 1
        if not crc_ok:
            self.crc_error += 1
        elif payload == self.expected:
            self.ok += 1
        else:
            self.mismatch += 1
            self.byte_errors += sum(a != b for a, b in zip(payload, self.expected)) + abs(len(payload) - len(self.expected))
            self.bit_errors += sum((a ^ b).bit_count() for a, b in zip(payload, self.expected)) + 8 * abs(len(payload) - len(self.expected))

    def report(self):
        print(f'RX累计: total={self.total}, ok={self.ok}, crc_error={self.crc_error}, '
              f'mismatch={self.mismatch}, byte_errors={self.byte_errors}, bit_errors={self.bit_errors}', flush=True)


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter, allow_abbrev=False)
    parser.add_argument("--dev-index", "--dev_index", type=number_in_range(0, 0xFFFFFFFF), default=0)
    parser.add_argument("--dll", type=Path, help="CH347DLLA64.DLL路径")
    parser.add_argument("--timeout", type=float, default=2.0, help="每次状态轮询超时秒数，默认2")
    parser.add_argument('--verbose', action='store_true', help='打印每笔SPI帧/寄存器读写，会降低FIFO服务速度')
    parser.add_argument('--mode', type=number_in_range(0, 1), default=0, help='0=TX，1=RX')
    parser.add_argument('--tx_test_mode', type=number_in_range(0, 1), default=1, help='0=单音，1=数据包(默认)')
    parser.add_argument('--chan', '--tx_chan', type=number_in_range(0, 80), default=40, help='频率=2400+chan MHz')
    parser.add_argument('--len', '--tx_len', type=number_in_range(0, 255), default=20, help='TX 0..255，RX 1..63字节')
    parser.add_argument('--pkt_type', '--tx_pkt_type', type=packet_type, default='0xff')
    parser.add_argument('--tx_cnt', type=number_in_range(1, 65535), default=65535, help='1..65534；65535=持续发送，Ctrl+C停止')
    parser.add_argument('--tx_interval', type=float, default=20, help='两包之间sleep毫秒，默认20')
    parser.add_argument('--out_rx_result_time', type=float, default=1000, help='RX累计结果上报周期，毫秒')
    sub = parser.add_subparsers(dest="command")
    read = sub.add_parser("read", help="独立读取32位寄存器，不执行初始化")
    read.add_argument("address", type=number_in_range(0, 0xFFFF))
    write = sub.add_parser("write", help="独立写入32位寄存器，不执行初始化")
    write.add_argument("address", type=number_in_range(0, 0xFFFF))
    write.add_argument("value", type=number_in_range(0, 0xFFFFFFFF))
    args = parser.parse_args(argv)
    if args.command and any(token.split('=')[0] in (
            '--mode', '--tx_test_mode', '--chan', '--tx_chan', '--len', '--tx_len',
            '--pkt_type', '--tx_pkt_type', '--tx_cnt', '--tx_interval', '--out_rx_result_time')
            for token in argv):
        parser.error('read/write独立访问寄存器，不能与收发测试参数混用')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout必须是有限正数")
    if not math.isfinite(args.tx_interval) or args.tx_interval < 0:
        parser.error('--tx_interval必须是有限非负毫秒数')
    if not math.isfinite(args.out_rx_result_time) or args.out_rx_result_time <= 0:
        parser.error('--out_rx_result_time必须是有限正毫秒数')
    if args.command is None and args.mode == 1:
        if not 1 <= args.len <= 63:
            parser.error('RX --len必须在1..63之间（与wl160x_test_mode.py一致）')
        if args.pkt_type == 'random':
            parser.error('RX不能用random生成预期数据')
    return args


def main(argv=None):
    args = parse_args(argv)
    spi = None
    try:
        spi = CH347SPI(args.dev_index, args.dll)
        ceres = Ceres(spi, args.timeout, verbose=args.verbose or args.command is not None)
        if args.command == "read":
            ceres.read_reg(args.address)
        elif args.command == "write":
            ceres.write_reg(args.address, args.value)
        else:
            ceres.init(args.chan, args.mode, args.tx_test_mode)
            if args.mode == 0 and args.tx_test_mode == 0:
                freq = 2400 + args.chan
                ceres.start_single_tone(freq)
                print(f"单音配置已发送: {freq} MHz；关闭USB句柄不会写入停止TX命令。")
            else:
                ceres.drv_ceres_set_rate(0)
                ceres.ceres_nordic_24g_mode_setup()
                ceres.drv_ceres_set_static_length(args.len)
                print(f"{'RX' if args.mode else 'TX'}: {2400 + args.chan} MHz, 1M, len={args.len}, pkt_type={args.pkt_type}")
                if args.mode:
                    ceres.receive_packets(make_payload(args.len, args.pkt_type), args.out_rx_result_time)
                else:
                    ceres.transmit_packets(args.len, args.pkt_type, args.tx_cnt, args.tx_interval)
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("操作被中断", file=sys.stderr)
        return 130
    finally:
        if spi is not None:
            spi.close()


if __name__ == "__main__":
    raise SystemExit(main())
