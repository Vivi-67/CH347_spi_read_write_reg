"""CH347 配置 Ceres Tx Single Tone，或读写寄存器。

用法（64 位 Windows Python，DLL 默认位于本脚本目录）：
    python Ceres_cfg_reg.py                         # channel=40，初始化 + 2440 MHz 单音
    python Ceres_cfg_reg.py tone --channel 40        # 0..80 对应 2400..2480 MHz
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


def register_command(address, write=False):
    """对应 drv_ceres_write_reg/read_reg 的 mod/bank/cmd 计算。"""
    if not 0 <= address <= 0xFFFF:
        raise ValueError("地址必须在0x0000..0xFFFF范围内")
    mod, offset = address >> 14, address & 0x3FFF
    bank = ((offset >> 5) | (mod << 6)) & 0xFF
    return bank, (offset & 31) | (0x20 if write else 0x00)


class Ceres:
    def __init__(self, spi, timeout=2.0):
        self.spi, self.timeout = spi, timeout
        self.last_bank = None          # 每次进程首次访问必选bank，避免依赖芯片旧状态

    def frame(self, tx, label):
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
        print(f"WRITE 0x{address:04X} <= 0x{value:08X}")

    def read_reg(self, address):
        bank, cmd = register_command(address)
        self.select_bank(bank)
        rx = self.frame(bytes((cmd, 0xFF, 0xFF, 0xFF, 0xFF)), f"READ 0x{address:04X}; 后4字节dummy")
        print(f"SPI RX: {rx.hex(' ').upper()}")
        value = int.from_bytes(rx[1:5], "little")
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

    def init(self, channel=80):
        """drv_ceres_init(0, CERES_TX_MODE, CERES_24G_MODE, CERES_CHANNEL)。"""
        print("--- drv_ceres_aon_init() ---")
        self.select_bank(0)
        self.write_reg(AON_CTRL, 1 << 16)
        self.write_reg(AON_CTRL, 0)
        self.delay_ms(5)
        # SOFT_RESETN|BYPASS|TXFIFO_LIMIT(30)|RSTB_DBB_REG|XTAL_UP|DBB_UP
        #self.write_reg(AON_CTRL, (1 << 16) | (1 << 17) | (30 << 10) | 0x13)
        self.write_reg(AON_CTRL, (1 << 16) | (1 << 17) | (20 << 10) | 0x13)
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

        print("--- drv_ceres_core_init() / drv_ceres_link_init() ---")
        self.write_reg(AGC_CTRL0, self.read_reg(AGC_CTRL0) | (3 << 2))
        self.write_reg(TXPWRDN, (30 << 16) | (100 << 8) | 10)  # PEER_PLATFORM=0

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
            value = int(token, 0)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"无效整数: {token}") from exc
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(f"必须在{low}..{high}范围内")
        return value
    return parse


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dev-index", type=number_in_range(0, 15), default=0)
    parser.add_argument("--dll", type=Path, help="CH347DLLA64.DLL路径")
    parser.add_argument("--timeout", type=float, default=2.0, help="每次状态轮询超时秒数，默认2")
    sub = parser.add_subparsers(dest="command")
    read = sub.add_parser("read", help="独立读取32位寄存器，不执行初始化")
    read.add_argument("address", type=number_in_range(0, 0xFFFF))
    write = sub.add_parser("write", help="独立写入32位寄存器，不执行初始化")
    write.add_argument("address", type=number_in_range(0, 0xFFFF))
    write.add_argument("value", type=number_in_range(0, 0xFFFFFFFF))
    tone = sub.add_parser("tone", help="执行SDK初始化并开启单音（默认命令）")
    tone.add_argument("--channel", type=number_in_range(0, 80), default=40, help="初始化AFC和单音共用信道，0..80对应2400..2480 MHz，默认40（2440 MHz）")
    args = parser.parse_args(argv)
    if not 0 < args.timeout < float("inf"):
        parser.error("--timeout必须是有限正数")
    spi = None
    try:
        spi = CH347SPI(args.dev_index, args.dll)
        ceres = Ceres(spi, args.timeout)
        if args.command == "read":
            ceres.read_reg(args.address)
        elif args.command == "write":
            ceres.write_reg(args.address, args.value)
        else:
            channel = getattr(args, "channel", 40)
            ceres.init(channel)
            freq = 2400 + channel
            ceres.start_single_tone(freq)
            print(f"单音配置已发送: {freq} MHz；关闭USB句柄不会写入停止TX命令。")
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
