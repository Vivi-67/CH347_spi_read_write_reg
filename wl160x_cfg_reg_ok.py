import ctypes
import time
from ctypes import Structure, c_ubyte, c_ushort, c_ulong, c_bool, POINTER
from pathlib import Path

# ==============================================================================
# 1. 定义 CH347 相关数据结构与 API 类型映射
# ==============================================================================

class SPI_CONFIG(Structure):
    _fields_ = [
        ("mMode", c_ubyte),               # SPI模式: 0=Mode0 (CPOL=0, CPHA=0)
        ("mClock", c_ubyte),              # 时钟频率: 5 代表 1.875 MHz (满足 SSCK <= 2.5MHz)
        ("mByteOrder", c_ubyte),          # 传输顺序: 0=MSB First
        ("mSPIWriteReadInterval", c_ushort), # 字节/数据块间硬件延时 (单位: us)
        ("mSpiOutDefaultData", c_ubyte),  # 读操作时 MOSI 默认发送的数据 (0x00)
        ("mChipSelect", c_ulong),         # 片选引脚: 0x80 对应 CS0
        ("CS1Polarity", c_ubyte),         # CS1 极性: 0=低有效
        ("CS2Polarity", c_ubyte),         # CS2 极性: 0=低有效
        ("mIsAutoDeactiveCS", c_ushort),  # 1=传输完成后由 CH347 硬件自动拉高 CS
        ("mActiveDelayNum", c_ushort),    # CS使能后的延时 (us)
        ("mDelayNum", c_ulong)            # CS撤销前的延时 (us)
    ]

# 加载 Windows 下的 CH347 驱动动态库
try:
    ch347 = ctypes.windll.LoadLibrary("CH347DLLA64.dll")
except Exception as e:
    print(f"加载 CH347DLLA64.dll 失败，请检查 DLL 文件位置: {e}")
    exit(1)

# API 参数与返回值类型设置
ch347.CH347OpenDevice.argtypes = [c_ulong]
ch347.CH347OpenDevice.restype = c_ulong  # 成功返回句柄/索引，失败返回 INVALID_HANDLE_VALUE (0xFFFFFFFF)

ch347.CH347CloseDevice.argtypes = [c_ulong]
ch347.CH347CloseDevice.restype = c_bool

ch347.CH347SPI_Init.argtypes = [c_ulong, POINTER(SPI_CONFIG)]
ch347.CH347SPI_Init.restype = c_bool

ch347.CH347SPI_WriteRead.argtypes = [c_ulong, c_ulong, c_ulong, POINTER(c_ubyte)]
ch347.CH347SPI_WriteRead.restype = c_bool


# ==============================================================================
# 2. 设备初始化与核心读写封装
# ==============================================================================

def init_ch347_spi(dev_index=0):
    """
    初始化 CH347 SPI 接口
    """
    # 1. 打开设备
    if ch347.CH347OpenDevice(dev_index) == 0xFFFFFFFF:
        print(f"错误: 无法打开 CH347 设备 (Device Index: {dev_index})")
        return False

    # 2. 配置 SPI 参数
    cfg = SPI_CONFIG()
    cfg.mMode = 1                  # SPI Mode 1 (CPOL=0, CPHA=1)
    cfg.mClock = 5                 # 1.875 MHz 时钟 (可设 4=3.75MHz, 5=1.875MHz, 6=937.5KHz)
    cfg.mByteOrder = 1             # 高位在前 (MSB First)
    cfg.mSPIWriteReadInterval = 2  # 硬件级字节间延时设为 2us (满足 >= 450ns 要求)
    cfg.mSpiOutDefaultData = 0x00  # 读操作时 MOSI 默认数据
    cfg.mChipSelect = 0x80         # 使能 CS0
    cfg.CS1Polarity = 0            # CS0 低电平有效
    cfg.CS2Polarity = 0
    cfg.mIsAutoDeactiveCS = 1      # 1: 传输完 2 个字节后自动拉高 CS
    cfg.mActiveDelayNum = 0        # CS 拉低后的初始延时 (us)
    cfg.mDelayNum = 0              # CS 拉高前的保持延时 (us)

    if not ch347.CH347SPI_Init(dev_index, ctypes.byref(cfg)):
        print("错误: CH347SPI_Init 初始化失败")
        return False
    
    print("CH347 SPI 初始化成功 (模式: Mode0, 速率: 1.875MHz, 字节间延时: 2us)")
    return True

def transceiver_transfer_8bit_one_shot(dev_index, reg_addr, data_val=0, is_read=False):
    """
    使用单次 CH347SPI_WriteRead 传输完成 [8bit地址] + [8bit数据]
    在第 1 和第 2 字节传输之间，CS 全程保持低电平，SCK 暂停 2us (满足 >450ns)
    
    :param dev_index: 设备索引 (通常为 0)
    :param reg_addr:  8位寄存器地址 (仅低 7 位有效, 0x00 ~ 0x7F)
    :param data_val:  需要写入的数据 (仅写模式有效)
    :param is_read:   True=读寄存器, False=写寄存器
    :return: (bool 成功状态, int 读取到的值/写入的值)
    """
    # 1. 构造 8-bit 地址字节 (Bit7: 1=Read, 0=Write)
    addr_byte = reg_addr & 0x7F
    if is_read:
        addr_byte |= 0x80   # 读模式 Bit7 = 1
    else:
        addr_byte &= ~0x80  # 写模式 Bit7 = 0

    # 2. 构造 8-bit 数据字节
    # 读模式下，第2个字节 MOSI 填充 0xFF (匹配 mSpiOutDefaultData)，由 MISO 接收 SDIO 数据
    data_byte = 0xFF if is_read else (data_val & 0xFF)

    # 3. 将 地址 和 数据 打包进连续的 2 字节缓冲区
    io_buf = (c_ubyte * 2)(addr_byte, data_byte)

    ch347.CH347SPI_ChangeCS(dev_index, 0)

    # 4. 调用 WriteRead API 执行 2 字节传输
    # 参数 0x80: 选择 CS0，CH347 硬件将在整个传输过程(共2字节)中保持 CS 低电平，
    # 并在字节 1 结束后自动暂停 SCK 时钟 2us，发送完字节 2 后拉高 CS。
    if not ch347.CH347SPI_WriteRead(dev_index, 0x80, 2, io_buf):
        print("错误: CH347SPI_WriteRead 传输失败")
        ch347.CH347SPI_ChangeCS(dev_index, 1)
        return False, 0

    ch347.CH347SPI_ChangeCS(dev_index, 1)
    # 5. 返回传输结果
    if is_read:
        # 传输完成后，io_buf 被 MISO 数据覆写：
        # io_buf[0] 是地址阶段收到的无效数据，io_buf[1] 是读出的寄存器数据
        read_data = io_buf[1]
        return True, read_data
    else:
        return True, data_val


# ==============================================================================
# 3. 从文本文件加载并执行寄存器操作
# ==============================================================================

def _parse_integer(token, file_path, line_number, field_name):
    try:
        return int(token, 0)
    except ValueError as exc:
        raise ValueError(
            f"{file_path}:{line_number}: {field_name} '{token}' 不是有效整数"
        ) from exc


def load_register_operations(file_path):
    """读取并校验寄存器操作文件，返回 (命令, 参数...) 列表。"""
    operations = []

    with file_path.open("r", encoding="utf-8-sig") as register_file:
        for line_number, raw_line in enumerate(register_file, start=1):
            line = raw_line.split("//", 1)[0].split("#", 1)[0].strip()
            if not line:
                continue

            parts = line.split()
            command = parts[0].lower()

            if command == "w" and len(parts) == 3:
                address = _parse_integer(parts[1], file_path, line_number, "地址")
                value = _parse_integer(parts[2], file_path, line_number, "数据")
                if not 0 <= address <= 0x7F:
                    raise ValueError(
                        f"{file_path}:{line_number}: 地址必须在 0x00 到 0x7F 之间"
                    )
                if not 0 <= value <= 0xFF:
                    raise ValueError(
                        f"{file_path}:{line_number}: 数据必须在 0x00 到 0xFF 之间"
                    )
                operations.append((command, address, value))

            elif command == "r" and len(parts) == 2:
                address = _parse_integer(parts[1], file_path, line_number, "地址")
                if not 0 <= address <= 0x7F:
                    raise ValueError(
                        f"{file_path}:{line_number}: 地址必须在 0x00 到 0x7F 之间"
                    )
                operations.append((command, address))

            elif command == "delay" and len(parts) == 2:
                delay_ms = _parse_integer(parts[1], file_path, line_number, "延时")
                if delay_ms < 0:
                    raise ValueError(
                        f"{file_path}:{line_number}: 延时不能为负数"
                    )
                operations.append((command, delay_ms))

            else:
                raise ValueError(
                    f"{file_path}:{line_number}: 不支持的格式 '{line}'；"
                    "请使用 w <地址> <数据>、r <地址> 或 delay <毫秒>"
                )

    if not operations:
        raise ValueError(f"{file_path}: 文件中没有可执行的寄存器操作")

    return operations


def execute_register_operations(dev_index, operations):
    """按文件中的先后顺序执行寄存器操作。"""
    for operation in operations:
        command = operation[0]

        if command == "w":
            _, address, value = operation
            success, _ = transceiver_transfer_8bit_one_shot(
                dev_index, reg_addr=address, data_val=value, is_read=False
            )
            if not success:
                return False
            print(f"[写寄存器成功] 地址 0x{address:02X} <= 0x{value:02X}")

        elif command == "r":
            _, address = operation
            success, read_value = transceiver_transfer_8bit_one_shot(
                dev_index, reg_addr=address, is_read=True
            )
            if not success:
                return False
            print(f"[读寄存器成功] 地址 0x{address:02X} => 0x{read_value:02X}")

        else:
            _, delay_ms = operation
            print(f"[延时] {delay_ms} ms")
            time.sleep(delay_ms / 1000.0)

    return True


def main():
    dev_index = 0
    register_list_path = Path(__file__).resolve().with_name("wl160x_reg_list.txt")

    try:
        operations = load_register_operations(register_list_path)
    except (OSError, ValueError) as exc:
        print(f"寄存器列表加载失败: {exc}")
        return 1

    print(f"已加载 {len(operations)} 条操作: {register_list_path}")
    if not init_ch347_spi(dev_index):
        return 1

    try:
        return 0 if execute_register_operations(dev_index, operations) else 1
    finally:
        ch347.CH347CloseDevice(dev_index)
        print("设备已关闭。")


if __name__ == "__main__":
    raise SystemExit(main())
