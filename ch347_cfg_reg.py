import ctypes
import argparse
import re
import shlex
import time
from ctypes import (
    c_ubyte,
    c_ushort,
    c_ulong,
    c_void_p,
    c_long,
    POINTER,
    Structure,
    byref,
)
from pathlib import Path
from typing import NamedTuple


# ============================================================
# CH347 configuration
# ============================================================

DEV_INDEX = 0

# CH347 CS selection:
# 0x80 -> enable CS1
# 0x81 -> enable CS2
CS1 = 0x80

sclk_dict = {0 : "60 MHz", 
             1 : "30 MHz",
             2 : "15 MHz",
             3 : "7.5 MHz",
             4 : "3.75 MHz",
             5 : "1.875 MHz",
             6 : "937.5 kHz",
             7 : "468.75 kHz"}

# ============================================================
# CH347 SPI configuration structure
# ============================================================

class SPI_CONFIG(Structure):
    _fields_ = [
        ("iMode", c_ubyte),
        ("iClock", c_ubyte),
        ("iByteOrder", c_ubyte),
        ("iSpiWriteReadInterval", c_ushort),
        ("iSpiOutDefaultData", c_ubyte),
        ("iChipSelect", c_ulong),
        ("CS1Polarity", c_ubyte),
        ("CS2Polarity", c_ubyte),
        ("iIsAutoDeativeCS", c_ushort),
        ("iActiveDelay", c_ushort),
        ("iDelayDeactive", c_ulong),
    ]


# ============================================================
# CH347 driver
# ============================================================

class CH347SPI:
    def __init__(self, dll_path=None, dev_index=DEV_INDEX):

        self.dev_index = dev_index

        if dll_path is None:
            dll_path = Path(__file__).resolve().parent / "CH347DLLA64.DLL"

        dll_path = Path(dll_path)

        if not dll_path.exists():
            raise FileNotFoundError(
                f"CH347 DLL not found:\n{dll_path}"
            )

        print(f"Loading DLL: {dll_path}")

        self.dll = ctypes.WinDLL(str(dll_path))

        # --------------------------------------------------------
        # Function prototypes
        # --------------------------------------------------------

        self.dll.CH347OpenDevice.argtypes = [
            c_ulong
        ]
        self.dll.CH347OpenDevice.restype = c_long

        self.dll.CH347CloseDevice.argtypes = [
            c_ulong
        ]
        self.dll.CH347CloseDevice.restype = None

        self.dll.CH347SPI_Init.argtypes = [
            c_ulong,
            POINTER(SPI_CONFIG)
        ]
        self.dll.CH347SPI_Init.restype = c_long

        self.dll.CH347SPI_WriteRead.argtypes = [
            c_ulong,
            c_ulong,
            c_ulong,
            c_void_p
        ]
        self.dll.CH347SPI_WriteRead.restype = c_long

        # --------------------------------------------------------
        # Open device
        # --------------------------------------------------------

        ret = self.dll.CH347OpenDevice(self.dev_index)

        if ret == -1:
            raise RuntimeError(
                f"CH347OpenDevice({self.dev_index}) failed"
            )

        print(f"CH347 device {self.dev_index} opened")

        # --------------------------------------------------------
        # SPI configuration
        # --------------------------------------------------------

        cfg = SPI_CONFIG()

        # Proxima:
        # SPI Mode 0
        cfg.iMode = 0

        # CH347 clock:
        #
        # 0 = 60 MHz
        # 1 = 30 MHz
        # 2 = 15 MHz
        # 3 = 7.5 MHz
        # 4 = 3.75 MHz
        # 5 = 1.875 MHz
        # 6 = 937.5 kHz
        # 7 = 468.75 kHz
        #
        # First test with 1.875 MHz
        cfg.iClock = 5

        # MSB first
        cfg.iByteOrder = 1

        # No extra interval
        cfg.iSpiWriteReadInterval = 0

        # During read, output FF on MOSI
        cfg.iSpiOutDefaultData = 0xFF

        # Enable CS1
        cfg.iChipSelect = CS1

        # CS active low
        cfg.CS1Polarity = 0
        cfg.CS2Polarity = 0

        # Automatically deassert CS after transaction
        cfg.iIsAutoDeativeCS = 1

        # No additional CS delay for first test
        cfg.iActiveDelay = 0
        cfg.iDelayDeactive = 0

        ret = self.dll.CH347SPI_Init(
            self.dev_index,
            byref(cfg)
        )

        if ret == 0:
            self.close()
            raise RuntimeError("CH347SPI_Init failed")

        print("SPI initialized")
        print("  Mode       = %d" %(cfg.iMode))
        print("  Clock      = %s" %(sclk_dict[cfg.iClock]))
        print("  Bit order  = MSB")
        print("  CS         = CS1")
        print()

    # ========================================================
    # Full duplex SPI transfer
    # ========================================================

    def transfer(self, tx: bytes) -> bytes:

        if not isinstance(tx, (bytes, bytearray)):
            raise TypeError("tx must be bytes or bytearray")

        if len(tx) == 0:
            return b""

        # ctypes buffer
        buf = (c_ubyte * len(tx))(*tx)

        ret = self.dll.CH347SPI_WriteRead(
            self.dev_index,
            CS1,
            len(tx),
            ctypes.cast(buf, c_void_p)
        )

        if ret == 0:
            raise RuntimeError(
                "CH347SPI_WriteRead failed"
            )

        # IMPORTANT:
        # The buffer contains the MISO data after WriteRead.
        return bytes(buf)

    # ========================================================
    # Close
    # ========================================================

    def close(self):

        if self.dll is not None:
            self.dll.CH347CloseDevice(self.dev_index)
            self.dll = None
            print("CH347 closed")


# ============================================================
# Proxima SPI protocol
# ============================================================

def build_frame(
    addr: int,
    data: int,
    write: bool,
    width: int = 32
) -> bytes:
    """
    Proxima SPI frame:

        A[15:0]
        1011
        DATA[31:0]
        W[3:0]

    Total:
        16 + 4 + 32 + 4 = 56 bits = 7 bytes

    W[3]:
        1 = write
        0 = read/no-write

    W[1:0]:
        00 = 8 bit
        01 = 16 bit
        10 = 24 bit
        11 = 32 bit
    """

    if not (0 <= addr <= 0xFFFF):
        raise ValueError("addr must be 16-bit")

    if not (0 <= data <= 0xFFFFFFFF):
        raise ValueError("data must be 32-bit")

    width_code = {
        8:  0b00,
        16: 0b01,
        24: 0b10,
        32: 0b11,
    }

    if width not in width_code:
        raise ValueError(
            "width must be 8, 16, 24 or 32"
        )

    w = (
        ((1 if write else 0) << 3)
        | width_code[width]
    )

    frame = (
        ((addr & 0xFFFF) << 40)
        | (0xB << 36)
        | ((data & 0xFFFFFFFF) << 4)
        | w
    )

    return frame.to_bytes(
        7,
        byteorder="big"
    )


def parse_rx_data(rx: bytes) -> int:
    """
    Extract DATA[31:0] from the 56-bit received SPI frame.
    """

    if len(rx) != 7:
        raise ValueError(
            f"Expected 7 bytes, got {len(rx)}"
        )

    value = int.from_bytes(
        rx,
        byteorder="big"
    )

    data = (
        value >> 4
    ) & 0xFFFFFFFF

    return data


# ============================================================
# Proxima register access
# ============================================================

class ProximaSPI:

    def __init__(self, dll_path=None):

        self.spi = CH347SPI(
            dll_path=dll_path
        )

    def write_reg(
        self,
        addr: int,
        value: int,
        width: int = 32,
    ) -> int:

        if width not in (8, 16, 24, 32):
            raise ValueError(
                "width must be 8, 16, 24 or 32"
            )

        max_value = (1 << width) - 1
        if not (0 <= value <= max_value):
            raise ValueError(
                f"value must fit in {width} bits"
            )

        tx = build_frame(
            addr=addr,
            data=value,
            write=True,
            width=width
        )

        print(
            f"WRITE{width:<2} addr=0x{addr:04X} "
            f"data=0x{value:0{width // 4}X}"
        )

        print(
            "  TX:",
            tx.hex(" ")
        )

        rx = self.spi.transfer(tx)

        print(
            "  RX:",
            rx.hex(" ")
        )

        old_value = parse_rx_data(rx) & max_value

        print(
            f"  OLD = 0x{old_value:0{width // 4}X}"
        )

        return old_value

    def read_reg(
        self,
        addr: int,
        width: int = 32,
    ) -> int:

        if width not in (8, 16, 24, 32):
            raise ValueError(
                "width must be 8, 16, 24 or 32"
            )

        tx = build_frame(
            addr=addr,
            data=0,
            write=False,
            width=width
        )

        print(
            f"READ{width:<3} addr=0x{addr:04X}"
        )

        print(
            "  TX:",
            tx.hex(" ")
        )

        rx = self.spi.transfer(tx)

        print(
            "  RX:",
            rx.hex(" ")
        )

        value = parse_rx_data(rx) & ((1 << width) - 1)

        print(
            f"  DATA = 0x{value:0{width // 4}X}"
        )

        return value

    def write_reg32(
        self,
        addr: int,
        value: int
    ) -> int:
        return self.write_reg(addr, value, width=32)

    def set_bits(self, addr: int, high: int, low: int, value: int) -> int:
        """Read a 32-bit register, replace the selected field, and write back."""
        if not (0 <= addr <= 0xFFFF):
            raise ValueError("address must be a 16-bit value")
        _validate_bit_field(high, low, value)
        mask = ((1 << (high - low + 1)) - 1) << low
        old_value = self.read_reg32(addr)
        new_value = (old_value & ~mask) | (value << low)
        print(
            f"SETBITS addr=0x{addr:04X} bits=[{high}:{low}] "
            f"value=0x{value:X} old=0x{old_value:08X} new=0x{new_value:08X}"
        )
        self.write_reg32(addr, new_value)
        return new_value

    def set_bit_list(self, addr: int, bits: tuple, values: tuple) -> int:
        """Replace discrete bits with one 32-bit read and one write."""
        if not (0 <= addr <= 0xFFFF):
            raise ValueError("address must be a 16-bit value")
        _validate_bit_list(bits, values)
        mask = sum(1 << bit for bit in bits)
        field_value = sum(value << bit for bit, value in zip(bits, values))
        old_value = self.read_reg32(addr)
        new_value = (old_value & ~mask) | field_value
        print(
            f"SETBITS addr=0x{addr:04X} bits={bits} values={values} "
            f"old=0x{old_value:08X} new=0x{new_value:08X}"
        )
        self.write_reg32(addr, new_value)
        return new_value

    def read_reg32(
        self,
        addr: int
    ) -> int:
        return self.read_reg(addr, width=32)

    def close(self):
        self.spi.close()


# ============================================================
# Command file
# ============================================================

class RegisterCommand(NamedTuple):
    line_number: int
    operation: str
    arguments: tuple


def _parse_integer(token: str, description: str) -> int:
    try:
        return int(token, 0)
    except ValueError as exc:
        raise ValueError(
            f"invalid {description}: {token!r}"
        ) from exc


def _validate_bit_field(high: int, low: int, value: int) -> None:
    if not (0 <= low <= high <= 31):
        raise ValueError("bit range must satisfy 0 <= low <= high <= 31")
    width = high - low + 1
    if not (0 <= value < (1 << width)):
        raise ValueError(f"value must fit in {width} bits")


def _validate_bit_list(bits: tuple, values: tuple) -> None:
    if not bits or len(bits) != len(values):
        raise ValueError("bit and value lists must be nonempty and have equal lengths")
    if any(not (0 <= bit <= 31) for bit in bits):
        raise ValueError("bit numbers must be between 0 and 31")
    if len(set(bits)) != len(bits):
        raise ValueError("duplicate bit numbers are not allowed")
    if any(value not in (0, 1) for value in values):
        raise ValueError("each bit value must be 0 or 1")


def load_commands(command_path) -> list:
    command_path = Path(command_path)
    commands = []

    with command_path.open("r", encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                fields = shlex.split(
                    line,
                    comments=True,
                    posix=True
                )
            except ValueError as exc:
                raise ValueError(
                    f"{command_path}:{line_number}: {exc}"
                ) from exc

            if not fields:
                continue

            operation = fields[0].lower()

            try:
                if operation == "setbits":
                    # Rejoin tokens so whitespace inside lists is accepted.
                    list_match = re.fullmatch(
                        r"(\S+)\s+\[\s*([0-9]+(?:\s*,\s*[0-9]+)*)\s*\]"
                        r"\s+\(\s*([^()]*)\s*\)",
                        " ".join(fields[1:])
                    )
                    if list_match is not None:
                        addr = _parse_integer(list_match.group(1), "address")
                        if not (0 <= addr <= 0xFFFF):
                            raise ValueError("address must be a 16-bit value")
                        bits = tuple(int(bit.strip()) for bit in list_match.group(2).split(","))
                        values = tuple(
                            _parse_integer(value.strip(), "bit value")
                            for value in list_match.group(3).split(",")
                        )
                        _validate_bit_list(bits, values)
                        commands.append(RegisterCommand(
                            line_number, operation, (addr, bits, values)
                        ))
                        continue
                    if len(fields) != 4:
                        raise ValueError(
                            "setbits requires three arguments: "
                            "setbits <address> [high:low] <value> (or [bit]), "
                            "or setbits <address> [bit,...] (value,...)"
                        )
                    addr = _parse_integer(fields[1], "address")
                    if not (0 <= addr <= 0xFFFF):
                        raise ValueError("address must be a 16-bit value")
                    match = re.fullmatch(r"\[([0-9]+)(?::([0-9]+))?\]", fields[2])
                    if match is None:
                        raise ValueError("bit field must be [high:low] or [bit]")
                    high = int(match.group(1))
                    low = int(match.group(2)) if match.group(2) is not None else high
                    value = _parse_integer(fields[3], "value")
                    _validate_bit_field(high, low, value)
                    commands.append(RegisterCommand(
                        line_number, operation, (addr, high, low, value)
                    ))
                    continue

                if operation == "delay":
                    if len(fields) != 2:
                        raise ValueError(
                            "delay requires one argument: delay <milliseconds>"
                        )

                    milliseconds = _parse_integer(
                        fields[1],
                        "delay"
                    )
                    if milliseconds < 0:
                        raise ValueError("delay must not be negative")

                    commands.append(RegisterCommand(
                        line_number,
                        operation,
                        (milliseconds,)
                    ))
                    continue

                if (
                    len(operation) != 2
                    or operation[0] not in ("r", "w")
                    or operation[1] not in "1234"
                ):
                    raise ValueError(
                        f"unknown operation {fields[0]!r}; "
                        "expected r1-r4, w1-w4, setbits or delay"
                    )

                width = int(operation[1]) * 8
                expected_fields = 2 if operation[0] == "r" else 3
                if len(fields) != expected_fields:
                    if operation[0] == "r":
                        usage = f"{operation} <address>"
                    else:
                        usage = f"{operation} <address> <value>"
                    raise ValueError(
                        f"{operation} requires {expected_fields - 1} "
                        f"argument(s): {usage}"
                    )

                addr = _parse_integer(fields[1], "address")
                if not (0 <= addr <= 0xFFFF):
                    raise ValueError("address must be a 16-bit value")

                if operation[0] == "r":
                    arguments = (addr, width)
                else:
                    value = _parse_integer(fields[2], "value")
                    if not (0 <= value < (1 << width)):
                        raise ValueError(
                            f"value must fit in {width} bits"
                        )
                    arguments = (addr, value, width)

                commands.append(RegisterCommand(
                    line_number,
                    operation,
                    arguments
                ))
            except ValueError as exc:
                raise ValueError(
                    f"{command_path}:{line_number}: {exc}"
                ) from exc

    return commands


def execute_commands(dev, commands) -> None:
    for command in commands:
        print(
            f"[{command.line_number}] {command.operation}"
        )

        if command.operation == "delay":
            milliseconds = command.arguments[0]
            print(f"DELAY   {milliseconds} ms")
            time.sleep(milliseconds / 1000.0)
        elif command.operation == "setbits":
            if len(command.arguments) == 3:
                dev.set_bit_list(*command.arguments)
            else:
                dev.set_bits(*command.arguments)
        elif command.operation.startswith("r"):
            dev.read_reg(*command.arguments)
        else:
            dev.write_reg(*command.arguments)

        print()


def protocol_self_test() -> None:
    tx = build_frame(
        addr=0x1014,
        data=0x0000AAAA,
        write=False,
        width=32
    )

    assert tx == bytes.fromhex(
        "10 14 B0 00 0A AA A3"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Execute Proxima SPI register commands"
    )
    parser.add_argument(
        "command_file",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parent / "cfg_reg_list.txt",
        help="command file (default: cfg_reg_list.txt beside this script)"
    )
    args = parser.parse_args()

    protocol_self_test()
    commands = load_commands(args.command_file)
    print(
        f"Loaded {len(commands)} command(s) from "
        f"{args.command_file}"
    )
    print()

    dev = ProximaSPI()

    try:
        execute_commands(dev, commands)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
